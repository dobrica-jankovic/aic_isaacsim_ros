"""Model constants for the insertion module, vendored the way this repo vendors
``asset_specs`` from IsaacLab: plain dataclasses, one source file, no runtime
dependency on the simulator.

Every number is *derived* from the repo's own sources — ``aic_sim.specs``
(scene layout, randomization envelope, goal offsets) and the vendored USDs
(port corner frames, joint frames, tool welds). ``scripts/verify_specs.py``
re-derives all of it and fails loudly on drift; do not edit numbers here
without re-running it.

Frames follow the repo conventions: metres, quaternions wxyz, world Z-up,
TF root frame ``World``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


WORLD_FRAME = "World"
TCP_FRAME = "gripper_tcp"
BASE_FRAME = "base_link"

CAMERA_NAMES = ("center_camera", "left_camera", "right_camera")
CAMERA_RGB_TOPIC = "/aic/{name}/rgb"
CAMERA_INFO_TOPIC = "/aic/{name}/camera_info"

PORT_POSE_TOPIC = "/aic/insertion/port_pose"
ENTRANCE_POSE_TOPIC = "/aic/insertion/entrance_pose"
SEAT_POSE_TOPIC = "/aic/insertion/seat_pose"
STATUS_TOPIC = "/aic/insertion/status"
DEBUG_IMAGE_TOPIC = "/aic/insertion/debug/{name}"

JOINT_STATES_TOPIC = "/joint_states"
JOINT_COMMAND_TOPIC = "/joint_command"
WRENCH_TOPIC = "/wrist_ft/wrench"

ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

#: Task default arm pose (mirrors ``aic_sim.specs.UR5E_ARM_JOINT_GROUP``).
HOME_JOINT_POSITIONS = (0.1597, -1.3542, -1.6648, -1.6933, 1.5710, -1.7306)

#: Camera-hover tip position for the OBSERVE state: near side of the card's
#: randomization region, 0.15 m above the entrance plane. Offline-verified to
#: track from home with a bent elbow and to keep the whole prior region inside
#: the wrist cameras' footprint.
OBSERVE_TIP_POS = (0.245, 0.20, 0.30)


@dataclass(frozen=True)
class ChainStep:
    """One link of the serial chain: ``T_parent_child(q) = T(pos, quat) . Rz(q)``.

    Extracted from the USD joint frames (``localPos0``/``localRot0``; every
    ``local1`` frame in this robot is identity and every revolute axis is the
    joint-frame Z). ``joint`` is None for welds.
    """

    pos: tuple[float, float, float]
    quat: tuple[float, float, float, float]  # wxyz
    joint: str | None = None


#: base_link -> gripper_tcp. Source: aic_unified_robot_cable_sdf.usd.
UR5E_TCP_CHAIN: tuple[ChainStep, ...] = (
    ChainStep((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    ChainStep((0.0, 0.0, 0.1625), (1.0, 0.0, 0.0, 0.0), "shoulder_pan_joint"),
    ChainStep((0.0, 0.0, 0.0), (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0), "shoulder_lift_joint"),
    ChainStep((-0.425, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), "elbow_joint"),
    ChainStep((-0.3922, 0.0, 0.1333), (1.0, 0.0, 0.0, 0.0), "wrist_1_joint"),
    ChainStep((0.0, -0.0997, 0.0), (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0), "wrist_2_joint"),
    ChainStep((0.0, 0.0996, 0.0), (math.sqrt(0.5), -math.sqrt(0.5), 0.0, 0.0), "wrist_3_joint"),
    ChainStep((0.0, 0.0, 0.0), (-0.5, 0.5, 0.5, 0.5)),          # wrist_3 -> flange
    ChainStep((0.0, 0.0, 0.0), (0.5, 0.5, 0.5, 0.5)),           # flange -> tool0
    ChainStep((0.0, 0.0, -0.0265), (1.0, 0.0, 0.0, 0.0)),       # tool0 -> cam_mount
    ChainStep((0.0, 0.0, 0.0265), (1.0, 0.0, 0.0, 0.0)),        # cam_mount -> ati_base
    ChainStep((0.0, 0.0, 0.0245), (1.0, 0.0, 0.0, 0.0)),        # ati_base -> ati_tool
    ChainStep((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),           # ati_tool -> gripper_base
    ChainStep((0.0, 0.0, 0.172), (1.0, 0.0, 0.0, 0.0)),         # gripper_base -> gripper_tcp
)

#: gripper_tcp -> sfp_tip_link. The tip is welded to the gripper through the
#: finger -> lc_plug -> sfp_module fixed-joint chain, so this is a rigid tool
#: constant (the compliant rope hangs off the plug and does not move the tip).
#: Measured from the RUNNING simulation (TF gripper_tcp vs the physical tip):
#: the authored USD xforms disagree with the fixed-joint constraint frames by
#: 90 degrees, and PhysX snaps the assembly onto the joints at Play — so this
#: cannot be read from the stage's authored poses.
TCP_TO_TIP_POS = (-0.00431098, -0.01745621, 0.05677496)
TCP_TO_TIP_QUAT = (0.0, 0.0, 0.98901471, 0.14781712)  # wxyz


@dataclass(frozen=True)
class PortRect:
    """One SFP entrance opening: 4 corners in ``nic_card_link`` frame.

    Corner order is the fixed cycle (+x,+z), (+x,-z), (-x,-z), (-x,+z) around
    the entrance origin — the estimator relies on it.
    """

    name: str
    entrance: tuple[float, float, float]
    seat: tuple[float, float, float]
    corners: tuple[tuple[float, float, float], ...]


SFP_PORT_0 = PortRect(
    name="sfp_port_0",
    entrance=(0.01295, -0.07737, 0.00501),
    seat=(0.01295, -0.03157, 0.00501),
    corners=(
        (0.01993, -0.07737, 0.01001),
        (0.01993, -0.07737, 0.00121),
        (0.00597, -0.07737, 0.00121),
        (0.00597, -0.07737, 0.01001),
    ),
)

SFP_PORT_1 = PortRect(
    name="sfp_port_1",
    entrance=(-0.01025, -0.07737, 0.00501),
    seat=(-0.01025, -0.03157, 0.00501),
    corners=(
        (-0.00327, -0.07737, 0.01001),
        (-0.00327, -0.07737, 0.00121),
        (-0.01723, -0.07737, 0.00121),
        (-0.01723, -0.07737, 0.01001),
    ),
)

#: Opening metric dimensions (metres) and the port pair spacing along card X.
PORT_RECT_WIDTH = 0.01396
PORT_RECT_HEIGHT = 0.0088
PORT_PAIR_SPACING = 0.0232

#: Where the tip must go relative to a port frame (mirrors
#: ``aic_sim.specs.AIC_NIC_PORT_0_GOAL.eef_pose_in_port_frame``). The cheat
#: topics publish port frames composed with exactly this offset; we do the
#: same so both streams are directly comparable.
EEF_POS_IN_PORT = (0.0, 0.0, 0.0013)
EEF_QUAT_IN_PORT = (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)  # +90 deg about X


@dataclass(frozen=True)
class CardPrior:
    """Layout knowledge from ``aic_sim.specs`` — the randomization envelope.

    The board moves only in x/y/yaw and the card is carried on a snap grid, so
    the card height, roll and pitch are fixture constants; the estimator's
    unknowns are (x, y, yaw-about-Z) only.
    """

    default_pos: tuple[float, float, float] = (0.25135, 0.25229, 0.0743)
    default_quat: tuple[float, float, float, float] = (0.0, 0.0, -0.7068252, 0.7073883)
    board_default_pos: tuple[float, float, float] = (0.2837, 0.229, 0.0)
    board_xy_range: float = 0.04
    board_yaw_range: float = 0.35
    board_local_offset: tuple[float, float] = (-0.03235, 0.02329)
    slide_range: tuple[float, float] = (0.0, 0.12)
    yaw_margin: float = 0.07
    xy_margin: float = 0.02

    def yaw_bounds(self) -> tuple[float, float]:
        limit = self.board_yaw_range + self.yaw_margin
        return (-limit, limit)

    def xy_bounds(self) -> tuple[tuple[float, float], tuple[float, float]]:
        """Loose (min, max) per axis for the card origin, by sampling the envelope."""
        base = np.asarray(self.board_default_pos[:2])
        points = []
        for yaw in np.linspace(-self.board_yaw_range, self.board_yaw_range, 15):
            c, s = math.cos(yaw), math.sin(yaw)
            for slide in np.linspace(*self.slide_range, 7):
                off = np.array(
                    [self.board_local_offset[0], self.board_local_offset[1] + slide]
                )
                rotated = np.array([c * off[0] - s * off[1], s * off[0] + c * off[1]])
                for dx in (-self.board_xy_range, self.board_xy_range):
                    for dy in (-self.board_xy_range, self.board_xy_range):
                        points.append(base + np.array([dx, dy]) + rotated)
        points = np.asarray(points)
        lo = points.min(axis=0) - self.xy_margin
        hi = points.max(axis=0) + self.xy_margin
        return ((float(lo[0]), float(hi[0])), (float(lo[1]), float(hi[1])))


CARD_PRIOR = CardPrior()


@dataclass(frozen=True)
class ControlSpec:
    """Motion and guard parameters. Mirrors the IsaacLab planner where noted."""

    servo_rate_hz: float = 20.0
    standoff_m: float = 0.05                 # planner: standoff above the entrance
    speed_scale_approach: float = 0.6        # planner phase speed scales
    speed_scale_align: float = 0.8
    speed_scale_insert: float = 0.1
    v_max: float = 0.10                      # m/s cap before phase scaling
    w_max: float = 0.50                      # rad/s cap before phase scaling
    dls_lambda: float = 0.01                 # upstream diff-IK damping
    max_joint_step: float = 0.05             # rad per servo cycle, safety clamp
    # |q_cmd - q_meas| clamp (anti-windup). The stiff PD drives sag under the
    # tool load, so the command must be allowed to lead the measurement by the
    # sag or the tip parks centimetres high; 0.06 rad was measured to cap out
    # at a 20 mm standing error.
    windup_rad: float = 0.25
    settle_tol_m: float = 0.003              # measured error that ends a transit segment
    settle_grace_s: float = 6.0              # extra settling time before moving on anyway
    success_pos_m: float = 0.003             # upstream termination spec
    success_rot_rad: float = math.radians(4.0)
    success_hold_s: float = 0.5
    settle_before_insert_s: float = 1.5
    estimate_max_age_s: float = 1.0
    estimate_max_std_m: float = 0.0015
    # Deviation from tare that means "jam". The wrist F/T carries the compliant
    # fibre cable, whose swing alone moves the reading by up to ~32 N with the
    # plug in free space (measured over an 18 s hold). Anything near a plastic
    # plug's real insertion force is therefore *below this platform's noise
    # floor*: this threshold only catches a hard crash, and jam detection
    # proper is the stall test. Both must persist to fire.
    contact_force_n: float = 45.0
    contact_persist_s: float = 0.5
    stall_progress_m: float = 0.0015         # commanded travel below this proves nothing
    stall_ratio: float = 0.35                # measured/commanded travel that means "stuck"
    stall_window_s: float = 2.0              # window the comparison is made over
    retreat_m: float = 0.02
    max_retries: int = 3
    retry_spiral_m: float = 0.0006
    fk_tf_warn_m: float = 0.005              # FK vs /tf divergence watchdog
    fk_tf_abort_m: float = 0.02


CONTROL = ControlSpec()


@dataclass(frozen=True)
class DetectorSpec:
    """Perception parameters (image side)."""

    roi_margin_m: float = 0.06         # metric margin around the prior/track ROI
    adaptive_block: int = 31           # adaptiveThreshold neighbourhood (odd)
    adaptive_c: int = 7
    min_area_px: float = 20.0
    side_tolerance: float = 0.20       # metric side-length gate, fractional
    angle_tolerance_deg: float = 20.0  # corner right-angle gate
    pair_spacing_tol_m: float = 0.004
    pair_parallel_tol_deg: float = 12.0
    subpix_window: int = 3
    ema_alpha: float = 0.25            # temporal filter on the card estimate
    window: int = 8                    # fits kept for convergence statistics
    min_cameras: int = 1               # cameras that must contribute to a fit
