#!/usr/bin/env python3
"""Cross-check aic_insertion's vendored constants against their sources.

Sources: ``aic_sim.specs`` (layout, goal offset, home pose, randomization)
and the vendored USDs (port frames, joint frames, tool weld). Run from the
repo root with pxr importable — the lightweight way, no Isaac Sim session:

    EXT=~/isaacsim-6.0/_build/linux-x86_64/release/extscache/omni.usd.libs-*
    PYTHONPATH=$EXT:. LD_LIBRARY_PATH=$EXT/bin \
        python3 ros2/aic_insertion/scripts/verify_specs.py

Exits non-zero on any drift.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "ros2" / "aic_insertion"))

from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402

from aic_insertion import specs as ins  # noqa: E402
from aic_sim.specs import (  # noqa: E402
    AIC_NIC_PORT_0_GOAL,
    AIC_PORT_INSERTION_LAYOUT,
    UR5E_ARM_JOINT_GROUP,
)

FAILURES: list[str] = []


def check(name: str, got, want, atol=1e-6) -> None:
    got = np.asarray(got, dtype=float)
    want = np.asarray(want, dtype=float)
    if got.shape != want.shape or not np.allclose(got, want, atol=atol):
        FAILURES.append(f"{name}: package has {got.tolist()}, source says {want.tolist()}")


def rel_pose(cache, stage, a: str, b: str):
    m = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(b)) * \
        cache.GetLocalToWorldTransform(stage.GetPrimAtPath(a)).GetInverse()
    t = m.ExtractTranslation()
    q = m.ExtractRotationQuat().GetNormalized()
    i = q.GetImaginary()
    return np.array([t[0], t[1], t[2]]), np.array([q.GetReal(), i[0], i[1], i[2]])


def verify_layout() -> None:
    layout = AIC_PORT_INSERTION_LAYOUT
    target = layout.target_slot
    board = layout.board_slot
    check("card default pos", ins.CARD_PRIOR.default_pos, target.pose.pos)
    check("card default quat", ins.CARD_PRIOR.default_quat, target.pose.rot)
    check("board default pos", ins.CARD_PRIOR.board_default_pos, board.pose.pos)

    rand = layout.randomization
    ranges = {r.axis: r.bounds for r in rand.board_ranges}
    check("board xy range", [ins.CARD_PRIOR.board_xy_range], [ranges["x"][1]])
    check("board yaw range", [ins.CARD_PRIOR.board_yaw_range], [ranges["yaw"][1]])
    part = next(p for p in rand.board_relative_parts if p.slot_name == target.name)
    check("board local offset", ins.CARD_PRIOR.board_local_offset, part.board_local_offset[:2])
    check("slide range", ins.CARD_PRIOR.slide_range, dict((r.axis, r.bounds) for r in part.pose_ranges)["y"])

    goal = AIC_NIC_PORT_0_GOAL
    check("eef pos in port", ins.EEF_POS_IN_PORT, goal.eef_pose_in_port_frame.pos)
    check("eef quat in port", ins.EEF_QUAT_IN_PORT, goal.eef_pose_in_port_frame.rot)
    home = [UR5E_ARM_JOINT_GROUP.default_positions[n] for n in ins.ARM_JOINTS]
    check("home joint positions", ins.HOME_JOINT_POSITIONS, home)


def verify_nic_usd() -> None:
    stage = Usd.Stage.Open(str(REPO / "assets/targets/nic_card/nic_card.usd"))
    cache = UsdGeom.XformCache()
    root = "/nic_card_link"
    for port, prefix in ((ins.SFP_PORT_0, "sfp_port_0"), (ins.SFP_PORT_1, "sfp_port_1")):
        ent = f"{root}/{prefix}_link/{prefix}_link_entrance"
        t, _ = rel_pose(cache, stage, root, f"{root}/{prefix}_link")
        check(f"{prefix} seat", port.seat, t, atol=1e-5)
        t, _ = rel_pose(cache, stage, root, ent)
        check(f"{prefix} entrance", port.entrance, t, atol=1e-5)
        for i, corner in enumerate(("front_left", "back_left", "back_right", "front_right")):
            t, _ = rel_pose(cache, stage, root, f"{ent}/{prefix}_{corner}")
            check(f"{prefix} corner {corner}", port.corners[i], t, atol=1e-5)


def _quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def _quat_rot(q, v):
    w = q[0]
    u = np.asarray(q[1:])
    v = np.asarray(v, dtype=float)
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def _joint_frames(stage, name):
    """(local0, local1) frames of a joint, each as (pos, quat wxyz)."""

    for prim in stage.Traverse():
        if prim.GetName() == name and prim.IsA(UsdPhysics.Joint):
            j = UsdPhysics.Joint(prim)
            out = []
            for pos_attr, rot_attr in (
                (j.GetLocalPos0Attr(), j.GetLocalRot0Attr()),
                (j.GetLocalPos1Attr(), j.GetLocalRot1Attr()),
            ):
                p = np.array(list(pos_attr.Get()))
                r = rot_attr.Get()
                i = r.GetImaginary()
                out.append((p, np.array([r.GetReal(), i[0], i[1], i[2]])))
            return out
    raise KeyError(name)


def _compose(a, b):
    return a[0] + _quat_rot(a[1], b[0]), _quat_mul(a[1], b[1])


def _invert(a):
    q = np.array([a[1][0], *(-np.asarray(a[1][1:]))])
    return -_quat_rot(q, a[0]), q


def _fixed_step(stage, joint):
    """T_body0->body1 enforced by a fixed joint: T(l0) . T(l1)^-1."""

    l0, l1 = _joint_frames(stage, joint)
    return _compose(l0, _invert(l1))


def verify_robot_usd() -> None:
    stage = Usd.Stage.Open(
        str(REPO / "assets/robots/ur5e_cable/aic_unified_robot_cable_sdf.usd")
    )

    # The tool constant must come from the JOINT constraint frames — the
    # authored xforms disagree with them by 90 deg and PhysX snaps the
    # assembly onto the joints at Play.
    base_tcp = _fixed_step(stage, "gripper_attach_tool_frame")
    base_finger = _fixed_step(stage, "gripper_right_finger_joint")
    tip = _compose(
        _compose(
            _compose(_invert(base_tcp), base_finger),
            _fixed_step(stage, "gripper_attach_joint"),
        ),
        _compose(
            _fixed_step(stage, "sfp_module_joint"),
            _fixed_step(stage, "sfp_tip_joint"),
        ),
    )
    check("tcp->tip pos (joint frames)", ins.TCP_TO_TIP_POS, tip[0], atol=2e-5)
    q = tip[1]
    if np.dot(q, ins.TCP_TO_TIP_QUAT) < 0:  # q and -q are the same rotation
        q = -q
    check("tcp->tip quat (joint frames)", ins.TCP_TO_TIP_QUAT, q, atol=1e-4)

    joints = {}
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Joint):
            joints[prim.GetName()] = prim
    revolute = [s for s in ins.UR5E_TCP_CHAIN if s.joint is not None]
    for step in revolute:
        prim = joints[step.joint]
        j = UsdPhysics.Joint(prim)
        pos0 = np.array(list(j.GetLocalPos0Attr().Get()))
        rot0 = j.GetLocalRot0Attr().Get()
        i = rot0.GetImaginary()
        quat0 = np.array([rot0.GetReal(), i[0], i[1], i[2]])
        if np.dot(quat0, step.quat) < 0:
            quat0 = -quat0
        check(f"{step.joint} frame pos", step.pos, pos0, atol=1e-5)
        check(f"{step.joint} frame quat", step.quat, quat0, atol=1e-5)
        axis = UsdPhysics.RevoluteJoint(prim).GetAxisAttr().Get()
        if axis != "Z":
            FAILURES.append(f"{step.joint}: axis is {axis}, chain assumes Z")


def main() -> int:
    verify_layout()
    verify_nic_usd()
    verify_robot_usd()
    if FAILURES:
        print("SPEC DRIFT DETECTED:")
        for failure in FAILURES:
            print(f"  {failure}")
        return 1
    print("aic_insertion specs match aic_sim.specs and the USDs")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
