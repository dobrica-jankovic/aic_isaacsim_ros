"""Scene-layout contracts for AIC task assemblies."""

from __future__ import annotations

from dataclasses import dataclass

from .base import PoseSpec, SceneSlotSpec, Vector3
from .robots import UR5E_CABLE_ASSET
from .targets import NIC_CARD_ASSET, SC_PORT_ASSET
from .workcells import AIC_WORKCELL_ASSET, TASK_BOARD_ASSET


SCENE_SLOT_ROBOT = "robot"
SCENE_SLOT_TARGET = "target"
SCENE_SLOT_BOARD = "board"
SCENE_SLOT_WORKCELL = "workcell"
SCENE_SLOT_SC_PORT_1 = "sc_port_1"
SCENE_SLOT_SC_PORT_2 = "sc_port_2"


@dataclass(frozen=True)
class AxisRangeSpec:
    """Randomization range for one named axis."""

    axis: str
    bounds: tuple[float, float]


@dataclass(frozen=True)
class AxisSnapSpec:
    """Optional grid snap for one randomized axis."""

    axis: str
    step: float


@dataclass(frozen=True)
class BoardPartRandomizationSpec:
    """A part whose reset pose is anchored to the board pose."""

    slot_name: str
    board_local_offset: Vector3
    pose_ranges: tuple[AxisRangeSpec, ...] = ()
    snap_steps: tuple[AxisSnapSpec, ...] = ()


@dataclass(frozen=True)
class LayoutRandomizationSpec:
    """Reset randomization for the board and board-relative parts."""

    board_slot_name: str
    board_ranges: tuple[AxisRangeSpec, ...]
    board_relative_parts: tuple[BoardPartRandomizationSpec, ...]
    sync_usd_xforms: bool = True


@dataclass(frozen=True)
class SceneLayoutSpec:
    """Physical arrangement of assets for one task layout."""

    name: str
    robot_slot: SceneSlotSpec
    target_slot: SceneSlotSpec
    board_slot: SceneSlotSpec | None = None
    workcell_slot: SceneSlotSpec | None = None
    auxiliary_slots: tuple[SceneSlotSpec, ...] = ()
    randomization: LayoutRandomizationSpec | None = None

    def all_slots(self) -> tuple[SceneSlotSpec, ...]:
        """Return every concrete scene slot in spawn order."""

        slots = [self.robot_slot]
        if self.workcell_slot is not None:
            slots.append(self.workcell_slot)
        if self.board_slot is not None:
            slots.append(self.board_slot)
        slots.extend(self.auxiliary_slots)
        slots.append(self.target_slot)
        return tuple(slots)

    def slot(self, name: str) -> SceneSlotSpec:
        """Return a scene slot by stable scene name."""

        for slot in self.all_slots():
            if slot.name == name:
                return slot
        raise KeyError(f"Unknown scene slot '{name}' for layout '{self.name}'.")


AIC_UR5E_ROBOT_SLOT = SceneSlotSpec(
    name=SCENE_SLOT_ROBOT,
    role="robot",
    asset=UR5E_CABLE_ASSET,
    prim_path="{ENV_REGEX_NS}/Robot",
    pose=PoseSpec(pos=(-0.18, -0.122, 0.0), rot=(0.0, 0.0, 0.0, 1.0)),
    purpose="controlled UR5e cable robot",
)

AIC_WORKCELL_SLOT = SceneSlotSpec(
    name=SCENE_SLOT_WORKCELL,
    role="workcell",
    asset=AIC_WORKCELL_ASSET,
    prim_path="{ENV_REGEX_NS}/workcell",
    pose=PoseSpec(pos=(0.0, 0.0, -1.15), rot=(1.0, 0.0, 0.0, 0.0)),
    purpose="static surrounding AIC workcell",
)

AIC_TASK_BOARD_SLOT = SceneSlotSpec(
    name=SCENE_SLOT_BOARD,
    role="board",
    asset=TASK_BOARD_ASSET,
    prim_path="{ENV_REGEX_NS}/board",
    pose=PoseSpec(pos=(0.2837, 0.229, 0.0), rot=(1.0, 0.0, 0.0, 0.0)),
    kinematic=True,
    purpose="task-board fixture that carries interchangeable parts",
)

AIC_SC_PORT_1_SLOT = SceneSlotSpec(
    name=SCENE_SLOT_SC_PORT_1,
    role="auxiliary",
    asset=SC_PORT_ASSET,
    prim_path="{ENV_REGEX_NS}/sc_port_1",
    pose=PoseSpec(pos=(0.2904, 0.1928, 0.005), rot=(0.73136, 0.0, 0.0, -0.682)),
    kinematic=True,
    purpose="passive SC port in the AIC workcell layout",
)

AIC_SC_PORT_2_SLOT = SceneSlotSpec(
    name=SCENE_SLOT_SC_PORT_2,
    role="auxiliary",
    asset=SC_PORT_ASSET,
    prim_path="{ENV_REGEX_NS}/sc_port_2",
    pose=PoseSpec(pos=(0.2913, 0.1507, 0.005), rot=(0.73136, 0.0, 0.0, -0.682)),
    kinematic=True,
    purpose="second passive SC port in the AIC workcell layout",
)

AIC_NIC_CARD_TARGET_SLOT = SceneSlotSpec(
    name=SCENE_SLOT_TARGET,
    role="target",
    asset=NIC_CARD_ASSET,
    prim_path="{ENV_REGEX_NS}/target",
    pose=PoseSpec(pos=(0.25135, 0.25229, 0.0743), rot=(0.0, 0.0, -0.7068252, 0.7073883)),
    kinematic=True,
    purpose="active target object for SFP insertion",
)

AIC_PORT_INSERTION_RANDOMIZATION = LayoutRandomizationSpec(
    board_slot_name=SCENE_SLOT_BOARD,
    board_ranges=(
        AxisRangeSpec("x", (-0.04, 0.04)),
        AxisRangeSpec("y", (-0.04, 0.04)),
        AxisRangeSpec("yaw", (-0.35, 0.35)),
    ),
    board_relative_parts=(
        BoardPartRandomizationSpec(
            slot_name=SCENE_SLOT_SC_PORT_1,
            board_local_offset=(0.0067, -0.0362, 0.005),
            pose_ranges=(AxisRangeSpec("x", (-0.005, 0.02)),),
        ),
        BoardPartRandomizationSpec(
            slot_name=SCENE_SLOT_SC_PORT_2,
            board_local_offset=(0.0076, -0.0783, 0.005),
            pose_ranges=(AxisRangeSpec("x", (-0.005, 0.02)),),
        ),
        BoardPartRandomizationSpec(
            slot_name=SCENE_SLOT_TARGET,
            board_local_offset=(-0.03235, 0.02329, 0.0743),
            pose_ranges=(AxisRangeSpec("y", (0.0, 0.12)),),
            snap_steps=(AxisSnapSpec("y", 0.04),),
        ),
    ),
)

AIC_PORT_INSERTION_LAYOUT = SceneLayoutSpec(
    name="aic_port_insertion",
    robot_slot=AIC_UR5E_ROBOT_SLOT,
    workcell_slot=AIC_WORKCELL_SLOT,
    board_slot=AIC_TASK_BOARD_SLOT,
    auxiliary_slots=(
        AIC_SC_PORT_1_SLOT,
        AIC_SC_PORT_2_SLOT,
    ),
    target_slot=AIC_NIC_CARD_TARGET_SLOT,
    randomization=AIC_PORT_INSERTION_RANDOMIZATION,
)
