"""Ground-truth ("cheat") topics for the insertion goal.

Mirrors the IsaacLab task's ``cheatcode`` observation group: the entrance and
seat goal poses, the insertion tip pose, and the ``insertion_fraction`` progress
scalar. All of it goes out as **standard ROS messages** -- ``PoseStamped`` and
``Float32`` -- so nothing downstream has to build a custom ``.msg`` package to
read it.

This is privileged information a real robot cannot measure. It is meant for
dataset recording, scripted demonstrations and asymmetric critics; a policy that
consumes it will not transfer.

Topics, under ``<namespace>/cheat``:

- ``entrance_pose``, ``seat_pose``, ``tip_pose`` -- ``geometry_msgs/PoseStamped``
  in the ``World`` frame
- ``insertion_fraction`` -- ``std_msgs/Float32`` in ``[0, 1]``

The poses are also derivable from ``/tf`` plus the target's port frames, but
publishing them explicitly keeps the fraction verifiable against the same
numbers it was computed from.
"""

from __future__ import annotations

from pxr import Usd

from .graph import GraphBuilder
from .script_nodes import script_body
from .specs import InsertionGoalSpec, SceneLayoutSpec, TargetAssetSpec
from .stage import (
    DEFAULT_ROOT,
    asset_root_prim_path,
    body_prim_path,
    slot_prim_path,
)


WORLD_FRAME = "World"

#: ScriptNode output suffix -> PoseStamped input field.
_POSE_FIELDS = (
    ("px", "pose:position:x"),
    ("py", "pose:position:y"),
    ("pz", "pose:position:z"),
    ("qx", "pose:orientation:x"),
    ("qy", "pose:orientation:y"),
    ("qz", "pose:orientation:z"),
    ("qw", "pose:orientation:w"),
)

_STAMP_FIELDS = (
    ("sec", "header:stamp:sec"),
    ("nanosec", "header:stamp:nanosec"),
)

_POSES = ("entrance", "seat", "tip")


def add_cheat_topics(
    builder: GraphBuilder,
    stage: Usd.Stage,
    layout: SceneLayoutSpec,
    goal: InsertionGoalSpec,
    *,
    root: str = DEFAULT_ROOT,
    namespace: str = "/aic",
) -> None:
    """Add the goal ScriptNode and its publishers. Field wiring is deferred.

    The ``ROS2Publisher`` nodes do not expose their message fields until the
    graph has ticked, so call :func:`connect_cheat_publishers` afterwards.
    """

    target_slot = layout.slot(goal.target_slot)
    target: TargetAssetSpec = target_slot.asset
    port = target.port(goal.port_name)

    target_root = asset_root_prim_path(
        stage, slot_prim_path(target_slot, root), target.usd.root_prim
    )
    if port.entrance_frame_path is None:
        raise ValueError(f"Port '{port.name}' has no entrance frame to publish.")

    robot = layout.robot_slot.asset
    tip_prim = body_prim_path(
        stage,
        slot_prim_path(layout.robot_slot, root),
        robot.body_name_for_role(goal.tip_body_role),
    )

    builder.node("ReadInsertionGoal", "omni.graph.scriptnode.ScriptNode")
    builder.set_input(
        "ReadInsertionGoal",
        "inputs:script",
        script_body(
            "insertion_goal",
            target_prim=target_root,
            entrance_prim=_join(target_root, port.entrance_frame_path),
            seat_prim=_join(target_root, port.seat_frame_path),
            tip_prim=tip_prim,
            eef_pos=list(goal.eef_pose_in_port_frame.pos),
            eef_quat=list(goal.eef_pose_in_port_frame.rot),
            lateral_threshold=goal.lateral_threshold_m,
        ),
    )
    outputs = [("outputs:fraction", "float"), ("outputs:sec", "int"),
               ("outputs:nanosec", "uint")]
    for pose in _POSES:
        outputs += [(f"outputs:{pose}_{field}", "double") for field, _ in _POSE_FIELDS]
    builder.add_outputs("ReadInsertionGoal", outputs)
    builder.connect("OnTick", "outputs:tick", "ReadInsertionGoal", "inputs:execIn")

    for pose in _POSES:
        node = _pose_node(pose)
        builder.node(node, "isaacsim.ros2.bridge.ROS2Publisher")
        builder.set_input(node, "inputs:messagePackage", "geometry_msgs")
        builder.set_input(node, "inputs:messageSubfolder", "msg")
        builder.set_input(node, "inputs:messageName", "PoseStamped")
        builder.set_input(
            node, "inputs:topicName", f"{namespace}/cheat/{pose}_pose"
        )
        builder.connect(
            "ReadInsertionGoal", "outputs:execOut", node, "inputs:execIn"
        )

    builder.node("CheatFractionPub", "isaacsim.ros2.bridge.ROS2Publisher")
    builder.set_input("CheatFractionPub", "inputs:messagePackage", "std_msgs")
    builder.set_input("CheatFractionPub", "inputs:messageSubfolder", "msg")
    builder.set_input("CheatFractionPub", "inputs:messageName", "Float32")
    builder.set_input(
        "CheatFractionPub", "inputs:topicName", f"{namespace}/cheat/insertion_fraction"
    )
    builder.connect(
        "ReadInsertionGoal", "outputs:execOut", "CheatFractionPub", "inputs:execIn"
    )


def connect_cheat_publishers(builder: GraphBuilder) -> None:
    """Wire the ScriptNode outputs into the publishers' message fields."""

    for pose in _POSES:
        node = _pose_node(pose)
        builder.set_input(node, "inputs:header:frame_id", WORLD_FRAME)
        for suffix, field in _POSE_FIELDS + _STAMP_FIELDS:
            source = suffix if suffix in ("sec", "nanosec") else f"{pose}_{suffix}"
            builder.connect(
                "ReadInsertionGoal", f"outputs:{source}", node, f"inputs:{field}"
            )
    builder.connect(
        "ReadInsertionGoal", "outputs:fraction", "CheatFractionPub", "inputs:data"
    )


def _pose_node(pose: str) -> str:
    return f"Cheat{pose.capitalize()}Pub"


def _join(root: str, relative: str) -> str:
    return f"{root.rstrip('/')}/{relative.lstrip('/')}"


__all__ = ["add_cheat_topics", "connect_cheat_publishers"]
