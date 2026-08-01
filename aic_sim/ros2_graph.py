"""Build the ROS 2 bridge action graph for an AIC layout.

Everything robot-specific -- joint names, camera prims, actuator gains, the F/T
sensor body -- is read from the layout's robot :class:`RobotAssetSpec`, so
another industrial robot means another spec, not another graph.

Topics (see also :mod:`aic_sim.cheat` for the ground-truth ones):

===================================  ==========================  ===
``/clock``                           ``rosgraph_msgs/Clock``     pub
``/joint_states``                    ``sensor_msgs/JointState``  pub
``/joint_command``                   ``sensor_msgs/JointState``  sub
``/tf``                              ``tf2_msgs/TFMessage``      pub
``/aic/<camera>/rgb``                ``sensor_msgs/Image``       pub
``/aic/<camera>/camera_info``        ``sensor_msgs/CameraInfo``  pub
``/wrist_ft/wrench``                 ``geometry_msgs/Wrench...`` pub
===================================  ==========================  ===

The bridge only publishes while the timeline is playing.
"""

from __future__ import annotations

import carb.settings
import omni.kit.app
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from .cheat import add_cheat_topics, connect_cheat_publishers
from .graph import GraphBuilder
from .script_nodes import script_body
from .specs import (
    AIC_CAMERA_LENS,
    AIC_NIC_PORT_0_GOAL,
    ROBOT_ROLE_WRIST_FT,
    CameraLensSpec,
    InsertionGoalSpec,
    RobotAssetSpec,
    SceneLayoutSpec,
)
from .stage import (
    DEFAULT_ROOT,
    articulation_root_path,
    asset_root_prim_path,
    slot_prim_path,
)


DEFAULT_GRAPH_PATH = "/World/aic/ROS2_Graph"
ARM_JOINT_GROUP = "arm"

_WRENCH_FIELDS = (
    ("fx", "wrench:force:x"),
    ("fy", "wrench:force:y"),
    ("fz", "wrench:force:z"),
    ("tx", "wrench:torque:x"),
    ("ty", "wrench:torque:y"),
    ("tz", "wrench:torque:z"),
    ("sec", "header:stamp:sec"),
    ("nanosec", "header:stamp:nanosec"),
)


def build_bridge(
    stage: Usd.Stage,
    layout: SceneLayoutSpec,
    *,
    root: str = DEFAULT_ROOT,
    graph_path: str = DEFAULT_GRAPH_PATH,
    namespace: str = "/aic",
    lens: CameraLensSpec = AIC_CAMERA_LENS,
    goal: InsertionGoalSpec | None = AIC_NIC_PORT_0_GOAL,
) -> GraphBuilder:
    """Create the bridge graph, replacing any graph already at ``graph_path``."""

    robot: RobotAssetSpec = layout.robot_slot.asset
    robot_slot = slot_prim_path(layout.robot_slot, root)
    robot_root = asset_root_prim_path(stage, robot_slot, robot.usd.root_prim)
    arm = robot.joint_group(ARM_JOINT_GROUP)

    # ScriptNodes are opt-in by default and the opt-in prompt is a UI dialog,
    # which never resolves headless.
    carb.settings.get_settings().set("/app/omni.graph.scriptnode/enable_opt_in", False)

    _apply_actuator_gains(stage, robot_root, robot)

    builder = GraphBuilder(stage, graph_path)
    builder.node("OnTick", "omni.graph.action.OnPlaybackTick")
    builder.node("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime")

    _add_clock(builder)
    _add_joint_states(builder, robot_root, arm.joint_names)
    _add_joint_command(builder, articulation_root_path(stage, robot_slot))
    _add_cameras(builder, stage, robot_slot, robot, lens, namespace)
    _add_transform_tree(builder, stage, robot_root)

    ft_body = robot.body_name_for_role(ROBOT_ROLE_WRIST_FT)
    _add_wrench(builder, robot_root, ft_body)
    if goal is not None:
        add_cheat_topics(builder, stage, layout, goal, root=root, namespace=namespace)

    # The generic ROS2Publisher nodes only materialise their message fields
    # (inputs:wrench:force:x and friends) once the graph has ticked, so every
    # connection into them has to wait.
    _pump(40)
    builder.set_input("WrenchPub", "inputs:header:frame_id", ft_body)
    for src, dst in _WRENCH_FIELDS:
        builder.connect("ReadWrench", f"outputs:{src}", "WrenchPub", f"inputs:{dst}")
    if goal is not None:
        connect_cheat_publishers(builder)
    _pump(40)
    return builder


def _pump(frames: int) -> None:
    app = omni.kit.app.get_app()
    for _ in range(frames):
        app.update()


def _apply_actuator_gains(
    stage: Usd.Stage, robot_root: str, robot: RobotAssetSpec
) -> None:
    """Write the spec's actuator gains onto the joint drives.

    The raw USD ships with much lower stiffness (~95-100) than the IsaacLab
    actuator model uses, and tracks ``/joint_command`` poorly under gravity.
    """

    for actuator in robot.actuators:
        joints = set(robot.joint_group(actuator.joint_group).joint_names)
        for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_root)):
            if prim.GetName() not in joints or not prim.IsA(UsdPhysics.Joint):
                continue
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if drive:
                drive.GetStiffnessAttr().Set(actuator.stiffness)
                drive.GetDampingAttr().Set(actuator.damping)


def _add_clock(builder: GraphBuilder) -> None:
    builder.node("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock")
    builder.set_input("PublishClock", "inputs:topicName", "/clock")
    builder.connect("OnTick", "outputs:tick", "PublishClock", "inputs:execIn")
    builder.connect(
        "ReadSimTime", "outputs:simulationTime", "PublishClock", "inputs:timeStamp"
    )


def _add_joint_states(
    builder: GraphBuilder, robot_root: str, joints: tuple[str, ...]
) -> None:
    builder.node("ReadArmState", "omni.graph.scriptnode.ScriptNode")
    builder.set_input(
        "ReadArmState",
        "inputs:script",
        script_body("arm_state", robot_prim=robot_root, joints=list(joints)),
    )
    builder.add_outputs(
        "ReadArmState",
        [
            ("outputs:names", "token[]"),
            ("outputs:positions", "double[]"),
            ("outputs:velocities", "double[]"),
            ("outputs:efforts", "double[]"),
            ("outputs:dofTypes", "uchar[]"),
        ],
    )

    builder.node("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState")
    builder.set_input("PublishJointState", "inputs:topicName", "/joint_states")
    builder.set_input("PublishJointState", "inputs:stageMetersPerUnit", 1.0)
    builder.connect("OnTick", "outputs:tick", "ReadArmState", "inputs:execIn")
    builder.connect(
        "ReadArmState", "outputs:execOut", "PublishJointState", "inputs:execIn"
    )
    builder.connect(
        "ReadSimTime", "outputs:simulationTime", "PublishJointState", "inputs:timeStamp"
    )
    for src, dst in (
        ("names", "jointNames"),
        ("positions", "jointPositions"),
        ("velocities", "jointVelocities"),
        ("efforts", "jointEfforts"),
        ("dofTypes", "jointDofTypes"),
    ):
        builder.connect(
            "ReadArmState", f"outputs:{src}", "PublishJointState", f"inputs:{dst}"
        )


def _add_joint_command(builder: GraphBuilder, articulation_root: str) -> None:
    builder.node("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState")
    builder.set_input("SubscribeJointState", "inputs:topicName", "/joint_command")
    builder.node("ArtController", "isaacsim.core.nodes.IsaacArticulationController")
    # The target is the prim carrying ArticulationRootAPI, not the slot Xform.
    builder.set_relationship("ArtController", "inputs:targetPrim", [articulation_root])
    builder.connect("OnTick", "outputs:tick", "SubscribeJointState", "inputs:execIn")
    builder.connect(
        "SubscribeJointState", "outputs:execOut", "ArtController", "inputs:execIn"
    )
    for field in ("jointNames", "positionCommand", "velocityCommand", "effortCommand"):
        builder.connect(
            "SubscribeJointState", f"outputs:{field}", "ArtController", f"inputs:{field}"
        )


def _add_cameras(
    builder: GraphBuilder,
    stage: Usd.Stage,
    robot_slot: str,
    robot: RobotAssetSpec,
    lens: CameraLensSpec,
    namespace: str,
) -> None:
    for frame in robot.camera_frames:
        camera_path = f"{robot_slot}/{frame.relative_prim_path.lstrip('/')}"
        camera = UsdGeom.Camera.Define(stage, camera_path)
        camera.CreateFocalLengthAttr(lens.focal_length)
        camera.CreateHorizontalApertureAttr(lens.horizontal_aperture)
        camera.CreateVerticalApertureAttr(lens.vertical_aperture)
        camera.CreateClippingRangeAttr(Gf.Vec2f(*lens.clipping_range))

        # USD cameras look down -Z with +Y up; ROS optical frames are +Z forward
        # with +Y down. 180 degrees about X converts between them.
        xform = UsdGeom.Xformable(camera.GetPrim())
        xform.ClearXformOpOrder()
        xform.AddOrientOp().Set(Gf.Quatf(0.0, 1.0, 0.0, 0.0))

        # The camera prim's parent is the optical frame /tf publishes it under.
        frame_id = camera_path.rsplit("/", 2)[1]

        # The render product must be created *by the graph*. A Python-side
        # rep.create.render_product authors into the session layer, which is
        # dropped on save, leaving the helpers pointing at a dead path. This
        # node recreates it from USD data on every play.
        product = f"Cam_{frame.name}_rp"
        builder.node(product, "isaacsim.core.nodes.IsaacCreateRenderProduct")
        builder.set_relationship(product, "inputs:cameraPrim", [camera_path])
        builder.set_input(product, "inputs:width", frame.width)
        builder.set_input(product, "inputs:height", frame.height)
        builder.connect("OnTick", "outputs:tick", product, "inputs:execIn")

        for data_type in frame.data_types:
            helper = f"Cam_{frame.name}_{data_type}"
            builder.node(helper, "isaacsim.ros2.bridge.ROS2CameraHelper")
            builder.set_input(helper, "inputs:type", data_type)
            builder.set_input(
                helper, "inputs:topicName", f"{namespace}/{frame.name}/{data_type}"
            )
            builder.set_input(helper, "inputs:frameId", frame_id)
            builder.connect(
                product, "outputs:renderProductPath", helper, "inputs:renderProductPath"
            )
            builder.connect(product, "outputs:execOut", helper, "inputs:execIn")

        info = f"Cam_{frame.name}_info"
        builder.node(info, "isaacsim.ros2.bridge.ROS2CameraInfoHelper")
        builder.set_input(
            info, "inputs:topicName", f"{namespace}/{frame.name}/camera_info"
        )
        builder.set_input(info, "inputs:frameId", frame_id)
        builder.connect(
            product, "outputs:renderProductPath", info, "inputs:renderProductPath"
        )
        builder.connect(product, "outputs:execOut", info, "inputs:execIn")


def _add_transform_tree(
    builder: GraphBuilder, stage: Usd.Stage, robot_root: str
) -> None:
    links = [
        prim.GetPath().pathString
        for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_root))
        if prim.HasAPI(UsdPhysics.RigidBodyAPI)
    ]
    builder.node("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree")
    builder.set_input("PublishTF", "inputs:topicName", "/tf")
    builder.set_relationship("PublishTF", "inputs:parentPrim", ["/World"])
    builder.set_relationship("PublishTF", "inputs:targetPrims", links)
    builder.connect("OnTick", "outputs:tick", "PublishTF", "inputs:execIn")
    builder.connect(
        "ReadSimTime", "outputs:simulationTime", "PublishTF", "inputs:timeStamp"
    )


def _add_wrench(builder: GraphBuilder, robot_root: str, body: str) -> None:
    builder.node("ReadWrench", "omni.graph.scriptnode.ScriptNode")
    builder.set_input(
        "ReadWrench",
        "inputs:script",
        script_body("wrench", robot_prim=robot_root, body=body),
    )
    builder.add_outputs(
        "ReadWrench",
        [(f"outputs:{name}", "double") for name in ("fx", "fy", "fz", "tx", "ty", "tz")]
        + [("outputs:sec", "int"), ("outputs:nanosec", "uint")],
    )

    builder.node("WrenchPub", "isaacsim.ros2.bridge.ROS2Publisher")
    builder.set_input("WrenchPub", "inputs:messagePackage", "geometry_msgs")
    builder.set_input("WrenchPub", "inputs:messageSubfolder", "msg")
    builder.set_input("WrenchPub", "inputs:messageName", "WrenchStamped")
    builder.set_input("WrenchPub", "inputs:topicName", "/wrist_ft/wrench")
    builder.connect("OnTick", "outputs:tick", "ReadWrench", "inputs:execIn")
    builder.connect("ReadWrench", "outputs:execOut", "WrenchPub", "inputs:execIn")


def build_report(builder: GraphBuilder) -> str:
    errors = builder.compute_errors()
    lines = [f"nodes: {builder.node_count()}"]
    lines.extend(f"  ERROR {error}" for error in errors)
    lines.append("ROS2 BRIDGE BUILT" if not errors else "ROS2 BRIDGE BUILT WITH ERRORS")
    return "\n".join(lines)


__all__ = ["DEFAULT_GRAPH_PATH", "build_bridge", "build_report"]
