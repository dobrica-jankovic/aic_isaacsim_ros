"""Robot asset contracts used by AIC tasks."""

from __future__ import annotations

from dataclasses import dataclass

from .base import (
    ActuatorSpec,
    AssetIdentity,
    AssetSpec,
    BodyRoleSpec,
    CameraFrameSpec,
    JointGroupSpec,
    RobotSpawnSpec,
    UsdAssetInterface,
    asset_path,
)


ROBOT_ROLE_TCP = "tcp"
ROBOT_ROLE_EEF = "eef"
ROBOT_ROLE_WRIST_FT = "wrist_ft"


@dataclass(frozen=True)
class RobotAssetSpec(AssetSpec):
    """Asset-level contract for a robot articulation."""

    joint_groups: tuple[JointGroupSpec, ...]
    body_roles: tuple[BodyRoleSpec, ...]
    camera_frames: tuple[CameraFrameSpec, ...]
    actuators: tuple[ActuatorSpec, ...]
    spawn: RobotSpawnSpec

    def joint_group(self, name: str) -> JointGroupSpec:
        """Return a named joint group."""

        for group in self.joint_groups:
            if group.name == name:
                return group
        raise KeyError(f"Unknown joint group '{name}' for robot asset '{self.name}'.")

    def body_role(self, role: str) -> BodyRoleSpec:
        """Return the concrete body currently assigned to a semantic role."""

        for body_role in self.body_roles:
            if body_role.role == role:
                return body_role
        raise KeyError(f"Unknown body role '{role}' for robot asset '{self.name}'.")

    def body_name_for_role(self, role: str) -> str:
        """Return only the concrete body name for a semantic role."""

        return self.body_role(role).body_name

    def camera_frame(self, name: str) -> CameraFrameSpec:
        """Return a named camera frame exposed by this robot asset."""

        for frame in self.camera_frames:
            if frame.name == name:
                return frame
        raise KeyError(f"Unknown camera frame '{name}' for robot asset '{self.name}'.")

    def actuator(self, name: str) -> ActuatorSpec:
        """Return actuator defaults by actuator name."""

        for actuator in self.actuators:
            if actuator.name == name:
                return actuator
        raise KeyError(f"Unknown actuator '{name}' for robot asset '{self.name}'.")


UR5E_ARM_JOINT_GROUP = JointGroupSpec(
    name="arm",
    joint_names=(
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ),
    default_positions={
        "shoulder_pan_joint": 0.1597,
        "shoulder_lift_joint": -1.3542,
        "elbow_joint": -1.6648,
        "wrist_1_joint": -1.6933,
        "wrist_2_joint": 1.5710,
        "wrist_3_joint": -1.7306,
    },
)

UR5E_ARM_ACTUATOR = ActuatorSpec(
    name="arm",
    joint_group=UR5E_ARM_JOINT_GROUP.name,
    effort_limit_sim=87.0,
    stiffness=2000.0,
    damping=100.0,
)

UR5E_CABLE_ASSET = RobotAssetSpec(
    identity=AssetIdentity(name="ur5e_cable", role="robot"),
    usd=UsdAssetInterface(kind="articulation", root_prim="aic_unified_robot"),
    usd_path=asset_path("robots", "ur5e_cable", "aic_unified_robot_cable_sdf.usd"),
    joint_groups=(UR5E_ARM_JOINT_GROUP,),
    body_roles=(
        BodyRoleSpec(
            role=ROBOT_ROLE_TCP,
            body_name="gripper_tcp",
            purpose="body controlled by the Differential IK action",
        ),
        BodyRoleSpec(
            role=ROBOT_ROLE_EEF,
            body_name="sfp_tip_link",
            purpose="physical insertion tip used for goals and terminations",
        ),
        BodyRoleSpec(
            role=ROBOT_ROLE_WRIST_FT,
            body_name="ati_tool_link",
            purpose="6D F/T sensor mount — source of the wrist wrench observation",
        ),
    ),
    camera_frames=(
        CameraFrameSpec(
            name="center_camera",
            relative_prim_path="aic_unified_robot/center_camera_optical/center_camera",
        ),
        CameraFrameSpec(
            name="left_camera",
            relative_prim_path="aic_unified_robot/left_camera_optical/left_camera",
        ),
        CameraFrameSpec(
            name="right_camera",
            relative_prim_path="aic_unified_robot/right_camera_optical/right_camera",
        ),
    ),
    actuators=(UR5E_ARM_ACTUATOR,),
    spawn=RobotSpawnSpec(
        max_depenetration_velocity=5.0,
        enabled_self_collisions=True,
        solver_position_iteration_count=32,
        solver_velocity_iteration_count=16,
        activate_contact_sensors=False,
    ),
)
