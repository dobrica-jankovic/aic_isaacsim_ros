# AIC UR5e → ROS 2 (Jazzy) Bridge

OmniGraph action graph that connects the AIC port-insertion scene (UR5e + cable)
to ROS 2. Built on the running Isaac Sim via the `isaac-sim-remote` skill.

## Topics

| Topic | Type | Dir | Source |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | pub | sim time |
| `/joint_states` | `sensor_msgs/JointState` | pub | 6 UR5e arm joints (arm-only, clean names) |
| `/joint_command` | `sensor_msgs/JointState` | **sub** | drives the arm via `IsaacArticulationController` |
| `/tf` | `tf2_msgs/TFMessage` | pub | every rigid-body link (arm, `ati_tool_link`, gripper, cameras) |
| `/aic/{center,left,right}_camera/rgb` | `sensor_msgs/Image` | pub | 224×224 RGB |
| `/aic/{center,left,right}_camera/camera_info` | `sensor_msgs/CameraInfo` | pub | pinhole intrinsics (fx=fy=240.3, cx=cy=112) |
| `/wrist_ft/wrench` | `geometry_msgs/WrenchStamped` | pub | 6D wrench at `ati_tool_link` |

All grounded in the IsaacLab task (`etf_robotics_aic`): arm joint names, camera
intrinsics (`builders.py`), actuator gains stiffness=2000/damping=100
(`robots.py`), and the wrist F/T body `ati_tool_link` (`body_incoming_joint_wrench_b`).

## Usage

```bash
# 1. Start the sim (see REMOTE_SIM_SETUP.md) and wait for port 8226.
# 2. Load the scene:
cd /home/etfrobot/isaacsim-6.0/skills/isaac-sim-remote
python3 scripts/isaacsim_send.py --file <repo>/scripts/load_aic_scene.py
# 3. Play the timeline (bridge only publishes while playing):
python3 scripts/isaacsim_send.py --file <repo>/scripts/simulation_control.py --arg action=play
# 4. Build the ROS 2 graph:
python3 scripts/isaacsim_send.py --file <repo>/scripts/build_ros2_bridge.py

# Verify (source ROS 2 first):
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic echo /joint_states --once
# Drive the arm:
ros2 topic pub /joint_command sensor_msgs/msg/JointState \
  "{name: ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'], position: [-0.5,-1.2,1.0,-1.5,1.0,0.3]}" -r 20
```

## Files (`scripts/`)

- `load_aic_scene.py` — references robot + workcell + board + ports + target at their scene poses.
- `build_ros2_bridge.py` — builds the whole action graph idempotently (removes and recreates `/World/aic/ROS2_Graph`).
- `armstate_body.py` — ScriptNode body: reads the 6 arm joints for `/joint_states`.
- `wrench_body.py` — ScriptNode body: reads the 6D `ati_tool_link` wrench for `/wrist_ft/wrench`.

## Design notes / gotchas

- **Arm-only joint_states**: the native `ROS2PublishJointState` is all-or-nothing
  and would emit all 46 DOF with duplicate cable-joint names (breaks MoveIt). We
  feed it in manual mode from `ReadArmState` (ScriptNode) — clean 6 joints.
- **Wrench**: no native OmniGraph WrenchStamped node. A ScriptNode reads the 6D
  reaction wrench and feeds a generic `ROS2Publisher` (flattened `wrench:force:x`…).
- **ScriptNode bodies live in files** and are loaded via `usePath`/`scriptPath` —
  inlining a multi-line script through `isaacsim_send` corrupts indentation.
- **Physics-view resets**: creating camera render products resets the physics sim
  view, invalidating cached articulation handles. Both ScriptNode bodies re-init
  on failure to stay robust.
- **Publish rate ≈ 10 Hz**: the 3 camera render products gate the shared
  `OnPlaybackTick`, so all publishers run at the render rate. If you need
  high-rate joint/clock/wrench, drive those publishers from a separate
  non-render tick (e.g. a physics-step trigger) instead of `OnPlaybackTick`.
- **JointState target** is the prim carrying `ArticulationRootAPI`
  (`…/aic_unified_robot/root_joint`), not the top Xform.
