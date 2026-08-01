# The ROS 2 bridge

One OmniGraph action graph at `/World/aic/ROS2_Graph`, built by
`aic_sim/ros2_graph.py` (plus `aic_sim/cheat.py`). It only publishes while the
timeline is playing.

## Topics

| Topic | Type | Dir | Source |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | pub | sim time |
| `/joint_states` | `sensor_msgs/JointState` | pub | the 6 UR5e arm joints, clean names |
| `/joint_command` | `sensor_msgs/JointState` | **sub** | drives the arm via `IsaacArticulationController` |
| `/tf` | `tf2_msgs/TFMessage` | pub | every rigid-body link under the robot, relative to `World` |
| `/aic/{center,left,right}_camera/rgb` | `sensor_msgs/Image` | pub | 224x224 RGB |
| `/aic/{center,left,right}_camera/camera_info` | `sensor_msgs/CameraInfo` | pub | pinhole, fx=fy=240.30, cx=cy=112 |
| `/wrist_ft/wrench` | `geometry_msgs/WrenchStamped` | pub | 6D wrench at `ati_tool_link` |
| `/aic/cheat/entrance_pose` | `geometry_msgs/PoseStamped` | pub | port entrance goal, `World` frame |
| `/aic/cheat/seat_pose` | `geometry_msgs/PoseStamped` | pub | port seat goal, `World` frame |
| `/aic/cheat/tip_pose` | `geometry_msgs/PoseStamped` | pub | `sfp_tip_link` pose, `World` frame |
| `/aic/cheat/insertion_fraction` | `std_msgs/Float32` | pub | insertion progress in `[0, 1]` |

Everything is grounded in the Isaac Lab task (`etf_robotics_aic`): arm joint
names and defaults, camera intrinsics (`builders.py`), actuator gains
stiffness=2000/damping=100 (`robots.py`), the wrist F/T body `ati_tool_link`
(`body_incoming_joint_wrench_b`), and the insertion goal (`InsertionGoalCommand`
plus the `insertion_fraction` observation).

## The cheat topics

`/aic/cheat/*` mirrors the task's `cheatcode` observation group. It is
**privileged information a real robot cannot measure** -- for dataset recording,
scripted demonstrations and asymmetric critics. A policy that consumes it will
not transfer. They are standard messages on purpose: no custom `.msg` package.

`insertion_fraction` decomposes the seat-to-tip error along the entrance->seat
axis and reports how far along the tip has travelled, clamped to `[0, 1]` and
forced to zero once the tip strays more than 2 mm off that line. Verified by
sliding the target along its own insertion axis: the published value tracks the
commanded fraction, reads exactly `1.0` with the tip at the seat, and gates to
zero off-axis.

## Design notes and gotchas

These four cost real time. Do not reintroduce them.

- **`og.Controller.set()` does not author USD.** It writes only the runtime
  (Fabric) value; the stage keeps no opinion, so a saved-and-reopened graph
  silently reverts every input to its OGN default -- `topicName` becomes `/rgb`,
  script bodies become empty. The bridge then comes up *wrong* rather than
  failing loudly. Every input goes through `GraphBuilder.set_input`, which sets
  the runtime value *and* authors the USD attribute. Relationships (`set_rel`)
  were always fine -- they are plain USD.

- **Render products must be created by the graph, not by Python.**
  `rep.create.render_product(...)` authors into the **session layer**, which is
  discarded on save; the reopened stage then has `ROS2CameraHelper` nodes
  pointing at a dead render-product path. Each camera therefore has an
  `isaacsim.core.nodes.IsaacCreateRenderProduct` node feeding the rgb/info
  helpers, so the render product is rebuilt from USD data on every Play.

- **All ScriptNode bodies share one globals dict.** `OgnScriptNode` execs each
  script and merges the names it defined into `compute.__globals__` -- which is
  the *`OgnScriptNode` module's* globals, shared process-wide. Two bodies that
  both define `CONFIG` or `_init` at module level overwrite each other,
  whichever initialised last, and the symptom is a nonsense `KeyError` in an
  unrelated node. Every body in `aic_sim/script_nodes/` therefore keeps its
  config and helpers **inside `compute`**, where they are locals and cannot
  collide.

- **Body names are not prim paths.** One articulation can span sibling branches
  of the slot: the UR5e's `ati_tool_link` sits under `aic_unified_robot` while
  its `sfp_tip_link` sits under `cable/sfp_module`, and there is a *different*
  prim called `sfp_tip_link_robot` next door. Resolve bodies with
  `aic_sim.stage.body_prim_path`, which searches for a rigid body with that
  exact name, rather than composing `<asset root>/<body name>`.

Also worth knowing:

- **Arm-only joint states.** `ROS2PublishJointState` is all-or-nothing and would
  emit all 46 DOF with duplicate cable-joint names, which breaks MoveIt. It runs
  in manual mode fed from the `ReadArmState` ScriptNode.
- **Wrench.** There is no native OmniGraph `WrenchStamped` node, so a ScriptNode
  feeds a generic `ROS2Publisher` with the fields flattened
  (`wrench:force:x` ...). Generic publishers only materialise their message
  fields after the graph has ticked, which is why the builder pumps frames
  before connecting into them.
- **Physics-view resets.** Creating camera render products resets the physics
  simulation view and invalidates cached articulation handles. Every ScriptNode
  body re-initialises on failure.
- **JointState target** is the prim carrying `ArticulationRootAPI`
  (`.../aic_unified_robot/root_joint`), not the top Xform.
- **Publish rate is ~10 Hz** for everything, because the three camera render
  products gate the shared `OnPlaybackTick`.

## The saved stage

`stages/aic_ros2_scene.usd` holds the scene *and* the whole graph, and opens
standalone: launch Isaac Sim with ROS 2 sourced, File -> Open, press Play, and
all 15 topics come up. The ScriptNode bodies are inlined into `inputs:script`,
so the stage does not depend on this repo being checked out.

The specs remain the source of truth. If you change the builders, regenerate it:

```bash
source /opt/ros/jazzy/setup.bash
~/isaacsim-6.0/_build/linux-x86_64/release/python.sh run_sim.py \
    --headless --save $PWD/stages/aic_ros2_scene.usd
```

**The test that catches the whole class of save/reload bug** is: regenerate the
stage, kill the process, relaunch, open the stage, press Play, and check all 15
topics. Nothing else finds it -- the bridge comes up wrong rather than failing.

```bash
~/isaacsim-6.0/_build/linux-x86_64/release/python.sh run_sim.py \
    --headless --stage $PWD/stages/aic_ros2_scene.usd
```

Because that failure is silent, `--stage` prints the reopened graph's node
count, every topic it actually asks for, and any compute errors, so a wrong
reload is visible without a ROS shell. A graph that reverted to OGN defaults
shows up there as `topic /rgb`.
