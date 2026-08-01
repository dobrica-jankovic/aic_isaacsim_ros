# AIC Isaac Sim + ROS 2

Runs the AIC port-insertion scene (UR5e with a fibre cable, workcell, task board,
SC ports, NIC-card target) in **plain Isaac Sim**, bridged to **ROS 2 Jazzy** over
an OmniGraph action graph.

Upstream the task exists only as an Isaac Lab task
(`~/IsaacLab/etf_robotics_aic/source/aic_task`). This repo is self-contained: the
USD assets are vendored under `assets/` via git-LFS and the layout dataclasses
under `aic_sim/specs/`, so nothing here needs an Isaac Lab checkout or torch at
runtime.

It covers three jobs:

1. **Sim + ROS 2** -- spawn the scene, drive the arm over `/joint_command`, read
   cameras, wrist F/T, TF, and ground-truth insertion goals.
2. **Scene only** -- the same workcell without the robot, for randomizing asset
   poses and lighting. The assets themselves are fixed.
3. **Vision-based insertion** -- `ros2/aic_insertion/`, a ROS 2 package that
   locates the NIC card from the wrist cameras and inserts the plug. It reads
   only what a real robot could measure; the `/aic/cheat/*` topics are its
   yardstick, never its input.

## Quickstart

Isaac Sim's interpreter is `~/isaacsim-6.0/_build/linux-x86_64/release/python.sh`.

### Sim + ROS 2

```bash
source /opt/ros/jazzy/setup.bash        # required: the bridge uses ros_distro=system_default
~/isaacsim-6.0/_build/linux-x86_64/release/python.sh run_sim.py
```

Then, from a ROS 2 shell:

```bash
ros2 topic list                          # 15 topics -- see docs/ros2-bridge.md
ros2 topic echo /joint_states --once
rviz2 -d rviz/aic.rviz                   # 3 camera images + TF

# drive the arm
ros2 topic pub /joint_command sensor_msgs/msg/JointState \
  "{name: ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'],
    position: [-0.5,-1.2,1.0,-1.5,1.0,0.3]}" -r 20
```

`scripts/move_arm.py` ramps to a joint target over `/joint_command` rather than
stepping to it; `--home` returns to the spec's default arm pose.

Useful flags: `--headless`, `--no-ros`, `--save <path.usd>` to write the built
stage out, `--stage <path.usd>` to open one instead of rebuilding.

### Scene only

```bash
~/isaacsim-6.0/_build/linux-x86_64/release/python.sh run_scene_only.py \
    --iterations 10 --interval 1.0 --seed 0
```

Each iteration jitters the board in x/y/yaw, carries the ports and the target
with it, slides the target along its snap grid, and resamples the dome light.
`--seed` makes that reproducible. Add `--headless --screenshot-dir <dir>` to
dump a capture per layout, or `--robot` to keep the arm in frame.

## How it fits together

`aic_sim/specs/` is the source of truth. Poses, joint names, camera prims,
actuator gains, the F/T body and the randomization ranges live there; the scene
builder and the graph builder read them. Adding a second industrial robot should
be a spec change, not a code change -- and nothing in this repo should hardcode a
pose that a spec already carries.

| | |
|---|---|
| `aic_sim/specs/` | asset + layout dataclasses. `base/robots/targets/workcells/scene` mirror Isaac Lab's `asset_specs` package -- keep them a straight mirror. `task.py` is where deliberate divergence lives. |
| `aic_sim/stage.py` | builds a scene on a USD stage from a layout spec |
| `aic_sim/randomize.py` | reset randomization for poses and lighting, in plain USD |
| `aic_sim/ros2_graph.py` | the ROS 2 bridge action graph |
| `aic_sim/cheat.py` | ground-truth insertion-goal topics |
| `aic_sim/graph.py` | OmniGraph authoring helpers |
| `aic_sim/script_nodes/` | ScriptNode bodies, inlined into the graph at build time |
| `run_sim.py`, `run_scene_only.py` | standalone entry points |
| `scripts/` | shims that send the same builders into an already-running sim |
| `stages/aic_ros2_scene.usd` | saved scene + graph, opens standalone in the GUI |
| `assets/` | vendored USD assets (git-LFS) |
| `rviz/aic.rviz` | RViz layout |
| `ros2/aic_insertion/` | ament package: vision-based SFP insertion. Consumes the bridge, drives `/joint_command`. See its own README to run it. |

## Docs

- [ros2/aic_insertion/README.md](ros2/aic_insertion/README.md) -- how to build
  and run the insertion module, its topics, and its measured accuracy.
- [docs/vision-insertion.md](docs/vision-insertion.md) -- the design behind it:
  frame analysis, the approach trade study, what the live runs changed, and
  where it currently stops. **Read §1.2 before touching any transform.**
- [docs/ros2-bridge.md](docs/ros2-bridge.md) -- topic map, design notes, and the
  gotchas that cost real time. **Read the design notes before touching the graph.**
- [docs/remote-sim.md](docs/remote-sim.md) -- driving an already-running Isaac Sim
  over its Python socket, which is much faster to iterate against than relaunching.

## Not done

- **Publish rates are all ~10 Hz.** The three camera render products gate the
  shared `OnPlaybackTick`, so clock, joint states and wrench run at the render
  rate. Driving those from `IsaacOnPhysicsStep` instead would decouple them.
- **Only `rgb` is published.** `CameraFrameSpec.data_types` already drives the
  helper nodes, so adding `depth` or `semantic_segmentation` to the spec should
  be enough -- untested.
- **Randomization authors USD only**, so it applies with the timeline stopped.
  Randomizing mid-playback would need the poses pushed through PhysX as well.
- **No URDF / `robot_state_publisher` / MoveIt.** The `/joint_states` and `/tf`
  we publish are already compatible with that stack if someone wants it.
