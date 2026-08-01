# aic_insertion

Vision-based SFP insertion for the AIC UR5e scene. Perception finds the NIC
card from the three wrist cameras; a guarded controller inserts the plug over
`/joint_command`. Design, frame analysis and the approach trade study live in
[`docs/vision-insertion.md`](../../docs/vision-insertion.md) at the repo root.

The runtime uses cameras + `camera_info`, `/tf`, `/joint_states` and
`/wrist_ft/wrench` only. The `/aic/cheat/*` topics are consumed exclusively by
the evaluation script.

## Run

```bash
# 1. Sim (own terminal). 448 px cameras are the documented operating point.
source /opt/ros/jazzy/setup.bash
~/isaacsim-6.0/_build/linux-x86_64/release/python.sh run_sim.py --camera-res 448

# 2. Build + launch the module (own terminal), from the repo root.
source /opt/ros/jazzy/setup.bash
(cd ros2 && colcon build --symlink-install)
source ros2/install/setup.bash
ros2 launch aic_insertion insertion.launch.py            # or control:=false

# 3. Watch (own terminal, repo root, ROS 2 sourced).
ros2 topic echo /aic/insertion/status
python3 ros2/aic_insertion/scripts/eval_perception.py --duration 60
```

The arm starts by ramping to the task home pose, then hovers over the card's
prior region so the wrist cameras can see the ports — expect ~60 s of transit
before the first descent. `/aic/insertion/status` names the current state.

Debug overlays: `/aic/insertion/debug/<camera>` (accepted rectangles green,
track prediction cyan) — add them to the RViz layout next to the raw images.

## Topics

| Topic | Type | Dir |
|---|---|---|
| `/aic/insertion/port_pose` | `PoseWithCovarianceStamped` | pub — card pose, `World` frame |
| `/aic/insertion/entrance_pose` | `PoseStamped` | pub — tip goal, comparable to `/aic/cheat/entrance_pose` |
| `/aic/insertion/seat_pose` | `PoseStamped` | pub — tip goal, comparable to `/aic/cheat/seat_pose` |
| `/aic/insertion/status` | `String` | pub — state machine state |
| `/aic/insertion/debug/<camera>` | `Image` | pub — detector overlay |
| `/joint_command` | `JointState` | **sub side effect** — the controller drives the arm |

## Layout

| | |
|---|---|
| `aic_insertion/specs.py` | vendored model constants (port CAD, tool weld, kinematic chain, priors, thresholds) — checked against sources by `scripts/verify_specs.py` |
| `aic_insertion/transforms.py` | wxyz pose math, ROS msg conversion, quintic |
| `aic_insertion/ur5e_kin.py` | USD-extracted FK, geometric Jacobian, DLS |
| `aic_insertion/fitting.py` | ray∩plane back-projection, metric gates, 2D rigid fit |
| `aic_insertion/detector.py` | classical dark-quad detector (swappable stage) |
| `aic_insertion/estimator.py` | port association, multi-view fusion, track |
| `aic_insertion/controller.py` | state machine + quintic segments (no ROS) |
| `aic_insertion/perception_node.py` | images + TF → goals |
| `aic_insertion/insertion_node.py` | goals → guarded servo → `/joint_command` |
| `scripts/verify_specs.py` | fails on drift between `specs.py` and `aic_sim`/USDs |
| `scripts/eval_perception.py` | vision vs cheat error stats (validation only) |
| `test/` | FK, estimator and projection round-trip unit tests (`python3 -m pytest`) |

## State machine

`WAIT_ESTIMATE → APPROACH → REFINE → (ALIGN ↔ REFINE) → INSERT → SEATED`,
with `INSERT → RETREAT → REFINE` on a force spike or stall (≤3 retries, then
`FAILED`). Motion mirrors the IsaacLab demonstration planner: quintic
segments, 0.6/0.8/0.1 phase speed scales, 5 cm standoff, tip→TCP endpoint
shift, 3 mm / 4° / 0.5 s success criteria. The wrench is tared at the standoff
before each descent.

## Measured behaviour

Numbers from live runs against the sim at `--camera-res 448`, default
(unrandomized) layout:

| | |
|---|---|
| Card pose error vs `/aic/cheat/*` | 0.35 mm in x, 0.61 mm in y, yaw 0.12° |
| Estimate scatter (`std_xy`) once locked | 0.14 mm |
| Corners fused per estimate | 24 (both ports × 4 corners × 3 cameras) |
| Fit residual (RMS) | 0.85 mm |
| Standoff arrival error | 0.7–0.8 mm |
| Lock-on | during the transit, before the hover completes |

Perception locks on while the arm is still moving, so the controller never
waits at the hover. Run `scripts/eval_perception.py` to reproduce the first
rows; `scripts/run_campaign.sh` sweeps randomized layouts end to end.

## Safety rails

- FK is cross-checked against the sim's `/tf` `gripper_tcp` every cycle
  (warn 5 mm, abort 20 mm) — catches any kinematic-convention drift live.
- A pair of openings with the model's 23.2 mm spacing is required to *start*
  a track; single rectangles only refine one (port-0/port-1 ambiguity).
- Estimates gate on residual, prior box, yaw window, freshness and
  sliding-window scatter before the controller may move.
