# AIC Isaac Sim + ROS2

## Goal

Take the AIC task (currently only Isaac Lab **assets + config**) and set it up to
run in **Isaac Sim**, then wire **OmniGraph action graphs** to **ROS2 (Jazzy)**.
Scripts to do this live in this repo.

## Status

- ✅ Scene loads and runs in Isaac Sim (UR5e cable robot + workcell + task board
  + ports + NIC-card target). See `scripts/load_aic_scene.py`.
- ✅ OmniGraph → ROS2 Jazzy bridge complete: joint state/command, 3 cameras,
  wrist F/T wrench, TF, clock. **See [ROS2_BRIDGE.md](ROS2_BRIDGE.md)** for the
  topic map, usage, continuity notes, and next steps.

## Directories

| Purpose | Path |
|---|---|
| Isaac Lab task source (assets + config; pkg `aic_task`) | `/home/etfrobot/IsaacLab/etf_robotics_aic/source` |
| Isaac Sim skills (ref knowledge, incl. `isaac-sim-ros2-bridge`) | `/home/etfrobot/isaacsim-6.0/skills` |
| This project (scripts) | `/home/etfrobot/Documents/dobrica/aic_isaacsim_ros` |

See [REMOTE_SIM_SETUP.md](REMOTE_SIM_SETUP.md) to start the remote sim.
