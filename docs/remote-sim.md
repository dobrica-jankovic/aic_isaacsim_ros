# Driving an already-running Isaac Sim

`run_sim.py` relaunches Isaac Sim every time, which costs a minute or two. When
iterating on the scene builder or the graph, keep one sim up and send Python
into it over its `isaacsim.code_editor.python_server` TCP socket
(`127.0.0.1:8226`) instead. Full reference:
`~/isaacsim-6.0/skills/isaac-sim-remote/SKILL.md`.

## 1. Launch

Source Jazzy first, or the ROS 2 bridge cannot find `librmw`/`libament` and
fails to start.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/isaacsim-6.0/_build/linux-x86_64/release
bash isaac-sim.sh --no-window --enable isaacsim.code_editor.python_server \
    > /tmp/isaac_sim.log 2>&1 &
```

Drop `--no-window` and add `DISPLAY=:1` if you want the UI or full-app
screenshots.

## 2. Wait until ready

```bash
for i in $(seq 1 120); do nc -z 127.0.0.1 8226 && break; sleep 2; done
grep -E "app ready|ros2.core|rclpy loaded" /tmp/isaac_sim.log
```

## 3. Build the scene and the bridge

`scripts/load_aic_scene.py` and `scripts/build_ros2_bridge.py` are shims: they
put this repo on `sys.path`, drop any cached `aic_sim` modules so edits take
effect, and call the same builders `run_sim.py` uses. Nothing is duplicated
between the two paths.

```bash
cd ~/isaacsim-6.0/skills/isaac-sim-remote
REPO=~/Documents/dobrica/aic_isaacsim_ros

python3 scripts/isaacsim_send.py --file $REPO/scripts/load_aic_scene.py
python3 scripts/isaacsim_send.py --file scripts/simulation_control.py --arg action=play
python3 scripts/isaacsim_send.py --file $REPO/scripts/build_ros2_bridge.py
```

The order matters: the ScriptNodes initialise against a live physics view on
their first compute, and the bridge only publishes during playback.

Both shims hardcode `REPO` -- they are shipped to the sim as *text*, so
`__file__` does not exist on the far side. Update the constant if the checkout
moves.

**Watch for stale sends.** The sim caches modules between calls. The shims purge
`aic_sim` on every send, but if you edit a file and send in the same breath you
can still ship the previous version -- and the symptom is a confusing error in
code you just fixed. When a fix appears not to take, send again before debugging
it.

## Reconnecting

The sim is a standalone OS process reachable purely by its socket, so a cleared
chat or a new shell needs no session state:

```bash
nc -z 127.0.0.1 8226 && echo "still up" || echo "gone -- relaunch"
python3 scripts/isaacsim_send.py 'print("pong")'
```

## Stop

```bash
pkill -9 -f "apps/isaacsim.exp.full.kit"
```

## Notes

- Verified against Isaac Sim **6.0.1**, bridge `isaacsim.ros2.core-1.9.4`.
- The server persists globals across calls; use `--context NAME` for isolation.
- Async is fine (top-level `await`); prefer `update_app_async()` in sent code.
