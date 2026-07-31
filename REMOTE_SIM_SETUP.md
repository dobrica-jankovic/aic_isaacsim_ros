# Remote Simulation — Start Here

Drive a running Isaac Sim from the shell by sending Python to its
`python_server` TCP socket (`127.0.0.1:8226`). Full reference:
`/home/etfrobot/isaacsim-6.0/skills/isaac-sim-remote/SKILL.md`.

## 1. Launch (headless, ROS2 Jazzy via system libs)

Source Jazzy first so the ROS2 bridge finds `librmw`/`libament` — otherwise the
bridge fails to start.

```bash
source /opt/ros/jazzy/setup.bash
cd /home/etfrobot/isaacsim-6.0/_build/linux-x86_64/release
bash isaac-sim.sh --no-window --enable isaacsim.code_editor.python_server \
    > /tmp/isaac_sim.log 2>&1 &
```

Add `DISPLAY=:1` (drop `--no-window`) if you want the UI / full-app screenshots.

## 2. Wait until ready

```bash
for i in $(seq 1 120); do nc -z 127.0.0.1 8226 && break; sleep 2; done
grep -E "app ready|ros2.core" /tmp/isaac_sim.log   # expect "app ready", bridge startup, no librmw error
```

## 3. Send code / control sim

```bash
cd /home/etfrobot/isaacsim-6.0/skills/isaac-sim-remote
python3 scripts/isaacsim_send.py 'print("pong")'                     # ping
python3 scripts/isaacsim_send.py --file scripts/health_check.py      # env info
python3 scripts/isaacsim_send.py --file scripts/simulation_control.py --arg action=play
```

## Reconnect (e.g. after clearing the chat)

The sim is a standalone OS process, reachable purely by its socket — no session
state needed. Check if it's still up, then talk to it again:

```bash
nc -z 127.0.0.1 8226 && echo "still up" || echo "gone — relaunch (step 1)"
cd /home/etfrobot/isaacsim-6.0/skills/isaac-sim-remote
python3 scripts/isaacsim_send.py 'print("pong")'
```

If the port is gone, relaunch from step 1.

## Stop

```bash
pkill -9 -f "apps/isaacsim.exp.full.kit"
```

## Notes

- Verified: Isaac Sim **6.0.1**, bridge `isaacsim.ros2.core-1.9.4`, assets on S3.
- Server persists globals across calls; use `--context NAME` for isolation.
- Async ok (top-level `await`); prefer `update_app_async()` in sent code.
