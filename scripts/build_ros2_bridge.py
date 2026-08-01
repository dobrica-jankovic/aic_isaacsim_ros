"""Build the ROS 2 bridge graph inside an already-running Isaac Sim.

Sent over the isaac-sim-remote socket (see ``load_aic_scene.py`` for why
:data:`REPO` is hardcoded). Prerequisites: the scene is loaded and the timeline
is playing -- the ScriptNodes initialise against a live physics view, and the
bridge only publishes during playback.

    python3 scripts/isaacsim_send.py --file <repo>/scripts/build_ros2_bridge.py

The real work is in :mod:`aic_sim.ros2_graph`; this is the transport shim.
"""

import sys

REPO = "/home/etfrobot/Documents/dobrica/aic_isaacsim_ros"

if REPO not in sys.path:
    sys.path.insert(0, REPO)
for _name in [m for m in list(sys.modules) if m.split(".")[0] == "aic_sim"]:
    del sys.modules[_name]

import omni.usd

from aic_sim.ros2_graph import build_bridge, build_report
from aic_sim.specs import AIC_PORT_INSERTION_LAYOUT

builder = build_bridge(omni.usd.get_context().get_stage(), AIC_PORT_INSERTION_LAYOUT)

print("=== BUILD REPORT ===")
print(build_report(builder))
