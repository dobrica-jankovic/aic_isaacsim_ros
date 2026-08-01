"""Build the AIC scene inside an already-running Isaac Sim.

Sent over the isaac-sim-remote socket, which ships this file's *text* to the
sim, so it cannot rely on ``__file__`` -- hence the hardcoded repo path. Update
:data:`REPO` if the checkout moves.

    cd ~/isaacsim-6.0/skills/isaac-sim-remote
    python3 scripts/isaacsim_send.py --file <repo>/scripts/load_aic_scene.py

The real work is in :mod:`aic_sim.stage`; this is the transport shim.
"""

import sys

REPO = "/home/etfrobot/Documents/dobrica/aic_isaacsim_ros"

if REPO not in sys.path:
    sys.path.insert(0, REPO)
# The sim process is long-lived and caches imports, so drop them to pick up
# edits made since the last send.
for _name in [m for m in list(sys.modules) if m.split(".")[0] == "aic_sim"]:
    del sys.modules[_name]

import omni.usd

from aic_sim.specs import AIC_PORT_INSERTION_LAYOUT
from aic_sim.stage import build_scene, describe_scene

stage = omni.usd.get_context().get_stage()
spawned = build_scene(stage, AIC_PORT_INSERTION_LAYOUT)

print("=== LOAD REPORT ===")
print(describe_scene(stage, spawned))
