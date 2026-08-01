"""AIC Isaac Sim + ROS 2 helpers.

Layout::

    specs/          asset + layout dataclasses (no IsaacLab, no torch)
    script_nodes/   ScriptNode bodies, inlined into the graph at build time
    stage.py        build a scene on a USD stage from a layout spec
    randomize.py    reset randomization for poses and lighting, in plain USD
    graph.py        OmniGraph authoring helpers
    ros2_graph.py   the ROS 2 bridge action graph
    cheat.py        ground-truth insertion-goal topics

``specs`` and ``randomize`` need only ``pxr``; everything else needs a running
Isaac Sim. Import them lazily so ``specs`` stays usable under plain ``python3``.
"""
