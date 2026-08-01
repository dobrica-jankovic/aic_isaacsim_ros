#!/usr/bin/env python3
"""Launch the AIC scene with its ROS 2 bridge in a standalone Isaac Sim.

    ~/isaacsim-6.0/_build/linux-x86_64/release/python.sh run_sim.py

Source ROS 2 first (``source /opt/ros/jazzy/setup.bash``) -- the bridge uses
``ros_distro=system_default`` and will not start without it.

The scene is built from ``aic_sim.specs``, which is the source of truth. Use
``--save`` to write the result out as a ``.usd`` that the GUI can open directly,
and ``--stage`` to open one of those instead of rebuilding.
"""

from __future__ import annotations

import argparse
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--stage", help="open this .usd instead of building the scene from specs"
    )
    parser.add_argument(
        "--save", help="save the built stage here (assets get repo-relative paths)"
    )
    parser.add_argument("--headless", action="store_true", help="run without a window")
    parser.add_argument(
        "--no-ros", action="store_true", help="build the scene but not the ROS 2 bridge"
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=0,
        help="exit after this many frames instead of running until closed",
    )
    return parser.parse_args()


def main() -> None:
    # Output is nearly always redirected to a log, and a build that looks
    # silent for minutes is hard to tell from a hang.
    sys.stdout.reconfigure(line_buffering=True)

    args = parse_args()

    # SimulationApp has to exist before anything imports omni.* or pxr, so the
    # imports below are deliberately not at module scope.
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": args.headless})

    import omni.timeline
    import omni.usd
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("omni.graph.scriptnode")
    enable_extension("isaacsim.ros2.bridge")
    app.update()

    from aic_sim.ros2_graph import build_bridge, build_report
    from aic_sim.specs import AIC_PORT_INSERTION_LAYOUT
    from aic_sim.stage import build_scene, describe_scene

    context = omni.usd.get_context()
    if args.stage:
        context.open_stage(args.stage)
    else:
        context.new_stage()
        if args.save:
            # Anchor the root layer before authoring, so asset references are
            # written relative to it and the saved stage stays portable.
            context.save_as_stage(args.save)
        spawned = build_scene(context.get_stage(), AIC_PORT_INSERTION_LAYOUT)
        print(describe_scene(context.get_stage(), spawned))
    for _ in range(10):
        app.update()

    # Play before building the graph: the ScriptNodes initialise against a live
    # physics view on their first compute.
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(10):
        app.update()

    if not args.no_ros and not args.stage:
        builder = build_bridge(context.get_stage(), AIC_PORT_INSERTION_LAYOUT)
        print(build_report(builder))

    if args.save:
        context.save_stage()
        print(f"saved stage: {args.save}")

    print("running -- Ctrl-C or close the window to stop")
    frames = 0
    while app.is_running() and (args.steps == 0 or frames < args.steps):
        app.update()
        frames += 1
    app.close()


if __name__ == "__main__":
    main()
