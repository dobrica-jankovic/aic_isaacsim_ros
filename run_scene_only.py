#!/usr/bin/env python3
"""Launch the AIC scene *without* the robot and randomize it.

    ~/isaacsim-6.0/_build/linux-x86_64/release/python.sh run_scene_only.py

For looking at asset-pose and lighting variation on its own: no arm in the way,
no ROS 2 bridge, no physics stepping. Randomization authors USD xforms directly
(see :mod:`aic_sim.randomize`), which is why the timeline stays stopped.

    # ten layouts, half a second apart, reproducible
    python.sh run_scene_only.py --iterations 10 --interval 0.5 --seed 0

    # same, headless, writing a viewport capture per layout
    python.sh run_scene_only.py --iterations 10 --headless --screenshot-dir /tmp/aic
"""

from __future__ import annotations

import argparse
import random
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, help="seed the sampler for reproducibility")
    parser.add_argument(
        "--iterations",
        type=int,
        default=0,
        help="number of layouts to sample; 0 keeps going until the window closes",
    )
    parser.add_argument(
        "--interval", type=float, default=2.0, help="seconds between layouts"
    )
    parser.add_argument("--headless", action="store_true", help="run without a window")
    parser.add_argument("--robot", action="store_true", help="include the robot too")
    parser.add_argument(
        "--no-light", action="store_true", help="randomize poses but not the dome light"
    )
    parser.add_argument("--screenshot-dir", help="write a viewport capture per layout")
    parser.add_argument("--save", help="save the scene here before randomizing")
    return parser.parse_args()


def main() -> None:
    # Output is nearly always redirected to a log, and a build that looks
    # silent for minutes is hard to tell from a hang.
    sys.stdout.reconfigure(line_buffering=True)

    args = parse_args()

    # SimulationApp has to exist before anything imports omni.* or pxr.
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": args.headless})

    import omni.usd

    from aic_sim.randomize import randomize
    from aic_sim.specs import AIC_DOME_LIGHT, AIC_PORT_INSERTION_LAYOUT
    from aic_sim.stage import build_scene, describe_scene

    context = omni.usd.get_context()
    context.new_stage()
    if args.save:
        # Anchor the root layer so asset references are written relative to it.
        context.save_as_stage(args.save)

    stage = context.get_stage()
    spawned = build_scene(
        stage, AIC_PORT_INSERTION_LAYOUT, include_robot=args.robot
    )
    print(describe_scene(stage, spawned))
    if args.save:
        context.save_stage()

    rng = random.Random(args.seed)
    frames_per_layout = max(1, int(args.interval * 60))
    layout_index = 0
    while app.is_running() and (
        args.iterations == 0 or layout_index < args.iterations
    ):
        poses, light = randomize(
            stage,
            AIC_PORT_INSERTION_LAYOUT,
            rng=rng,
            light=None if args.no_light else AIC_DOME_LIGHT,
        )
        print(f"--- layout {layout_index} ---")
        for name, pose in poses.items():
            pos = ", ".join(f"{value:+.4f}" for value in pose.pos)
            print(f"  {name:12s} pos=({pos})")
        if light is not None:
            color = ", ".join(f"{value:.2f}" for value in light.color)
            print(f"  {'light':12s} intensity={light.intensity:.0f} color=({color})")

        for _ in range(frames_per_layout):
            app.update()
        if args.screenshot_dir:
            _capture(args.screenshot_dir, layout_index, app)
        layout_index += 1

    app.close()


def _capture(directory: str, index: int, app) -> None:
    import os

    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, f"layout_{index:03d}.png")
    capture_viewport_to_file(get_active_viewport(), file_path=path)
    # The capture is queued on the render thread; give it frames to land.
    for _ in range(10):
        app.update()
    print(f"  captured {path}")


if __name__ == "__main__":
    main()
