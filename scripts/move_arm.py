#!/usr/bin/env python3
"""Move the AIC UR5e to a joint-space target via /joint_command.

Ramps from the current /joint_states pose to the target so the arm moves
smoothly instead of snapping (the drive gains are stiff: 2000/100).

    source /opt/ros/jazzy/setup.bash
    python3 scripts/move_arm.py -- -0.5 -1.2 1.0 -1.5 1.0 0.3
    python3 scripts/move_arm.py --duration 4 --home

Runs on the ROS side under plain python3. It reads the joint names and the home
pose out of ``aic_sim.specs``, which needs neither Isaac Sim nor torch.
"""
import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from aic_sim.specs import UR5E_ARM_JOINT_GROUP  # noqa: E402

ARM = list(UR5E_ARM_JOINT_GROUP.joint_names)

#: The task's default arm pose, not a generic UR5e home.
HOME = [UR5E_ARM_JOINT_GROUP.default_positions[n] for n in ARM]

RATE_HZ = 50.0


class ArmMover(Node):
    def __init__(self):
        super().__init__("aic_move_arm")
        self.pub = self.create_publisher(JointState, "/joint_command", 10)
        self.current = None
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)

    def _on_state(self, msg):
        # /joint_states is arm-only and ordered like ARM, but match by name anyway
        idx = {n: i for i, n in enumerate(msg.name)}
        if all(n in idx for n in ARM):
            self.current = [msg.position[idx[n]] for n in ARM]

    def wait_for_state(self, timeout=5.0):
        deadline = time.time() + timeout
        while self.current is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.current

    def send(self, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ARM
        msg.position = [float(p) for p in positions]
        self.pub.publish(msg)

    def move_to(self, target, duration, hold=2.0):
        start = self.wait_for_state()
        if start is None:
            self.get_logger().warn("no /joint_states yet — stepping straight to target")
            start = list(target)

        steps = max(1, int(duration * RATE_HZ))
        for i in range(steps + 1):
            # cosine ease-in/out
            a = 0.5 * (1.0 - math.cos(math.pi * i / steps))
            self.send([s + a * (t - s) for s, t in zip(start, target)])
            rclpy.spin_once(self, timeout_sec=1.0 / RATE_HZ)

        # keep commanding the target so it holds against gravity
        for _ in range(int(hold * RATE_HZ)):
            self.send(target)
            rclpy.spin_once(self, timeout_sec=1.0 / RATE_HZ)

        return self.current


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("positions", nargs="*", type=float,
                    help="6 joint angles in radians (%s)" % ", ".join(ARM))
    ap.add_argument("--home", action="store_true",
                    help="go to the spec default arm pose")
    ap.add_argument("--duration", type=float, default=3.0, help="ramp time, seconds")
    args = ap.parse_args()

    if args.home:
        target = HOME
    elif len(args.positions) == 6:
        target = args.positions
    else:
        ap.error("give exactly 6 joint angles, or --home")

    rclpy.init()
    node = ArmMover()
    print("target :", [round(v, 4) for v in target])
    reached = node.move_to(target, args.duration)
    print("reached:", [round(v, 4) for v in (reached or [])])
    if reached:
        err = max(abs(a - b) for a, b in zip(reached, target))
        print("max joint error: %.4f rad" % err)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
