#!/usr/bin/env python3
"""Compare the vision estimate against the ground-truth cheat topics.

Validation-only tooling: subscribes both `/aic/insertion/*` and
`/aic/cheat/*`, prints running error statistics, and optionally appends one
CSV row per sample. The runtime pipeline itself never touches the cheat
topics.

    source /opt/ros/jazzy/setup.bash
    python3 ros2/aic_insertion/scripts/eval_perception.py --duration 30 --csv out.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32, String


class EvalNode(Node):
    def __init__(self, csv_path: str | None):
        super().__init__("aic_insertion_eval")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.latest: dict[str, PoseStamped] = {}
        self.fraction = 0.0
        self.status = ""
        self.errors: list[tuple[float, float, float]] = []
        self.writer = None
        if csv_path:
            handle = open(csv_path, "a", newline="")
            self.writer = csv.writer(handle)
            self.writer.writerow(
                ["t", "err_xy_mm", "err_z_mm", "err_yaw_deg", "fraction", "status"]
            )
        for key, topic in (
            ("est", "/aic/insertion/entrance_pose"),
            ("gt", "/aic/cheat/entrance_pose"),
        ):
            self.create_subscription(
                PoseStamped, topic, lambda msg, k=key: self._on_pose(k, msg), 10
            )
        self.create_subscription(
            Float32, "/aic/cheat/insertion_fraction", self._on_fraction, 10
        )
        self.create_subscription(String, "/aic/insertion/status", self._on_status, 10)
        self.create_timer(0.5, self._sample)

    def _on_pose(self, key, msg):
        self.latest[key] = msg

    def _on_fraction(self, msg):
        self.fraction = float(msg.data)

    def _on_status(self, msg):
        self.status = msg.data

    def _sample(self):
        if "est" not in self.latest or "gt" not in self.latest:
            return
        est, gt = self.latest["est"].pose, self.latest["gt"].pose
        dx = est.position.x - gt.position.x
        dy = est.position.y - gt.position.y
        dz = est.position.z - gt.position.z
        err_xy = math.hypot(dx, dy)
        # Quaternion angle (both wxyz-agnostic: fields are explicit).
        dot = abs(
            est.orientation.w * gt.orientation.w + est.orientation.x * gt.orientation.x
            + est.orientation.y * gt.orientation.y + est.orientation.z * gt.orientation.z
        )
        err_ang = 2.0 * math.degrees(math.acos(min(dot, 1.0)))
        self.errors.append((err_xy, abs(dz), err_ang))
        now = self.get_clock().now().nanoseconds * 1e-9
        line = (
            f"t={now:8.2f}  err_xy={err_xy * 1000:6.2f} mm  err_z={dz * 1000:6.2f} mm  "
            f"err_rot={err_ang:5.2f} deg  fraction={self.fraction:4.2f}  {self.status}"
        )
        print(line, flush=True)
        if self.writer:
            self.writer.writerow(
                [f"{now:.2f}", f"{err_xy * 1000:.3f}", f"{dz * 1000:.3f}",
                 f"{err_ang:.3f}", f"{self.fraction:.3f}", self.status]
            )

    def summary(self):
        if not self.errors:
            return "no samples — did both pipelines publish?"
        arr = np.asarray(self.errors)
        return (
            f"samples={len(arr)}  "
            f"err_xy mm: mean={arr[:, 0].mean() * 1000:.2f} p95={np.percentile(arr[:, 0], 95) * 1000:.2f}  "
            f"err_z mm: mean={arr[:, 1].mean() * 1000:.2f}  "
            f"err_rot deg: mean={arr[:, 2].mean():.2f} p95={np.percentile(arr[:, 2], 95):.2f}  "
            f"final fraction={self.fraction:.2f}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0, help="wall seconds")
    parser.add_argument("--csv", default=None)
    args = parser.parse_args()

    rclpy.init()
    node = EvalNode(args.csv)
    deadline = time.time() + args.duration
    try:
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    print("\nSUMMARY:", node.summary())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
