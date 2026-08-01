"""ROS 2 node: wrist-camera images -> card pose and insertion tip goals.

Subscribes the three wrist cameras plus TF, runs the classical detector and
the multi-view estimator, and publishes:

- ``/aic/insertion/port_pose``      PoseWithCovarianceStamped, the card pose
- ``/aic/insertion/entrance_pose``  PoseStamped, tip goal (cheat-compatible)
- ``/aic/insertion/seat_pose``      PoseStamped, tip goal (cheat-compatible)
- ``/aic/insertion/debug/<camera>`` Image overlays (param ``debug_overlay``)

Runs with ``use_sim_time`` so stamps line up with the bridge.
"""

from __future__ import annotations

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
import cv2
import tf2_ros

from .detector import ClassicalPortDetector, draw_overlay
from .estimator import PortPoseEstimator
from .fitting import project_plane_points
from .specs import (
    CAMERA_INFO_TOPIC,
    CAMERA_NAMES,
    CAMERA_RGB_TOPIC,
    DEBUG_IMAGE_TOPIC,
    DetectorSpec,
    ENTRANCE_POSE_TOPIC,
    PORT_POSE_TOPIC,
    SEAT_POSE_TOPIC,
    WORLD_FRAME,
)
from .transforms import pose_to_msg, transform_from_msg


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("aic_port_perception")
        self.declare_parameter("debug_overlay", True)
        self.debug_overlay = bool(self.get_parameter("debug_overlay").value)

        spec = DetectorSpec()
        self.estimator = PortPoseEstimator(spec)
        self.detector = ClassicalPortDetector(spec, self.estimator.plane_z)
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.camera_info: dict[str, np.ndarray] = {}
        self._published_any = False
        self.pub_port = self.create_publisher(PoseWithCovarianceStamped, PORT_POSE_TOPIC, 10)
        self.pub_entrance = self.create_publisher(PoseStamped, ENTRANCE_POSE_TOPIC, 10)
        self.pub_seat = self.create_publisher(PoseStamped, SEAT_POSE_TOPIC, 10)
        self.pub_debug: dict[str, object] = {}

        for name in CAMERA_NAMES:
            self.create_subscription(
                CameraInfo, CAMERA_INFO_TOPIC.format(name=name),
                lambda msg, n=name: self._on_info(n, msg), qos_profile_sensor_data,
            )
            self.create_subscription(
                Image, CAMERA_RGB_TOPIC.format(name=name),
                lambda msg, n=name: self._on_image(n, msg), qos_profile_sensor_data,
            )
            if self.debug_overlay:
                self.pub_debug[name] = self.create_publisher(
                    Image, DEBUG_IMAGE_TOPIC.format(name=name), 2
                )
        self.get_logger().info("port perception up; waiting for images")

    def _on_info(self, name: str, msg: CameraInfo) -> None:
        self.camera_info[name] = np.asarray(msg.k, dtype=float).reshape(3, 3)

    def _on_image(self, name: str, msg: Image) -> None:
        K = self.camera_info.get(name)
        if K is None:
            return
        cam_pose = self._camera_pose(msg)
        if cam_pose is None:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        rects = self.detector.detect(
            gray, K, cam_pose, self.estimator.roi_world_xy(), name, stamp
        )
        estimate = self.estimator.add_observations(rects, stamp)
        if estimate is not None:
            self._published_any = True
            self._publish(estimate, msg.header.stamp)
        elif not self._published_any:
            # Without this the controller just sits in WAIT_ESTIMATE forever
            # with no stated reason. The usual cause is camera resolution: at
            # the spec default of 224 px the openings are ~11 px wide and the
            # detector finds nothing, so nothing is ever published.
            self.get_logger().warn(
                f"no port estimate yet ({name}: {len(rects)} opening candidates, "
                f"{msg.width}x{msg.height} px). A pair of openings is required to "
                "start a track; if this persists, the cameras are too coarse — "
                "run the sim with --camera-res 448.",
                throttle_duration_sec=5.0,
            )
        if self.debug_overlay and name in self.pub_debug:
            self._publish_overlay(name, bgr, rects, K, cam_pose, msg.header)

    def _camera_pose(self, msg: Image):
        # Look up at the image stamp: the wrist cameras move, and latest-TF
        # skew shifts back-projections by millimetres mid-motion. Fall back to
        # latest only when the buffer cannot serve the stamp yet.
        for when in (rclpy.time.Time.from_msg(msg.header.stamp), rclpy.time.Time()):
            try:
                tf = self.tf_buffer.lookup_transform(
                    WORLD_FRAME, msg.header.frame_id, when
                )
                return transform_from_msg(tf.transform)
            except tf2_ros.TransformException as exc:
                error = exc
        self.get_logger().warn(
            f"TF {msg.header.frame_id}: {error}", throttle_duration_sec=5.0
        )
        return None

    def _publish(self, est, stamp) -> None:
        port = PoseWithCovarianceStamped()
        port.header.stamp = stamp
        port.header.frame_id = WORLD_FRAME
        pose_to_msg((est.card_pos, est.card_quat), port.pose.pose)
        cov = np.zeros((6, 6))
        var_xy = max(est.std_xy, 1e-4) ** 2
        cov[0, 0] = cov[1, 1] = var_xy
        cov[2, 2] = 1e-8
        cov[3, 3] = cov[4, 4] = 1e-8
        cov[5, 5] = max(est.std_yaw, 1e-3) ** 2
        port.pose.covariance = cov.flatten().tolist()
        self.pub_port.publish(port)

        for pub, goal in ((self.pub_entrance, est.entrance_goal), (self.pub_seat, est.seat_goal)):
            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = WORLD_FRAME
            pose_to_msg(goal, msg.pose)
            pub.publish(msg)

    def _publish_overlay(self, name, bgr, rects, K, cam_pose, header) -> None:
        predicted = self.estimator.predicted_corners_xy()
        predicted_px = None
        if predicted is not None:
            predicted_px = project_plane_points(
                predicted, self.estimator.plane_z, K, cam_pose
            )
        out = draw_overlay(bgr, rects, predicted_px)
        msg = self.bridge.cv2_to_imgmsg(out, encoding="bgr8")
        msg.header = header
        self.pub_debug[name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
