"""ROS 2 node: perception goals -> guarded insertion over ``/joint_command``.

Owns the servo loop: the state machine emits a tip pose target, differential
IK (damped least squares, matching the upstream controller) turns the pose
error into joint steps, and ramped position targets stream to the bridge.
An FK-vs-TF watchdog cross-checks the kinematic model against the simulator
every cycle and aborts on divergence.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import tf2_ros

from . import ur5e_kin as kin
from .controller import Goals, InsertionStateMachine
from .specs import (
    ARM_JOINTS,
    BASE_FRAME,
    CARD_PRIOR,
    CONTROL,
    EEF_QUAT_IN_PORT,
    ENTRANCE_POSE_TOPIC,
    HOME_JOINT_POSITIONS,
    OBSERVE_TIP_POS,
    JOINT_COMMAND_TOPIC,
    JOINT_STATES_TOPIC,
    PORT_POSE_TOPIC,
    SEAT_POSE_TOPIC,
    STATUS_TOPIC,
    TCP_FRAME,
    WORLD_FRAME,
    WRENCH_TOPIC,
)
from .transforms import (
    compose,
    pose_from_msg,
    quat_mul,
    quat_normalize,
    rotvec_between,
    transform_from_msg,
)

KP_POS = 4.0
KP_ROT = 4.0
HOME_RAMP_S = 4.0


def _observe_pose() -> tuple:
    """Camera hover: tip above the near side of the card's prior region, in
    the insertion orientation, so all three wrist cameras look down at the
    openings. Near-side keeps the arm well inside the workspace and the card
    out of the gripper's blind cone."""

    quat = quat_normalize(
        quat_mul(np.asarray(CARD_PRIOR.default_quat), np.asarray(EEF_QUAT_IN_PORT))
    )
    return np.asarray(OBSERVE_TIP_POS), quat


class InsertionNode(Node):
    def __init__(self):
        super().__init__("aic_insertion")
        self.spec = CONTROL
        self.machine = InsertionStateMachine(self.spec, observe_pose=_observe_pose())
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.q_meas: np.ndarray | None = None
        self.q_cmd: np.ndarray | None = None
        self.T_wb: tuple | None = None
        self.entrance: tuple | None = None
        self.seat: tuple | None = None
        self.goal_stamp = 0.0
        self.std_xy = float("inf")
        self.force_filt: np.ndarray | None = None
        self.force_tare: np.ndarray | None = None
        self.home_start: float | None = None
        self.q_home_from: np.ndarray | None = None
        self.ramp_queue: list | None = None
        self.last_target: tuple | None = None
        self.last_state = ""

        self.pub_cmd = self.create_publisher(JointState, JOINT_COMMAND_TOPIC, 10)
        self.pub_status = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_joints, 10)
        self.create_subscription(WrenchStamped, WRENCH_TOPIC, self._on_wrench, 10)
        self.create_subscription(PoseStamped, ENTRANCE_POSE_TOPIC, self._on_entrance, 10)
        self.create_subscription(PoseStamped, SEAT_POSE_TOPIC, self._on_seat, 10)
        self.create_subscription(PoseWithCovarianceStamped, PORT_POSE_TOPIC, self._on_port, 10)
        self.timer = self.create_timer(1.0 / self.spec.servo_rate_hz, self._tick)
        self.get_logger().info("insertion node up; ramping to home, then waiting on perception")

    # -------------------------------------------------------------- callbacks

    def _on_joints(self, msg: JointState) -> None:
        index = {name: i for i, name in enumerate(msg.name)}
        if all(name in index for name in ARM_JOINTS):
            self.q_meas = np.array([msg.position[index[n]] for n in ARM_JOINTS])

    def _on_wrench(self, msg: WrenchStamped) -> None:
        f = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        self.force_filt = f if self.force_filt is None else 0.8 * self.force_filt + 0.2 * f

    def _on_entrance(self, msg: PoseStamped) -> None:
        self.entrance = pose_from_msg(msg.pose)
        self.goal_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_seat(self, msg: PoseStamped) -> None:
        self.seat = pose_from_msg(msg.pose)

    def _on_port(self, msg: PoseWithCovarianceStamped) -> None:
        self.std_xy = math.sqrt(max(msg.pose.covariance[0] + msg.pose.covariance[7], 0.0))

    # ------------------------------------------------------------------ servo

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.q_meas is None:
            return
        if self.q_cmd is None:
            self.q_cmd = self.q_meas.copy()
        if self.T_wb is None and not self._lookup_base():
            return

        if not self._home_done(now):
            return

        tip_meas = self._tip_pose(self.q_meas)
        gap = 0.0
        if self.last_target is not None:
            gap = float(np.linalg.norm(self.last_target[0] - tip_meas[0]))
        if not self._watchdog(tip_meas):
            self.machine.state = "FAILED"

        target = self.machine.update(
            now, tip_meas, self._goals(now), self._wrench_dev(), gap
        )
        if self.machine.wants_tare and self.force_filt is not None:
            self.force_tare = self.force_filt.copy()
        if target is not None:
            self.last_target = target
        if self.last_target is not None:
            self._servo_to(self.last_target)
        self._publish_cmd()
        self._publish_status()

    def _servo_to(self, tip_target: tuple) -> None:
        # Closed loop on the MEASURED joints: the drives are a stiff PD and sag
        # ~1-2 deg under gravity, so an open-loop command integration parks the
        # tip centimetres off target. Servoing the measurement lets q_cmd float
        # above the target by exactly the sag.
        dt = 1.0 / self.spec.servo_rate_hz
        tcp_target = kin.tcp_from_tip(tip_target)
        tcp_meas = compose(self.T_wb, kin.fk_tcp(self.q_meas))

        v = KP_POS * (tcp_target[0] - tcp_meas[0])
        w = KP_ROT * rotvec_between(tcp_meas[1], tcp_target[1])
        scale = self.machine.segment.speed_scale if self.machine.segment else 1.0
        v = _clamp_norm(v, max(scale * self.spec.v_max, 0.02))
        w = _clamp_norm(w, max(scale * self.spec.w_max, 0.1))

        # Twist into base_link axes: the Jacobian lives there.
        R_wb_inv_v = _rotate_inv(self.T_wb[1], v)
        R_wb_inv_w = _rotate_inv(self.T_wb[1], w)
        twist = np.concatenate([R_wb_inv_v, R_wb_inv_w])
        dq = kin.dls_step(kin.jacobian(self.q_meas), twist, self.spec.dls_lambda) * dt
        dq = np.clip(dq, -self.spec.max_joint_step, self.spec.max_joint_step)
        # Anti-windup: never let the command run far from the measurement, or a
        # jam would integrate unbounded contact force. The descent runs on a
        # tighter clamp — that clamp *is* the force limit for the press fit.
        windup = (
            self.spec.windup_rad_insert
            if self.machine.state == "INSERT"
            else self.spec.windup_rad
        )
        self.q_cmd = np.clip(self.q_cmd + dq, self.q_meas - windup, self.q_meas + windup)

    # ---------------------------------------------------------------- helpers

    def _home_done(self, now: float) -> bool:
        """Joint-space cosine ramp to the task home pose before the machine
        takes over; every later motion is a Cartesian segment from there."""

        if self.ramp_queue is None:
            self.ramp_queue = [(np.asarray(HOME_JOINT_POSITIONS), HOME_RAMP_S)]
        while self.ramp_queue:
            target, duration = self.ramp_queue[0]
            if self.home_start is None:
                if float(np.max(np.abs(self.q_meas - target))) < 0.02:
                    self.ramp_queue.pop(0)
                    continue
                self.home_start = now
                self.q_home_from = self.q_cmd.copy()
            tau = (now - self.home_start) / duration
            if tau >= 1.0:
                self.q_cmd = target.copy()
                self.home_start = None
                self.ramp_queue.pop(0)
                # Hold each waypoint one tick so the drives settle in order.
                self._publish_cmd()
                return False
            a = 0.5 * (1.0 - math.cos(math.pi * tau))
            self.q_cmd = self.q_home_from + a * (target - self.q_home_from)
            self._publish_cmd()
            return False
        return True

    def _goals(self, now: float) -> Goals | None:
        if self.entrance is None or self.seat is None:
            return None
        return Goals(
            entrance=self.entrance, seat=self.seat,
            stamp=self.goal_stamp, std_xy=self.std_xy,
            age=now - self.goal_stamp,
        )

    def _wrench_dev(self) -> float:
        if self.force_filt is None or self.force_tare is None:
            return 0.0
        return float(np.linalg.norm(self.force_filt - self.force_tare))

    def _tip_pose(self, q: np.ndarray) -> tuple:
        return kin.tip_from_tcp(compose(self.T_wb, kin.fk_tcp(q)))

    def _lookup_base(self) -> bool:
        try:
            tf = self.tf_buffer.lookup_transform(WORLD_FRAME, BASE_FRAME, rclpy.time.Time())
        except tf2_ros.TransformException:
            return False
        self.T_wb = transform_from_msg(tf.transform)
        return True

    def _watchdog(self, tip_meas: tuple) -> bool:
        """Compare the kinematic model against the simulator's own TF."""

        try:
            tf = self.tf_buffer.lookup_transform(WORLD_FRAME, TCP_FRAME, rclpy.time.Time())
        except tf2_ros.TransformException:
            return True
        tcp_tf = transform_from_msg(tf.transform)
        tcp_fk = compose(self.T_wb, kin.fk_tcp(self.q_meas))
        err = float(np.linalg.norm(tcp_tf[0] - tcp_fk[0]))
        if err > self.spec.fk_tf_abort_m:
            self.get_logger().error(f"FK vs TF diverged: {err * 1000:.1f} mm — aborting")
            return False
        if err > self.spec.fk_tf_warn_m:
            self.get_logger().warn(
                f"FK vs TF gap {err * 1000:.1f} mm", throttle_duration_sec=5.0
            )
        return True

    def _publish_cmd(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ARM_JOINTS)
        msg.position = [float(v) for v in self.q_cmd]
        self.pub_cmd.publish(msg)

    def _publish_status(self) -> None:
        state = self.machine.state
        if state != self.last_state:
            self.get_logger().info(
                f"state: {self.last_state or 'START'} -> {state} | {self._diagnostics()}"
            )
            self.last_state = state
        msg = String()
        msg.data = f"{state} retries={self.machine.retries}"
        self.pub_status.publish(msg)

    def _diagnostics(self) -> str:
        """One line of the signals that decide transitions — read this first
        when a run ends in FAILED."""

        tip = self._tip_pose(self.q_meas)
        parts = [f"tip=({tip[0][0]:.4f},{tip[0][1]:.4f},{tip[0][2]:.4f})"]
        if self.last_target is not None:
            err = np.linalg.norm(self.last_target[0] - tip[0])
            parts.append(f"track_err={err * 1000:.1f}mm")
        if self.seat is not None:
            parts.append(
                f"seat_err={np.linalg.norm(self.seat[0] - tip[0]) * 1000:.1f}mm"
            )
        parts.append(f"wrench_dev={self._wrench_dev():.1f}N")
        parts.append(f"std_xy={self.std_xy * 1000:.2f}mm")
        parts.append(f"windup={np.max(np.abs(self.q_cmd - self.q_meas)):.3f}rad")
        return " ".join(parts)


def _clamp_norm(v: np.ndarray, limit: float) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n <= limit or n < 1e-12:
        return v
    return v * (limit / n)


def _rotate_inv(quat: np.ndarray, v: np.ndarray) -> np.ndarray:
    from .transforms import quat_inv, quat_rotate

    return quat_rotate(quat_inv(quat), v)


def main(args=None):
    rclpy.init(args=args)
    node = InsertionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
