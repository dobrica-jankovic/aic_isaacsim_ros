"""Pose math in the repo's conventions: metres, quaternions **wxyz**.

ROS messages carry xyzw fields; only :func:`pose_from_msg` /
:func:`pose_to_msg` touch that ordering, everything else stays wxyz.
A pose is an ``(pos, quat)`` pair of numpy arrays, shapes ``(3,)``/``(4,)``.
"""

from __future__ import annotations

import numpy as np


def quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q)


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product of wxyz quaternions."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def quat_inv(q: np.ndarray) -> np.ndarray:
    """Inverse of a unit wxyz quaternion (its conjugate)."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector ``v`` by unit wxyz quaternion ``q``."""
    w, x, y, z = q
    u = np.array([x, y, z])
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = quat_normalize(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def quat_from_matrix(R: np.ndarray) -> np.ndarray:
    """wxyz quaternion from a rotation matrix (Shepperd's method)."""
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]
    trace = m00 + m11 + m22
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        q = np.array([0.25 / s, (m21 - m12) * s, (m02 - m20) * s, (m10 - m01) * s])
    elif m00 > m11 and m00 > m22:
        s = 2.0 * np.sqrt(1.0 + m00 - m11 - m22)
        q = np.array([(m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s])
    elif m11 > m22:
        s = 2.0 * np.sqrt(1.0 + m11 - m00 - m22)
        q = np.array([(m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s])
    else:
        s = 2.0 * np.sqrt(1.0 + m22 - m00 - m11)
        q = np.array([(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s])
    return quat_normalize(q)


def quat_angle(q1: np.ndarray, q2: np.ndarray) -> float:
    """Short-path rotation magnitude between two unit quaternions, radians."""
    dot = min(abs(float(np.dot(q1, q2))), 1.0)
    return 2.0 * np.arccos(dot)


def quat_slerp(q1: np.ndarray, q2: np.ndarray, s: float) -> np.ndarray:
    """Short-path SLERP; ``s`` in [0, 1]."""
    q1 = quat_normalize(q1)
    q2 = quat_normalize(q2)
    dot = float(np.dot(q1, q2))
    if dot < 0.0:
        q2, dot = -q2, -dot
    if dot > 0.9995:
        return quat_normalize(q1 + s * (q2 - q1))
    theta = np.arccos(min(dot, 1.0))
    return (np.sin((1 - s) * theta) * q1 + np.sin(s * theta) * q2) / np.sin(theta)


def rotvec_between(q_from: np.ndarray, q_to: np.ndarray) -> np.ndarray:
    """Axis-angle vector (world frame) taking ``q_from`` to ``q_to``."""
    dq = quat_mul(q_to, quat_inv(q_from))
    if dq[0] < 0.0:
        dq = -dq
    axis = dq[1:]
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        return np.zeros(3)
    angle = 2.0 * np.arctan2(norm, dq[0])
    return axis / norm * angle


def yaw_quat(yaw: float) -> np.ndarray:
    """wxyz quaternion for a rotation about world Z."""
    return np.array([np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)])


def compose(parent: tuple, child: tuple) -> tuple:
    """Compose child pose onto parent pose; both ``(pos, quat wxyz)``."""
    p_pos, p_quat = parent
    c_pos, c_quat = child
    return p_pos + quat_rotate(p_quat, c_pos), quat_mul(p_quat, c_quat)


def invert(pose: tuple) -> tuple:
    pos, quat = pose
    inv_q = quat_inv(quat)
    return -quat_rotate(inv_q, pos), inv_q


def pose_from_msg(msg) -> tuple:
    """geometry_msgs/Pose -> (pos, quat wxyz)."""
    p, o = msg.position, msg.orientation
    return np.array([p.x, p.y, p.z]), np.array([o.w, o.x, o.y, o.z])


def pose_to_msg(pose: tuple, msg) -> None:
    """(pos, quat wxyz) -> geometry_msgs/Pose (written in place)."""
    pos, quat = pose
    msg.position.x, msg.position.y, msg.position.z = map(float, pos)
    msg.orientation.w = float(quat[0])
    msg.orientation.x = float(quat[1])
    msg.orientation.y = float(quat[2])
    msg.orientation.z = float(quat[3])


def transform_from_msg(msg) -> tuple:
    """geometry_msgs/Transform -> (pos, quat wxyz)."""
    t, r = msg.translation, msg.rotation
    return np.array([t.x, t.y, t.z]), np.array([r.w, r.x, r.y, r.z])


def quintic(tau: float) -> float:
    """Rest-to-rest quintic time scaling ``s(tau) = 10t^3 - 15t^4 + 6t^5``."""
    tau = min(max(tau, 0.0), 1.0)
    return tau ** 3 * (10.0 - 15.0 * tau + 6.0 * tau ** 2)
