"""UR5e forward kinematics and differential IK from the USD-extracted chain.

The chain in ``specs.UR5E_TCP_CHAIN`` was read out of the robot USD's PhysX
joint frames, so FK here reproduces the simulated robot exactly (validated in
``test/test_kinematics.py`` and, live, against the ``/tf`` ``gripper_tcp``
frame by the insertion node's watchdog).

All poses are ``(pos, quat wxyz)`` in the ``base_link`` frame unless stated.
"""

from __future__ import annotations

import numpy as np

from .specs import ARM_JOINTS, UR5E_TCP_CHAIN, TCP_TO_TIP_POS, TCP_TO_TIP_QUAT
from .transforms import quat_mul, quat_normalize, quat_rotate, yaw_quat

_TCP_TIP = (np.asarray(TCP_TO_TIP_POS), np.asarray(TCP_TO_TIP_QUAT))


def fk_tcp(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Pose of ``gripper_tcp`` in ``base_link`` for arm joint vector ``q``.

    ``q`` is ordered like ``specs.ARM_JOINTS``.
    """

    pose, _, _ = _fk_with_joints(q)
    return pose


def tip_from_tcp(tcp_pose: tuple) -> tuple:
    """Pose of ``sfp_tip_link`` given a ``gripper_tcp`` pose (any frame)."""

    pos, quat = tcp_pose
    return pos + quat_rotate(quat, _TCP_TIP[0]), quat_mul(quat, _TCP_TIP[1])


def tcp_from_tip(tip_pose: tuple) -> tuple:
    """Inverse of :func:`tip_from_tcp`: the TCP pose that puts the tip there."""

    pos, quat = tip_pose
    inv_q = np.array([_TCP_TIP[1][0], *(-_TCP_TIP[1][1:])])
    tcp_quat = quat_mul(quat, inv_q)
    return pos - quat_rotate(tcp_quat, _TCP_TIP[0]), tcp_quat


def jacobian(q: np.ndarray) -> np.ndarray:
    """Geometric Jacobian (6x6) of the TCP point, in ``base_link`` axes.

    Rows: linear x/y/z then angular x/y/z.
    """

    (tcp_pos, _), origins, axes = _fk_with_joints(q)
    J = np.zeros((6, 6))
    for i in range(6):
        z = axes[i]
        J[:3, i] = np.cross(z, tcp_pos - origins[i])
        J[3:, i] = z
    return J


def dls_step(J: np.ndarray, twist: np.ndarray, lam: float) -> np.ndarray:
    """Damped-least-squares joint velocity for a Cartesian twist."""

    JT = J.T
    return JT @ np.linalg.solve(J @ JT + lam * lam * np.eye(6), twist)


def _fk_with_joints(q: np.ndarray):
    """FK returning the TCP pose plus each revolute joint's origin and axis."""

    q_by_name = dict(zip(ARM_JOINTS, np.asarray(q, dtype=float)))
    pos = np.zeros(3)
    quat = np.array([1.0, 0.0, 0.0, 0.0])
    origins, axes = [], []
    for step in UR5E_TCP_CHAIN:
        pos = pos + quat_rotate(quat, np.asarray(step.pos))
        quat = quat_normalize(quat_mul(quat, np.asarray(step.quat)))
        if step.joint is not None:
            origins.append(pos.copy())
            axes.append(quat_rotate(quat, np.array([0.0, 0.0, 1.0])))
            quat = quat_normalize(quat_mul(quat, yaw_quat(q_by_name[step.joint])))
    return (pos, quat), origins, axes
