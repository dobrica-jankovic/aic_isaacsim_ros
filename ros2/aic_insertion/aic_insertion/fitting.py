"""Plane-geometry primitives for port perception. Pure numpy, no ROS.

The entrance openings lie in a horizontal world plane at a fixture-constant
height (see specs), which turns pose estimation into 2D: pixel rays are
intersected with that plane and the known rectangle model is fitted to the
resulting points with a closed-form 2D rigid alignment.
"""

from __future__ import annotations

import numpy as np

from .transforms import quat_rotate, quat_to_matrix


def backproject_to_plane(
    pts_px: np.ndarray,
    K: np.ndarray,
    cam_pose: tuple,
    plane_z: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Intersect pixel rays with the horizontal plane ``z = plane_z``.

    ``cam_pose`` is the optical frame in world, ``(pos, quat wxyz)``, ROS
    optical convention (+Z forward). Returns ``(xy (N,2), valid (N,))``;
    points whose rays run (near) parallel to the plane are marked invalid.
    """

    pts = np.atleast_2d(np.asarray(pts_px, dtype=float))
    cam_pos, cam_quat = cam_pose
    R = quat_to_matrix(cam_quat)
    ones = np.ones((pts.shape[0], 1))
    rays_optical = np.linalg.solve(K, np.hstack([pts, ones]).T).T
    rays_world = rays_optical @ R.T
    dz = rays_world[:, 2]
    valid = np.abs(dz) > 1e-6
    lam = np.where(valid, (plane_z - cam_pos[2]) / np.where(valid, dz, 1.0), 0.0)
    valid &= lam > 0.0
    points = cam_pos[None, :] + lam[:, None] * rays_world
    return points[:, :2], valid


def project_plane_points(
    pts_xy: np.ndarray,
    plane_z: float,
    K: np.ndarray,
    cam_pose: tuple,
) -> np.ndarray:
    """Project world points on the plane into pixel coordinates. (N,2)->(N,2)."""

    pts = np.atleast_2d(np.asarray(pts_xy, dtype=float))
    world = np.hstack([pts, np.full((pts.shape[0], 1), plane_z)])
    cam_pos, cam_quat = cam_pose
    R = quat_to_matrix(cam_quat)
    optical = (world - cam_pos[None, :]) @ R
    z = np.where(np.abs(optical[:, 2]) < 1e-9, 1e-9, optical[:, 2])
    uvw = optical @ K.T
    return uvw[:, :2] / z[:, None]


def order_cyclic_ccw(pts: np.ndarray) -> np.ndarray:
    """Order 4 points counter-clockwise (in standard xy axes) around their centroid."""

    pts = np.asarray(pts, dtype=float)
    centroid = pts.mean(axis=0)
    angles = np.arctan2(pts[:, 1] - centroid[1], pts[:, 0] - centroid[0])
    return pts[np.argsort(angles)]


def rect_metrics(plane_pts: np.ndarray) -> dict:
    """Side lengths, corner angles, centre and long-axis angle of a plane quad.

    ``plane_pts`` must be cyclically ordered. ``long_axis`` is the direction of
    the longer side pair, mod pi, in ``[-pi/2, pi/2)``.
    """

    pts = np.asarray(plane_pts, dtype=float)
    sides = np.roll(pts, -1, axis=0) - pts
    lengths = np.linalg.norm(sides, axis=1)
    angles = []
    for i in range(4):
        a = -sides[i - 1]
        b = sides[i]
        cosang = np.dot(a, b) / max(np.linalg.norm(a) * np.linalg.norm(b), 1e-12)
        angles.append(np.degrees(np.arccos(np.clip(cosang, -1.0, 1.0))))
    pair_a = 0.5 * (lengths[0] + lengths[2])
    pair_b = 0.5 * (lengths[1] + lengths[3])
    if pair_a >= pair_b:
        long_vec = sides[0] - sides[2]
        long_len, short_len = pair_a, pair_b
    else:
        long_vec = sides[1] - sides[3]
        long_len, short_len = pair_b, pair_a
    axis = np.arctan2(long_vec[1], long_vec[0])
    if axis >= np.pi / 2:
        axis -= np.pi
    elif axis < -np.pi / 2:
        axis += np.pi
    return {
        "center": pts.mean(axis=0),
        "long_len": float(long_len),
        "short_len": float(short_len),
        "angles_deg": np.asarray(angles),
        "long_axis": float(axis),
    }


def rect_gate(metrics: dict, width: float, height: float, side_tol: float, angle_tol_deg: float) -> bool:
    """Metric acceptance test for a candidate opening."""

    ok_w = abs(metrics["long_len"] - width) <= side_tol * width
    ok_h = abs(metrics["short_len"] - height) <= side_tol * height
    ok_a = np.all(np.abs(metrics["angles_deg"] - 90.0) <= angle_tol_deg)
    return bool(ok_w and ok_h and ok_a)


def kabsch_2d(P: np.ndarray, V: np.ndarray) -> tuple[np.ndarray, float]:
    """Closed-form rigid fit: find ``t, yaw`` minimising ``sum |p - (t + Rz(yaw) v)|^2``."""

    P = np.asarray(P, dtype=float)
    V = np.asarray(V, dtype=float)
    p_bar = P.mean(axis=0)
    v_bar = V.mean(axis=0)
    Pc = P - p_bar
    Vc = V - v_bar
    # M = sum v_c p_c^T; yaw from its skew/trace parts.
    M = Vc.T @ Pc
    yaw = float(np.arctan2(M[0, 1] - M[1, 0], M[0, 0] + M[1, 1]))
    c, s = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[c, -s], [s, c]])
    t = p_bar - Rz @ v_bar
    return t, yaw


def rotate_2d(pts: np.ndarray, yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[c, -s], [s, c]])
    return np.atleast_2d(pts) @ Rz.T


def fit_residual_rms(P: np.ndarray, V: np.ndarray, t: np.ndarray, yaw: float) -> float:
    pred = rotate_2d(V, yaw) + t
    return float(np.sqrt(np.mean(np.sum((P - pred) ** 2, axis=1))))


def card_plane_model(
    default_quat: np.ndarray, card_z: float, ports: tuple
) -> dict:
    """Precompute the card-frame model mapped through the default rotation.

    Returns per-port corner/center/entrance/seat vectors ``v`` such that a card
    at ``(x, y, yaw)`` places them at ``(x, y) + Rz(yaw) v_xy`` on the plane
    ``z = card_z + v_z`` (yaw about world Z never changes v_z).
    """

    model = {}
    for port in ports:
        corners = np.array([quat_rotate(default_quat, np.asarray(c)) for c in port.corners])
        entrance = quat_rotate(default_quat, np.asarray(port.entrance))
        seat = quat_rotate(default_quat, np.asarray(port.seat))
        model[port.name] = {
            "corners_xy": corners[:, :2],
            "corners_z": card_z + corners[:, 2],
            "center_xy": corners[:, :2].mean(axis=0),
            "entrance": entrance,
            "seat": seat,
        }
    model["plane_z"] = float(np.mean([model[p.name]["corners_z"].mean() for p in ports]))
    return model
