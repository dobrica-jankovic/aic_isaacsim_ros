"""Synthetic round-trip tests: known card pose -> observations -> recovered pose."""

import numpy as np
import pytest

from aic_insertion.detector import RectObservation
from aic_insertion.estimator import PortPoseEstimator
from aic_insertion.fitting import (
    backproject_to_plane,
    project_plane_points,
    rect_metrics,
    rotate_2d,
)
from aic_insertion.specs import CARD_PRIOR, DetectorSpec, SFP_PORT_0, SFP_PORT_1
from aic_insertion.transforms import quat_mul, yaw_quat


def _estimator():
    return PortPoseEstimator(DetectorSpec())


def _true_corners(est, txy, yaw):
    """World-plane corners of both openings for a card at (txy, yaw)."""
    out = {}
    for port in (SFP_PORT_0, SFP_PORT_1):
        v = est.model[port.name]["corners_xy"]
        out[port.name] = rotate_2d(v, yaw) + np.asarray(txy)
    return out


def _observations(est, txy, yaw, stamp=0.0, noise=0.0, rng=None, ports=None):
    obs = []
    corners = _true_corners(est, txy, yaw)
    for name in ports or (SFP_PORT_0.name, SFP_PORT_1.name):
        pts = corners[name].copy()
        if noise:
            pts = pts + rng.normal(0.0, noise, pts.shape)
        order = np.argsort(
            np.arctan2(pts[:, 1] - pts[:, 1].mean(), pts[:, 0] - pts[:, 0].mean())
        )
        pts = pts[order]
        obs.append(
            RectObservation(
                camera="synthetic", stamp=stamp, image_pts=np.zeros((4, 2)),
                plane_pts=pts, metrics=rect_metrics(pts),
            )
        )
    return obs


def test_exact_recovery():
    est = _estimator()
    txy, yaw = np.array([0.26, 0.24]), 0.2
    result = est.add_observations(_observations(est, txy, yaw), stamp=0.0)
    assert result is not None
    assert np.allclose(result.card_pos[:2], txy, atol=1e-9)
    assert abs(result.yaw - yaw) < 1e-9
    assert result.card_pos[2] == pytest.approx(CARD_PRIOR.default_pos[2])


def test_noisy_recovery_and_convergence():
    est = _estimator()
    rng = np.random.default_rng(7)
    txy, yaw = np.array([0.30, 0.20]), -0.25
    result = None
    for k in range(12):
        result = est.add_observations(
            _observations(est, txy, yaw, stamp=0.05 * k, noise=2e-4, rng=rng),
            stamp=0.05 * k,
        ) or result
    assert result is not None
    assert np.linalg.norm(result.card_pos[:2] - txy) < 5e-4
    assert abs(result.yaw - yaw) < 0.01
    assert result.std_xy < 1e-3


def test_single_rect_cannot_start_track():
    est = _estimator()
    result = est.add_observations(
        _observations(est, np.array([0.26, 0.24]), 0.0, ports=[SFP_PORT_0.name]),
        stamp=0.0,
    )
    assert result is None


def test_single_rect_refines_existing_track():
    est = _estimator()
    txy, yaw = np.array([0.26, 0.24]), 0.1
    assert est.add_observations(_observations(est, txy, yaw), stamp=0.0) is not None
    est.obs.clear()
    moved = txy + np.array([0.001, 0.0])
    result = est.add_observations(
        _observations(est, moved, yaw, stamp=0.2, ports=[SFP_PORT_0.name]), stamp=0.2
    )
    assert result is not None
    assert np.linalg.norm(result.card_pos[:2] - moved) < 1.5e-3


def test_yaw_outside_prior_is_rejected():
    est = _estimator()
    result = est.add_observations(
        _observations(est, np.array([0.26, 0.24]), 1.2), stamp=0.0
    )
    assert result is None


def test_projection_backprojection_round_trip():
    est = _estimator()
    # A camera 0.3 m above the plane, looking straight down (optical +Z fwd).
    cam_pos = np.array([0.25, 0.24, est.plane_z + 0.3])
    cam_quat = quat_mul(np.array([0.0, 1.0, 0.0, 0.0]), yaw_quat(0.3))
    K = np.array([[480.6, 0, 224], [0, 480.6, 224], [0, 0, 1.0]])
    pts_xy = _true_corners(est, np.array([0.26, 0.24]), 0.15)[SFP_PORT_0.name]
    px = project_plane_points(pts_xy, est.plane_z, K, (cam_pos, cam_quat))
    back, valid = backproject_to_plane(px, K, (cam_pos, cam_quat), est.plane_z)
    assert np.all(valid)
    assert np.allclose(back, pts_xy, atol=1e-10)
