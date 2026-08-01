"""FK/Jacobian checks against USD-derived ground truth.

The zero-pose reference comes from the robot USD's authored state (all joints
at 0), where ``base_link -> gripper_tcp`` was measured with pxr's XformCache.
"""

import numpy as np
import pytest

from aic_insertion import ur5e_kin as kin
from aic_insertion.specs import TCP_TO_TIP_POS
from aic_insertion.transforms import quat_rotate


def test_fk_zero_pose_matches_usd():
    pos, quat = kin.fk_tcp(np.zeros(6))
    assert np.allclose(pos, [0.8172, 0.4294, 0.0628], atol=2e-4)
    # At q=0 the tool axes: tcp +Z is world... derive via quat action instead
    # of hardcoding: the USD showed tcp quat (0, 0, sqrt(.5), sqrt(.5)).
    expected = np.array([0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)])
    assert min(np.linalg.norm(quat - expected), np.linalg.norm(quat + expected)) < 1e-3


def test_tip_round_trip():
    rng = np.random.default_rng(0)
    for _ in range(20):
        q = rng.uniform(-2.5, 2.5, 6)
        tcp = kin.fk_tcp(q)
        tip = kin.tip_from_tcp(tcp)
        back = kin.tcp_from_tip(tip)
        assert np.allclose(back[0], tcp[0], atol=1e-8)
        assert min(np.linalg.norm(back[1] - tcp[1]), np.linalg.norm(back[1] + tcp[1])) < 1e-8
        # Tip offset magnitude is the tool constant.
        assert np.isclose(
            np.linalg.norm(tip[0] - tcp[0]), np.linalg.norm(TCP_TO_TIP_POS), atol=1e-9
        )


def test_jacobian_matches_finite_differences():
    rng = np.random.default_rng(1)
    q = rng.uniform(-1.5, 1.5, 6)
    J = kin.jacobian(q)
    eps = 1e-6
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = eps
        p0, q0 = kin.fk_tcp(q - dq)
        p1, q1 = kin.fk_tcp(q + dq)
        lin = (p1 - p0) / (2 * eps)
        assert np.allclose(J[:3, i], lin, atol=1e-5), f"joint {i} linear"


def test_dls_reduces_pose_error():
    q = np.array([0.16, -1.35, -1.66, -1.69, 1.57, -1.73])
    target_pos, target_quat = kin.fk_tcp(q + 0.02)
    cur_pos, cur_quat = kin.fk_tcp(q)
    twist = np.concatenate([target_pos - cur_pos, np.zeros(3)])
    dq = kin.dls_step(kin.jacobian(q), twist, 0.01)
    new_pos, _ = kin.fk_tcp(q + dq)
    assert np.linalg.norm(new_pos - target_pos) < np.linalg.norm(cur_pos - target_pos)
