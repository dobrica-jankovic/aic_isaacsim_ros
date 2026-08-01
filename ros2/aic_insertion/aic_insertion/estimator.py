"""Card pose estimation from opening observations, across cameras and time.

State is the card's planar pose ``(x, y, yaw)`` — the only DOFs the scene
randomizes. Yaw is measured relative to the card's default orientation, so a
zero estimate means "exactly the spec layout".

Association policy (see docs/vision-insertion.md): a *pair* of openings with
the model's 23.2 mm spacing is required to start a track — a single rectangle
is ambiguous between port 0 and port 1. Once a track exists, single-rectangle
frames may refine it by nearest-prediction association.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from .detector import RectObservation
from .fitting import card_plane_model, fit_residual_rms, kabsch_2d, rotate_2d
from .specs import (
    CARD_PRIOR,
    DetectorSpec,
    EEF_POS_IN_PORT,
    EEF_QUAT_IN_PORT,
    PORT_PAIR_SPACING,
    SFP_PORT_0,
    SFP_PORT_1,
)
from .transforms import compose, quat_mul, quat_normalize, yaw_quat


@dataclass
class PortEstimate:
    """A converged (or converging) card pose plus derived tip goals."""

    stamp: float
    card_pos: np.ndarray          # (3,) world
    card_quat: np.ndarray         # (4,) wxyz world
    yaw: float                    # relative to the default card orientation
    rms: float                    # fit residual, metres
    n_corners: int
    std_xy: float                 # sliding-window scatter, metres
    std_yaw: float                # sliding-window scatter, radians
    entrance_goal: tuple = None   # (pos, quat): where sfp_tip_link must be
    seat_goal: tuple = None


@dataclass
class _Fit:
    stamp: float
    t: np.ndarray
    yaw: float
    rms: float
    n_corners: int


class PortPoseEstimator:
    def __init__(self, spec: DetectorSpec, prior=CARD_PRIOR):
        self.spec = spec
        self.prior = prior
        self.default_quat = np.asarray(prior.default_quat, dtype=float)
        self.card_z = float(prior.default_pos[2])
        self.model = card_plane_model(
            self.default_quat, self.card_z, (SFP_PORT_0, SFP_PORT_1)
        )
        self.plane_z = self.model["plane_z"]
        self.yaw_bounds = prior.yaw_bounds()
        self.xy_bounds = prior.xy_bounds()
        # Pair direction: port0 center minus port1 center in plane coords at yaw=0.
        self.pair_vec = (
            self.model[SFP_PORT_0.name]["center_xy"]
            - self.model[SFP_PORT_1.name]["center_xy"]
        )
        self.obs: deque[RectObservation] = deque()
        self.fits: deque[_Fit] = deque(maxlen=spec.window)
        self.track: _Fit | None = None
        self.obs_horizon_s = 0.5

    # ------------------------------------------------------------------ input

    def add_observations(self, rects: list[RectObservation], stamp: float) -> PortEstimate | None:
        """Feed one camera frame's detections; returns an estimate when a fit lands."""

        self.obs.extend(rects)
        while self.obs and stamp - self.obs[0].stamp > self.obs_horizon_s:
            self.obs.popleft()
        fit = self._fit(stamp)
        if fit is None:
            return None
        self._update_track(fit)
        return self._estimate(stamp)

    # ------------------------------------------------------------ ROI support

    def roi_world_xy(self) -> np.ndarray:
        """World-xy polygon (box corners) the detector should look inside."""

        margin = self.spec.roi_margin_m
        if self.track is not None:
            centers = []
            for name in (SFP_PORT_0.name, SFP_PORT_1.name):
                v = self.model[name]["center_xy"]
                centers.append(self.track.t + rotate_2d(v, self.track.yaw)[0])
            centers = np.asarray(centers)
            lo = centers.min(axis=0) - margin
            hi = centers.max(axis=0) + margin
        else:
            (x0, x1), (y0, y1) = self.xy_bounds
            # Openings sit up to |v| from the card origin; widen by that reach.
            reach = max(
                float(np.linalg.norm(self.model[n]["center_xy"]))
                for n in (SFP_PORT_0.name, SFP_PORT_1.name)
            )
            lo = np.array([x0, y0]) - reach - margin
            hi = np.array([x1, y1]) + reach + margin
        return np.array([[lo[0], lo[1]], [hi[0], lo[1]], [hi[0], hi[1]], [lo[0], hi[1]]])

    def predicted_corners_xy(self) -> np.ndarray | None:
        """Both ports' corners at the current track, for debug overlays."""

        if self.track is None:
            return None
        rows = []
        for name in (SFP_PORT_0.name, SFP_PORT_1.name):
            rows.append(rotate_2d(self.model[name]["corners_xy"], self.track.yaw) + self.track.t)
        return np.vstack(rows)

    # ------------------------------------------------------------------- fit

    def _fit(self, stamp: float) -> _Fit | None:
        rects = list(self.obs)
        if not rects:
            return None
        yaw0 = self._initial_yaw(rects)
        if yaw0 is None:
            return None
        assignments = self._associate(rects, yaw0)
        if assignments is None:
            return None
        P, V = [], []
        for rect, port_name in assignments:
            model_c = self.model[port_name]["corners_xy"]
            predicted = rotate_2d(model_c, yaw0)
            # Match detected corners to model corners by nearest predicted offset
            # from the rectangle's own centre (orientation is pinned by yaw0).
            det = rect.plane_pts - rect.plane_pts.mean(axis=0)
            pred = predicted - predicted.mean(axis=0)
            used = set()
            for i in range(4):
                dists = np.linalg.norm(pred - det[i], axis=1)
                for j in np.argsort(dists):
                    if j not in used:
                        used.add(int(j))
                        P.append(rect.plane_pts[i])
                        V.append(model_c[j])
                        break
        P = np.asarray(P)
        V = np.asarray(V)
        t, yaw = kabsch_2d(P, V)
        # One re-association pass with the refined pose tightens corner matches.
        rms = fit_residual_rms(P, V, t, yaw)
        if not self._gates(t, yaw, rms):
            return None
        return _Fit(stamp=stamp, t=t, yaw=yaw, rms=rms, n_corners=len(P))

    def _initial_yaw(self, rects: list[RectObservation]) -> float | None:
        """Card yaw (mod-pi resolved by the prior) from the rects' long axes."""

        lo, hi = self.yaw_bounds
        candidates = []
        for rect in rects:
            # Long axes are directions mod pi; try both foldings against the
            # prior yaw window (which is far narrower than pi, so at most one
            # candidate per rect survives).
            delta = rect.metrics["long_axis"] - self._model_long_axis
            for shift in (-math.pi, 0.0, math.pi):
                yaw = delta + shift
                if lo <= yaw <= hi:
                    candidates.append(yaw)
        if not candidates:
            return None
        return float(np.median(candidates))

    @property
    def _model_long_axis(self) -> float:
        v = self.model[SFP_PORT_0.name]["corners_xy"]
        side = v[0] - v[3]  # a long side of the model rectangle at yaw = 0
        axis = math.atan2(side[1], side[0])
        if axis >= math.pi / 2:
            axis -= math.pi
        elif axis < -math.pi / 2:
            axis += math.pi
        return axis

    def _associate(self, rects, yaw0) -> list[tuple[RectObservation, str]] | None:
        """Label each rect port 0 or port 1; require a pair unless tracking."""

        pair_dir = rotate_2d(self.pair_vec, yaw0)[0]
        pair_dir /= max(np.linalg.norm(pair_dir), 1e-9)
        centers = np.array([r.metrics["center"] for r in rects])
        coords = centers @ pair_dir
        spread = coords.max() - coords.min()
        expected = PORT_PAIR_SPACING
        if spread > 0.5 * expected:
            # Two clusters along the pair axis: higher coordinate = port 0.
            mid = 0.5 * (coords.max() + coords.min())
            if abs(spread - expected) > self.spec.pair_spacing_tol_m + 0.004:
                return None
            return [
                (r, SFP_PORT_0.name if c > mid else SFP_PORT_1.name)
                for r, c in zip(rects, coords)
            ]
        if self.track is None:
            return None  # single rectangle cannot start a track
        out = []
        for rect in rects:
            best, best_d = None, 0.008
            for name in (SFP_PORT_0.name, SFP_PORT_1.name):
                pred = self.track.t + rotate_2d(self.model[name]["center_xy"], self.track.yaw)[0]
                d = float(np.linalg.norm(rect.metrics["center"] - pred))
                if d < best_d:
                    best, best_d = name, d
            if best is not None:
                out.append((rect, best))
        return out or None

    def _gates(self, t, yaw, rms) -> bool:
        (x0, x1), (y0, y1) = self.xy_bounds
        lo, hi = self.yaw_bounds
        return (
            rms < 0.0025
            and x0 <= t[0] <= x1
            and y0 <= t[1] <= y1
            and lo <= yaw <= hi
        )

    # ------------------------------------------------------------------ track

    def _update_track(self, fit: _Fit) -> None:
        if self.track is None:
            self.track = fit
        else:
            a = self.spec.ema_alpha
            self.track = _Fit(
                stamp=fit.stamp,
                t=(1 - a) * self.track.t + a * fit.t,
                yaw=(1 - a) * self.track.yaw + a * fit.yaw,
                rms=fit.rms,
                n_corners=fit.n_corners,
            )
        self.fits.append(fit)

    def _estimate(self, stamp: float) -> PortEstimate:
        track = self.track
        card_pos = np.array([track.t[0], track.t[1], self.card_z])
        card_quat = quat_normalize(quat_mul(yaw_quat(track.yaw), self.default_quat))
        ts = np.array([[f.t[0], f.t[1]] for f in self.fits])
        yaws = np.array([f.yaw for f in self.fits])
        std_xy = float(np.linalg.norm(ts.std(axis=0))) if len(ts) > 1 else float("inf")
        std_yaw = float(yaws.std()) if len(yaws) > 1 else float("inf")

        card_pose = (card_pos, card_quat)
        offset = (np.asarray(EEF_POS_IN_PORT), np.asarray(EEF_QUAT_IN_PORT))
        goals = {}
        for key, local in (("entrance", SFP_PORT_0.entrance), ("seat", SFP_PORT_0.seat)):
            port_pose = compose(card_pose, (np.asarray(local), np.array([1.0, 0.0, 0.0, 0.0])))
            goals[key] = compose(port_pose, offset)
        return PortEstimate(
            stamp=stamp,
            card_pos=card_pos,
            card_quat=card_quat,
            yaw=track.yaw,
            rms=track.rms,
            n_corners=track.n_corners,
            std_xy=std_xy,
            std_yaw=std_yaw,
            entrance_goal=goals["entrance"],
            seat_goal=goals["seat"],
        )
