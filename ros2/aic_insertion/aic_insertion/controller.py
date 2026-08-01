"""Insertion state machine and Cartesian segment logic. Pure logic, no ROS.

Motion mirrors the IsaacLab demonstration planner: rest-to-rest quintic
segments, per-phase speed scales, a standoff above the entrance, then a slow
guarded descent to the seat. The inputs are perception goals instead of the
privileged cheat topics, so extra states exist for waiting on convergence,
re-refining at the standoff, and force-triggered retreat/retry.

All poses are ``sfp_tip_link`` poses in ``World``, ``(pos, quat wxyz)``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .specs import ControlSpec
from .transforms import quat_angle, quat_slerp, quintic

_QUINTIC_PEAK_VEL = 15.0 / 8.0


@dataclass
class Segment:
    start: tuple
    end: tuple
    duration: float
    speed_scale: float
    t0: float = 0.0

    def sample(self, t: float) -> tuple:
        tau = min(max((t - self.t0) / max(self.duration, 1e-6), 0.0), 1.0)
        s = quintic(tau)
        pos = self.start[0] + s * (self.end[0] - self.start[0])
        quat = quat_slerp(self.start[1], self.end[1], s)
        return pos, quat

    def done(self, t: float) -> bool:
        return t - self.t0 >= self.duration


def make_segment(
    start: tuple, end: tuple, speed_scale: float, spec: ControlSpec, t0: float
) -> Segment:
    """Size the duration so quintic peak velocity honours the phase speed cap."""

    L = float(np.linalg.norm(end[0] - start[0]))
    ang = quat_angle(start[1], end[1])
    T_pos = _QUINTIC_PEAK_VEL * L / max(speed_scale * spec.v_max, 1e-9)
    T_rot = _QUINTIC_PEAK_VEL * ang / max(speed_scale * spec.w_max, 1e-9)
    duration = max(T_pos, T_rot, 0.25)
    return Segment(start=start, end=end, duration=duration, speed_scale=speed_scale, t0=t0)


def spiral_offset(attempt: int, radius: float) -> np.ndarray:
    """Lateral retry offset: attempt 0 is centred, then a small ring."""

    if attempt <= 0:
        return np.zeros(3)
    angle = (attempt - 1) * (math.pi * 2.0 / 3.0)
    return np.array([math.cos(angle), math.sin(angle), 0.0]) * radius


@dataclass
class Goals:
    """Perception output the machine consumes, all tip poses in World."""

    entrance: tuple
    seat: tuple
    stamp: float
    std_xy: float
    age: float = 0.0


class InsertionStateMachine:
    """States: WAIT_ESTIMATE -> APPROACH -> REFINE -> INSERT -> SEATED,
    with RETREAT looping back to REFINE on a guarded failure."""

    def __init__(self, spec: ControlSpec):
        self.spec = spec
        self.state = "WAIT_ESTIMATE"
        self.segment: Segment | None = None
        self.goals: Goals | None = None
        self.retries = 0
        self._refine_enter_t: float | None = None
        self._hold_start: float | None = None
        self.wants_tare = False

    # The node calls this every servo tick.
    def update(
        self,
        t: float,
        tip_cmd: tuple,
        tip_meas: tuple,
        goals: Goals | None,
        wrench_dev: float,
        tracking_gap: float,
    ) -> tuple | None:
        """Advance the machine; returns the tip pose target (or None = hold)."""

        self.wants_tare = False
        if goals is not None:
            self.goals = goals
        handler = getattr(self, f"_state_{self.state.lower()}")
        return handler(t, tip_cmd, tip_meas, wrench_dev, tracking_gap)

    # ----------------------------------------------------------------- states

    def _state_wait_estimate(self, t, tip_cmd, tip_meas, wrench_dev, gap):
        if not self._estimate_ready():
            return None
        self.segment = make_segment(
            tip_cmd, self._standoff_pose(), self.spec.speed_scale_approach, self.spec, t
        )
        self.state = "APPROACH"
        return self.segment.sample(t)

    def _state_approach(self, t, tip_cmd, tip_meas, wrench_dev, gap):
        if self.segment.done(t):
            self.state = "REFINE"
            self._refine_enter_t = t
            return None
        return self.segment.sample(t)

    def _state_refine(self, t, tip_cmd, tip_meas, wrench_dev, gap):
        if t - self._refine_enter_t < self.spec.settle_before_insert_s:
            return None
        if not self._estimate_ready():
            self._refine_enter_t = t  # estimate went stale: keep waiting
            return None
        standoff = self._standoff_pose()
        lateral = float(np.linalg.norm((standoff[0] - tip_cmd[0])[:2]))
        if lateral > 0.001:
            self.segment = make_segment(
                tip_cmd, standoff, self.spec.speed_scale_align, self.spec, t
            )
            self.state = "ALIGN"
            return self.segment.sample(t)
        self.wants_tare = True
        self.segment = make_segment(
            tip_cmd, self._seat_pose(), self.spec.speed_scale_insert, self.spec, t
        )
        self.state = "INSERT"
        return self.segment.sample(t)

    def _state_align(self, t, tip_cmd, tip_meas, wrench_dev, gap):
        if self.segment.done(t):
            self.state = "REFINE"
            self._refine_enter_t = t - self.spec.settle_before_insert_s  # recheck now
            return None
        return self.segment.sample(t)

    def _state_insert(self, t, tip_cmd, tip_meas, wrench_dev, gap):
        jammed = wrench_dev > self.spec.contact_force_n or gap > self.spec.stall_pos_m
        if jammed and not self.segment.done(t):
            return self._begin_retreat(t, tip_cmd)
        if self.segment.done(t):
            err_pos = float(np.linalg.norm(tip_meas[0] - self._seat_pose()[0]))
            err_rot = quat_angle(tip_meas[1], self._seat_pose()[1])
            if err_pos < self.spec.success_pos_m and err_rot < self.spec.success_rot_rad:
                if self._hold_start is None:
                    self._hold_start = t
                elif t - self._hold_start >= self.spec.success_hold_s:
                    self.state = "SEATED"
            else:
                self._hold_start = None
                return self._begin_retreat(t, tip_cmd)
        return self.segment.sample(t)

    def _state_retreat(self, t, tip_cmd, tip_meas, wrench_dev, gap):
        if self.segment.done(t):
            self.state = "REFINE"
            self._refine_enter_t = t
            return None
        return self.segment.sample(t)

    def _state_seated(self, t, tip_cmd, tip_meas, wrench_dev, gap):
        return None  # hold position; the run is a success

    def _state_failed(self, t, tip_cmd, tip_meas, wrench_dev, gap):
        return None

    # ---------------------------------------------------------------- helpers

    def _begin_retreat(self, t, tip_cmd):
        self.retries += 1
        self._hold_start = None
        if self.retries > self.spec.max_retries:
            self.state = "FAILED"
            return None
        up = tip_cmd[0] + np.array([0.0, 0.0, self.spec.retreat_m])
        self.segment = make_segment(
            tip_cmd, (up, self._standoff_pose()[1]),
            self.spec.speed_scale_align, self.spec, t,
        )
        self.state = "RETREAT"
        return self.segment.sample(t)

    def _estimate_ready(self) -> bool:
        g = self.goals
        return (
            g is not None
            and g.age < self.spec.estimate_max_age_s
            and g.std_xy < self.spec.estimate_max_std_m
        )

    def _standoff_pose(self) -> tuple:
        entrance, seat = self.goals.entrance, self.goals.seat
        axis = seat[0] - entrance[0]
        axis = axis / max(np.linalg.norm(axis), 1e-9)
        pos = entrance[0] - self.spec.standoff_m * axis + spiral_offset(
            self.retries, self.spec.retry_spiral_m
        )
        return pos, entrance[1]

    def _seat_pose(self) -> tuple:
        pos = self.goals.seat[0] + spiral_offset(self.retries, self.spec.retry_spiral_m)
        return pos, self.goals.seat[1]
