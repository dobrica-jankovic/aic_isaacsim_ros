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
from .ur5e_kin import tcp_from_tip, tip_from_tcp

_QUINTIC_PEAK_VEL = 15.0 / 8.0


@dataclass
class Segment:
    """Quintic pose segment, interpolated in **TCP space**.

    Endpoints arrive as tip poses but are stored and interpolated as TCP
    poses, exactly like the upstream planner ("shifts each segment endpoint
    to the TCP frame at plan time"). Rotating about the TCP keeps the wrist
    nearly stationary; rotating about the tip would orbit the whole 26 cm
    tool through the reach boundary — that difference is what lets the ALIGN
    rotation happen next to the port without unwinding the elbow.
    ``sample`` converts back, so callers stay in tip space. ``end`` keeps the
    tip endpoint for settle checks.
    """

    start_tcp: tuple
    end_tcp: tuple
    end: tuple  # tip-space endpoint
    duration: float
    speed_scale: float
    t0: float = 0.0

    def sample(self, t: float) -> tuple:
        tau = min(max((t - self.t0) / max(self.duration, 1e-6), 0.0), 1.0)
        s = quintic(tau)
        pos = self.start_tcp[0] + s * (self.end_tcp[0] - self.start_tcp[0])
        quat = quat_slerp(self.start_tcp[1], self.end_tcp[1], s)
        return tip_from_tcp((pos, quat))

    def done(self, t: float) -> bool:
        return t - self.t0 >= self.duration


def make_segment(
    start: tuple, end: tuple, speed_scale: float, spec: ControlSpec, t0: float
) -> Segment:
    """Size the duration so quintic peak velocity honours the phase speed cap.

    ``start``/``end`` are tip poses; see :class:`Segment` for the TCP shift.
    """

    start_tcp = tcp_from_tip(start)
    end_tcp = tcp_from_tip(end)
    L = float(np.linalg.norm(end_tcp[0] - start_tcp[0]))
    ang = quat_angle(start_tcp[1], end_tcp[1])
    T_pos = _QUINTIC_PEAK_VEL * L / max(speed_scale * spec.v_max, 1e-9)
    T_rot = _QUINTIC_PEAK_VEL * ang / max(speed_scale * spec.w_max, 1e-9)
    duration = max(T_pos, T_rot, 0.25)
    return Segment(
        start_tcp=start_tcp, end_tcp=end_tcp, end=end,
        duration=duration, speed_scale=speed_scale, t0=t0,
    )


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
    """States: OBSERVE -> WAIT_ESTIMATE -> APPROACH -> REFINE -> INSERT ->
    SEATED, with RETREAT looping back to REFINE on a guarded failure.

    OBSERVE exists because the task's home pose keeps the wrist cameras away
    from the board: the machine first moves the tip above the prior card
    region, cameras down, so perception has something to look at.
    """

    def __init__(self, spec: ControlSpec, observe_pose: tuple | None = None):
        self.spec = spec
        self.observe_pose = observe_pose
        self.state = "OBSERVE" if observe_pose is not None else "WAIT_ESTIMATE"
        self.segment: Segment | None = None
        self.goals: Goals | None = None
        self.retries = 0
        self._refine_enter_t: float | None = None
        self._hold_start: float | None = None
        self._stall_mark: tuple = (0.0, None, None)
        self._overload_since: float | None = None
        self.wants_tare = False

    # The node calls this every servo tick. ``tip`` is the MEASURED tip pose;
    # segments are planned from it so trajectories start where the arm really is.
    def update(
        self,
        t: float,
        tip: tuple,
        goals: Goals | None,
        wrench_dev: float,
        tracking_gap: float,
    ) -> tuple | None:
        """Advance the machine; returns the tip pose target (or None = hold)."""

        self.wants_tare = False
        if goals is not None:
            self.goals = goals
        handler = getattr(self, f"_state_{self.state.lower()}")
        return handler(t, tip, wrench_dev, tracking_gap)

    def _overloaded(self, t: float, wrench_dev: float) -> bool:
        """True once the wrist load has stayed over threshold long enough.

        The cable swinging off the plug moves the reading by tens of newtons
        in free space, so a single sample over threshold means nothing; only a
        sustained overload does.
        """

        if wrench_dev <= self.spec.contact_force_n:
            self._overload_since = None
            return False
        if self._overload_since is None:
            self._overload_since = t
        return t - self._overload_since >= self.spec.contact_persist_s

    def _stalled(self, t: float, tip: tuple) -> bool:
        """True when the tip stops advancing *while the command is advancing*.

        Both halves matter. Tracking error alone is not stall — the stiff
        drives sag under load, and that offset is constant and harmless. Nor
        is measured travel alone: the segment is a rest-to-rest quintic, so
        near its ends the command barely moves and the tip rightly follows.
        Stall is measured travel falling short of *commanded* travel.
        """

        mark_t, mark_pos, mark_cmd = self._stall_mark
        if mark_pos is None:
            self._stall_mark = (t, tip[0].copy(), self.segment.sample(t)[0])
            return False
        if t - mark_t < self.spec.stall_window_s:
            return False
        moved = float(np.linalg.norm(tip[0] - mark_pos))
        commanded = float(np.linalg.norm(self.segment.sample(t)[0] - mark_cmd))
        self._stall_mark = (t, tip[0].copy(), self.segment.sample(t)[0])
        return (
            commanded > self.spec.stall_progress_m
            and moved < self.spec.stall_ratio * commanded
        )

    def _segment_settled(self, t: float, tip: tuple) -> bool:
        """Segment time elapsed AND the measured pose reached its endpoint
        (with a grace timeout so a marginal residual cannot wedge the machine)."""

        if not self.segment.done(t):
            return False
        err = float(np.linalg.norm(tip[0] - self.segment.end[0]))
        overtime = t - self.segment.t0 - self.segment.duration
        return err < self.spec.settle_tol_m or overtime > self.spec.settle_grace_s

    # ----------------------------------------------------------------- states

    def _state_observe(self, t, tip, wrench_dev, gap):
        if self.segment is None:
            self.segment = make_segment(
                tip, self.observe_pose, self.spec.speed_scale_approach, self.spec, t
            )
        if self._segment_settled(t, tip):
            self.segment = None
            self.state = "WAIT_ESTIMATE"
            return None
        return self.segment.sample(t)

    def _state_wait_estimate(self, t, tip, wrench_dev, gap):
        if not self._estimate_ready():
            return None
        self.segment = make_segment(
            tip, self._standoff_pose(), self.spec.speed_scale_approach, self.spec, t
        )
        self.state = "APPROACH"
        return self.segment.sample(t)

    def _state_approach(self, t, tip, wrench_dev, gap):
        if self._segment_settled(t, tip):
            self.state = "REFINE"
            self._refine_enter_t = t
            return None
        return self.segment.sample(t)

    def _state_refine(self, t, tip, wrench_dev, gap):
        if t - self._refine_enter_t < self.spec.settle_before_insert_s:
            return None
        if not self._estimate_ready():
            self._refine_enter_t = t  # estimate went stale: keep waiting
            return None
        standoff = self._standoff_pose()
        # Full 3D, not just lateral: RETREAT lifts along the insertion axis, so
        # a lateral-only test would let each retry start higher than the last.
        if float(np.linalg.norm(standoff[0] - tip[0])) > 0.0015:
            self.segment = make_segment(
                tip, standoff, self.spec.speed_scale_align, self.spec, t
            )
            self.state = "ALIGN"
            return self.segment.sample(t)
        self.wants_tare = True
        # Re-primed on the first INSERT tick, once the segment exists.
        self._stall_mark = (t, None, None)
        self._overload_since = None
        self.segment = make_segment(
            tip, self._seat_pose(), self.spec.speed_scale_insert, self.spec, t
        )
        self.state = "INSERT"
        return self.segment.sample(t)

    def _state_align(self, t, tip, wrench_dev, gap):
        if self._segment_settled(t, tip):
            self.state = "REFINE"
            self._refine_enter_t = t - self.spec.settle_before_insert_s  # recheck now
            return None
        return self.segment.sample(t)

    def _state_insert(self, t, tip, wrench_dev, gap):
        jammed = self._overloaded(t, wrench_dev) or self._stalled(t, tip)
        if jammed and not self.segment.done(t):
            return self._begin_retreat(t, tip)
        if self.segment.done(t):
            err_pos = float(np.linalg.norm(tip[0] - self._seat_pose()[0]))
            err_rot = quat_angle(tip[1], self._seat_pose()[1])
            if err_pos < self.spec.success_pos_m and err_rot < self.spec.success_rot_rad:
                if self._hold_start is None:
                    self._hold_start = t
                elif t - self._hold_start >= self.spec.success_hold_s:
                    self.state = "SEATED"
            elif t - self.segment.t0 - self.segment.duration > self.spec.settle_grace_s:
                self._hold_start = None
                return self._begin_retreat(t, tip)
        return self.segment.sample(t)

    def _state_retreat(self, t, tip, wrench_dev, gap):
        if self.segment.done(t):
            self.state = "REFINE"
            self._refine_enter_t = t
            return None
        return self.segment.sample(t)

    def _state_seated(self, t, tip, wrench_dev, gap):
        return None  # hold position; the run is a success

    def _state_failed(self, t, tip, wrench_dev, gap):
        return None

    # ---------------------------------------------------------------- helpers

    def _begin_retreat(self, t, tip):
        self.retries += 1
        self._hold_start = None
        if self.retries > self.spec.max_retries:
            self.state = "FAILED"
            return None
        up = tip[0] + np.array([0.0, 0.0, self.spec.retreat_m])
        self.segment = make_segment(
            tip, (up, self._standoff_pose()[1]),
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
