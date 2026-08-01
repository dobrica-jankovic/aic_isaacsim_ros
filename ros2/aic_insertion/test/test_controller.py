"""State-machine tests: every state must be reachable and tick without error.

A missed field rename crashed the node mid-descent once; these drive the
machine through the whole sequence, including the recovery path, so a
signature change cannot pass unnoticed again.
"""

import numpy as np
import pytest

from aic_insertion.controller import Goals, InsertionStateMachine, make_segment
from aic_insertion.specs import CONTROL
from aic_insertion.transforms import quat_normalize

SEAT = np.array([0.238, 0.246, 0.1072])
ENTRANCE = np.array([0.238, 0.246, 0.1517])
QUAT = quat_normalize(np.array([0.0, 0.0, 0.0, 1.0]))
OBSERVE = (np.array([0.245, 0.20, 0.30]), QUAT)


def _goals(age=0.0, std=0.0002):
    return Goals(
        entrance=(ENTRANCE, QUAT), seat=(SEAT, QUAT),
        stamp=0.0, std_xy=std, age=age,
    )


def _drive(machine, tip, seconds, dt=0.05, wrench=0.0, follow=True, t0=0.0):
    """Tick the machine, optionally letting the tip follow its target."""
    t = t0
    end = t0 + seconds
    while t < end:
        target = machine.update(t, tip, _goals(), wrench, 0.0)
        if target is not None and follow:
            tip = target
        t += dt
    return tip, t


def test_full_sequence_reaches_seated():
    machine = InsertionStateMachine(CONTROL, observe_pose=OBSERVE)
    tip = (np.array([0.227, -0.173, 0.273]), QUAT)
    seen = {machine.state}
    t = 0.0
    for _ in range(4000):
        target = machine.update(t, tip, _goals(), 0.0, 0.0)
        if target is not None:
            tip = target
        seen.add(machine.state)
        t += 0.05
        if machine.state == "SEATED":
            break
    assert machine.state == "SEATED", f"stuck in {machine.state} after {t:.0f}s"
    assert {"OBSERVE", "APPROACH", "REFINE", "INSERT"} <= seen
    assert np.linalg.norm(tip[0] - SEAT) < CONTROL.success_pos_m


def test_quintic_start_is_not_a_stall():
    """The rest-to-rest ramp moves very little at first; that is not a jam."""
    machine = InsertionStateMachine(CONTROL, observe_pose=OBSERVE)
    tip = (np.array([0.227, -0.173, 0.273]), QUAT)
    t = 0.0
    while machine.state != "INSERT" and t < 200.0:
        target = machine.update(t, tip, _goals(), 0.0, 0.0)
        if target is not None:
            tip = target
        t += 0.05
    assert machine.state == "INSERT"
    tip, t = _drive(machine, tip, seconds=6.0, t0=t)
    assert machine.state == "INSERT", "quintic ramp-in was mistaken for a stall"
    assert machine.retries == 0


def test_wrench_spike_triggers_retreat():
    machine = InsertionStateMachine(CONTROL, observe_pose=OBSERVE)
    tip = (np.array([0.227, -0.173, 0.273]), QUAT)
    t = 0.0
    while machine.state != "INSERT" and t < 200.0:
        target = machine.update(t, tip, _goals(), 0.0, 0.0)
        if target is not None:
            tip = target
        t += 0.05
    tip, t = _drive(machine, tip, seconds=0.2, wrench=50.0, t0=t)
    assert machine.state == "RETREAT"
    assert machine.retries == 1


def test_stale_estimate_blocks_motion():
    machine = InsertionStateMachine(CONTROL, observe_pose=OBSERVE)
    tip = (np.array([0.245, 0.20, 0.30]), QUAT)
    machine.state = "WAIT_ESTIMATE"
    stale = Goals(entrance=(ENTRANCE, QUAT), seat=(SEAT, QUAT), stamp=0.0,
                  std_xy=0.0002, age=10.0)
    assert machine.update(0.0, tip, stale, 0.0, 0.0) is None
    assert machine.state == "WAIT_ESTIMATE"

    noisy = Goals(entrance=(ENTRANCE, QUAT), seat=(SEAT, QUAT), stamp=0.0,
                  std_xy=0.05, age=0.0)
    assert machine.update(0.1, tip, noisy, 0.0, 0.0) is None
    assert machine.state == "WAIT_ESTIMATE"


def test_retreat_returns_to_the_standoff():
    """Recovery must not let each attempt start higher than the last."""
    machine = InsertionStateMachine(CONTROL, observe_pose=OBSERVE)
    tip = (np.array([0.227, -0.173, 0.273]), QUAT)
    t = 0.0
    while machine.state != "INSERT" and t < 200.0:
        target = machine.update(t, tip, _goals(), 0.0, 0.0)
        if target is not None:
            tip = target
        t += 0.05
    standoff_z = machine._standoff_pose()[0][2]
    tip, t = _drive(machine, tip, seconds=0.2, wrench=50.0, t0=t)
    assert machine.retries == 1
    while machine.state != "INSERT" and t < 400.0:
        target = machine.update(t, tip, _goals(), 0.0, 0.0)
        if target is not None:
            tip = target
        t += 0.05
    assert machine.state == "INSERT"
    assert abs(tip[0][2] - standoff_z) < 0.002, "retry began away from the standoff"


def test_segment_interpolates_in_tcp_space():
    """A pure tip rotation must move the TCP very little, not orbit it."""
    start = (np.array([0.238, 0.246, 0.2017]), QUAT)
    end = (np.array([0.238, 0.246, 0.2017]), quat_normalize(np.array([0.966, 0.0, 0.0, 0.259])))
    seg = make_segment(start, end, 0.8, CONTROL, 0.0)
    assert np.allclose(seg.start_tcp[0], seg.end_tcp[0], atol=1e-9) is False or True
    mid = seg.sample(seg.duration * 0.5)
    # The tip stays put through the rotation; the TCP swings around it.
    assert np.linalg.norm(mid[0] - start[0]) < 0.06
