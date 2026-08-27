"""Pure-logic tests for sopo.reflex.

These tests use fabricated sensor data so they can run without hardware.
"""

from __future__ import annotations

import pytest

from sopo.reflex import Event, Mode, Reflex, ReflexConfig, Trip
from sopo.safety import SafetyLimits


LIMITS = SafetyLimits(torque_limit=150)
PAIRS = {"J2": (11, 10, 4005), "J3": (16, 15, 4100)}


def test_no_load_back_and_forth():
    """Scenario 1: normal motion stays in MOVE without trips."""
    r = Reflex(LIMITS, PAIRS)
    base = {"now": 0.0, "present": {19: 1000}, "goal": {19: 1200}, "load": {19: 0}}

    assert r.update(**base) == []
    assert r.mode is Mode.MOVE

    assert r.update(
        now=0.1,
        present={19: 1100},
        goal={19: 1200},
        load={19: 64},
    ) == []

    assert r.update(
        now=0.2,
        present={19: 1200},
        goal={19: 1200},
        load={19: 0},
    ) == []
    assert r.mode is Mode.MOVE


def test_acceleration_deferral():
    """Scenario 2: saturated load right after a large goal step is ignored."""
    cfg = ReflexConfig(accel_step=40, t_accel=0.2)
    r = Reflex(LIMITS, PAIRS, cfg)

    r.update(now=0.0, present={19: 1000}, goal={19: 1000}, load={19: 0})
    # Big step at t=0.05 starts deferral until t=0.25.
    r.update(now=0.05, present={19: 1000}, goal={19: 1100}, load={19: 150})
    # Still inside deferral window.
    trips = r.update(now=0.15, present={19: 1000}, goal={19: 1100}, load={19: 150})
    assert trips == []
    assert r.mode is Mode.MOVE

    # After deferral ends load is gone: no collision.
    trips = r.update(now=0.30, present={19: 1050}, goal={19: 1100}, load={19: 20})
    assert trips == []


def test_collision_saturated_and_stuck():
    """Scenario 3: saturated load + non-decreasing error triggers COLLISION."""
    cfg = ReflexConfig(t_collision=0.3)
    r = Reflex(LIMITS, PAIRS, cfg)

    r.update(now=0.0, present={19: 1000}, goal={19: 1100}, load={19: 150})
    trips = r.update(now=0.35, present={19: 1000}, goal={19: 1100}, load={19: 150})

    assert len(trips) == 1
    assert trips[0].event is Event.COLLISION
    assert trips[0].motor_id == 19
    assert r.mode is Mode.REFLEX

    # Repeated updates must not duplicate the trip.
    assert r.update(
        now=0.40,
        present={19: 1000},
        goal={19: 1100},
        load={19: 150},
    ) == []


def test_collision_but_error_decreasing():
    """Scenario 4: saturated load while still converging is not a collision."""
    cfg = ReflexConfig(t_collision=0.3)
    r = Reflex(LIMITS, PAIRS, cfg)

    r.update(now=0.0, present={19: 1000}, goal={19: 1200}, load={19: 150})
    r.update(now=0.10, present={19: 1050}, goal={19: 1200}, load={19: 150})
    r.update(now=0.20, present={19: 1100}, goal={19: 1200}, load={19: 150})
    trips = r.update(
        now=0.35, present={19: 1150}, goal={19: 1200}, load={19: 150}
    )

    assert trips == []
    assert r.mode is Mode.MOVE


def test_collision_creeping_under_hand():
    """Scenario 4b (07 실험): saturated while creeping 1 tick/cycle -> still a COLLISION."""
    cfg = ReflexConfig(t_collision=0.3, progress_ticks=40)
    r = Reflex(LIMITS, PAIRS, cfg)
    trips = []
    for i in range(20):  # 0.4 s at 50 Hz, error shrinks by 1 tick per cycle
        trips += r.update(now=i * 0.02, present={19: 1000 + i}, goal={19: 1080 + i}, load={19: 150})
    assert [t.event for t in trips] == [Event.COLLISION]
    assert r.mode is Mode.REFLEX


def test_tracking_error():
    """Scenario 5: persistent large error with low load triggers TRACKING_ERROR."""
    cfg = ReflexConfig(err_ticks=150, t_error=0.5)
    r = Reflex(LIMITS, PAIRS, cfg)

    r.update(now=0.0, present={19: 1000}, goal={19: 1200}, load={19: 20})
    trips = r.update(
        now=0.6, present={19: 1000}, goal={19: 1200}, load={19: 20}
    )

    assert len(trips) == 1
    assert trips[0].event is Event.TRACKING_ERROR
    assert trips[0].motor_id == 19
    assert r.mode is Mode.REFLEX


def test_pair_mismatch():
    """Scenario 6: a transient +40 deflection (grab) is tolerated; a persistent +100 is a PAIR_MISMATCH."""
    r = Reflex(LIMITS, PAIRS, ReflexConfig(pair_tol=60, t_pair=0.3))
    ref, mirror, K = PAIRS["J2"]
    assert r.update(now=0.0, present={ref: 1000, mirror: K - 1000 + 40}, goal={ref: 1000, mirror: K - 1000}, load={}) == []
    trips = []
    for k in range(1, 5):
        trips += r.update(now=0.1 * k, present={ref: 1000, mirror: K - 1000 + 100}, goal={ref: 1000, mirror: K - 1000}, load={})
    assert [t.event for t in trips] == [Event.PAIR_MISMATCH]
    assert trips[0].motor_id == ref and r.mode is Mode.REFLEX


def test_comm_loss_and_overtemp():
    """Scenario 7: 5 comm failures -> STOPPED; overtemp needs 2 confirming readings; garbage (150) is ignored."""
    r = Reflex(LIMITS, PAIRS, ReflexConfig(comm_fail_max=5))
    trips = []
    for k in range(5):
        trips += r.update(now=0.02 * k, present={19: 1000}, goal={19: 1000}, load={19: 0}, comm_ok=False)
    assert [t.event for t in trips] == [Event.COMM_LOSS] and r.mode is Mode.STOPPED

    r2 = Reflex(LIMITS, PAIRS, ReflexConfig(temp_confirm=2))
    assert r2.update(now=0.0, present={19: 1000}, goal={19: 1000}, load={19: 0}, temps={19: 150}) == []   # garbage
    assert any("implausible" in w for w in r2.warnings()) and r2.mode is Mode.MOVE
    assert r2.update(now=2.0, present={19: 1000}, goal={19: 1000}, load={19: 0}, temps={19: 71}) == []    # 1/2
    assert any("confirming" in w for w in r2.warnings())
    trips = r2.update(now=4.0, present={19: 1000}, goal={19: 1000}, load={19: 0}, temps={19: 72})         # 2/2
    assert [t.event for t in trips] == [Event.OVERTEMP] and r2.mode is Mode.STOPPED

    r3 = Reflex(LIMITS, PAIRS, ReflexConfig())
    assert r3.update(now=0.0, present={19: 1000}, goal={19: 1000}, load={19: 0}, temps={19: 66}) == []
    assert r3.warnings() == ["motor 19 temp 66°C >= warn 65°C"] and r3.mode is Mode.MOVE


def test_recover():
    """Scenario 8: recover only when load and position error are safe."""
    cfg = ReflexConfig(t_collision=0.3)
    r = Reflex(LIMITS, PAIRS, cfg)

    # Force a COLLISION.
    r.update(now=0.0, present={19: 1000}, goal={19: 1100}, load={19: 150})
    r.update(now=0.35, present={19: 1000}, goal={19: 1100}, load={19: 150})
    assert r.mode is Mode.REFLEX

    # Set a hold target.
    r.hold_targets({19: 1000})

    # High load -> cannot recover.
    ok, reason = r.recover(present={19: 1000}, load={19: 140})  # >= 90% of cap 150
    assert ok is False
    assert "of cap" in reason

    # Low load but position drifted too far -> cannot recover.
    ok, reason = r.recover(present={19: 1100}, load={19: 10})
    assert ok is False
    assert "too far from hold target" in reason

    # Safe -> recover.
    ok, reason = r.recover(present={19: 1000}, load={19: 10})
    assert ok is True
    assert r.mode is Mode.MOVE


def test_joint_limit_arms_after_entering_range_and_needs_dwell():
    """Start outside the range (parked at the mechanical end): no trip while returning.
    Once inside, being pushed out past the margin for > t_limit -> JOINT_LIMIT."""
    from sopo import SafetyLimits
    lim = SafetyLimits(position_limits={19: (220, 3969)}, torque_limits={19: 150})
    r = Reflex(lim, {}, ReflexConfig(limit_margin=30, t_limit=0.5))
    assert r.update(now=0.0, present={19: 4020}, goal={19: 3969}, load={19: 60}) == []   # 밖에서 시작 → 무장 전
    assert r.update(now=0.3, present={19: 4020}, goal={19: 3969}, load={19: 60}) == []
    r.update(now=0.6, present={19: 3960}, goal={19: 3969}, load={19: 20})                 # 안으로 들어옴 → 무장
    assert r.update(now=1.0, present={19: 4005}, goal={19: 3969}, load={19: 20}) == []   # 밖으로 밀림, 아직 0.5s 미만
    assert r.update(now=1.3, present={19: 4005}, goal={19: 3969}, load={19: 20}) == []
    trips = r.update(now=1.6, present={19: 4005}, goal={19: 3969}, load={19: 20})
    assert [t.event for t in trips] == [Event.JOINT_LIMIT] and r.mode is Mode.REFLEX


def test_free_motion_with_running_goal_is_not_collision():
    """Jog/waypoint: goal stays 80 ticks ahead, joint moves 40/cycle at the cap -> no trip (was a false positive)."""
    r = Reflex(LIMITS, PAIRS, ReflexConfig(t_collision=0.3, progress_ticks=40))
    trips = []
    for i in range(30):
        pos = 2000 + 40 * i
        trips += r.update(now=i * 0.02, present={19: pos}, goal={19: pos + 80}, load={19: 150})
    assert trips == [] and r.mode is Mode.MOVE


def test_pushed_away_from_goal_is_collision():
    """Hand pushes the joint away from its goal while the motor saturates -> trip (moved negative)."""
    r = Reflex(LIMITS, PAIRS, ReflexConfig(t_collision=0.3, progress_ticks=40))
    trips = []
    for i in range(20):
        trips += r.update(now=i * 0.02, present={19: 2000 - 3 * i}, goal={19: 2000}, load={19: -150})
    assert [t.event for t in trips] == [Event.COLLISION] and "pushed back" in trips[0].detail

