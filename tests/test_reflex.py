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
    """Scenario 6: dual pair sum outside tolerance triggers PAIR_MISMATCH."""
    cfg = ReflexConfig(pair_tol=20)
    r = Reflex(LIMITS, {"J2": (11, 10, 4005)}, cfg)

    # Within tolerance.
    trips = r.update(
        now=0.0,
        present={11: 2000, 10: 2005},
        goal={11: 2000, 10: 2005},
        load={11: 0, 10: 0},
    )
    assert trips == []

    # Sum is 4045, K=4005 -> deviation 40 > 20.
    trips = r.update(
        now=0.1,
        present={11: 2000, 10: 2045},
        goal={11: 2000, 10: 2045},
        load={11: 0, 10: 0},
    )
    assert len(trips) == 1
    assert trips[0].event is Event.PAIR_MISMATCH
    assert trips[0].motor_id == 11
    assert r.mode is Mode.REFLEX


def test_comm_loss_and_overtemp():
    """Scenario 7: repeated comm failures and over-temperature enter STOPPED."""
    cfg = ReflexConfig(comm_fail_max=5, temp_warn=65, temp_stop=70)
    r = Reflex(LIMITS, PAIRS, cfg)

    # Comm failure must reach comm_fail_max.
    for i in range(1, 5):
        trips = r.update(
            now=i * 0.05,
            present={19: 1000},
            goal={19: 1000},
            load={19: 0},
            comm_ok=False,
        )
        assert trips == []
    trips = r.update(
        now=0.30,
        present={19: 1000},
        goal={19: 1000},
        load={19: 0},
        comm_ok=False,
    )
    assert len(trips) == 1
    assert trips[0].event is Event.COMM_LOSS
    assert r.mode is Mode.STOPPED

    # Overtemp.
    r2 = Reflex(LIMITS, PAIRS, cfg)
    trips = r2.update(
        now=0.0,
        present={19: 1000},
        goal={19: 1000},
        load={19: 0},
        temps={19: 71},
    )
    assert len(trips) == 1
    assert trips[0].event is Event.OVERTEMP
    assert r2.mode is Mode.STOPPED

    # Warning only.
    r3 = Reflex(LIMITS, PAIRS, cfg)
    trips = r3.update(
        now=0.0,
        present={19: 1000},
        goal={19: 1000},
        load={19: 0},
        temps={19: 66},
    )
    assert trips == []
    assert r3.warnings() == ["motor 19 temp 66°C >= warn 65°C"]
    assert r3.mode is Mode.MOVE


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
    ok, reason = r.recover(present={19: 1000}, load={19: 80})
    assert ok is False
    assert "load too high" in reason

    # Low load but position drifted too far -> cannot recover.
    ok, reason = r.recover(present={19: 1100}, load={19: 10})
    assert ok is False
    assert "too far from hold target" in reason

    # Safe -> recover.
    ok, reason = r.recover(present={19: 1000}, load={19: 10})
    assert ok is True
    assert r.mode is Mode.MOVE
