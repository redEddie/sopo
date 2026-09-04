"""Pure-logic tests for sopo.reflex.

These tests use fabricated sensor data so they can run without hardware.
"""

from __future__ import annotations

import pytest

from sopo.safety.reflex import Event, Mode, Reflex, ReflexConfig, Trip
from sopo.safety.limits import SafetyLimits


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


def test_external_force_trips_after_persistence():
    """τ_ext가 trip 임계를 t_ext 이상 넘으면 EXTERNAL_FORCE 1회 트립 (래칭, 중복 없음)."""
    r = Reflex(LIMITS, PAIRS, ReflexConfig(ext_trip_nm=1.0, t_ext=0.3))
    base = dict(present={11: 1000}, goal={11: 1000}, load={11: 0})
    ext = {"J2": 1.5}
    m = {"J2": 11}
    assert r.update(now=0.0, **base, ext_torque=ext, ext_joint_motor=m) == []
    assert r.update(now=0.1, **base, ext_torque=ext, ext_joint_motor=m) == []
    trips = r.update(now=0.35, **base, ext_torque=ext, ext_joint_motor=m)
    assert [t.event for t in trips] == [Event.EXTERNAL_FORCE]
    assert trips[0].motor_id == 11 and r.mode is Mode.REFLEX
    # 래칭: 계속 넣어도 중복 트립 없음 (REFLEX에서는 신규 판정 자체를 안 함)
    assert r.update(now=0.5, **base, ext_torque=ext, ext_joint_motor=m) == []


def test_external_force_short_spike_ignored():
    """t_ext 미만의 스파이크는 무시되고, 사라지면 타이머도 리셋된다."""
    r = Reflex(LIMITS, PAIRS, ReflexConfig(ext_trip_nm=1.0, t_ext=0.3))
    m = {"J2": 11}
    base = dict(present={11: 1000}, goal={11: 1000}, load={11: 0})
    r.update(now=0.0, **base, ext_torque={"J2": 1.5}, ext_joint_motor=m)
    assert r.update(now=0.1, **base, ext_torque={"J2": 0.0}, ext_joint_motor=m) == []   # 사라짐 → 리셋
    assert r.update(now=0.4, **base, ext_torque={"J2": 1.5}, ext_joint_motor=m) == []   # 타이머 재시작
    assert r.update(now=0.6, **base, ext_torque={"J2": 1.5}, ext_joint_motor=m) == []   # 0.2s < t_ext
    assert r.mode is Mode.MOVE


def test_external_force_warn_band_only():
    """warn~trip 사이는 경고만 쌓이고 트립하지 않는다."""
    r = Reflex(LIMITS, PAIRS, ReflexConfig(ext_warn_nm=0.3, ext_trip_nm=1.0, t_ext=0.3))
    m = {"J2": 11}
    base = dict(present={11: 1000}, goal={11: 1000}, load={11: 0})
    for k in range(5):
        assert r.update(now=0.1 * k, **base, ext_torque={"J2": 0.5}, ext_joint_motor=m) == []
        assert any("J2" in w for w in r.warnings())
    assert r.mode is Mode.MOVE


def test_external_force_gated_while_moving():
    """사이클당 ext_motion_ticks 이상 움직이는 동안은 타이머가 리셋되어 트립하지 않는다."""
    # accel_step을 크게 잡아 목표 추종에 의한 deferral은 끄고, 위치 변화 게이팅만 본다
    r = Reflex(LIMITS, PAIRS, ReflexConfig(ext_trip_nm=1.0, t_ext=0.3, ext_motion_ticks=40, accel_step=1000))
    m = {"J2": 11}
    trips = []
    for k in range(30):  # 0.6s 동안 매 사이클 50틱 이동, τ_ext는 계속 초과
        pos = 1000 + 50 * k
        trips += r.update(now=0.02 * k, present={11: pos}, goal={11: pos}, load={11: 0},
                          ext_torque={"J2": 1.5}, ext_joint_motor=m)
    assert trips == [] and r.mode is Mode.MOVE
    # 정지하면 타이머가 진행되어 트립
    pos = 1000 + 50 * 29
    trips = []
    for k in range(30, 50):
        trips += r.update(now=0.02 * k, present={11: pos}, goal={11: pos}, load={11: 0},
                          ext_torque={"J2": 1.5}, ext_joint_motor=m)
    assert [t.event for t in trips] == [Event.EXTERNAL_FORCE]


def test_external_force_gated_during_accel_deferral():
    """목표 급변 직후(t_accel)에는 τ_ext 초과가 지속돼도 타이머가 안 잡힌다."""
    r = Reflex(LIMITS, PAIRS, ReflexConfig(ext_trip_nm=1.0, t_ext=0.2, accel_step=40, t_accel=0.5,
                                           ext_motion_ticks=1000))  # 위치 게이트는 무력화
    m = {"J2": 11}
    ext = {"J2": 1.5}
    r.update(now=0.0, present={11: 1000}, goal={11: 1000}, load={11: 0}, ext_torque=ext, ext_joint_motor=m)
    # 목표 급변 → deferral 시작 (~0.55). 타이머는 리셋.
    r.update(now=0.05, present={11: 1000}, goal={11: 1060}, load={11: 0}, ext_torque=ext, ext_joint_motor=m)
    assert r.update(now=0.4, present={11: 1000}, goal={11: 1060}, load={11: 0}, ext_torque=ext, ext_joint_motor=m) == []
    # deferral 종료 후 타이머 재시작 → t_ext 전엔 안 뜸
    assert r.update(now=0.7, present={11: 1000}, goal={11: 1060}, load={11: 0}, ext_torque=ext, ext_joint_motor=m) == []
    assert r.update(now=0.85, present={11: 1000}, goal={11: 1060}, load={11: 0}, ext_torque=ext, ext_joint_motor=m) == []
    trips = r.update(now=0.95, present={11: 1000}, goal={11: 1060}, load={11: 0}, ext_torque=ext, ext_joint_motor=m)
    assert [t.event for t in trips] == [Event.EXTERNAL_FORCE]


def test_external_force_none_disabled():
    """ext_torque=None(관측기 없음)이면 판정 비활성 — 기존 호출과 동일하게 동작."""
    r = Reflex(LIMITS, PAIRS)
    for k in range(40):
        assert r.update(now=0.02 * k, present={11: 1000}, goal={11: 1000}, load={11: 0}) == []
    assert r.mode is Mode.MOVE
