"""Torque/effort limiting so the arm cannot hurt a person or itself.

Three layers, applied together:
1. Motor-side effort cap   - Torque_Limit / Max_Torque_Limit (0-1000 = 0-100%)
2. Motor-side overload trip - Overload_Torque + Protection_Time + Protective_Torque
3. Host-side clamping       - joint limits and max step per control cycle
"""

from __future__ import annotations

from dataclasses import dataclass, field

from ..hal.bus import FeetechBus

# Datasheet stall torque at 12V in kg.cm. Torque_Limit is per-mille of the
# servo's maximum output, which equals this at standstill.
#   sts3215: 12V variant (C018) = 30.0; the 7.4V variant (C001) is 19.5 - check the label
MODEL_STALL_TORQUE_KGCM = {
    "sts3215": 30.0,
    "sts3250": 50.0,
    "sm8512bl": 85.0,
}
KGCM_TO_NM = 0.0980665  # kg·cm → N·m 변환은 이 상수 한 곳에서만
CURRENT_UNIT_MA = 6.5  # Present_Current / Protection_Current register unit


def stall_nm(model: str, count: int = 1) -> float:
    """모델의 스톨 토크 [N·m @12V]. count는 관절의 모터 수 (듀얼이면 2, 합산)."""
    return MODEL_STALL_TORQUE_KGCM[model] * KGCM_TO_NM * count


def torque_limit_from_kgcm(model: str, kgcm: float) -> int:
    """kg.cm -> Torque_Limit per-mille for a given motor model (exact at stall)."""
    stall = MODEL_STALL_TORQUE_KGCM[model]
    return max(0, min(1000, round(kgcm / stall * 1000)))


def torque_limit_to_kgcm(model: str, limit: int) -> float:
    """Torque_Limit per-mille -> kg.cm upper bound (exact at stall, less while moving)."""
    return MODEL_STALL_TORQUE_KGCM[model] * limit / 1000


@dataclass
class SafetyLimits:
    # Effort cap in per-mille of stall torque. 300 = 30%: an STS3215 (~19 kg.cm
    # stall at 12V) then pushes with at most ~5.7 kg.cm - uncomfortable, not injuring.
    torque_limit: int = 300
    # Load % that starts the overload timer (per-mille). If the motor pushes against
    # something at >= this fraction of its allowed torque...
    overload_torque: int = 80
    # ...for this long (units of 10ms), it drops to protective torque.
    protection_time: int = 50  # 500 ms
    # Torque % it falls back to after tripping (per-mille of Max_Torque_Limit).
    protective_torque: int = 20
    # Acceleration register, units of ~8.7 deg/s^2. Low = soft starts/stops.
    acceleration: int = 30
    # Joint limits in ticks (0-4095 for STS). Defaults keep clear of the wrap point.
    min_position: int = 200
    max_position: int = 3896
    # Max goal change per control cycle in ticks. At 50 Hz, 80 ticks/cycle is
    # about 350 deg/s worst case; lower it for a heavier arm.
    max_relative_target: int = 80
    # Franka의 position-based velocity limit 단순판: 리밋(또는 J1 케이블 범위)까지 남은 거리가
    # brake_zone_ticks 안이면 허용 스텝을 거리에 비례해 줄인다 (하한 brake_min_step). 도달 전에 감속.
    brake_zone_ticks: int = 200
    brake_min_step: int = 30
    # Stop policy (IEC 60204-1 stop category 2 / ISO 10218-1 safety-rated monitored stop): on a fault the
    # arm FREEZES with torque kept - goal = present and Torque_Limit raised to hold_torque_limit so it
    # stays rigid under gravity and a held object. Torque is dropped only by an explicit idle/guiding.
    hold_torque_limit: int = 600
    # EPROM Max_Torque_Limit written by cookbook/10_persist_caps: the hardware ceiling. RAM Torque_Limit
    # (motion caps, lower) is set by apply_safety() on every torque-on; hold raises it up to the ceiling.
    eprom_torque_ceiling: int = 600  # >= ~30: below that the P controller output (~8 permille/tick) cannot move a loaded joint

    # Optional per-joint overrides: motor_id -> (min_position, max_position)
    position_limits: dict[int, tuple[int, int]] = field(default_factory=dict)
    # Optional per-joint effort caps: motor_id -> per-mille. Measured with
    # cookbook/06_gravity_load.py so each joint gets just enough for its own weight.
    torque_limits: dict[int, int] = field(default_factory=dict)
    # Optional absolute current trip in mA (Protection_Current, EPROM). Unlike
    # Torque_Limit (a duty-cycle fraction) this is a physical quantity: the motor
    # cuts output when current exceeds it for Over_Current_Protection_Time.
    protection_current_ma: int | None = None

    def joint_range(self, motor_id: int) -> tuple[int, int]:
        return self.position_limits.get(motor_id, (self.min_position, self.max_position))

    def torque_for(self, motor_id: int) -> int:
        return self.torque_limits.get(motor_id, self.torque_limit)


def apply_safety(bus: FeetechBus, motor_ids: list[int], limits: SafetyLimits) -> None:
    """Writes the motor-side caps. Call after connect(), before enabling torque.

    Only touches RAM (Torque_Limit) plus the overload EPROM registers; the
    persistent Max_Torque_Limit is left alone unless you call
    persist_torque_limit() explicitly.
    """
    for motor_id in motor_ids:
        with bus.eprom_unlocked(motor_id):
            # Return_Delay_Time > 0 desynchronizes the sync-read response chain:
            # a delayed reply corrupts every reply after it in the group
            # (observed on sm8512bl shipping with 250 = 500us; sts32xx ship with 0).
            bus.write("Return_Delay_Time", motor_id, 0)
            bus.write("Overload_Torque", motor_id, limits.overload_torque)
            bus.write("Protection_Time", motor_id, limits.protection_time)
            bus.write("Protective_Torque", motor_id, limits.protective_torque)
            if limits.protection_current_ma is not None:
                bus.write("Protection_Current", motor_id, round(limits.protection_current_ma / CURRENT_UNIT_MA))
        bus.write("Torque_Limit", motor_id, limits.torque_for(motor_id))
        bus.write("Acceleration", motor_id, limits.acceleration)


def persist_torque_limit(bus: FeetechBus, motor_ids: list[int], torque_limit: int) -> None:
    """Writes Max_Torque_Limit to EPROM so the cap survives power cycles.

    Torque_Limit (RAM) re-initializes from this value at power-on, so a
    persisted cap protects you even if a script forgets apply_safety().
    """
    if not 0 <= torque_limit <= 1000:
        raise ValueError(f"torque_limit must be 0-1000, got {torque_limit}")
    for motor_id in motor_ids:
        with bus.eprom_unlocked(motor_id):
            bus.write("Max_Torque_Limit", motor_id, torque_limit)


def clamp_goal(
    goal: dict[int, int], present: dict[int, int], limits: SafetyLimits
) -> dict[int, int]:
    """Host-side clamp: joint limits + max step per cycle relative to present."""
    safe = {}
    for motor_id, target in goal.items():
        lo, hi = limits.joint_range(motor_id)
        target = max(lo, min(hi, target))
        if motor_id in present:
            pos = present[motor_id]
            step = limits.max_relative_target
            target = max(pos - step, min(pos + step, target))
        safe[motor_id] = target
    return safe


def read_effort(bus: FeetechBus, motor_ids: list[int]) -> dict[int, float]:
    """Present_Load as a signed fraction of stall torque (-1.0 to 1.0)."""
    loads = bus.sync_read("Present_Load", motor_ids)
    return {motor_id: load / 1000 for motor_id, load in loads.items()}


def verify_eprom(bus: FeetechBus, limits: SafetyLimits, motor_ids: list[int]) -> list[str]:
    """모터 EPROM이 캘리브레이션과 다르면 경고 목록. Max_Torque_Limit은 상한(eprom_torque_ceiling)과 비교한다."""
    problems: list[str] = []
    for motor_id in motor_ids:
        eprom_cap = bus.read("Max_Torque_Limit", motor_id)
        if eprom_cap != limits.eprom_torque_ceiling:
            problems.append(f"ID{motor_id}: EPROM Max_Torque_Limit {eprom_cap}‰ != ceiling {limits.eprom_torque_ceiling}‰ (run cookbook/10_persist_caps.py)")
        if motor_id in limits.position_limits:
            lo, hi = limits.position_limits[motor_id]
            got = (bus.read("Min_Position_Limit", motor_id), bus.read("Max_Position_Limit", motor_id))
            if got != (lo, hi):
                problems.append(f"ID{motor_id}: EPROM 위치 한계 {got} != 캘리브레이션 ({lo}, {hi})")
    return problems


def freeze(bus: FeetechBus, motor_ids: list[int], limits: SafetyLimits) -> dict[int, int]:
    """Category-2 stop: hold the present position rigidly with torque ON.

    goal = present (no further motion), Torque_Limit = hold cap, Torque_Enable = 1. Returns the held
    positions. Uses sync_write (no reply needed) so it works even when the receive path is degraded.
    """
    present = bus.sync_read("Present_Position", motor_ids)
    bus.sync_write("Goal_Position", present)
    bus.sync_write("Torque_Limit", {i: limits.hold_torque_limit for i in motor_ids})
    bus.sync_write("Torque_Enable", {i: 1 for i in motor_ids})
    return present

