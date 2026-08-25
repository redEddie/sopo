"""Torque/effort limiting so the arm cannot hurt a person or itself.

Three layers, applied together:
1. Motor-side effort cap   - Torque_Limit / Max_Torque_Limit (0-1000 = 0-100%)
2. Motor-side overload trip - Overload_Torque + Protection_Time + Protective_Torque
3. Host-side clamping       - joint limits and max step per control cycle
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .bus import FeetechBus


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

    # Optional per-joint overrides: motor_id -> (min_position, max_position)
    position_limits: dict[int, tuple[int, int]] = field(default_factory=dict)
    # Optional per-joint effort caps: motor_id -> per-mille. Measured with
    # cookbook/06_gravity_load.py so each joint gets just enough for its own weight.
    torque_limits: dict[int, int] = field(default_factory=dict)

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
