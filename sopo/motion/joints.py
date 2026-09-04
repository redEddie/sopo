"""Joint-level abstraction over raw Feetech motor IDs.

Provides a single-controller view of joints that may be:
- one motor (J4/J5/J6/J7),
- two mirrored motors (J2/J3), or
- a cable-limited continuous joint that wraps at 0/4095 (J1).
"""

from __future__ import annotations

import math
from typing import Protocol


from ..hal.bus import FeetechBus

TICKS_PER_REV = 4096
MAX_SINGLE_MOVE = TICKS_PER_REV // 2  # half a turn: last-resort guard for continuous joints


class Joint(Protocol):
    """Logical joint that hides motor-count and wrap details."""

    name: str

    @property
    def motor_ids(self) -> tuple[int, ...]:
        ...

    def read(self, bus: FeetechBus) -> int:
        """Return logical joint position in the reference frame."""
        ...

    def command(self, bus: FeetechBus, logical_goal: int) -> None:
        """Write motor goal(s) corresponding to the logical joint goal."""
        ...


class SingleMotorJoint:
    """One-to-one joint, e.g. J4/J5/J6/J7."""

    def __init__(self, name: str, motor_id: int):
        self.name = name
        self.motor_id = motor_id

    @property
    def motor_ids(self) -> tuple[int, ...]:
        return (self.motor_id,)

    def read(self, bus: FeetechBus) -> int:
        return bus.read("Present_Position", self.motor_id)

    def command(self, bus: FeetechBus, logical_goal: int) -> None:
        bus.write("Goal_Position", self.motor_id, _clamp_ticks(logical_goal))


class DualMotorJoint:
    """Two motors mounted in opposition, e.g. J2(10,11) / J3(15,16).

    One motor is the reference; the joint angle is read from it. The other
    motor is commanded so the pair sums to the measured constant K, keeping
    the joint stiff without fighting.
    """

    def __init__(self, name: str, ids: tuple[int, int], reference_id: int, K: int, preload_ticks: int = 0):
        if reference_id not in ids:
            raise ValueError(f"reference_id {reference_id} must be one of {ids}")
        self.name = name
        self.ids = ids
        self.reference_id = reference_id
        self.mirror_id = ids[1] if ids[0] == reference_id else ids[0]
        self.K = K
        # Anti-backlash: command the mirror motor preload_ticks *past* K - goal so the two gear trains
        # push against each other and the joint has no free play. Costs a constant small load;
        # keep it a few ticks (backlash is ~5 ticks on sts3250).
        self.preload_ticks = preload_ticks

    @property
    def motor_ids(self) -> tuple[int, ...]:
        return self.ids

    def read(self, bus: FeetechBus) -> int:
        return bus.read("Present_Position", self.reference_id)

    def command(self, bus: FeetechBus, logical_goal: int) -> None:
        goal = _clamp_ticks(logical_goal)
        mirror_goal = _clamp_ticks(self.K - goal - self.preload_ticks)
        bus.write("Goal_Position", self.reference_id, goal)
        bus.write("Goal_Position", self.mirror_id, mirror_goal)


class ContinuousJoint:
    """Continuous-rotation joint whose raw position wraps at 0/4095, e.g. J1.

    There is no mechanical stop; the cable harness limits rotation. This class
    unwraps the 12-bit reading into a monotonic logical angle by counting wraps,
    and clamps goals to ``home ± range_ticks`` where ``home`` is the first
    reading after start. Clamping is silent (no error, no torque cut).

    The turn count lives in memory only, so the program must be started with the
    cable relaxed (that pose becomes the centre of the allowed range).

    Firmware modes (verified on sm8512bl, 2026-08-28):
      single-turn (Phase bit4 off): the servo never crosses raw 4095/0 - it takes the long way
        round, which with a step-clamped goal shows up as a +-5 deg chatter at the seam.
      multi-turn (Phase bit4 on, Min/Max_Position_Limit 0/0): position is reported and
        accepted beyond 0..4095 continuously (+415 deg measured). Use firmware_multiturn=True.
        The turn count resets at power-on, so the start pose is still the range centre.
    """

    def __init__(self, name: str, motor_id: int, range_ticks: int | None = None, firmware_multiturn: bool = False,
                 home_abs: int | None = None):
        self.name = name
        self.motor_id = motor_id
        self.range_ticks = range_ticks
        # Absolute single-turn reading of the cable-relaxed pose (set with cookbook/09 --center, usually 2048).
        # The encoder is absolute within a turn, so home = the copy of home_abs nearest to the first reading.
        # Only the turn count is unknown at power-on; with range_ticks <= 2048 that choice is unique.
        self.home_abs = home_abs
        # True: servo runs with Phase bit4 set and Min/Max_Position_Limit 0/0 -> it reports and accepts
        # multi-turn positions itself, so goals are written as-is (no mod 4096). Software turn
        # counting stays harmless (deltas never exceed half a turn).
        self.firmware_multiturn = firmware_multiturn
        self.turn_count = 0
        self.home: int | None = None
        self._last_raw: int | None = None

    @property
    def motor_ids(self) -> tuple[int, ...]:
        return (self.motor_id,)

    def range_bounds(self) -> tuple[int, int] | None:
        if self.home is None or self.range_ticks is None:
            return None
        return (self.home - self.range_ticks, self.home + self.range_ticks)

    def clamp(self, logical_goal: int) -> int:
        """Silently limit a logical goal to the cable range (not an error)."""
        bounds = self.range_bounds()
        if bounds is None:
            return logical_goal
        return max(bounds[0], min(bounds[1], logical_goal))

    def _update_turn(self, raw: int) -> None:
        # Only reads may change the turn count: a jump of more than half a turn
        # between consecutive readings means the encoder wrapped.
        if self._last_raw is not None:
            delta = raw - self._last_raw
            if delta > TICKS_PER_REV // 2:
                self.turn_count -= 1
            elif delta < -(TICKS_PER_REV // 2):
                self.turn_count += 1
        self._last_raw = raw

    def read(self, bus: FeetechBus) -> int:
        raw = bus.read("Present_Position", self.motor_id)
        self._update_turn(raw)
        logical = raw + self.turn_count * TICKS_PER_REV
        if self.home is None:
            if self.home_abs is None:
                self.home = logical
            else:
                self.home = self.home_abs + round((logical - self.home_abs) / TICKS_PER_REV) * TICKS_PER_REV
        return logical

    def command(self, bus: FeetechBus, logical_goal: int) -> None:
        if self._last_raw is None:
            self.read(bus)
        current = self._last_raw + self.turn_count * TICKS_PER_REV
        goal = self.clamp(logical_goal)
        # Never ask for more than half a turn in one command: the caller's step
        # clamp normally keeps this far smaller, this is a last-resort guard.
        move = max(-MAX_SINGLE_MOVE, min(MAX_SINGLE_MOVE, goal - current))
        target = current + move
        # Do NOT touch turn_count here - it tracks the *present* position and is
        # updated by read() when the motor actually crosses the wrap.
        if self.firmware_multiturn:
            bus.write("Goal_Position", self.motor_id, target)  # firmware counts turns; +/-32767 ticks (sign-magnitude)
            return
        raw_goal = target - math.floor(target / TICKS_PER_REV) * TICKS_PER_REV
        bus.write("Goal_Position", self.motor_id, raw_goal)


def _clamp_ticks(value: int) -> int:
    """Clamp to the motor's native 0..4095 range."""
    return max(0, min(TICKS_PER_REV - 1, value))


def build_joints(configs: list[dict]) -> list[Joint]:
    """Build Joint objects from a config list (see configs/arm.yaml)."""
    joints: list[Joint] = []
    for cfg in configs:
        name = cfg["name"]
        jtype = cfg["type"]
        if jtype == "single":
            joints.append(SingleMotorJoint(name, cfg["motor_id"]))
        elif jtype == "dual":
            ids = tuple(cfg["ids"])
            joints.append(DualMotorJoint(name, ids, cfg["reference_id"], cfg["K"], int(cfg.get("preload_ticks", 0))))
        elif jtype == "continuous":
            joints.append(ContinuousJoint(name, cfg["motor_id"], cfg.get("range_ticks"), bool(cfg.get("firmware_multiturn", False)),
                                          cfg.get("home_abs")))
        else:
            raise ValueError(f"Unknown joint type: {jtype}")
    return joints
