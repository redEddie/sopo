"""Host-side reflex / collision detector for sopo arms.

Pure logic: this module never touches the bus. Feed it sensor readings every
cycle and it returns events and a requested safety mode. The caller is
responsible for writing hold targets or disabling torque.

Design: docs/reflex-spec.md
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .safety import SafetyLimits


class Mode(Enum):
    IDLE = "idle"
    MOVE = "move"
    REFLEX = "reflex"
    STOPPED = "stopped"


class Event(Enum):
    COLLISION = "collision"
    TRACKING_ERROR = "tracking_error"
    PAIR_MISMATCH = "pair_mismatch"
    COMM_LOSS = "comm_loss"
    OVERTEMP = "overtemp"


@dataclass
class ReflexConfig:
    sat_ratio: float = 0.95
    t_collision: float = 0.3
    accel_step: int = 40
    t_accel: float = 0.2
    err_ticks: int = 150
    t_error: float = 0.5
    pair_tol: int = 20
    comm_fail_max: int = 5
    temp_warn: int = 65
    temp_stop: int = 70
    backoff_ticks: int = 0


@dataclass
class Trip:
    event: Event
    motor_id: int | None
    detail: str


class _MotorState:
    """Per-motor runtime state for the reflex detector."""

    def __init__(self) -> None:
        self.sat_start: float | None = None
        self.min_err: int | None = None
        self.err_start: float | None = None
        self.accel_until: float = 0.0
        self.last_goal: int | None = None


class Reflex:
    """Host-side safety reflex.

    Args:
        limits: SafetyLimits used to read per-motor torque caps.
        pairs: Mapping joint_name -> (reference_id, mirror_id, K) for dual-motor
            joints. Used only for PAIR_MISMATCH detection.
        cfg: Tunable thresholds.
    """

    def __init__(
        self,
        limits: SafetyLimits,
        pairs: dict[str, tuple[int, int, int]],
        cfg: ReflexConfig = ReflexConfig(),
    ):
        self._limits = limits
        self._pairs = dict(pairs)
        self._cfg = cfg

        # All motor ids we ever need to reason about.
        ids: set[int] = set()
        for ref, mirror, _ in pairs.values():
            ids.add(ref)
            ids.add(mirror)
        ids.update(limits.torque_limits.keys())
        ids.update(limits.position_limits.keys())
        self._motor_ids: tuple[int, ...] = tuple(sorted(ids))

        self._mode = Mode.IDLE
        self._trips: set[tuple[Event, int | None]] = set()
        self._motor_states: dict[int, _MotorState] = {mid: _MotorState() for mid in ids}
        self._comm_fail_count = 0
        self._warnings: list[str] = []
        self._last_hold_target: dict[int, int] = {}
        self._last_collision: Trip | None = None
        self._last_load: dict[int, int] = {}

    @property
    def mode(self) -> Mode:
        return self._mode

    def update(
        self,
        now: float,
        present: dict[int, int],
        goal: dict[int, int],
        load: dict[int, int],
        temps: dict[int, int] | None = None,
        comm_ok: bool = True,
    ) -> list[Trip]:
        """Evaluate one control cycle and return any new trips."""
        self._warnings.clear()
        new_trips: list[Trip] = []

        # Ensure we know about every id that appears in live data.
        live_ids = set(present) | set(goal) | set(load)
        for mid in live_ids:
            if mid not in self._motor_states:
                self._motor_states[mid] = _MotorState()
        if live_ids - set(self._motor_ids):
            self._motor_ids = tuple(sorted(set(self._motor_ids) | live_ids))

        # Communication loss is checked first; it overrides everything.
        if not comm_ok:
            self._comm_fail_count += 1
            if self._comm_fail_count >= self._cfg.comm_fail_max:
                trip = Trip(
                    Event.COMM_LOSS,
                    None,
                    f"comm failed {self._comm_fail_count} cycles",
                )
                new_trips.extend(self._emit(trip))
        else:
            self._comm_fail_count = 0

        # Temperature checks.
        if temps:
            for motor_id, temp in temps.items():
                if temp >= self._cfg.temp_stop:
                    trip = Trip(
                        Event.OVERTEMP,
                        motor_id,
                        f"temp {temp}°C >= stop {self._cfg.temp_stop}°C",
                    )
                    new_trips.extend(self._emit(trip))
                elif temp >= self._cfg.temp_warn:
                    self._warnings.append(
                        f"motor {motor_id} temp {temp}°C >= warn {self._cfg.temp_warn}°C"
                    )

        # Once STOPPED, nothing else matters.
        if self._mode is Mode.STOPPED:
            return new_trips

        # Promote from IDLE to MOVE on the first live cycle, unless a trip
        # already forced REFLEX / STOPPED above.
        if self._mode is Mode.IDLE:
            self._mode = Mode.MOVE

        # Per-motor checks.
        for motor_id in self._motor_ids:
            if self._mode is Mode.REFLEX:
                # In REFLEX only comm/temp can create new trips.
                break

            pos = present.get(motor_id)
            tgt = goal.get(motor_id)
            if pos is None or tgt is None:
                continue

            state = self._motor_states[motor_id]
            cap = self._limits.torque_for(motor_id)
            err = abs(tgt - pos)
            load_val = load.get(motor_id, 0)
            self._last_load[motor_id] = load_val
            load_abs = abs(load_val)

            # Acceleration deferral: big goal jump suspends collision detection.
            if state.last_goal is not None:
                if abs(tgt - state.last_goal) >= self._cfg.accel_step:
                    state.accel_until = now + self._cfg.t_accel
            state.last_goal = tgt

            # COLLISION detection.
            if now >= state.accel_until:
                if load_abs >= self._cfg.sat_ratio * cap:
                    if state.sat_start is None:
                        state.sat_start = now
                        state.min_err = err
                    else:
                        if state.min_err is not None and err < state.min_err:
                            state.min_err = err
                            state.sat_start = now
                        # else: error is not decreasing
                        if (
                            now - state.sat_start >= self._cfg.t_collision
                            and state.min_err is not None
                            and err >= state.min_err
                        ):
                            trip = Trip(
                                Event.COLLISION,
                                motor_id,
                                f"load {load_val}‰ saturated for {now - state.sat_start:.3f}s",
                            )
                            new_trips.extend(self._emit(trip))
                else:
                    state.sat_start = None
                    state.min_err = None

            # TRACKING_ERROR detection.
            if err > self._cfg.err_ticks:
                if state.err_start is None:
                    state.err_start = now
                elif now - state.err_start >= self._cfg.t_error:
                    trip = Trip(
                        Event.TRACKING_ERROR,
                        motor_id,
                        f"error {err} ticks for {now - state.err_start:.3f}s",
                    )
                    new_trips.extend(self._emit(trip))
            else:
                state.err_start = None

        # PAIR_MISMATCH detection.
        if self._mode is not Mode.REFLEX:
            for joint_name, (ref, mirror, K) in self._pairs.items():
                if ref not in present or mirror not in present:
                    continue
                if abs(present[ref] + present[mirror] - K) > self._cfg.pair_tol:
                    trip = Trip(
                        Event.PAIR_MISMATCH,
                        ref,
                        f"{joint_name} sum {present[ref] + present[mirror]} != K {K}±{self._cfg.pair_tol}",
                    )
                    new_trips.extend(self._emit(trip))

        return new_trips

    def hold_targets(self, present: dict[int, int]) -> dict[int, int]:
        """Return motor goals that hold the arm at the measured position.

        If a collision was detected and backoff_ticks is configured, the
        offending motor is backed off opposite to the load direction.
        """
        targets = {mid: present[mid] for mid in self._motor_ids if mid in present}

        if self._mode is Mode.REFLEX and self._cfg.backoff_ticks:
            trip = self._last_collision
            if trip and trip.motor_id is not None and trip.motor_id in present:
                load_sign = 1
                if trip.motor_id in self._last_load:
                    load_sign = 1 if self._last_load[trip.motor_id] >= 0 else -1
                raw = present.get(trip.motor_id, 0)
                backed = max(0, min(4095, raw - load_sign * self._cfg.backoff_ticks))
                targets[trip.motor_id] = backed

        self._last_hold_target = dict(targets)
        return targets

    def recover(
        self,
        present: dict[int, int],
        load: dict[int, int],
    ) -> tuple[bool, str]:
        """Attempt to leave REFLEX and resume MOVE.

        Requires all motors to be below 50% of their torque cap and within
        30 ticks of the last hold target.
        """
        if self._mode is not Mode.REFLEX:
            return False, f"not in REFLEX mode (mode={self._mode.value})"

        for motor_id in self._motor_ids:
            cap = self._limits.torque_for(motor_id)
            if abs(load.get(motor_id, 0)) >= 0.5 * cap:
                return False, f"motor {motor_id} load too high"
            hold = self._last_hold_target.get(motor_id)
            if hold is not None and motor_id in present:
                if abs(present[motor_id] - hold) > 30:
                    return False, f"motor {motor_id} too far from hold target"

        self._mode = Mode.MOVE
        self._trips.clear()
        self._last_collision = None
        for state in self._motor_states.values():
            state.sat_start = None
            state.min_err = None
            state.err_start = None
        return True, ""

    def warnings(self) -> list[str]:
        """Return non-fatal warnings accumulated this cycle and clear them."""
        out = list(self._warnings)
        self._warnings.clear()
        return out

    def _emit(self, trip: Trip) -> list[Trip]:
        """Emit a trip if it is new for the current latched episode."""
        key = (trip.event, trip.motor_id)
        if key in self._trips:
            return []
        self._trips.add(key)

        if trip.event in (Event.COMM_LOSS, Event.OVERTEMP):
            self._mode = Mode.STOPPED
        elif self._mode is not Mode.STOPPED:
            self._mode = Mode.REFLEX

        if trip.event is Event.COLLISION and self._mode is not Mode.STOPPED:
            self._last_collision = trip

        return [trip]


# Re-export for type checkers / importers.
__all__ = ["Mode", "Event", "ReflexConfig", "Trip", "Reflex"]
