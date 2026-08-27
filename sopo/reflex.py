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
    JOINT_LIMIT = "joint_limit"  # 측정 위치가 소프트 리밋 밖 (libfranka joint_position_limits_violation)


@dataclass
class ReflexConfig:
    sat_ratio: float = 0.95
    t_collision: float = 0.3
    progress_ticks: int = 40  # 포화 창(t_collision) 동안 이만큼 못 나아가면 막힌 것
    accel_step: int = 40
    t_accel: float = 0.2
    err_ticks: int = 150
    t_error: float = 0.5
    pair_tol: int = 60   # measured: free +-6, both motors saturated by a grab up to +21; a real fight is hundreds
    t_pair: float = 0.3  # must persist (gear deflection under load is transient)
    comm_fail_max: int = 5
    temp_warn: int = 65
    temp_stop: int = 70
    temp_confirm: int = 2       # consecutive readings >= temp_stop before OVERTEMP (a garbled packet must not stop the arm)
    temp_plausible: int = 100   # readings outside 0..this are ignored as bus garbage
    backoff_ticks: int = 0
    recover_load_ratio: float = 0.9  # recover() allowed while hold load < this * cap (gravity load is not external force)
    limit_margin: int = 30  # 소프트 리밋을 이만큼 넘어야 JOINT_LIMIT (클램프 자체는 이벤트 아님)
    t_limit: float = 0.5    # 리밋 밖에 이만큼 머물러야 JOINT_LIMIT (되돌아오는 중이면 안 뜸)


@dataclass
class Trip:
    event: Event
    motor_id: int | None
    detail: str


class _MotorState:
    """Per-motor runtime state for the reflex detector."""

    def __init__(self) -> None:
        self.sat_start: float | None = None
        self.pos_at_sat: int | None = None
        self.dir_at_sat: int = 0  # 포화 시작 시 목표 방향 (+1/-1)
        self.err_start: float | None = None
        self.accel_until: float = 0.0
        self.last_goal: int | None = None
        self.limit_armed: bool = False   # 범위 안에 한 번 들어온 뒤에만 리밋 감시
        self.limit_start: float | None = None


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
        self._extra_limits: dict[int, tuple[int, int]] = {}  # continuous joints: home +/- range (logical frame)
        self._pair_start: dict[str, float] = {}
        self._temp_over: dict[int, int] = {}

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

        # Temperature checks: implausible values are bus garbage (one shifted byte can put a position
        # byte in the temperature slot); a real thermal event persists, so require temp_confirm samples.
        if temps:
            for motor_id, temp in temps.items():
                if not 0 <= temp <= self._cfg.temp_plausible:
                    self._warnings.append(f"motor {motor_id} implausible temp {temp}°C ignored (bus garbage?)")
                    continue
                if temp >= self._cfg.temp_stop:
                    n = self._temp_over.get(motor_id, 0) + 1
                    self._temp_over[motor_id] = n
                    if n >= self._cfg.temp_confirm:
                        trip = Trip(Event.OVERTEMP, motor_id, f"temp {temp}°C >= stop {self._cfg.temp_stop}°C ({n} readings)")
                        new_trips.extend(self._emit(trip))
                    else:
                        self._warnings.append(f"motor {motor_id} temp {temp}°C >= stop, confirming ({n}/{self._cfg.temp_confirm})")
                else:
                    self._temp_over[motor_id] = 0
                    if temp >= self._cfg.temp_warn:
                        self._warnings.append(f"motor {motor_id} temp {temp}°C >= warn {self._cfg.temp_warn}°C")

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

            # COLLISION detection: saturated for a full window AND little *movement toward the goal*
            # in that window. Measured on position, not on error: with a jog/waypoint the goal keeps
            # running ahead so the error never shrinks even though the joint moves freely.
            #   free motion following the goal  -> large positive progress -> no trip
            #   held by a hand / blocked         -> ~0                      -> trip
            #   pushed by a hand away from goal  -> negative                -> trip
            #   creeping / lifting at the cap    -> small positive          -> trip (cap insufficient)
            if now >= state.accel_until:
                if load_abs >= self._cfg.sat_ratio * cap:
                    if state.sat_start is None:
                        state.sat_start = now
                        state.pos_at_sat = pos
                        state.dir_at_sat = 1 if tgt >= pos else -1
                    elif now - state.sat_start >= self._cfg.t_collision:
                        progress = (pos - (state.pos_at_sat or pos)) * state.dir_at_sat
                        if progress < self._cfg.progress_ticks:
                            hint = " (pushed back)" if progress < 0 else (" (slow progress, cap too low?)" if progress > 0 else "")
                            trip = Trip(
                                Event.COLLISION,
                                motor_id,
                                f"load {load_val}‰ saturated {now - state.sat_start:.2f}s, "
                                f"moved {progress:+d} ticks toward goal < {self._cfg.progress_ticks}{hint}",
                            )
                            new_trips.extend(self._emit(trip))
                        else:
                            state.sat_start = now  # 진행 중: 새 창 시작
                            state.pos_at_sat = pos
                            state.dir_at_sat = 1 if tgt >= pos else -1
                else:
                    state.sat_start = None
                    state.pos_at_sat = None

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

        # JOINT_LIMIT: 명령은 이미 클램프되므로 여기 걸리는 건 외력으로 밀렸거나 캡 부족으로 처진 경우.
        # 토크 OFF 중 기계 끝에 놓인 채 시작하는 일이 흔하므로, 범위 안에 한 번 들어온 뒤(armed)에만,
        # 그리고 t_limit 이상 밖에 머물 때만 판정한다 (되돌아오는 중이면 이벤트 아님).
        # 캘리브레이션된(position_limits에 있는) 모터만 — 연속 관절(J1)은 논리각이라 제외.
        if self._mode is not Mode.REFLEX:
            for motor_id, (lo, hi) in {**self._limits.position_limits, **self._extra_limits}.items():
                pos = present.get(motor_id)
                if pos is None:
                    continue
                st = self._motor_states.setdefault(motor_id, _MotorState())
                if lo <= pos <= hi:
                    st.limit_armed = True
                    st.limit_start = None
                    continue
                outside = pos < lo - self._cfg.limit_margin or pos > hi + self._cfg.limit_margin
                if not (st.limit_armed and outside):
                    st.limit_start = None
                    continue
                if st.limit_start is None:
                    st.limit_start = now
                elif now - st.limit_start >= self._cfg.t_limit:
                    trip = Trip(Event.JOINT_LIMIT, motor_id,
                                f"position {pos} outside [{lo}, {hi}] for {now - st.limit_start:.2f}s")
                    new_trips.extend(self._emit(trip))

        # PAIR_MISMATCH detection (with dwell: a grab deflects the two gear trains transiently).
        if self._mode is not Mode.REFLEX:
            for joint_name, (ref, mirror, K) in self._pairs.items():
                if ref not in present or mirror not in present:
                    continue
                dev = present[ref] + present[mirror] - K
                if abs(dev) <= self._cfg.pair_tol:
                    self._pair_start.pop(joint_name, None)
                    continue
                start = self._pair_start.setdefault(joint_name, now)
                if now - start >= self._cfg.t_pair:
                    trip = Trip(
                        Event.PAIR_MISMATCH,
                        ref,
                        f"{joint_name} sum {present[ref] + present[mirror]} != K {K} by {dev:+d} for {now - start:.2f}s (tol {self._cfg.pair_tol})",
                    )
                    new_trips.extend(self._emit(trip))

        return new_trips

    def set_limit(self, motor_id: int, lo: int, hi: int) -> None:
        """Register a range for JOINT_LIMIT (used for continuous joints once home is known)."""
        self._extra_limits[motor_id] = (lo, hi)

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
                backed = raw - load_sign * self._cfg.backoff_ticks
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
            if abs(load.get(motor_id, 0)) >= self._cfg.recover_load_ratio * cap:
                return False, f"motor {motor_id} load {abs(load.get(motor_id, 0))}‰ >= {self._cfg.recover_load_ratio:.0%} of cap {cap}"
            hold = self._last_hold_target.get(motor_id)
            if hold is not None and motor_id in present:
                if abs(present[motor_id] - hold) > 30:
                    return False, f"motor {motor_id} too far from hold target"

        self._mode = Mode.MOVE
        self._trips.clear()
        self._last_collision = None
        for state in self._motor_states.values():
            state.sat_start = None
            state.pos_at_sat = None
            state.err_start = None
            state.limit_start = None
        self._pair_start.clear()
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
