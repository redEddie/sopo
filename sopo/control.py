"""다관절 안전 제어 루프 — 나중에 sopod 데몬의 심장이 될 부분 (docs/architecture.md L5).

루프 한 사이클:
    관절 읽기 → source.get_action() → 클램프(리밋·스텝·J1 범위) → 명령
    → reflex.update() → REFLEX면 홀드 + 복구 대기 / STOPPED면 예외
토크 ON/OFF는 호출자 책임 (apply_safety → enable_torque → run_control_loop → finally disable).
"""

from __future__ import annotations

import csv
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Callable

from .bus import FeetechBus
from .joints import ContinuousJoint, DualMotorJoint, Joint
from .reflex import Event, Mode, Reflex, Trip
from .safety import SafetyLimits, freeze
from .sources import ActionSource

ARRIVAL_TICKS = 30
SOFT_START_STEP = 20
TICKS_PER_REV = 4096


def read_joints(bus: FeetechBus, joints: list[Joint]) -> dict[str, int]:
    return {j.name: j.read(bus) for j in joints}


def command_joints(bus: FeetechBus, joints: list[Joint], goals: dict[str, int]) -> None:
    for j in joints:
        j.command(bus, goals[j.name])


def _braked_step(pos: int, target: int, bounds: tuple[int, int] | None, step: int, zone: int, min_step: int) -> int:
    """리밋에 가까워질수록 허용 스텝을 줄인다 (Franka position-based velocity limit의 단순판)."""
    if bounds is None or zone <= 0 or target == pos:
        return step
    lo, hi = bounds
    dist = (hi - pos) if target > pos else (pos - lo)
    if dist >= zone:
        return step
    return max(min_step, int(step * max(dist, 0) / zone))


def clamp_joint_goals(
    goal: dict[str, int],
    present: dict[str, int],
    joint_limits: dict[str, tuple[int, int]],
    max_step: int,
    joints: list[Joint],
    brake_zone: int = 0,
    brake_min_step: int = 10,
) -> dict[str, int]:
    """관절 리밋(연속 관절은 home ± range_ticks) + 사이클당 스텝 제한 + 리밋 접근 감속. 점프 불가."""
    by_name = {j.name: j for j in joints}
    safe: dict[str, int] = {}
    for name, target in goal.items():
        j = by_name[name]
        if isinstance(j, ContinuousJoint):
            target = j.clamp(target)  # 케이블 범위: 잘라내기만, 오류 아님
            bounds = j.range_bounds()
        else:
            bounds = joint_limits.get(name, (0, TICKS_PER_REV - 1))
            target = max(bounds[0], min(bounds[1], target))
        pos = present[name]
        step = _braked_step(pos, target, bounds, max_step, brake_zone, brake_min_step)
        safe[name] = max(pos - step, min(pos + step, target))
    return safe


def reflex_present_view(bus: FeetechBus, joints: list[Joint], joint_pos: dict[str, int]) -> dict[int, int]:
    """리플렉스용 모터별 현재 위치: 듀얼 쌍은 raw 둘 다(합 검사), 연속 관절은 논리각(goal과 같은 프레임)."""
    ids = [mid for j in joints for mid in j.motor_ids]
    present = bus.sync_read("Present_Position", ids)
    for j in joints:
        if isinstance(j, ContinuousJoint) and j.name in joint_pos:
            present[j.motor_id] = joint_pos[j.name]
    return present


def motor_goals(joints: list[Joint], joint_goals: dict[str, int]) -> dict[int, int]:
    out: dict[int, int] = {}
    for j in joints:
        g = joint_goals[j.name]
        if isinstance(j, DualMotorJoint):
            out[j.reference_id] = g
            out[j.mirror_id] = max(0, min(TICKS_PER_REV - 1, j.K - g))
        else:
            out[j.motor_ids[0]] = g
    return out


def hold_joint_goals(joints: list[Joint], hold: dict[int, int]) -> dict[str, int]:
    return {j.name: hold[j.reference_id if isinstance(j, DualMotorJoint) else j.motor_ids[0]] for j in joints}


def describe_trip(trip: Trip, joints: list[Joint]) -> str:
    """One line: [REFLEX] KIND joint (ID): detail."""
    name_of = {mid: j.name for j in joints for mid in j.motor_ids}
    where = f"{name_of.get(trip.motor_id, '?')} (ID {trip.motor_id})" if trip.motor_id is not None else "all"
    return f"[REFLEX] {trip.event.name} {where}: {trip.detail}"


def end_session(bus: FeetechBus, motor_ids: list[int], limits: SafetyLimits, release: bool = False) -> None:
    """프로그램 종료 시 기본은 홀드(토크 유지, Cat 2). release=True일 때만 토크 해제."""
    if release:
        print("releasing torque - support the arm, it may drop")
        bus.disconnect(disable_torque_ids=motor_ids)
        return
    try:
        freeze(bus, motor_ids, limits)
        print(f"arm HOLDS position (torque on, cap {limits.hold_torque_limit}‰). release with: python cookbook/11_torque_off.py")
    except Exception as e:
        print(f"freeze failed ({e}) - servos keep their last goal", file=sys.stderr)
    bus.disconnect()


def prompt_recover(bus: FeetechBus, joints: list[Joint], reflex: Reflex) -> str:
    """REFLEX 상태 기본 처리: 키 입력. 'r'은 지금 값을 다시 읽어 복구, 'q'는 종료. 반환 'continue' | 'quit'."""
    ids = [mid for j in joints for mid in j.motor_ids]
    while reflex.mode is Mode.REFLEX:
        try:
            key = input("REFLEX latched. [r] recover / [q] quit: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            return "quit"
        if key == "q":
            return "quit"
        if key == "r":
            joint_pos = read_joints(bus, joints)  # 연속 관절 turn 갱신
            present = reflex_present_view(bus, joints, joint_pos)
            load = bus.sync_read("Present_Load", ids)
            ok, reason = reflex.recover(present, load)
            if ok:
                print("recovered, resuming (soft start)")
                return "continue"
            print(f"recover refused: {reason}")
    return "continue"


class Blackbox:
    """최근 N초의 사이클 기록(링버퍼). 리플렉스/정지/종료 시 CSV로 덤프 — Franka last_motion_errors에 대응."""

    def __init__(self, ids: list[int], seconds: float = 60.0, rate_hz: float = 50.0, out_dir: str | Path = "logs"):
        self.ids = ids
        self.rows: deque = deque(maxlen=int(seconds * rate_hz))
        self.out_dir = Path(out_dir)

    def record(self, t: float, mode: str, present: dict[int, int], goal: dict[int, int], load: dict[int, int], note: str = "", volt: dict[int, int] | None = None) -> None:
        row = [f"{t:.3f}", mode]
        for i in self.ids:
            row += [present.get(i, ""), goal.get(i, ""), load.get(i, "")]
        row.append(note)
        row.append(min(volt.values()) / 10 if volt else "")
        self.rows.append(row)

    def dump(self, reason: str) -> Path | None:
        if not self.rows:
            return None
        self.out_dir.mkdir(parents=True, exist_ok=True)
        path = self.out_dir / f"blackbox_{datetime.now():%Y%m%d_%H%M%S}_{reason}.csv"
        with path.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "mode"] + [f"{k}{i}" for i in self.ids for k in ("pos", "goal", "load")] + ["note", "vmin"])
            w.writerows(self.rows)
        self.rows.clear()  # 다음 파일에 같은 구간이 다시 실리지 않도록
        return path


def run_control_loop(
    bus: FeetechBus,
    joints: list[Joint],
    limits: SafetyLimits,
    joint_limits: dict[str, tuple[int, int]],
    reflex: Reflex,
    source: ActionSource,
    rate_hz: float = 50.0,
    on_reflex: Callable[[FeetechBus, list[Joint], Reflex], str] = prompt_recover,
    status_every: float = 0.5,
    verbose: bool = False,
    blackbox: Blackbox | None = None,
    status_inline: bool = False,
    dump_on_exit: bool = False,
) -> None:
    """source가 끝나거나(is_done) 사용자가 종료할 때까지 돈다. 토크는 켜진 채로 돌려받는다."""
    try:
        _run(bus, joints, limits, joint_limits, reflex, source, rate_hz, on_reflex, status_every, verbose, blackbox, status_inline)
    finally:
        if blackbox and dump_on_exit:
            path = blackbox.dump('exit')
            if path:
                print(f"blackbox saved: {path}", file=sys.stderr)


def _run(bus, joints, limits, joint_limits, reflex, source, rate_hz, on_reflex, status_every, verbose, blackbox, status_inline):
    ids = [mid for j in joints for mid in j.motor_ids]
    period = 1.0 / rate_hz
    soft = True
    last_temp = last_status = 0.0
    goal: dict[str, int] = {}
    load: dict[int, int] = {}
    rp: dict[int, int] = {}
    jitter_max = 0.0
    warned_limit: set[str] = set()
    flagged: dict[tuple[int, str], float] = {}
    last_volt = 0.0
    volt_span = (99.0, 0.0)
    first = read_joints(bus, joints)
    for j in joints:
        if isinstance(j, ContinuousJoint) and j.range_bounds():
            lo, hi = j.range_bounds()
            reflex.set_limit(j.motor_id, lo, hi)
            print(f"{j.name}: home {j.home} (abs {j.home_abs}), range [{lo}, {hi}], now {first[j.name]}")
    for n, (lo, hi) in joint_limits.items():
        if not lo <= first[n] <= hi:
            print(f"note: {n} at {first[n]} is outside soft limit [{lo}, {hi}] (parked past the end), soft start brings it back")

    while not source.is_done():
        t0 = time.monotonic()
        comm_ok = True
        try:
            present = read_joints(bus, joints)
            load = bus.sync_read("Present_Load", ids)
            action = source.get_action(present, t0)
            step = SOFT_START_STEP if soft else limits.max_relative_target
            goal = clamp_joint_goals(action, present, joint_limits, step, joints,
                                     limits.brake_zone_ticks, limits.brake_min_step)
            for n, (lo, hi) in joint_limits.items():
                if n in action and not lo <= action[n] <= hi and n not in warned_limit:
                    warned_limit.add(n)
                    print(f"warn: {n} goal {action[n]} outside soft limit [{lo}, {hi}], clamped", file=sys.stderr)
            command_joints(bus, joints, goal)
            if soft and all(abs(action[n] - present[n]) < ARRIVAL_TICKS for n in action):
                soft = False
                print("soft start done")
            rp = reflex_present_view(bus, joints, present)
        except KeyboardInterrupt:
            bus.port.clearPort()  # 트랜잭션 도중 중단: 수신 버퍼 정리 후 상위로
            raise
        except Exception as e:  # comm failure or garbled packet: never let one cycle kill the loop
            comm_ok = False
            print(f"comm error: {e}", file=sys.stderr)

        for mid, text in bus.pop_motor_errors().items():
            key = (mid, text)
            if key not in flagged or t0 - flagged[key] > 5.0:
                flagged[key] = t0
                print(f"motor {mid} status flag: {text}", file=sys.stderr)
        volt = {}
        if comm_ok and t0 - last_volt > 0.2:
            last_volt = t0
            try:
                volt = bus.sync_read("Present_Voltage", ids)
                vmin, vmax = min(volt.values()) / 10, max(volt.values()) / 10
                volt_span = (min(volt_span[0], vmin), max(volt_span[1], vmax))
                if vmin < 11.0 or vmax > 14.5:
                    print(f"warn: bus voltage {vmin:.1f}-{vmax:.1f} V", file=sys.stderr)
            except Exception:
                pass

        temps = None
        if t0 - last_temp > 2.0 and comm_ok:
            last_temp = t0
            temps = bus.sync_read("Present_Temperature", ids)

        trips = []
        if goal:
            mg = motor_goals(joints, goal)
            trips = reflex.update(t0, rp, mg, load, temps=temps, comm_ok=comm_ok)
            for trip in trips:
                print("\n" + describe_trip(trip, joints), file=sys.stderr)
            for w in reflex.warnings():
                print(f"warn: {w}", file=sys.stderr)
            if blackbox:
                blackbox.record(t0, reflex.mode.value, rp, mg, load, ";".join(t.event.value for t in trips), volt or None)

        if reflex.mode is Mode.STOPPED:
            if blackbox:
                print(f"blackbox saved: {blackbox.dump('stopped')}", file=sys.stderr)
            raise RuntimeError("STOPPED (comm loss / overtemp): torque off")

        if reflex.mode is Mode.REFLEX:
            hold = reflex.hold_targets(rp)
            command_joints(bus, joints, hold_joint_goals(joints, hold))  # 논리각·듀얼 변환은 command()가
            print("hold sent", file=sys.stderr)
            if blackbox and trips:
                print(f"blackbox saved: {blackbox.dump('reflex')}", file=sys.stderr)
            if on_reflex(bus, joints, reflex) == "quit":
                raise RuntimeError("aborted by user in REFLEX")
            soft = True
            continue

        elapsed = time.monotonic() - t0
        jitter_max = max(jitter_max, elapsed - period) if elapsed > period else jitter_max
        if verbose and t0 - last_status >= status_every:
            last_status = t0
            line = getattr(source, "last_status", "") or "  ".join(f"{n}:{present[n]}->{goal[n]}" for n in goal)
            raws = "  ".join(f"{j.name} raw={j._last_raw} turn={j.turn_count}" for j in joints if isinstance(j, ContinuousJoint))
            if raws:
                line += "  | " + raws
            text = f"[{source.name}] {line}  | {volt_span[0]:.1f}-{volt_span[1]:.1f}V | cycle {elapsed * 1e3:.1f}ms"
            if status_inline:
                print("\r" + text.ljust(140), end="", flush=True)  # 한 줄에서 갱신
            else:
                print(text)
        if elapsed < period:
            time.sleep(period - elapsed)
