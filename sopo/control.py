"""다관절 안전 제어 루프 — 나중에 sopod 데몬의 심장이 될 부분 (docs/architecture.md L5).

루프 한 사이클:
    관절 읽기 → source.get_action() → 클램프(리밋·스텝·J1 범위) → 명령
    → reflex.update() → REFLEX면 홀드 + 복구 대기 / STOPPED면 예외
토크 ON/OFF는 호출자 책임 (apply_safety → enable_torque → run_control_loop → finally disable).
"""

from __future__ import annotations

import sys
import time
from typing import Callable

from .bus import FeetechBus
from .joints import ContinuousJoint, DualMotorJoint, Joint
from .reflex import Mode, Reflex
from .safety import SafetyLimits
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


def prompt_recover(bus: FeetechBus, joints: list[Joint], reflex: Reflex) -> str:
    """REFLEX 상태 기본 처리: 키 입력. 'r'은 지금 값을 다시 읽어 복구, 'q'는 종료. 반환 'continue' | 'quit'."""
    ids = [mid for j in joints for mid in j.motor_ids]
    while reflex.mode is Mode.REFLEX:
        try:
            key = input("리플렉스 발동. [r] 복구 시도, [q] 종료: ").strip().lower()
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
                print("복구 성공. 재개합니다 (소프트스타트).")
                return "continue"
            print(f"복구 불가: {reason}")
    return "continue"


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
) -> None:
    """source가 끝나거나(is_done) 사용자가 종료할 때까지 돈다. 토크는 켜진 채로 돌려받는다."""
    ids = [mid for j in joints for mid in j.motor_ids]
    period = 1.0 / rate_hz
    soft = True  # 첫 목표(또는 복구 직후)까지는 느린 스텝
    last_temp = last_status = 0.0
    goal: dict[str, int] = {}
    load: dict[int, int] = {}
    rp: dict[int, int] = {}
    jitter_max = 0.0
    warned_limit: set[str] = set()

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
                    print(f"경고: {n} 목표 {action[n]}이 소프트 리밋 [{lo}, {hi}] 밖 — 리밋에서 잘립니다 (이벤트 아님)", file=sys.stderr)
            command_joints(bus, joints, goal)
            if soft and all(abs(action[n] - present[n]) < ARRIVAL_TICKS for n in action):
                soft = False
                print("소프트스타트 완료.")
            rp = reflex_present_view(bus, joints, present)
        except ConnectionError as e:
            comm_ok = False
            print(f"통신 오류: {e}", file=sys.stderr)

        temps = None
        if t0 - last_temp > 2.0 and comm_ok:
            last_temp = t0
            temps = bus.sync_read("Present_Temperature", ids)

        if goal:
            trips = reflex.update(t0, rp, motor_goals(joints, goal), load, temps=temps, comm_ok=comm_ok)
            for trip in trips:
                print(f"[!] {trip.event.value}: {trip.detail}", file=sys.stderr)
            for w in reflex.warnings():
                print(f"경고: {w}", file=sys.stderr)

        if reflex.mode is Mode.STOPPED:
            raise RuntimeError("STOPPED: 통신 두절/과열로 안전을 위해 중단합니다.")

        if reflex.mode is Mode.REFLEX:
            hold = reflex.hold_targets(rp)
            command_joints(bus, joints, hold_joint_goals(joints, hold))  # 논리각·듀얼 변환은 command()가
            print("홀드 목표 전송.", file=sys.stderr)
            if on_reflex(bus, joints, reflex) == "quit":
                raise RuntimeError("사용자가 리플렉스 상태에서 종료했습니다.")
            soft = True
            continue

        elapsed = time.monotonic() - t0
        jitter_max = max(jitter_max, elapsed - period) if elapsed > period else jitter_max
        if verbose and t0 - last_status >= status_every:
            last_status = t0
            print(f"[{source.name}] " + "  ".join(f"{n}:{present[n]}->{goal[n]}" for n in goal)
                  + f"  | cycle {elapsed * 1e3:.1f}ms, max over {jitter_max * 1e3:.1f}ms")
        if elapsed < period:
            time.sleep(period - elapsed)
