#!/usr/bin/env python3
"""설정의 관절 정의를 이용해 한 관절을 안전하게 이동시킨다.

듀얼 모터 관절(J2/J3)의 대칭 동작과 리플렉스를 확인할 수 있다.

예시:
    python cookbook/08_move_joint.py --config configs/arm.yaml --joint J2 --goal 2000
    python cookbook/08_move_joint.py --config configs/arm.yaml --joint J3 --goal 1500 --torque-limit 200
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import argparse
import time

import yaml

from sopo import (
    ContinuousJoint,
    DualMotorJoint,
    FeetechBus,
    Mode,
    Reflex,
    ReflexConfig,
    SafetyLimits,
    SingleMotorJoint,
    apply_safety,
    clamp_goal,
)


def load_config(path: str) -> dict:
    with open(path) as f:
        cfg = yaml.safe_load(f)

    calib_path = Path(path).parent / "calibration.yaml"
    if calib_path.exists():
        calib = yaml.safe_load(calib_path.read_text()) or {}
        safety = cfg.setdefault("safety", {})
        for key in ("position_limits", "torque_limits"):
            if key in calib:
                safety.setdefault(key, {}).update(calib[key])
        print(f"캘리브레이션 적용: {calib_path}")
    return cfg


def build_joint(cfg: dict, joint_name: str):
    """설정에서 지정한 이름의 Joint 객체 하나를 만든다."""
    for jcfg in cfg["joints"]:
        if jcfg["name"] != joint_name:
            continue
        jtype = jcfg["type"]
        if jtype == "single":
            return SingleMotorJoint(joint_name, jcfg["motor_id"])
        if jtype == "dual":
            return DualMotorJoint(joint_name, tuple(jcfg["ids"]), jcfg["reference_id"], jcfg["K"])
        if jtype == "continuous":
            return ContinuousJoint(joint_name, jcfg["motor_id"], jcfg.get("range_ticks"))
        raise ValueError(f"알 수 없는 관절 타입: {jtype}")
    raise ValueError(f"관절 '{joint_name}'을 찾을 수 없습니다.")


def read_motors(bus: FeetechBus, motor_ids: list[int]) -> dict[int, int]:
    return {mid: bus.read("Present_Position", mid) for mid in motor_ids}


def read_loads(bus: FeetechBus, motor_ids: list[int]) -> dict[int, int]:
    return {mid: bus.read("Present_Load", mid) for mid in motor_ids}


def build_pairs(cfg: dict) -> dict[str, tuple[int, int, int]]:
    pairs: dict[str, tuple[int, int, int]] = {}
    for jcfg in cfg.get("joints", []):
        if jcfg.get("type") == "dual":
            ref = jcfg["reference_id"]
            ids = tuple(jcfg["ids"])
            mirror = ids[1] if ids[0] == ref else ids[0]
            pairs[jcfg["name"]] = (ref, mirror, jcfg["K"])
    return pairs


def wait_recover(reflex: Reflex, refresh) -> str:
    """Block until user recovers ('r') or quits ('q'). Returns 'continue' or 'quit'."""
    while True:
        try:
            key = input("리플렉스 발동. [r] 복구 시도, [q] 종료: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            return "quit"
        if key == "q":
            return "quit"
        if key == "r":
            present, load = refresh()  # 낡은 값이 아니라 지금 값으로 판정
            ok, reason = reflex.recover(present, load)
            if ok:
                print("복구 성공. 이동 재개.")
                return "continue"
            print(f"복구 불가: {reason}")


def main() -> None:
    parser = argparse.ArgumentParser(description="관절 단위 이동 테스트")
    parser.add_argument("--config", default="configs/arm.yaml")
    parser.add_argument("--joint", required=True, help="이동시킬 관절 이름 (예: J2)")
    parser.add_argument("--goal", type=int, required=True, help="목표 위치 (reference 모터 기준 틱)")
    parser.add_argument("--torque-limit", type=int, default=200, help="토크 제한 (0-1000)")
    parser.add_argument("--timeout", type=float, default=10.0, help="최대 대기 시간 (초)")
    args = parser.parse_args()

    cfg = load_config(args.config)
    joint = build_joint(cfg, args.joint)
    motor_ids = list(joint.motor_ids)

    bus = FeetechBus(cfg["arm"]["port"], cfg["arm"].get("baudrate", 1_000_000))
    bus.connect()

    s = cfg.get("safety", {})
    limits = SafetyLimits(
        torque_limit=args.torque_limit,
        max_relative_target=s.get("max_relative_target", 80),
        position_limits={int(k): tuple(v) for k, v in (s.get("position_limits") or {}).items()},
    )
    apply_safety(bus, motor_ids, limits)

    pairs = build_pairs(cfg)
    reflex = Reflex(limits, pairs)

    ref_id = joint.reference_id if isinstance(joint, DualMotorJoint) else joint.motor_id

    def refresh():
        """리플렉스 판정/복구용 현재 상태. 연속 관절은 논리각으로."""
        present = read_motors(bus, motor_ids)
        if isinstance(joint, ContinuousJoint):
            present = {joint.motor_id: joint.read(bus)}
        return present, read_loads(bus, motor_ids)
    bus.enable_torque(motor_ids)

    print(f"관절 {args.joint} 토크 ON, 모터 IDs: {motor_ids}")
    print(f"초기 위치: {read_motors(bus, motor_ids)}")
    print(f"목표: {args.goal}")

    start = time.monotonic()
    last_temp_check = start
    try:
        while True:
            now = time.monotonic()
            present = read_motors(bus, motor_ids)
            load = read_loads(bus, motor_ids)

            # 점프 금지: 사이클당 max_relative_target만큼만, 관절 리밋 안에서 (연속 관절은 자체 범위 클램프)
            if isinstance(joint, ContinuousJoint):
                cur = joint.read(bus)
                step = limits.max_relative_target
                stepped = max(cur - step, min(cur + step, joint.clamp(args.goal)))
                err = joint.clamp(args.goal) - cur
                # Reflex는 논리적 좌표계로 비교해야 한다.
                reflex_present = {joint.motor_id: cur}
            else:
                stepped = clamp_goal({ref_id: args.goal}, {ref_id: present[ref_id]}, limits)[ref_id]
                lo, hi = limits.joint_range(ref_id)
                err = max(lo, min(hi, args.goal)) - present[ref_id]
                reflex_present = present

            goal = {ref_id: stepped}
            if isinstance(joint, DualMotorJoint):
                mirror_goal = max(0, min(4095, joint.K - stepped))
                goal = {joint.reference_id: stepped, joint.mirror_id: mirror_goal}
            elif isinstance(joint, ContinuousJoint):
                goal = {joint.motor_id: stepped}

            trips = reflex.update(now, reflex_present, goal, load)
            for trip in trips:
                print(f"  [!] {trip.event.value}: {trip.detail}", file=sys.stderr)

            if reflex.mode is Mode.STOPPED:
                print("STOPPED: 토크를 해제하고 종료합니다.", file=sys.stderr)
                break

            if reflex.mode is Mode.REFLEX:
                hold = reflex.hold_targets(reflex_present)
                joint.command(bus, hold[ref_id])  # 연속 관절 논리각 변환·듀얼 미러는 command()가 처리
                print("  홀드 목표 전송.", file=sys.stderr)
                action = wait_recover(reflex, refresh)
                if action == "quit":
                    break
                continue

            joint.command(bus, stepped)

            # 온도 경고 (2초마다)
            if now - last_temp_check > 2.0:
                last_temp_check = now
                temps = {mid: bus.read("Present_Temperature", mid) for mid in motor_ids}
                trips = reflex.update(now, reflex_present, goal, load, temps=temps)
                for trip in trips:
                    print(f"  [!] {trip.event.value}: {trip.detail}", file=sys.stderr)
                for w in reflex.warnings():
                    print(f"  [경고] {w}", file=sys.stderr)
                if reflex.mode is Mode.STOPPED:
                    print("STOPPED: 과열로 토크 해제 후 종료합니다.", file=sys.stderr)
                    break

            print(f"  t={now - start:5.2f}s  pos={present}  err={err:5d}")

            if abs(err) < 20:
                print("목표 도달.")
                break
            if now - start > args.timeout:
                print("시간 초과.")
                break
            time.sleep(0.05)
    finally:
        bus.disable_torque(motor_ids)
        bus.disconnect()
        print("토크 OFF.")


if __name__ == "__main__":
    main()
