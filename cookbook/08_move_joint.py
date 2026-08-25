#!/usr/bin/env python3
"""설정의 관절 정의를 이용해 한 관절을 안전하게 이동시킨다.

리더 암 없이도 듀얼 모터 관절(J2/J3)의 대칭 동작을 확인할 수 있다.

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

    bus = FeetechBus(cfg["leader"]["port"], cfg["leader"].get("baudrate", 1_000_000))
    bus.connect()

    s = cfg.get("safety", {})
    limits = SafetyLimits(
        torque_limit=args.torque_limit,
        max_relative_target=s.get("max_relative_target", 80),
        position_limits={int(k): tuple(v) for k, v in (s.get("position_limits") or {}).items()},
    )
    apply_safety(bus, motor_ids, limits)
    ref_id = joint.reference_id if isinstance(joint, DualMotorJoint) else joint.motor_id
    bus.enable_torque(motor_ids)

    print(f"관절 {args.joint} 토크 ON, 모터 IDs: {motor_ids}")
    print(f"초기 위치: {read_motors(bus, motor_ids)}")
    print(f"목표: {args.goal}")

    start = time.monotonic()
    try:
        while True:
            present = read_motors(bus, motor_ids)
            # 점프 금지: 사이클당 max_relative_target만큼만, 관절 리밋 안에서 (연속 관절은 자체 범위 클램프)
            if isinstance(joint, ContinuousJoint):
                cur = joint.read(bus)
                step = limits.max_relative_target
                stepped = max(cur - step, min(cur + step, joint.clamp(args.goal)))
                err = joint.clamp(args.goal) - cur
            else:
                stepped = clamp_goal({ref_id: args.goal}, {ref_id: present[ref_id]}, limits)[ref_id]
                lo, hi = limits.joint_range(ref_id)
                err = max(lo, min(hi, args.goal)) - present[ref_id]
            joint.command(bus, stepped)
            print(f"  t={time.monotonic() - start:5.2f}s  pos={present}  err={err:5d}")

            if abs(err) < 20:
                print("목표 도달.")
                break
            if time.monotonic() - start > args.timeout:
                print("시간 초과.")
                break
            time.sleep(0.05)
    finally:
        bus.disable_torque(motor_ids)
        bus.disconnect()
        print("토크 OFF.")


if __name__ == "__main__":
    main()
