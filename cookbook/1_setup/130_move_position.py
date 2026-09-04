#!/usr/bin/env python3
"""한 모터를 안전하게 목표 위치로 이동시킨다.

예시:
    python cookbook/1_setup/130_move_position.py --port /dev/ttyACM0 --id 1 --goal 2500
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import time

from sopo import FeetechBus, SafetyLimits, apply_safety, clamp_goal


def main() -> None:
    parser = argparse.ArgumentParser(description="서보 위치 이동")
    parser.add_argument("--port", default="/dev/ttyACM0", help="시리얼 포트")
    parser.add_argument("--id", type=int, required=True, help="모터 ID")
    parser.add_argument("--goal", type=int, required=True, help="목표 위치 (ticks)")
    parser.add_argument("--torque-limit", type=int, default=300, help="토크 제한 (0-1000)")
    args = parser.parse_args()

    motor_id = args.id
    limits = SafetyLimits(torque_limit=args.torque_limit)

    # 소프트 리밋 밖의 목표는 미리 잘라내고, 도달 판정도 잘린 목표 기준으로 한다.
    lo, hi = limits.joint_range(motor_id)
    target = max(lo, min(hi, args.goal))
    if target != args.goal:
        print(f"목표 {args.goal}이 소프트 리밋 [{lo}, {hi}] 밖이라 {target}으로 조정합니다.")

    bus = FeetechBus(args.port)
    bus.connect()

    apply_safety(bus, [motor_id], limits)
    bus.enable_torque([motor_id])

    start = time.monotonic()
    timeout = 10.0

    try:
        while True:
            present = bus.read("Present_Position", motor_id)
            error = target - present

            # max_relative_target만큼씩 접근하므로 목표 점프가 없다.
            clamped = clamp_goal({motor_id: target}, {motor_id: present}, limits)
            bus.write("Goal_Position", motor_id, clamped[motor_id])

            print(f"present={present:5d}  goal={clamped[motor_id]:5d}  err={error:5d}")

            if abs(error) < 20:
                print("목표 도달.")
                break
            if time.monotonic() - start > timeout:
                print("시간 초과.")
                break

            time.sleep(1.0 / 50.0)
    finally:
        try:
            bus.disable_torque([motor_id])
        finally:
            bus.disconnect()


if __name__ == "__main__":
    main()
