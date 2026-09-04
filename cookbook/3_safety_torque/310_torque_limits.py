#!/usr/bin/env python3
"""토크 한계를 낮추고 손으로 밀었을 때 전류/부하가 포화되는지 보여준다.

예시:
    python cookbook/3_safety_torque/310_torque_limits.py --port /dev/ttyACM0 --id 1
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import time

from sopo import FeetechBus, SafetyLimits, apply_safety, persist_torque_limit, read_effort


def main() -> None:
    parser = argparse.ArgumentParser(description="토크 제한 데모")
    parser.add_argument("--port", default="/dev/ttyACM0", help="시리얼 포트")
    parser.add_argument("--id", type=int, required=True, help="모터 ID")
    parser.add_argument("--torque-limit", type=int, default=200, help="토크 제한 (0-1000)")
    parser.add_argument("--persist", action="store_true", help="EPROM에 Max_Torque_Limit로 영구 저장")
    args = parser.parse_args()

    motor_id = args.id
    limits = SafetyLimits(torque_limit=args.torque_limit)

    bus = FeetechBus(args.port)
    bus.connect()

    # 예외가 나더라도 토크는 반드시 끄고 종료한다.
    try:
        apply_safety(bus, [motor_id], limits)
        bus.enable_torque([motor_id])

        # 현재 위치를 잡고 유지한다.
        hold = bus.read("Present_Position", motor_id)
        bus.write("Goal_Position", motor_id, hold)

        print(f"ID {motor_id} 토크 제한 {args.torque_limit}/1000. 관절을 손으로 밀어보세요.")
        print(f"{'Effort':>8} | {'Current(A)':>11} | {'Load‰':>7}")
        print("-" * 32)

        # 과부하 보호: Overload_Torque 이상의 부하가 Protection_Time(x10ms) 동안
        # 지속되면 모터가 Torque_Limit를 Protective_Torque로 낮춰 자기 보호에 들어간다.
        try:
            while True:
                effort = read_effort(bus, [motor_id])
                load = bus.read("Present_Load", motor_id)
                current_raw = bus.read("Present_Current", motor_id)
                current_a = current_raw * 0.0065
                print(f"{effort.get(motor_id, 0.0):>8.3f} | {current_a:>11.4f} | {load:>7}")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n사용자 중단.")

        if args.persist:
            answer = input(
                "\nEPROM에 Max_Torque_Limit를 영구 저장합니다. 계속하려면 'yes'를 입력하세요: "
            )
            if answer.strip().lower() == "yes":
                persist_torque_limit(bus, [motor_id], args.torque_limit)
                print("EPROM에 저장 완료.")
            else:
                print("저장 취소.")
    finally:
        try:
            bus.disable_torque([motor_id])
        finally:
            bus.disconnect()


if __name__ == "__main__":
    main()
