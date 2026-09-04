#!/usr/bin/env python3
"""연속 관절(J1/J4) 서보 설정: (선택) 센터링 → 펌웨어 멀티턴 모드.

순서가 중요하다: Homing_Offset은 단일턴 모드에서만 예측대로 동작하므로 센터링을 먼저 하고
그 다음 멀티턴(Phase bit4 ON, Min/Max_Position_Limit 0/0)을 켠다.

예시:
    # 관절을 케이블 풀린 자세에 놓고 (토크 OFF)
    python cookbook/2_pose_calibration/211_continuous_setup.py --port /dev/ttyACM0 --id 19 --center
    python cookbook/2_pose_calibration/211_continuous_setup.py --port /dev/ttyACM0 --id 19 --status
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import time

from sopo import FeetechBus


def main() -> None:
    parser = argparse.ArgumentParser(description="continuous joint servo setup")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--id", type=int, required=True)
    parser.add_argument("--center", action="store_true", help="current pose -> reads 2048 (do this in the cable-relaxed pose)")
    parser.add_argument("--status", action="store_true", help="show registers only")
    args = parser.parse_args()

    bus = FeetechBus(args.port)
    bus.connect()
    i = args.id
    rd = lambda reg: bus.read(reg, i)
    try:
        bus.torque_off_verified([i])
        print(f"ID{i}: Phase={rd('Phase'):08b} (bit4 multiturn={'on' if rd('Phase') & 0x10 else 'off'}) "
              f"limits={rd('Min_Position_Limit')}/{rd('Max_Position_Limit')} offset={rd('Homing_Offset')} pos={rd('Present_Position')}")
        if args.status:
            return
        phase = rd("Phase")
        if args.center:
            # 센터링은 단일턴 모드에서
            with bus.eprom_unlocked(i):
                bus.write("Phase", i, phase & ~0x10)
                bus.write("Min_Position_Limit", i, 0)
                bus.write("Max_Position_Limit", i, 4095)
            time.sleep(0.05)
            raw, old = rd("Present_Position"), rd("Homing_Offset")
            new = max(-2047, min(2047, (old + (raw - 2048) + 2048) % 4096 - 2048))
            with bus.eprom_unlocked(i):
                bus.write("Homing_Offset", i, new)
            time.sleep(0.05)
            print(f"centered: Homing_Offset {old} -> {new}, reading {raw} -> {rd('Present_Position')} (target 2048)")
        with bus.eprom_unlocked(i):
            bus.write("Phase", i, phase | 0x10)
            bus.write("Min_Position_Limit", i, 0)
            bus.write("Max_Position_Limit", i, 0)
        time.sleep(0.05)
        print(f"multiturn ON: Phase={rd('Phase'):08b} limits={rd('Min_Position_Limit')}/{rd('Max_Position_Limit')} pos={rd('Present_Position')}")
        print("verify: python cookbook/2_pose_calibration/210_continuous_angle.py --id", i, " (rotate by hand past 360 deg: angle keeps growing, turns stay +0)")
    finally:
        bus.disconnect()


if __name__ == "__main__":
    main()
