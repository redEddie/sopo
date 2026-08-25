#!/usr/bin/env python3
"""랩(4095→0)을 넘어가는 관절의 연속 각도를 시작 자세 기준 도(°)로 보여준다 (토크 OFF).

J1처럼 한 바퀴 넘게 도는 관절의 허용 범위를 정할 때 쓴다. 전선이 풀린 자세에서 시작해
손으로 한쪽으로 돌리다가 전선이 당기기 시작하는 지점의 각도를 읽고, 반대쪽도 읽는다.
그 두 값(또는 더 보수적인 값)이 yaml의 range_ticks가 된다.

모터는 한 바퀴를 0~4095로만 보고하므로, 이 스크립트가 값이 튀는 순간(4095→0, 0→4095)을
감지해 바퀴 수를 더해 연속 각도를 만든다. 사용자가 셀 필요 없다.

예시:
    python cookbook/09_continuous_angle.py --port /dev/ttyACM0 --id 1
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import argparse
import time

from sopo import FeetechBus

TICKS_PER_REV = 4096
DEG_PER_TICK = 360.0 / TICKS_PER_REV


def main() -> None:
    parser = argparse.ArgumentParser(description="연속 각도 표시 (랩 자동 처리)")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--id", type=int, default=1)
    args = parser.parse_args()

    bus = FeetechBus(args.port)
    bus.connect()
    bus.disable_torque([args.id])

    start_raw = last_raw = bus.read("Present_Position", args.id)
    turns = 0
    lo = hi = 0.0
    print(f"토크 OFF. 지금 자세를 0°로 잡았습니다 (raw {start_raw}). 손으로 돌려보세요. Ctrl+C로 종료.\n")
    try:
        while True:
            raw = bus.read("Present_Position", args.id)
            delta = raw - last_raw
            if delta > TICKS_PER_REV // 2:      # 0 → 4095 방향으로 튐 = 한 바퀴 뒤로
                turns -= 1
            elif delta < -(TICKS_PER_REV // 2):  # 4095 → 0 방향으로 튐 = 한 바퀴 앞으로
                turns += 1
            last_raw = raw
            logical = raw + turns * TICKS_PER_REV - start_raw
            deg = logical * DEG_PER_TICK
            lo, hi = min(lo, deg), max(hi, deg)
            print(f"\r각도 {deg:+8.1f}°  (누적 최소 {lo:+7.1f}° / 최대 {hi:+7.1f}°, {logical:+6d} ticks, {turns:+d}바퀴)   ",
                  end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        bus.disconnect()
    print(f"\n\n결과: 시작 자세 기준 {lo:+.1f}° ~ {hi:+.1f}°")
    print(f"yaml 예시 (보수적으로 안쪽 값 사용): range_ticks: {int(min(abs(lo), abs(hi)) / DEG_PER_TICK)}")


if __name__ == "__main__":
    main()
