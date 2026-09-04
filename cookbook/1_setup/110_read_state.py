#!/usr/bin/env python3
"""토크를 켜지 않고 지정된 모터의 상태를 10Hz로 읽어 출력한다.

예시:
    python cookbook/1_setup/110_read_state.py --port /dev/ttyACM0 --ids 1,2,3
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import time

from sopo import FeetechBus


def parse_ids(text: str) -> list[int]:
    return [int(x.strip()) for x in text.split(",") if x.strip()]


def main() -> None:
    parser = argparse.ArgumentParser(description="서보 상태 읽기 (토크 OFF)")
    parser.add_argument("--port", default="/dev/ttyACM0", help="시리얼 포트")
    parser.add_argument("--ids", default="1,2,3", help="쉼표로 구분된 모터 ID")
    args = parser.parse_args()

    motor_ids = parse_ids(args.ids)
    bus = FeetechBus(args.port)
    bus.connect()

    print(f"{'ID':>4} | {'Pos':>6} | {'Vel':>6} | {'Load%':>6} | {'Volt':>5} | {'Temp':>4} | {'Status':>6}")
    print("-" * 55)

    try:
        while True:
            pos = bus.sync_read("Present_Position", motor_ids)
            vel = bus.sync_read("Present_Velocity", motor_ids)
            load = bus.sync_read("Present_Load", motor_ids)
            volt = bus.sync_read("Present_Voltage", motor_ids)
            temp = bus.sync_read("Present_Temperature", motor_ids)
            status = bus.sync_read("Status", motor_ids)

            for mid in motor_ids:
                load_pct = load.get(mid, 0) / 10.0
                voltage = volt.get(mid, 0) / 10.0
                print(
                    f"{mid:>4} | "
                    f"{pos.get(mid, -1):>6} | "
                    f"{vel.get(mid, -1):>6} | "
                    f"{load_pct:>6.1f} | "
                    f"{voltage:>5.1f} | "
                    f"{temp.get(mid, -1):>4} | "
                    f"{status.get(mid, -1):>6}"
                )
            print()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n사용자 중단.")
    finally:
        bus.disconnect()


if __name__ == "__main__":
    main()
