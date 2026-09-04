#!/usr/bin/env python3
"""모든(또는 지정) 모터의 토크를 확실히 끈다 — 재시도 + 읽기 검증.

예시:
    python cookbook/1_setup/150_torque_off.py --port /dev/ttyACM0            # ID 0~30 스캔 후 전부 OFF
    python cookbook/1_setup/150_torque_off.py --port /dev/ttyACM0 --ids 10,11
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse

from sopo import FeetechBus


def main() -> None:
    parser = argparse.ArgumentParser(description="torque off (verified)")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--ids", default=None, help="comma-separated ids (default: scan 0-30)")
    args = parser.parse_args()

    bus = FeetechBus(args.port)
    bus.connect()
    try:
        ids = [int(x) for x in args.ids.split(",")] if args.ids else sorted(bus.scan(range(0, 31)))
        print("SUPPORT THE ARM - it will drop. motors:", ids)
        still_on = bus.torque_off_verified(ids)
        state = {i: bus.read("Torque_Enable", i) for i in ids}
        print("Torque_Enable:", state)
        print("torque off OK" if not still_on else f"!!! still on: {still_on}")
    finally:
        bus.port.closePort()


if __name__ == "__main__":
    main()
