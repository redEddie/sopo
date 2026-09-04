#!/usr/bin/env python3
"""서보 위치 PID(P/D/I, EPROM)를 읽거나 쓴다. 강성·진동 튜닝용.

Feetech STS 기본값 P=32 D=32 I=0. P를 올리면 캡 안에서 강성이 오르고(포화되면 효과 없음), D는 감쇠, I는 정상상태 처짐 제거.
한 관절씩 바꾸고 07/jog로 진동·소음을 확인한다. 값은 EPROM이라 전원을 꺼도 유지된다.

예시:
    python cookbook/4_torque_model/420_set_pid.py --port /dev/ttyACM0 --ids 19,20,21                    # 읽기
    python cookbook/4_torque_model/420_set_pid.py --port /dev/ttyACM0 --ids 19 --p 48 --d 48            # 쓰기
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse

from sopo import FeetechBus


def main() -> None:
    parser = argparse.ArgumentParser(description="read/write servo PID")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--ids", required=True)
    parser.add_argument("--p", type=int); parser.add_argument("--d", type=int); parser.add_argument("--i", type=int)
    args = parser.parse_args()
    ids = [int(x) for x in args.ids.split(",")]
    bus = FeetechBus(args.port); bus.connect()
    try:
        for mid in ids:
            before = tuple(bus.read(r, mid) for r in ("P_Coefficient", "D_Coefficient", "I_Coefficient"))
            if any(v is not None for v in (args.p, args.d, args.i)):
                with bus.eprom_unlocked(mid):
                    if args.p is not None: bus.write("P_Coefficient", mid, args.p)
                    if args.d is not None: bus.write("D_Coefficient", mid, args.d)
                    if args.i is not None: bus.write("I_Coefficient", mid, args.i)
                after = tuple(bus.read(r, mid) for r in ("P_Coefficient", "D_Coefficient", "I_Coefficient"))
                print(f"ID{mid}: P/D/I {before} -> {after}")
            else:
                print(f"ID{mid}: P/D/I {before}")
    finally:
        bus.disconnect()


if __name__ == "__main__":
    main()
