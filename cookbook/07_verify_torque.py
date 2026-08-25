#!/usr/bin/env python3
"""토크 캡을 걸고 관절을 ±delta 왕복시키며 실제로 움직이는지, 부하/전류가 얼마나 쓰이는지 본다.

06_gravity_load.py가 낸 권장 Torque_Limit을 검증하는 용도. 이동 중 max load가 캡에
붙어 있으면(예: 캡 150에 load 128) 가속 여유가 없다는 뜻이니 캡을 올린다.

듀얼 모터 관절(J2: 10,11 / J3: 15,16)은 반전 장착이라 한쪽을 --invert로 지정해
서로 반대 방향으로 같은 양을 움직인다. 안 그러면 두 모터가 서로 싸운다.

예시:
    python cookbook/07_verify_torque.py --port /dev/ttyACM0 --ids 19 --torque-limit 150
    python cookbook/07_verify_torque.py --port /dev/ttyACM0 --ids 15,16 --invert 16 --torque-limit 300 --delta 100
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import argparse
import time

from sopo import FeetechBus, SafetyLimits, apply_safety, clamp_goal

CURRENT_UNIT_A = 0.0065
ARRIVE_TICKS = 15


def parse_ids(text: str) -> list[int]:
    return [int(x) for x in text.split(",") if x.strip()]


def move_and_log(bus, ids, targets, limits, timeout):
    """targets까지 클램프 스텝으로 이동하며 관절별 max load/current 기록."""
    max_load = dict.fromkeys(ids, 0)
    max_cur = dict.fromkeys(ids, 0)
    t0 = time.monotonic()
    arrived = None
    while time.monotonic() - t0 < timeout:
        present = bus.sync_read("Present_Position", ids)
        bus.sync_write("Goal_Position", clamp_goal(targets, present, limits))
        load = bus.sync_read("Present_Load", ids)
        cur = bus.sync_read("Present_Current", ids)
        for i in ids:
            max_load[i] = max(max_load[i], abs(load[i]))
            max_cur[i] = max(max_cur[i], cur[i])
        if all(abs(targets[i] - present[i]) < ARRIVE_TICKS for i in ids):
            arrived = time.monotonic() - t0
            break
        time.sleep(0.02)
    final = bus.sync_read("Present_Position", ids)
    return arrived, final, max_load, max_cur


def main() -> None:
    parser = argparse.ArgumentParser(description="토크 캡 검증 왕복 이동")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--ids", required=True, help="쉼표로 구분된 모터 ID")
    parser.add_argument("--invert", default="", help="반대 방향으로 움직일 ID (듀얼 관절의 반전 모터)")
    parser.add_argument("--torque-limit", type=int, default=150, help="검증할 Torque_Limit (‰)")
    parser.add_argument("--delta", type=int, default=200, help="왕복 이동량 (틱, 200 ≈ 17.6°)")
    parser.add_argument("--timeout", type=float, default=6.0, help="구간당 제한 시간 (초)")
    args = parser.parse_args()

    ids = parse_ids(args.ids)
    invert = set(parse_ids(args.invert))
    sign = {i: -1 if i in invert else 1 for i in ids}

    bus = FeetechBus(args.port)
    bus.connect()
    limits = SafetyLimits(torque_limit=args.torque_limit)
    apply_safety(bus, ids, limits)

    start = bus.sync_read("Present_Position", ids)
    print(f"Torque_Limit {args.torque_limit}‰ ({args.torque_limit / 10:.0f}%), 시작 위치 {start}")
    try:
        bus.sync_write("Goal_Position", start)  # 토크 ON 시 점프 방지
        bus.enable_torque(ids)
        legs = [
            (f"+{args.delta}", {i: start[i] + sign[i] * args.delta for i in ids}),
            ("복귀", dict(start)),
        ]
        for label, targets in legs:
            arrived, final, max_load, max_cur = move_and_log(bus, ids, targets, limits, args.timeout)
            status = f"{arrived:.2f}s" if arrived else "타임아웃"
            print(f"\n[{label}] {status}")
            print(f"{'ID':>4} | {'목표':>5} | {'도달':>5} | {'오차':>4} | {'max load‰':>9} | {'max A':>6} | 비고")
            for i in ids:
                err = final[i] - targets[i]
                note = ""
                if max_load[i] >= args.torque_limit * 0.85:
                    note = "캡에 근접 — 가속 여유 부족"
                print(f"{i:>4} | {targets[i]:>5} | {final[i]:>5} | {err:>+4} | "
                      f"{max_load[i]:>9} | {max_cur[i] * CURRENT_UNIT_A:>6.2f} | {note}")
            time.sleep(0.3)

        time.sleep(0.5)
        hold_load = bus.sync_read("Present_Load", ids)
        print(f"\n정지 홀드 load‰: {hold_load}")
    finally:
        bus.disable_torque(ids)
        bus.disconnect()
        print("토크 OFF.")


if __name__ == "__main__":
    main()
