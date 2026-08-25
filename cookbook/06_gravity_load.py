#!/usr/bin/env python3
"""최악 자세에서 자중을 버티는 데 필요한 토크를 측정해 권장 Torque_Limit을 낸다.

Present_Load는 모터 출력 듀티(‰)라서, 정지 자세를 유지할 때의 값이 곧 중력을 버티는
토크 비율이다. 로봇 모델 없이 실측한다.

절차: 토크 OFF → 관절을 손으로 최악 자세(레버 수평, 아래 링크 완전 신전)에 놓고 Enter →
현재 위치를 목표로 잡고 토크 ON(--hold-torque 캡) → --seconds 동안 부하/전류/처짐 측정 →
토크 OFF. 자세를 바꿔 반복할 수 있고, 최댓값 x --margin을 권장값으로 낸다.

듀얼 모터 관절(J2: 10,11 / J3: 15,16)은 두 ID를 함께 지정해 동시에 홀드시킨다.

예시:
    python cookbook/06_gravity_load.py --port /dev/ttyACM0 --ids 19
    python cookbook/06_gravity_load.py --port /dev/ttyACM0 --ids 15,16 --save
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import argparse
import math
import time

import yaml

from sopo import FeetechBus, SafetyLimits, apply_safety

CURRENT_UNIT_A = 0.0065


def parse_ids(text: str) -> list[int]:
    return [int(x) for x in text.split(",") if x.strip()]


def measure(bus: FeetechBus, ids: list[int], seconds: float) -> dict[int, dict]:
    start_pos = bus.sync_read("Present_Position", ids)
    stats = {i: {"max_load": 0, "sum_load": 0, "n": 0, "max_cur": 0} for i in ids}
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        load = bus.sync_read("Present_Load", ids)
        cur = bus.sync_read("Present_Current", ids)
        for i in ids:
            s = stats[i]
            s["max_load"] = max(s["max_load"], abs(load[i]))
            s["sum_load"] += abs(load[i])
            s["n"] += 1
            s["max_cur"] = max(s["max_cur"], cur[i])
        time.sleep(0.05)
    end_pos = bus.sync_read("Present_Position", ids)
    for i in ids:
        stats[i]["mean_load"] = stats[i]["sum_load"] / max(stats[i]["n"], 1)
        stats[i]["sag"] = end_pos[i] - start_pos[i]
    return stats


def main() -> None:
    parser = argparse.ArgumentParser(description="자중 토크 측정")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--ids", required=True, help="쉼표로 구분된 모터 ID (듀얼 관절은 둘 다)")
    parser.add_argument("--hold-torque", type=int, default=600, help="측정 중 토크 캡 (0-1000)")
    parser.add_argument("--seconds", type=float, default=3.0, help="자세당 측정 시간")
    parser.add_argument("--margin", type=float, default=1.5, help="권장값 = 최대 부하 x margin")
    parser.add_argument("--floor", type=int, default=150, help="권장값 하한 (‰)")
    parser.add_argument("--calibration", default="configs/calibration.yaml")
    parser.add_argument("--save", action="store_true", help="권장값을 calibration.yaml torque_limits에 저장")
    args = parser.parse_args()

    ids = parse_ids(args.ids)
    bus = FeetechBus(args.port)
    bus.connect()
    bus.disable_torque(ids)
    apply_safety(bus, ids, SafetyLimits(torque_limit=args.hold_torque))

    overall = {i: 0 for i in ids}
    pose = 0
    try:
        while True:
            ans = input(
                f"\n[자세 {pose + 1}] ID {ids} 관절을 최악 자세에 손으로 놓고 Enter (종료: q): "
            ).strip().lower()
            if ans in ("q", "ㅂ"):  # ㅂ = 한글 IME 상태의 q
                break
            if ans:
                print(f"'{ans}'는 무시합니다. 측정하려면 빈 Enter, 종료는 q.")
                continue
            present = bus.sync_read("Present_Position", ids)
            bus.sync_write("Goal_Position", present)  # 토크 ON 시 점프 방지
            bus.enable_torque(ids)
            print(f"토크 ON (캡 {args.hold_torque}‰). 천천히 손을 떼세요... 측정 {args.seconds:.0f}초")
            time.sleep(1.0)
            stats = measure(bus, ids, args.seconds)
            bus.disable_torque(ids)
            print("토크 OFF — 관절이 내려올 수 있으니 잡아주세요.")

            print(f"{'ID':>4} | {'pos':>5} | {'max load‰':>9} | {'mean‰':>6} | {'max A':>6} | {'sag(tick)':>9}")
            for i in ids:
                s = stats[i]
                overall[i] = max(overall[i], s["max_load"])
                print(
                    f"{i:>4} | {present[i]:>5} | {s['max_load']:>9} | {s['mean_load']:>6.0f} | "
                    f"{s['max_cur'] * CURRENT_UNIT_A:>6.2f} | {s['sag']:>+9}"
                )
                if s["max_load"] >= args.hold_torque * 0.95:
                    print(f"     ID{i}: 측정 캡에 포화 — --hold-torque를 올려 다시 측정")
            pose += 1
    except KeyboardInterrupt:
        print(f"\n중단. (측정 중이던 자세 {pose + 1}은 집계에서 제외)")
    finally:
        bus.disable_torque(ids)

    if pose == 0:
        bus.disconnect()
        return

    print(f"\n=== 권장 Torque_Limit (최대 부하 x {args.margin}, 하한 {args.floor}) ===")
    rec: dict[int, int] = {}
    for i in ids:
        value = math.ceil(overall[i] * args.margin / 10) * 10
        rec[i] = min(1000, max(args.floor, value))
        print(f"ID{i}: 측정 최대 {overall[i]}‰ -> 권장 {rec[i]}‰ ({rec[i] / 10:.0f}%)")
    if len(ids) > 1:
        # 함께 측정한 모터들은 한 관절을 나눠 지는 듀얼 쌍이다. 부하 분담은 자세와
        # 각 모터의 위치 오차에 따라 뒤바뀌므로 캡은 쌍에 동일하게(최댓값) 준다.
        shared = max(rec.values())
        rec = dict.fromkeys(ids, shared)
        print(f"듀얼 쌍 공통 권장: {shared}‰ (분담이 뒤바뀔 수 있어 최댓값으로 통일)")
    print("검증: python cookbook/02_move_position.py --id <ID> --goal <목표> --torque-limit <권장값>")

    if args.save:
        path = Path(args.calibration)
        data = yaml.safe_load(path.read_text()) if path.exists() else {}
        data = data or {}
        data.setdefault("torque_limits", {})
        data["torque_limits"].update(rec)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True))
        print(f"저장: {path} (torque_limits)")

    bus.disconnect()


if __name__ == "__main__":
    main()
