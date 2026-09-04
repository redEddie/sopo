#!/usr/bin/env python3
"""관절을 손으로 움직여 최소/최대 위치를 기록한다 (토크 OFF).

기록값은 안쪽으로 --margin만큼 줄여 configs/calibration.yaml의 position_limits에 저장한다.
--write-eprom을 주면 모터의 Min/Max_Position_Limit(EPROM)에도 써서 펌웨어가 범위 밖
목표를 거부하게 만든다 — 호스트 소프트웨어 버그에 대한 2차 방어선.

예시:
    python cookbook/2_pose_calibration/200_find_limits.py --port /dev/ttyACM0 --ids 19
    python cookbook/2_pose_calibration/200_find_limits.py --port /dev/ttyACM0 --ids 15,16 --write-eprom
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import time

import yaml

from sopo import FeetechBus

RESOLUTION = 4096
WRAP_GUARD = 50  # 이 안쪽이면 0/4095 랩 지점에 너무 가까움


def parse_ids(text: str) -> list[int]:
    return [int(x) for x in text.split(",") if x.strip()]


def load_yaml(path: Path) -> dict:
    if path.exists():
        return yaml.safe_load(path.read_text()) or {}
    return {}


def save_yaml(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True))


def main() -> None:
    parser = argparse.ArgumentParser(description="관절 위치 한계 기록 (손으로 이동)")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--ids", required=True, help="쉼표로 구분된 모터 ID (듀얼 관절은 둘 다)")
    parser.add_argument("--margin", type=int, default=50, help="기계 한계에서 안쪽으로 둘 여유 (틱)")
    parser.add_argument("--calibration", default="configs/calibration.yaml")
    parser.add_argument("--write-eprom", action="store_true", help="모터 Min/Max_Position_Limit에도 기록")
    args = parser.parse_args()

    ids = parse_ids(args.ids)
    bus = FeetechBus(args.port)
    bus.connect()
    bus.disable_torque(ids)

    print(f"토크 OFF. ID {ids} 관절을 손으로 양쪽 끝까지 천천히 움직이세요.")
    print("끝나면 Ctrl+C. (표시: 현재 [min-max])\n")

    mins: dict[int, int] = {}
    maxs: dict[int, int] = {}
    try:
        while True:
            pos = bus.sync_read("Present_Position", ids)
            for i in ids:
                mins[i] = min(mins.get(i, pos[i]), pos[i])
                maxs[i] = max(maxs.get(i, pos[i]), pos[i])
            line = "   ".join(f"ID{i}: {pos[i]:4d} [{mins[i]:4d}-{maxs[i]:4d}]" for i in ids)
            print("\r" + line, end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n")
    finally:
        bus.disable_torque(ids)

    limits: dict[int, list[int]] = {}
    print(f"{'ID':>4} | {'raw min':>7} | {'raw max':>7} | {'limit min':>9} | {'limit max':>9} | 비고")
    for i in ids:
        lo, hi = mins[i] + args.margin, maxs[i] - args.margin
        note = ""
        if mins[i] < WRAP_GUARD or maxs[i] > RESOLUTION - 1 - WRAP_GUARD:
            note = "랩(0/4095) 근접 — 조립 기준각 재검토"
        if hi - lo < 100:
            note = "범위가 100틱 미만 — 충분히 움직였는지 확인"
        print(f"{i:>4} | {mins[i]:>7} | {maxs[i]:>7} | {lo:>9} | {hi:>9} | {note}")
        limits[i] = [lo, hi]

    if any(v[1] - v[0] < 100 for v in limits.values()):
        raise SystemExit("범위가 너무 좁은 관절이 있어 저장하지 않습니다.")

    calib_path = Path(args.calibration)
    data = load_yaml(calib_path)
    data.setdefault("position_limits", {})
    for i, v in limits.items():
        data["position_limits"][i] = v
    save_yaml(calib_path, data)
    print(f"\n저장: {calib_path} (position_limits)")

    if args.write_eprom:
        for i, (lo, hi) in limits.items():
            with bus.eprom_unlocked(i):
                bus.write("Min_Position_Limit", i, lo)
                bus.write("Max_Position_Limit", i, hi)
            got = (bus.read("Min_Position_Limit", i), bus.read("Max_Position_Limit", i))
            status = "OK" if got == (lo, hi) else f"불일치 {got}"
            print(f"EPROM ID{i}: Min/Max_Position_Limit = {lo}/{hi} ... {status}")

    bus.disconnect()


if __name__ == "__main__":
    main()
