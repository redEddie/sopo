#!/usr/bin/env python3
"""모터 틱 증가 방향과 URDF +q 방향의 일치를 손으로 확인해 gravity.dir을 정한다.

토크는 켜지 않는다 (OFF). 관절마다 URDF +q 방향(물리 힌트)이 뜨고, 손으로 그
방향으로 움직이면 틱 변화의 부호를 읽어 자동으로 dir을 정한다:
  틱이 늘면 dir=+1 (일치), 줄면 dir=-1 (반대).

(규약: description/README.md의 REP-103 섹션. 예전 버전처럼 모터를 조그하지 않는다 —
움직이는 방향을 눈으로 읽기 어렵기 때문.)

예시:
    python cookbook/2_pose_calibration/231_check_directions.py
    python cookbook/2_pose_calibration/231_check_directions.py --joint J2
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import time

import yaml

from sopo import FeetechBus

# URDF +q 방향의 물리 설명 (description/README.md 규약 표와 동일)
DIRECTION_HINTS = {
    "J1": "위에서 봐서 반시계 회전 (+z 오른손)",
    "J2": "팔이 전방(+x)으로 기울기",
    "J3": "팔꿈치가 전방(+x)으로",
    "J4": "어프로치(+x) 방향 오른손 롤",
    "J5": "팁이 전방(+x)으로 (J2/J3과 같은 규약)",
    "J6": "어프로치(+x) 방향 오른손 롤",
}

MIN_DELTA_TICKS = 100  # 이보다 작은 변화는 미동으로 간주 (노이즈/건드림 방지)


def main() -> None:
    parser = argparse.ArgumentParser(description="모터 방향 ↔ URDF +q 방향 확인 (손 가이드)")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--config", default="configs/arm.yaml")
    parser.add_argument("--calibration", default="configs/calibration.yaml")
    parser.add_argument("--joint", help="이 관절만 확인 (예: J2)")
    args = parser.parse_args()

    data = yaml.safe_load(Path(args.config).read_text())
    joints = {}
    for j in data["joints"]:
        joints[j["name"]] = j["reference_id"] if j["type"] == "dual" else j["motor_id"]
    names = [args.joint] if args.joint else [n for n in joints if n in DIRECTION_HINTS]
    if args.joint and args.joint not in joints:
        print(f"알 수 없는 관절: {args.joint}")
        return

    calib_path = Path(args.calibration)
    calib = yaml.safe_load(calib_path.read_text()) if calib_path.exists() else {}
    calib = calib or {}
    dirs = calib.setdefault("gravity", {}).setdefault("dir", {})

    bus = FeetechBus(args.port)
    bus.connect()
    bus.disable_torque([joints[n] for n in names])

    print("방향 확인. 토크 OFF — 손으로 움직입니다. 안내 방향으로 천천히 확실히 움직이세요.")
    try:
        for n in names:
            ref = joints[n]
            print(f"\n[{n}] +q 방향: {DIRECTION_HINTS[n]}")
            while True:
                ans = input("  준비되면 Enter (건너뛰기: s / 종료: q): ").strip().lower()
                if ans in ("s", "q", "ㅂ"):
                    break
                before = bus.sync_read("Present_Position", [ref])[ref]
                print("  지금 손으로 안내 방향으로 움직이고 Enter...")
                input()
                after = bus.sync_read("Present_Position", [ref])[ref]
                delta = after - before
                if abs(delta) < MIN_DELTA_TICKS:
                    print(f"  변화 {delta:+d}틱 — 너무 작습니다. 더 크게 움직이고 다시.")
                    continue
                dirs[n] = 1 if delta > 0 else -1
                verdict = "일치 (dir=+1)" if delta > 0 else "반대 (dir=-1)"
                print(f"  변화 {delta:+d}틱 → {verdict}")
                break
            if ans in ("q", "ㅂ"):
                break
    except KeyboardInterrupt:
        print("\n중단.")

    calib_path.write_text(yaml.safe_dump(calib, sort_keys=False, allow_unicode=True))
    print(f"\n저장: {calib_path} gravity.dir = {dirs}")
    print("확인: python cookbook/4_torque_model/410_gravity_check.py (여러 자세에서 측정/예측 비교)")
    bus.disconnect()


if __name__ == "__main__":
    main()
