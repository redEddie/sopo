#!/usr/bin/env python3
"""URDF zero pose를 관절별로 순차 맞춰 gravity.zero_ticks에 기록한다.

토크는 켜지 않는다 (OFF). 관절마다 zero 방향 안내가 뜨고, 손으로 맞춘 뒤 Enter를
누르면 그 관절의 현재 틱을 zero로 저장한다. 전부 끝나면 calibration.yaml의
gravity.zero_ticks만 갱신된다 (dir/scale은 유지).

zero 자세는 뷰어(description/viewer.py)의 초기 자세 = 모든 슬라이더 0.

예시:
    python cookbook/2_pose_calibration/230_calibrate_zero.py
    python cookbook/2_pose_calibration/230_calibrate_zero.py --joint J5   # 한 관절만 다시
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse

import yaml

from sopo import FeetechBus

# zero pose에서 각 관절이 만족해야 하는 방향 (description/README.md 규약 참조)
ZERO_GUIDES = {
    "J1": "팔이 베이스 정면(+x, 어프로치 방향)을 향하게 yaw를 맞춘다",
    "J2": "link_2가 수직으로 서게 한다 (어깨 pitch 0)",
    "J3": "link_3이 link_2와 일직선으로 위를 향하게 한다 (팔꿈치 0)",
    "J4": "손목 롤 0 — 손목 하우징의 기준면이 뷰어 자세와 같게",
    "J5": "손목 구간이 수평으로 정면(+x)을 향하게 한다 (팔 전체가 서고 손목만 앞으로)",
    "J6": "플랜지 롤 0 — 플랜지/그리퍼의 기준 방향이 뷰어와 같게",
}


def load_joints(config: str) -> dict:
    data = yaml.safe_load(Path(config).read_text())
    out = {}
    for j in data["joints"]:
        out[j["name"]] = j["reference_id"] if j["type"] == "dual" else j["motor_id"]
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description="URDF zero pose 관절별 캘리브레이션")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--config", default="configs/arm.yaml")
    parser.add_argument("--calibration", default="configs/calibration.yaml")
    parser.add_argument("--joint", help="이 관절만 캘리브레이션 (예: J5)")
    args = parser.parse_args()

    joints = load_joints(args.config)
    names = [args.joint] if args.joint else [n for n in joints if n in ZERO_GUIDES]
    if args.joint and args.joint not in joints:
        print(f"알 수 없는 관절: {args.joint} (있는 것: {sorted(joints)})")
        return

    calib_path = Path(args.calibration)
    calib = yaml.safe_load(calib_path.read_text()) if calib_path.exists() else {}
    calib = calib or {}
    grav = calib.setdefault("gravity", {})
    zero_ticks = grav.setdefault("zero_ticks", {})

    bus = FeetechBus(args.port)
    bus.connect()
    # 토크는 건드리지 않는다 — 손으로 움직이는 동안 OFF 상태여야 함
    bus.disable_torque([joints[n] for n in names])

    print("URDF zero 캘리브레이션. 토크 OFF — 손으로 맞춥니다.")
    print("기준 자세: description/viewer.py 초기 화면 (슬라이더 전부 0)")
    try:
        for n in names:
            print(f"\n[{n}] {ZERO_GUIDES[n]}")
            while True:
                ans = input("  맞췄으면 Enter (재측정: r / 건너뛰기: s / 종료: q): ").strip().lower()
                if ans in ("q", "ㅂ"):
                    break
                if ans in ("s", ""):
                    if ans == "s":
                        break
                    pos = bus.sync_read("Present_Position", [joints[n]])
                    ticks = pos[joints[n]]
                    print(f"  {n} zero_ticks: {zero_ticks.get(n)} -> {ticks}")
                    zero_ticks[n] = ticks
                    break
                print("  Enter=저장, r=재측정, s=건너뛰기, q=종료")
            if ans in ("q", "ㅂ"):
                break
    except KeyboardInterrupt:
        print("\n중단.")

    calib_path.write_text(yaml.safe_dump(calib, sort_keys=False, allow_unicode=True))
    print(f"\n저장: {calib_path} gravity.zero_ticks = {zero_ticks}")
    print("다음: 여러 자세에서 검증 — python cookbook/4_torque_model/410_gravity_check.py")
    bus.disconnect()


if __name__ == "__main__":
    main()
