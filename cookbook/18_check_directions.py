#!/usr/bin/env python3
"""조인트별 모터 틱 증가 방향과 URDF +q 방향의 일치를 실기로 확인해 gravity.dir을 정한다.

관절마다: 현재 위치에서 +Δ틱 이동 → 실제로 움직인 방향을 눈으로 보고
URDF +q 방향(화면에 표시)과 일치하면 y, 반대면 n → calibration.yaml gravity.dir 갱신.
(규약: description/README.md의 REP-103 섹션 참조)

듀얼 관절(J2/J3)은 reference 모터 기준으로 조그한다.
J1/J4(연속 멀티턴)는 상대 이동이라도 안전하다.

예시:
    python cookbook/18_check_directions.py
    python cookbook/18_check_directions.py --delta 200 --torque 250
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import argparse
import time

import yaml

from sopo import FeetechBus, SafetyLimits, apply_safety

# URDF +q 방향의 물리 설명 (zero pose 근처에서 성립)
DIRECTION_HINTS = {
    "J1": "위에서 봐서 반시계(+z 오른손)",
    "J2": "팔이 전방(+x)으로 기울기",
    "J3": "팔꿈치가 전방(+x)으로",
    "J4": "어프로치(+x) 오른손 롤",
    "J5": "팁이 전방(+x)으로 (J2/J3과 같은 규약)",
    "J6": "어프로치(+x) 오른손 롤",
}


def main() -> None:
    parser = argparse.ArgumentParser(description="모터 방향 ↔ URDF +q 방향 확인")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--config", default="configs/arm.yaml")
    parser.add_argument("--calibration", default="configs/calibration.yaml")
    parser.add_argument("--delta", type=int, default=150, help="조그 크기 (틱)")
    parser.add_argument("--torque", type=int, default=200, help="토크 캡 (‰)")
    args = parser.parse_args()

    data = yaml.safe_load(Path(args.config).read_text())
    joints = {}
    for j in data["joints"]:
        if j["type"] == "dual":
            joints[j["name"]] = {"ref": j["reference_id"], "ids": j["ids"], "K": j["K"]}
        else:
            joints[j["name"]] = {"ref": j["motor_id"], "ids": [j["motor_id"]], "K": None}
    names = [n for n in joints if n in DIRECTION_HINTS]
    all_ids = sorted({i for j in joints.values() for i in j["ids"]})

    calib_path = Path(args.calibration)
    calib = yaml.safe_load(calib_path.read_text()) if calib_path.exists() else {}
    calib = calib or {}
    dirs = calib.get("gravity", {}).get("dir") or {n: 1 for n in names}

    bus = FeetechBus(args.port)
    bus.connect()
    bus.disable_torque(all_ids)
    apply_safety(bus, all_ids, SafetyLimits(torque_limit=args.torque))

    print(f"조그 +{args.delta}틱 (캡 {args.torque}‰). 각 관절에서 실제 방향이 힌트와 같은지 확인.")
    try:
        for n in names:
            j = joints[n]
            hint = DIRECTION_HINTS[n]
            while True:
                ans = input(f"\n[{n}] URDF +q = {hint}\n  조그 실행: Enter / 건너뛰기: s / 종료: q: ").strip().lower()
                if ans in ("s", "q", "ㅂ"):
                    break
                before = bus.sync_read("Present_Position", j["ids"])
                goal_ref = before[j["ref"]] + args.delta
                if j["K"] is not None:  # 듀얼: 미러는 K - ref_goal
                    mirror = [i for i in j["ids"] if i != j["ref"]][0]
                    bus.sync_write("Goal_Position", {j["ref"]: goal_ref, mirror: j["K"] - goal_ref})
                else:
                    bus.sync_write("Goal_Position", {j["ref"]: goal_ref})
                bus.enable_torque(j["ids"])
                time.sleep(0.8)
                after = bus.sync_read("Present_Position", j["ids"])
                # 원위치
                if j["K"] is not None:
                    bus.sync_write("Goal_Position", {j["ref"]: before[j["ref"]],
                                                     mirror: j["K"] - before[j["ref"]]})
                else:
                    bus.sync_write("Goal_Position", {j["ref"]: before[j["ref"]]})
                time.sleep(0.8)
                bus.disable_torque(j["ids"])
                moved = after[j["ref"]] - before[j["ref"]]
                print(f"  ref 위치 변화: {moved:+d}틱")
                ok = input("  실제 움직임이 힌트 방향이었나요? [y/n]: ").strip().lower()
                if ok in ("y", "n", "ㅛ", "ㅜ"):
                    dirs[n] = 1 if ok == "y" else -1
                    break
                print("  다시 테스트합니다.")
            if ans in ("q", "ㅂ"):
                break
    except KeyboardInterrupt:
        print("\n중단.")
    finally:
        bus.disable_torque(all_ids)

    calib.setdefault("gravity", {})["dir"] = dirs
    calib_path.write_text(yaml.safe_dump(calib, sort_keys=False, allow_unicode=True))
    print(f"\n저장: {calib_path} gravity.dir = {dirs}")
    bus.disconnect()


if __name__ == "__main__":
    main()
