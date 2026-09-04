#!/usr/bin/env python3
"""최대 페이로드 m_max [kg]를 선언하고 실현성을 검사해 arm.yaml payload.max_kg에 기록한다.

자세 그리드(J2/J3 스윕, 나머지 0)에서 관절별 필요 duty를 계산한다:
    필요 duty‰ = (|G_j(q)| + |extra_load_torque_j(q, m_max)|) / (stall_j × scale_j) × 1000
(scale < 1이면 필요 duty를 크게 추정 — 보수적 방향. scale은 calibration.yaml gravity 섹션)

최악 duty가 전력 캡(safety.power_torque_limit, EPROM 상한)을 넘으면 거부(--force 없으면 저장 안 함),
90%를 넘으면 경고한다. 통과 시 arm.yaml payload.max_kg에 기록하고 포락선 미리보기를 보여준다.
이 스크립트만이 payload.max_kg를 기록한다 (docs/payload-safety-requirements.md FR-1/FR-5).

예시:
    python cookbook/3_safety_torque/350_define_payload.py 0.5
    python cookbook/3_safety_torque/350_define_payload.py 1.0 --force
"""

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import math

import yaml

from sopo.config import load_estimation_limits, load_gravity_cal, make_gravity_model

GRID_DEG = range(-90, 91, 15)  # J2/J3 스윕 범위·간격 [deg]


def main() -> None:
    parser = argparse.ArgumentParser(description="최대 페이로드 선언 + 실현성 검사 → arm.yaml payload.max_kg")
    parser.add_argument("max_kg", type=float, help="선언할 최대 페이로드 [kg]")
    parser.add_argument("--config", default="configs/arm.yaml")
    parser.add_argument("--calibration", default="configs/calibration.yaml")
    parser.add_argument("--force", action="store_true", help="실현성 검사 실패여도 강제 기록")
    args = parser.parse_args()
    if args.max_kg <= 0:
        print("max_kg는 양수여야 합니다 (0으로 끄려면 arm.yaml을 직접 편집하세요).")
        return

    gm = make_gravity_model(args.config)
    cal = load_gravity_cal(args.calibration)
    floor, k = load_estimation_limits(args.calibration)
    arm_path = Path(args.config)
    arm = yaml.safe_load(arm_path.read_text())
    cap = int((arm.get("safety") or {}).get("power_torque_limit", 600))

    # 실현성 검사: 자세 그리드에서 최악 필요 duty
    names = list(gm.joint_map)
    worst = {n: 0.0 for n in names}
    worst_pose = {n: (0, 0) for n in names}
    for a2 in GRID_DEG:
        for a3 in GRID_DEG:
            q = {n: 0.0 for n in names}
            q["J2"], q["J3"] = math.radians(a2), math.radians(a3)
            g = gm.gravity(q)
            extra = gm.extra_load_torque(q, args.max_kg)
            for n in names:
                s = cal.scale.get(n, 1.0)
                duty = (abs(g[n]) + abs(extra[n])) / (gm.joint_map[n]["stall"] * s) * 1000
                if duty > worst[n]:
                    worst[n] = duty
                    worst_pose[n] = (a2, a3)

    print(f"필요 duty 계산: {args.max_kg}kg, J2/J3 {GRID_DEG.start}~{GRID_DEG.stop - 1}° 그리드, 전력 캡 {cap}‰")
    print(f"\n{'관절':>4} | {'최악 duty‰':>10} | {'자세(J2,J3)':>12} | {'캡 대비':>6}")
    for n in names:
        p = worst_pose[n]
        print(f"{n:>4} | {worst[n]:10.0f} | {str(p):>12} | {worst[n] / cap * 100:5.0f}%")
    worst_all = max(worst.values())

    if worst_all > cap:
        print(f"\n거부: 최악 필요 duty {worst_all:.0f}‰가 전력 캡 {cap}‰ 초과 — 이 페이로드는 이 모터로 못 듭니다.")
        print("더 작은 값으로 다시 시도하세요. (--force로 강제 기록 가능, 비추천)")
        if not args.force:
            return
        print("--force: 강제로 기록합니다.")
    elif worst_all > 0.9 * cap:
        print(f"\n경고: 최악 필요 duty가 캡의 90%를 넘습니다 ({worst_all / cap * 100:.0f}%) — 여유가 거의 없습니다.")
    else:
        print(f"\n실현 가능: 최악 필요 duty {worst_all:.0f}‰ (캡의 {worst_all / cap * 100:.0f}%)")

    # 포락선 미리보기 (floor/k가 있어야 의미 있음)
    if k is None or not floor:
        print("\n참고: calibration.yaml에 estimation floor/k가 없습니다 — 340_estimator_limits.py를 먼저 실행하세요.")
        print("      (없으면 데몬이 전력 모드로 올라가지 않고 순한 캡을 유지합니다)")
    else:
        zero = {n: 0.0 for n in names}
        print(f"\n포락선 미리보기 (k={k}): thr_j = |extra_j(q)|×k + floor_j [N·m]")
        for label, q in (("zero 자세", zero),
                         (f"최악 자세 J2,J3={worst_pose[max(worst, key=worst.get)]}",
                          {**zero, "J2": math.radians(worst_pose[max(worst, key=worst.get)][0]),
                           "J3": math.radians(worst_pose[max(worst, key=worst.get)][1])})):
            env = gm.payload_envelope(q, args.max_kg, k, floor)
            print(f"  {label}: " + "  ".join(f"{n}:{env[n]:.2f}" for n in names))

    # arm.yaml 갱신 — 주석 보존을 위해 정규식으로 한 줄만 교체 (220_capture_pose와 같은 방식)
    text = arm_path.read_text()
    pat = re.compile(r"(payload:\s*\n\s*max_kg:\s*)[0-9.]+")
    if pat.search(text):
        text = pat.sub(rf"\g<1>{args.max_kg}", text, count=1)
    else:
        text = text.rstrip("\n") + f"\n\npayload:\n  max_kg: {args.max_kg}\n"
    arm_path.write_text(text)
    print(f"\n저장: {arm_path} payload.max_kg = {args.max_kg}")
    print("다음: 360_payload_check.py로 합격시험을 진행하세요.")


if __name__ == "__main__":
    main()
