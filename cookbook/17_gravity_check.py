#!/usr/bin/env python3
"""모델 기반 중력 토크 G(q)와 실제 모터 부하(Present_Load)를 비교 검증한다.

URDF(실측 질량 주입)에서 계산한 중력 토크가 실제 하드웨어와 맞는지 자세별로 확인하고,
외력 토크 추정(측정 토크 - 중력 토크)도 보여준다.

사전 준비:
  1. description/link_masses.yaml 실측 완료 + postprocess.py 실행
  2. --calibrate-vertical 로 zero 기준 캡처 (최초 1회):
     팔을 URDF zero 자세(직립 + 손목 수평 전방, 뷰어의 초기 자세)에 손으로 맞추고 실행.
     calibration.yaml의 gravity.zero_ticks/dir에 저장된다.

흐름: 토크 OFF → 자세를 손으로 만들고 Enter → 토크 ON(--hold-torque 캡) → 측정 →
토크 OFF → 자세별 측정/예측 비교표. 반복 후 q로 종료하면 자세별 오차 요약을 보여준다.

예시:
    python cookbook/17_gravity_check.py --calibrate-vertical
    python cookbook/17_gravity_check.py
    python cookbook/17_gravity_check.py --hold-torque 400 --seconds 2
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import argparse
import time

import yaml

from sopo import FeetechBus, SafetyLimits, apply_safety
from sopo.config import load_arm_config, load_gravity_cal, make_gravity_model, make_joints
from sopo.motion.joints import DualMotorJoint


def save_zero_ticks(path: Path, zero_ticks: dict, dirs: dict) -> None:
    data = yaml.safe_load(path.read_text()) if path.exists() else {}
    data = data or {}
    data["gravity"] = {"zero_ticks": zero_ticks, "dir": dirs}
    path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True))
    print(f"저장: {path} gravity.zero_ticks = {zero_ticks}")


def sample(bus, ids, seconds):
    """토크 ON 상태에서 부하/위치를 평균낸다."""
    n = 0
    sum_load = {i: 0 for i in ids}
    sum_pos = {i: 0 for i in ids}
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        load = bus.sync_read("Present_Load", ids)
        pos = bus.sync_read("Present_Position", ids)
        for i in ids:
            sum_load[i] += load[i]
            sum_pos[i] += pos[i]
        n += 1
        time.sleep(0.05)
    n = max(n, 1)
    return ({i: sum_load[i] / n for i in ids},
            {i: round(sum_pos[i] / n) for i in ids})


def run_scale_calibration(bus, gm, ids_of, names, all_ids, read_q, args, calib_path, cal):
    """알려진 무게를 플랜지에 매달아 ‰↔토크 스케일을 보정해 calibration.yaml에 저장.

    measured‰ ≈ scale × (G/stall×1000) 의 scale을 관절별로 구한다.
    팔을 뻗은 자세에서만 의미가 있다 (토크가 커야 신호가 좋음).
    """
    input(f"팔을 수평 쪽으로 크게 뻗은 자세로 놓고 Enter (무게 {args.scale}kg 준비): ")
    present = bus.sync_read("Present_Position", all_ids)
    bus.sync_write("Goal_Position", present)
    bus.enable_torque(all_ids)
    print(f"토크 ON (캡 {args.hold_torque}‰) — 자세 유지 중")
    time.sleep(1.0)
    load0, _ = sample(bus, all_ids, args.seconds)
    print("베이스라인 측정 완료.")

    input(f"플랜지에 {args.scale}kg을 매달고 Enter (토크 유지 중, 팔을 흔들지 말 것): ")
    time.sleep(1.0)
    load1, pos1 = sample(bus, all_ids, args.seconds)
    bus.disable_torque(all_ids)
    print("토크 OFF — 무게를 내리고 팔을 잡아주세요.")

    q1 = read_q(pos1)
    delta_nm = gm.extra_load_torque(q1, args.scale)

    print(f"\n{'관절':>4} | {'모터별 Δ측정‰':>14} | {'예측 Δ‰':>8} | {'scale':>6}")
    new_scale = {}
    for n in names:
        loads = [load1[i] - load0[i] for i in ids_of[n]]
        # 미러 부호 반전을 흡수: 관절 측정 Δ는 모터별 |Δ|의 평균
        measured = sum(abs(d) for d in loads) / len(loads)
        pred_pm = delta_nm[n] / gm.joint_map[n]["stall"] * 1000
        loads_str = "/".join(f"{d:+.0f}" for d in loads)
        if abs(pred_pm) < 50:  # 신호가 약한 관절은 보정 불가 → 1.0 유지
            print(f"{n:>4} | {loads_str:>14} | {pred_pm:+8.0f} |  (신호 약함, 1.0 유지)")
            continue
        new_scale[n] = measured / abs(pred_pm)
        print(f"{n:>4} | {loads_str:>14} | {pred_pm:+8.0f} | {new_scale[n]:6.3f}")

    if not new_scale:
        print("보정 가능한 관절이 없습니다. 팔을 더 뻗은 자세에서 다시 시도하세요.")
        return
    cal.scale = {**cal.scale, **new_scale}
    data = yaml.safe_load(calib_path.read_text())
    data.setdefault("gravity", {})["scale"] = cal.scale
    calib_path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True))
    print(f"\n저장: {calib_path} gravity.scale = {cal.scale}")
    print("이후 인자 없는 실행에서는 예측/외력에 이 스케일이 자동 적용됩니다.")


def main() -> None:
    parser = argparse.ArgumentParser(description="중력 토크 모델 검증 + 외력 추정")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--config", default="configs/arm.yaml")
    parser.add_argument("--calibration", default="configs/calibration.yaml")
    parser.add_argument("--hold-torque", type=int, default=400, help="측정 중 토크 캡 (‰)")
    parser.add_argument("--seconds", type=float, default=2.0, help="자세당 측정 시간")
    parser.add_argument("--calibrate-vertical", action="store_true",
                        help="현재 자세를 URDF zero로 저장하고 종료")
    parser.add_argument("--scale", type=float, metavar="KG",
                        help="스케일 보정: 플랜지에 매다는 무게[kg]. 팔을 뻗은 자세에서 시작")
    args = parser.parse_args()

    cfg = load_arm_config(args.config)
    joint_objs = make_joints(cfg)
    names = [j.name for j in joint_objs]      # J1..J6 (arm.yaml 순서)
    ref_of = {j.name: (j.reference_id if isinstance(j, DualMotorJoint) else j.motor_ids[0])
              for j in joint_objs}
    ids_of = {j.name: list(j.motor_ids) for j in joint_objs}
    ref_ids = [ref_of[n] for n in names]
    all_ids = sorted({i for ids in ids_of.values() for i in ids})
    calib_path = Path(args.calibration)

    bus = FeetechBus(args.port)
    bus.connect()
    bus.disable_torque(all_ids)

    if args.calibrate_vertical:
        pos = bus.sync_read("Present_Position", ref_ids)
        zero = {n: pos[ref_of[n]] for n in names}
        dirs = load_gravity_cal(calib_path).dir or {n: 1 for n in names}
        save_zero_ticks(calib_path, zero, dirs)
        print("URDF zero 기준 저장 완료. 이후 자세 비교는 인자 없이 실행하세요.")
        bus.disconnect()
        return

    cal = load_gravity_cal(calib_path)
    if not cal.zero_ticks:
        print("zero 기준이 없습니다. 먼저 --calibrate-vertical 실행 (팔을 직립 자세로).")
        bus.disconnect()
        return

    gm = make_gravity_model(args.config)

    def read_q(pos):
        return {n: cal.q(n, pos[ref_of[n]]) for n in names}

    apply_safety(bus, all_ids, SafetyLimits(torque_limit=args.hold_torque))

    if args.scale:
        run_scale_calibration(bus, gm, ids_of, names, all_ids, read_q,
                              args, calib_path, cal)
        bus.disconnect()
        return

    pose = 0
    history = []  # (측정‰ 모터별, 예측‰ 관절별)
    try:
        while True:
            ans = input(f"\n[자세 {pose + 1}] 팔을 움직여 자세를 잡고 Enter (종료: q): ").strip().lower()
            if ans in ("q", "ㅂ"):
                break
            present = bus.sync_read("Present_Position", all_ids)
            bus.sync_write("Goal_Position", present)
            bus.enable_torque(all_ids)
            print(f"토크 ON (캡 {args.hold_torque}‰). 손을 떼세요... 측정 {args.seconds:.0f}초")
            time.sleep(1.0)
            load, pos = sample(bus, all_ids, args.seconds)
            bus.disable_torque(all_ids)
            print("토크 OFF — 팔을 잡아주세요.")

            q = read_q(pos)
            pred_nm = gm.gravity(q)
            pred_pm = gm.load_permille(q, cal.scale)
            ext_nm = gm.external_torque_from_loads(q, load, cal.scale)

            print(f"\n{'관절':>4} | {'측정‰':>14} | {'예측‰':>8} | {'예측N·m':>8} | {'외력N·m':>8}")
            for n in names:
                loads = "/".join(f"{load[i]:+.0f}" for i in ids_of[n])
                print(f"{n:>4} | {loads:>14} | {pred_pm[n]:+8.0f} | {pred_nm[n]:+8.3f} | {ext_nm[n]:+8.3f}")
            history.append((dict(load), pred_pm))
            pose += 1
    except KeyboardInterrupt:
        print("\n중단.")
    finally:
        bus.disable_torque(all_ids)

    if len(history) >= 2:
        print("\n=== 자세별 오차 요약 (측정 - 예측) ===")
        for n in names:
            ids = ids_of[n]
            if len(ids) == 1:
                errs = [m[ids[0]] - p[n] for m, p in history]
                mean_err = sum(errs) / len(errs)
                same_dir = sum(1 for m, p in history
                               if (m[ids[0]] - history[0][0][ids[0]]) * (p[n] - history[0][1][n]) > 0)
                hint = "" if same_dir >= len(history) / 2 else "  ← dir 반전 의심 (calibration gravity.dir)"
                print(f"{n:>4}: 평균 오차 {mean_err:+.0f}‰ (자세 {len(errs)}개){hint}")
            else:
                # 듀얼: arm.yaml의 mount_sign으로 부호를 맞춘 모터별 평균 오차
                signs = gm.joint_map[n].get("mount_sign") or [1] * len(ids)
                per_motor = [sum(s * m[i] - p[n] for m, p in history) / len(history)
                             for s, i in zip(signs, ids)]
                detail = ", ".join(f"ID{i}({'+' if s > 0 else '-'}장착) {e:+.0f}‰"
                                   for s, i, e in zip(signs, ids, per_motor))
                print(f"{n:>4}: 모터별 평균 오차 {detail} (자세 {len(history)}개)")

    bus.disconnect()


if __name__ == "__main__":
    main()
