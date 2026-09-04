#!/usr/bin/env python3
"""외력 추정기의 오차 한계를 실측해 calibration.yaml의 estimation 섹션에 기록한다.

페이로드 안전망(docs/payload-safety-requirements.md)의 입력 두 가지를 만든다:
  --phase floor : 정지 잔차(노이즈+히스테리시스) 바닥 floor_j = mean + 3σ (최소 3자세)
  --phase k     : 알려진 추를 반복 탈부착해 복원률 분포를 재서 k = 1/최저 복원률

사전 조건: 230(zero) → 231(dir) → 410 --scale 완료. sopod는 꺼져 있을 것 (버스 독점).
재실행 주기: 월 1회 또는 하드웨어 변경 시 (scale 감도는 세션 간 변동이 있다).

예시:
    python cookbook/3_safety_torque/340_estimator_limits.py --phase floor
    python cookbook/3_safety_torque/340_estimator_limits.py --phase k --mass 0.5
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import statistics
import time

import yaml

from sopo import FeetechBus, SafetyLimits, apply_safety
from sopo.config import load_arm_config, load_gravity_cal, make_gravity_model, make_joints
from sopo.model.estimation import ExternalTorqueEstimator
from sopo.motion.joints import DualMotorJoint

MIN_POSES = 3  # floor 측정에 필요한 최소 자세 수


def build_estimator(config: str, calibration: str, **est_kw):
    """쿡북 독립 실행용 관측기 (데몬 없이 버스 직접). 반환: (est, gm, all_ids)."""
    cfg = load_arm_config(config)
    joints = make_joints(cfg)
    ref_ids = {j.name: (j.reference_id if isinstance(j, DualMotorJoint) else j.motor_ids[0])
               for j in joints}
    all_ids = sorted({i for j in joints for i in j.motor_ids})
    cal = load_gravity_cal(calibration)
    if not cal.zero_ticks:
        raise SystemExit("gravity.zero_ticks가 없습니다. cookbook/2_pose_calibration/230_calibrate_zero.py 먼저.")
    gm = make_gravity_model(config)
    return ExternalTorqueEstimator(gm, cal, ref_ids=ref_ids, **est_kw), gm, all_ids


def run_estimator(bus, est, all_ids, seconds, hz=50.0):
    """토크 ON 상태에서 관측기를 seconds 동안 돌려 (사이클별 τ_ext 목록, 평균 q)를 돌려준다."""
    outs = []
    q_sum: dict[str, float] = {}
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        pos = bus.sync_read("Present_Position", all_ids)
        load = bus.sync_read("Present_Load", all_ids)
        try:
            vel = bus.sync_read("Present_Velocity", all_ids)
        except Exception:
            vel = None
        outs.append(est.update(pos, load, vel))
        for n, r in est.ref_ids.items():
            q_sum[n] = q_sum.get(n, 0.0) + est.cal.q(n, pos[r])
        time.sleep(1.0 / hz)
    n = max(len(outs), 1)
    return outs, {k: v / n for k, v in q_sum.items()}


def save_estimation(calib_path: Path, key: str, value) -> None:
    data = yaml.safe_load(calib_path.read_text()) or {}
    data.setdefault("estimation", {})[key] = value
    calib_path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True))
    print(f"\n저장: {calib_path} estimation.{key} = {value}")


def phase_floor(bus, args, calib_path: Path) -> None:
    """여러 자세의 정지 잔차를 모아 관절별 floor를 구한다 (mean + 3σ, 잔차 절댓값 기준)."""
    # 측정용 추정기는 필터/데드밴드 OFF (α=1 → 사이클 원시 잔차 그대로)
    est, _, all_ids = build_estimator(args.config, args.calibration, ema_alpha=1.0, deadband_nm=0.0)
    apply_safety(bus, all_ids, SafetyLimits(torque_limit=args.hold_torque))
    samples: dict[str, list[float]] = {n: [] for n in est.ref_ids}
    pose = 0
    try:
        while True:
            ans = input(f"\n[자세 {pose + 1}] 팔을 움직여 자세를 잡고 Enter (최소 {MIN_POSES}자세, 종료: q): ").strip().lower()
            if ans in ("q", "ㅂ"):
                break
            present = bus.sync_read("Present_Position", all_ids)
            bus.sync_write("Goal_Position", present)
            bus.enable_torque(all_ids)
            print(f"토크 ON (캡 {args.hold_torque}‰). 손을 떼세요... 측정 {args.seconds:.0f}초")
            time.sleep(1.0)  # 정착 대기
            outs, _ = run_estimator(bus, est, all_ids, args.seconds)
            bus.disable_torque(all_ids)
            print("토크 OFF — 팔을 잡아주세요.")
            for n in est.ref_ids:
                vals = [abs(o[n]) for o in outs if n in o]
                samples[n] += vals
                print(f"  {n}: n={len(vals)} |r| mean={statistics.mean(vals):.3f} σ={statistics.pstdev(vals):.3f}")
            pose += 1
    except KeyboardInterrupt:
        print("\n중단.")
    finally:
        bus.disable_torque(all_ids)

    if pose < MIN_POSES:
        print(f"자세가 {pose}개뿐입니다 — 최소 {MIN_POSES}자세 필요. 저장하지 않습니다.")
        return
    floor = {n: round(statistics.mean(v) + 3 * statistics.pstdev(v), 4)
             for n, v in samples.items() if v}
    print("\n=== floor (|잔차| mean + 3σ, 전 자세 합산) ===")
    for n, f in floor.items():
        print(f"  {n}: {f:.3f} N·m  (n={len(samples[n])})")
    save_estimation(calib_path, "floor", floor)
    print("다음: --phase k 로 추정 감도 여유율을 측정하세요.")


def phase_k(bus, args, calib_path: Path) -> None:
    """알려진 무게 탈부착의 복원률(측정 Δτ / 모델 Δτ) 분포에서 k = 1/최저 복원률."""
    # 측정용은 deadband OFF (작은 출력을 0으로 클램프하면 Δ가 왜곡됨). tare 없음 — Δ에서 바이어스 상쇄.
    est, gm, all_ids = build_estimator(args.config, args.calibration, deadband_nm=0.0)
    apply_safety(bus, all_ids, SafetyLimits(torque_limit=args.hold_torque))
    input(f"팔을 수평 쪽으로 크게 뻗은 자세로 놓고 Enter (무게 {args.mass}kg 준비): ")
    present = bus.sync_read("Present_Position", all_ids)
    bus.sync_write("Goal_Position", present)
    bus.enable_torque(all_ids)
    print(f"토크 ON (캡 {args.hold_torque}‰) — 이 자세를 유지한 채 진행합니다")
    ratios = []
    try:
        for rep in range(args.reps):
            input(f"\n[{rep + 1}/{args.reps}] 무게를 내린 상태에서 Enter: ")
            time.sleep(1.0)
            outs0, _ = run_estimator(bus, est, all_ids, args.seconds)
            base = {n: statistics.mean(o.get(n, 0.0) for o in outs0) for n in est.ref_ids}
            input(f"[{rep + 1}/{args.reps}] 플랜지에 {args.mass}kg을 매달고 Enter (팔을 흔들지 말 것): ")
            time.sleep(1.0)
            outs1, q1 = run_estimator(bus, est, all_ids, args.seconds)
            loaded = {n: statistics.mean(o.get(n, 0.0) for o in outs1) for n in est.ref_ids}

            delta = {n: abs(loaded[n] - base[n]) for n in est.ref_ids}
            jstar = max(delta, key=delta.get)  # 신호가 가장 큰 관절 기준
            expect = abs(gm.extra_load_torque(q1, args.mass)[jstar])
            if expect < 0.05:
                print("  모델 기대 토크가 너무 작습니다 — 팔을 더 뻗은 자세로 다시 시작하세요.")
                break
            ratio = delta[jstar] / expect
            ratios.append(ratio)
            print(f"  Δτ 최대 관절 {jstar}: 측정 {delta[jstar]:.3f} / 모델 {expect:.3f} → 복원률 {ratio:.2f}")
    except KeyboardInterrupt:
        print("\n중단.")
    finally:
        bus.disable_torque(all_ids)
        print("토크 OFF — 무게를 내리고 팔을 잡아주세요.")

    if len(ratios) < 3:
        print(f"유효 반복이 {len(ratios)}회뿐입니다 — 저장하지 않습니다.")
        return
    k = 1.0 / min(ratios)
    print(f"\n복원률 분포: {[round(r, 2) for r in ratios]} (최저 {min(ratios):.2f}) → k = 1/최저 = {k:.2f}")
    save_estimation(calib_path, "k", round(k, 3))
    print("다음: 350_define_payload.py 로 최대 페이로드를 선언하세요.")


def main() -> None:
    parser = argparse.ArgumentParser(description="외력 추정기 오차 한계 실측 (floor/k → calibration.yaml estimation)")
    parser.add_argument("--phase", required=True, choices=("floor", "k"))
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--config", default="configs/arm.yaml")
    parser.add_argument("--calibration", default="configs/calibration.yaml")
    parser.add_argument("--hold-torque", type=int, default=400, help="측정 중 토크 캡 (‰)")
    parser.add_argument("--seconds", type=float, default=2.0, help="자세/단계당 측정 시간")
    parser.add_argument("--mass", type=float, default=0.5, help="phase k: 플랜지에 매다는 알려진 무게 [kg]")
    parser.add_argument("--reps", type=int, default=5, help="phase k: 탈부착 반복 횟수")
    args = parser.parse_args()

    bus = FeetechBus(args.port)
    bus.connect()
    bus.disable_torque(sorted({i for j in yaml.safe_load(Path(args.config).read_text())["joints"]
                               for i in (j["ids"] if j["type"] == "dual" else [j["motor_id"]])}))
    calib_path = Path(args.calibration)
    try:
        if args.phase == "floor":
            phase_floor(bus, args, calib_path)
        else:
            phase_k(bus, args, calib_path)
    finally:
        bus.disconnect()


if __name__ == "__main__":
    main()
