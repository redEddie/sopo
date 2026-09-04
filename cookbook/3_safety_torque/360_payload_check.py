#!/usr/bin/env python3
"""페이로드 안전망 합격시험 (sopod 클라이언트 — 버스를 직접 만지지 않는다).

Phase A: m_max 추를 매달고 유지 + 서서히 이동 → 트립 없으면 합격 (허용 페이로드).
Phase B: m_max×k 초과 추를 매달면 EXTERNAL_FORCE 트립 → 부착(Enter)부터 REFLEX까지 지연 측정.

사전 조건: sopod 실행 중 + power_ok 상태 (340 floor/k + 350 payload + tare 완료여야 함).
무게 준비: 보정용으로 쓴 것과 초과시험용(m_max×k 상당) 두 개.

예시:
    python cookbook/3_safety_torque/360_payload_check.py
    python cookbook/3_safety_torque/360_payload_check.py --mass 0.5 --over 0.9
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import time

import yaml

from sopo.config import load_estimation_limits
from sopo.runtime.client import SopoClient

MOVE_TICKS = 150  # Phase A 왕복 이동량 (≈13°)


def wait_idle_quiet(c, seconds: float) -> bool:
    """seconds 동안 감시. REFLEX로 떨어지면 False."""
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        s = c.state(0.5)
        if not s:
            continue
        if s["mode"] == "reflex":
            print(f"\n  트립: {s['trips'][-1] if s['trips'] else '?'}")
            return False
        time.sleep(0.1)
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description="페이로드 안전망 합격시험 (360)")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--config", default="configs/arm.yaml")
    parser.add_argument("--calibration", default="configs/calibration.yaml")
    parser.add_argument("--mass", type=float, help="Phase A 무게 [kg] (기본: arm.yaml payload.max_kg)")
    parser.add_argument("--over", type=float, help="Phase B 무게 [kg] (기본: max_kg × estimation.k)")
    parser.add_argument("--timeout", type=float, default=6.0, help="Phase B 트립 대기 상한 [s]")
    args = parser.parse_args()

    arm = yaml.safe_load(Path(args.config).read_text())
    m_max = args.mass or float((arm.get("payload") or {}).get("max_kg") or 0.0)
    _, k = load_estimation_limits(args.calibration)
    m_over = args.over or (round(m_max * k, 3) if k else None)
    if not m_max or not m_over:
        print("payload.max_kg / estimation.k가 없습니다. 340/350을 먼저 실행하세요.")
        return

    c = SopoClient(args.host)
    s = c.state(2.0)
    if not s:
        print("sopod 상태가 없습니다 — sopod를 먼저 띄우세요.")
        return
    if not s.get("power_ok"):
        print("power_ok가 아닙니다 — 페이로드 안전망이 서 있지 않습니다.")
        print("순서: 340(floor/k) → 350(payload 선언) → sopod에서 move 후 정착(tare 자동) 또는 tare_ext")
        return
    if s["mode"] != "move":
        r = c.command("move")
        if not r.get("ok"):
            print(f"move 거부: {r.get('error')}")
            return
        time.sleep(1.0)

    results = {}
    print(f"\n=== Phase A: 허용 페이로드 {m_max}kg — 트립 없어야 합격 ===")
    input(f"플랜지에 {m_max}kg을 매달고 Enter: ")
    time.sleep(1.0)  # 정착
    ok = wait_idle_quiet(c, 3.0)
    if ok:
        # 서서히 왕복 이동 (J2 ±MOVE_TICKS)
        pos = c.state(1.0)["joints"]["J2"]["pos"]
        for target in (pos + MOVE_TICKS, pos):
            r = c.command("goto", action={"J2": target}, unit="ticks")
            if not r.get("ok"):
                print(f"  goto 거부: {r.get('error')}")
                ok = False
                break
            ok = wait_idle_quiet(c, 4.0)
            if not ok:
                break
    results["A (허용 페이로드 무반응)"] = ok
    print(f"Phase A: {'PASS' if ok else 'FAIL'}")

    if not ok:
        print("\n허용 페이로드에서 트립 — 340의 floor가 작거나 k가 낡았을 수 있습니다. 재측정하세요.")
    else:
        print(f"\n=== Phase B: 초과 페이로드 {m_over}kg — 트립돼야 합격 ===")
        input(f"플랜지의 무게를 {m_over}kg으로 바꿔 매달고 Enter (누르는 순간부터 지연 측정): ")
        t0 = time.monotonic()
        delay = None
        while time.monotonic() - t0 < args.timeout:
            s = c.state(0.2)
            if s and s["mode"] == "reflex":
                delay = time.monotonic() - t0
                trip = s["trips"][-1] if s["trips"] else "?"
                print(f"  트립 감지: {trip}")
                break
            time.sleep(0.05)
        ok = delay is not None and "EXTERNAL_FORCE" in (trip or "")
        results["B (초과 페이로드 트립)"] = ok
        if ok:
            print(f"Phase B: PASS (부착→REFLEX 지연 {delay:.2f}s — 목표 ≤ 0.5s + 정착 시간)")
        else:
            print(f"Phase B: FAIL ({args.timeout}s 안에 EXTERNAL_FORCE 없음)")
            print("  힌트: 추가 팔을 계속 끌어내려 정착하지 못하면 외력 판정이 게이팅됩니다 — 무게/자세를 확인하세요.")

    print("\n=== 결과 ===")
    for name, ok in results.items():
        print(f"  Phase {name}: {'PASS' if ok else 'FAIL'}")
    print("후속: 무게를 내리고 팔을 잡은 뒤 `python -m sopo.runtime.cli idle` → `recover` 순서로 복구하세요.")


if __name__ == "__main__":
    main()
