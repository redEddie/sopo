#!/usr/bin/env python3
"""리플렉스 합격 판정 (docs/reflex-spec.md 8절): 한 관절을 왕복시키며 오탐을 세고, 잡았을 때 감지되는지 본다.

Phase A: 손 대지 않고 +delta/-delta 왕복 N회 → 리플렉스 0건이어야 합격.
Phase B: 왕복을 계속하며 "관절을 손으로 잡으세요" → COLLISION이 뜨면 감지 지연(포화 시작→판정)을 표시,
         홀드 후 손을 놓아도 움직이지 않는지 확인, [r] 복구 → 재개되는지 확인.

예시:
    python cookbook/14_reflex_check.py --config configs/arm.yaml --joint J4 --delta 300 --cycles 10
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import argparse
import time

from sopo import FeetechBus, apply_safety
from sopo.config import all_motor_ids, load_arm_config, make_joint_limits, make_joints, make_limits, make_pairs
from sopo.control import Blackbox, describe_trip, prompt_recover, read_joints, run_control_loop
from sopo.reflex import Reflex, ReflexConfig
from sopo.sources import WaypointSource


class CountingReflex(Reflex):
    """update()를 감싸 트립을 세고 기록한다."""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.trips = []

    def update(self, now, *a, **k):
        out = super().update(now, *a, **k)
        for t in out:
            self.trips.append((now, t))
        return out


def main() -> None:
    parser = argparse.ArgumentParser(description="reflex pass/fail check")
    parser.add_argument("--config", default="configs/arm.yaml")
    parser.add_argument("--joint", default="J4")
    parser.add_argument("--delta", type=int, default=300, help="round-trip amplitude (ticks)")
    parser.add_argument("--cycles", type=int, default=10, help="Phase A round trips")
    parser.add_argument("--torque-limit", type=int, default=None)
    args = parser.parse_args()

    cfg = load_arm_config(args.config)
    joints = make_joints(cfg)
    limits = make_limits(cfg, args.torque_limit)
    if args.torque_limit is not None:
        limits.torque_limits = {}
    joint_limits = make_joint_limits(cfg, joints)
    ids = all_motor_ids(joints)
    bus = FeetechBus(cfg["arm"]["port"], cfg["arm"].get("baudrate", 1_000_000))
    bus.connect()
    try:
        apply_safety(bus, ids, limits)
        bus.enable_torque(ids)
        start = read_joints(bus, joints)
        p0 = start[args.joint]
        wps = [{args.joint: p0 + args.delta}, {args.joint: p0}] * args.cycles

        # ---- Phase A: 오탐 검사 ----
        reflex = CountingReflex(limits, make_pairs(cfg), ReflexConfig())
        src = WaypointSource(wps, dwell_s=0.3)
        src.connect()
        print(f"Phase A: {args.joint} {p0} <-> {p0 + args.delta}, {args.cycles} round trips, hands off")
        t0 = time.monotonic()
        try:
            run_control_loop(bus, joints, limits, joint_limits, reflex, src, rate_hz=cfg.get("rate_hz", 50),
                             on_reflex=lambda b, j, r: "quit")
        except RuntimeError as e:
            print(f"Phase A aborted: {e}", file=sys.stderr)
        fp = len(reflex.trips)
        print(f"Phase A result: {fp} trips in {time.monotonic() - t0:.1f}s -> {'PASS' if fp == 0 else 'FAIL (false positives)'}")
        for t, trip in reflex.trips:
            print("   ", describe_trip(trip, joints))
        if fp:
            print("   -> raise sat_ratio / t_accel, or raise the cap: free motion must not saturate for 300 ms")
            return

        # ---- Phase B: 감지 검사 ----
        input("\nPhase B: press Enter to start; then GRAB the joint while it moves...")
        reflex = CountingReflex(limits, make_pairs(cfg), ReflexConfig())
        src = WaypointSource(wps * 5, dwell_s=0.3)
        src.connect()
        detected = {}

        def on_reflex(b, j, r):
            now = time.monotonic()
            t, trip = reflex.trips[-1]
            detected["trip"] = trip
            print("\n" + describe_trip(trip, joints))
            print("   check: joint holds and does NOT push. Release your hand, then press r to recover, q to quit")
            return prompt_recover(b, j, r)

        try:
            run_control_loop(bus, joints, limits, joint_limits, reflex, src, rate_hz=cfg.get("rate_hz", 50),
                             on_reflex=on_reflex, blackbox=Blackbox(ids, rate_hz=cfg.get("rate_hz", 50)))
        except RuntimeError as e:
            print(f"Phase B ended: {e}", file=sys.stderr)
        if "trip" in detected:
            print(f"Phase B result: detected {detected['trip'].event.name} -> PASS if it fired within ~0.5 s of grabbing")
        else:
            print("Phase B result: no trip -> FAIL (grab was not detected)")
    except KeyboardInterrupt:
        print("\nquit requested")
    finally:
        print("torque off - support the arm, it may drop")
        bus.disconnect(disable_torque_ids=ids)


if __name__ == "__main__":
    main()
