#!/usr/bin/env python3
"""흔들림 측정: 지정 관절을 크게 들었다 내리는 왕복을 반복하며, 나머지 관절의 밀림과 도착 후 진동을 수치화한다.

프리로드/PID/가속 변경 전후에 같은 명령으로 돌려 비교한다. 현재 자세(보통 standby)에서 시작한다.

지표 (모터별):
  disturb_max / disturb_rms : 지정 관절이 움직이는 동안 |goal - pos| (밀림)
  settle_pp / settle_osc / settle_t : 지정 관절 도착 후 1초 창의 위치 피크-투-피크, 부호 반전 횟수, 6틱 안 정착 시간

예시:
    python cookbook/16_wobble_test.py --config configs/arm.yaml --move J2:+300,J3:+300 --cycles 3
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import argparse
import statistics as st

from sopo import FeetechBus, apply_safety
from sopo.config import all_motor_ids, load_arm_config, make_joint_limits, make_joints, make_limits, make_pairs
from sopo.motion.control import Blackbox, read_joints, run_control_loop
from sopo.motion.joints import DualMotorJoint
from sopo.safety.reflex import Reflex, ReflexConfig
from sopo.runtime.sources import WaypointSource

SETTLE_WIN = 1.0
SETTLE_TOL = 6


def main() -> None:
    parser = argparse.ArgumentParser(description="wobble measurement")
    parser.add_argument("--config", default="configs/arm.yaml")
    parser.add_argument("--move", default="J2:+300,J3:+300", help="joint:delta,...")
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--dwell", type=float, default=1.5)
    args = parser.parse_args()

    cfg = load_arm_config(args.config)
    joints = make_joints(cfg)
    limits = make_limits(cfg)
    joint_limits = make_joint_limits(cfg, joints)
    ids = all_motor_ids(joints)
    moves = {k: int(v) for k, v in (m.split(":") for m in args.move.split(","))}
    name_of = {j.name: j for j in joints}
    ref_of = {j.name: (j.reference_id if isinstance(j, DualMotorJoint) else j.motor_ids[0]) for j in joints}
    moving_ids = {ref_of[n] for n in moves}

    bus = FeetechBus(cfg["arm"]["port"], cfg["arm"].get("baudrate", 1_000_000))
    bus.connect()
    rate = cfg.get("rate_hz", 50)
    bb = Blackbox(ids, seconds=600, rate_hz=rate)
    try:
        apply_safety(bus, ids, limits)
        bus.enable_torque(ids)
        start = read_joints(bus, joints)
        up = {n: start[n] + d for n, d in moves.items()}
        src = WaypointSource([up, {n: start[n] for n in moves}] * args.cycles, dwell_s=args.dwell)
        src.connect()
        print(f"start {start}; lifting {moves} x{args.cycles} (speed {limits.max_relative_target} ticks/cycle, accel {limits.acceleration})")
        reflex = Reflex(limits, make_pairs(cfg), ReflexConfig())
        run_control_loop(bus, joints, limits, joint_limits, reflex, src, rate_hz=rate, blackbox=bb,
                         on_reflex=lambda b, j, r: "quit")
    except RuntimeError as e:
        print(f"stopped: {e}", file=sys.stderr)
    except KeyboardInterrupt:
        print("\nquit requested")
    finally:
        bus.disconnect(disable_torque_ids=ids)

    # ---- analysis on the ring buffer ----
    rows = list(bb.rows)
    if len(rows) < 50:
        print("not enough data"); return
    cols = {i: (2 + 3 * k) for k, i in enumerate(ids)}  # pos index; goal +1, load +2
    T = [float(r[0]) for r in rows]
    def pos(i, r): return int(r[cols[i]]) if r[cols[i]] != "" else None
    def goal(i, r): return int(r[cols[i] + 1]) if r[cols[i] + 1] != "" else None

    # moving windows: any moving joint's goal changed vs previous row
    moving = [False] * len(rows)
    for k in range(1, len(rows)):
        moving[k] = any(goal(i, rows[k]) != goal(i, rows[k - 1]) for i in moving_ids if goal(i, rows[k]) is not None)
    # arrival instants: moving -> not moving transitions
    arrivals = [k for k in range(1, len(rows)) if moving[k - 1] and not moving[k]]

    print(f"\n{'motor':>6} | {'disturb_max':>11} | {'disturb_rms':>11} | {'settle_pp':>9} | {'settle_osc':>10} | {'settle_t':>8}")
    for i in ids:
        errs = [abs(goal(i, r) - pos(i, r)) for k, r in enumerate(rows) if moving[k] and pos(i, r) is not None and goal(i, r) is not None]
        dmax = max(errs) if errs else 0
        drms = (sum(e * e for e in errs) / len(errs)) ** 0.5 if errs else 0
        pps, oscs, ts = [], [], []
        for a in arrivals:
            win = [(T[k], pos(i, rows[k])) for k in range(a, len(rows)) if T[k] - T[a] <= SETTLE_WIN and pos(i, rows[k]) is not None]
            if len(win) < 5: continue
            ps = [p for _, p in win]; final = st.median(ps[-5:])
            pps.append(max(ps) - min(ps))
            dev = [p - final for p in ps]
            oscs.append(sum(1 for a1, b1 in zip(dev, dev[1:]) if a1 * b1 < 0 and abs(a1) > 3 and abs(b1) > 3))
            settled = next((t - win[0][0] for (t, p) in win if all(abs(q - final) <= SETTLE_TOL for _, q in win[win.index((t, p)):])), SETTLE_WIN)
            ts.append(settled)
        tag = " <- moving" if i in moving_ids else ""
        print(f"{i:>6} | {dmax:>11d} | {drms:>11.1f} | {st.mean(pps) if pps else 0:>9.1f} | {st.mean(oscs) if oscs else 0:>10.1f} | {st.mean(ts) if ts else 0:>7.2f}s{tag}")
    print("\nlower is better. disturb_* = how much other joints get pushed while the lifted joints move; settle_* = ringing after arrival.")


if __name__ == "__main__":
    main()
