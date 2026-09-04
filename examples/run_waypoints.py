#!/usr/bin/env python3
"""다관절 웨이포인트 주행 — 안전 루프(클램프·리플렉스·워치독)의 첫 클라이언트.

명령 소스는 ActionSource 경계(sopo/sources.py)로 갈아끼운다:
  - 지금: WaypointSource (configs/waypoints.example.yaml)
  - 나중: LeaderArmSource(리더 암, lerobot Teleoperator 구조) / PolicySource(정책) — sources.py 플레이스홀더 참조

사용 예:
    python examples/run_waypoints.py --config configs/arm.yaml --waypoints configs/waypoints.example.yaml
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sopo import FeetechBus, apply_safety
from sopo.config import all_motor_ids, load_arm_config, make_joint_limits, make_joints, make_limits, make_pairs
from sopo.motion.control import end_session, Blackbox, run_control_loop
from sopo.safety.reflex import Reflex, ReflexConfig
from sopo.safety.limits import verify_eprom
from sopo.runtime.sources import WaypointSource


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default="configs/arm.yaml")
    parser.add_argument("--waypoints", default="configs/waypoints.example.yaml")
    parser.add_argument("--torque-limit", type=int, default=None, help="전 관절 공통 캡 오버라이드 (기본: 설정/캘리브레이션 값)")
    parser.add_argument("--rate", type=float, default=None, help="루프 주파수 Hz (기본: 설정 rate_hz 또는 50)")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--release", action="store_true", help="drop torque at exit (default: hold, Cat 2 stop)")
    args = parser.parse_args()

    cfg = load_arm_config(args.config)
    joints = make_joints(cfg)
    limits = make_limits(cfg, args.torque_limit)
    if args.torque_limit is not None:
        limits.torque_limits = {}  # 오버라이드 시 관절별 값 대신 공통 캡
    joint_limits = make_joint_limits(cfg, joints)
    ids = all_motor_ids(joints)
    reflex = Reflex(limits, make_pairs(cfg), ReflexConfig())
    source = WaypointSource.from_yaml(args.waypoints)

    bus = FeetechBus(cfg["arm"]["port"], cfg["arm"].get("baudrate", 1_000_000))
    bus.connect()
    for w in verify_eprom(bus, limits, ids):
        print(f"warn (EPROM drift): {w} -> run cookbook/3_safety_torque/320_persist_caps.py", file=sys.stderr)
    source.connect()
    try:
        apply_safety(bus, ids, limits)                      # 토크를 켜기 전 반드시
        caps = ", ".join(f"{i}:{limits.torque_for(i) / 10:.0f}%" for i in ids)
        print(f"torque caps {caps} | step {limits.max_relative_target} ticks | {len(source.waypoints)} waypoints")
        if any(j.__class__.__name__ == "ContinuousJoint" for j in joints):
            print("note: continuous joint (J1) must start with the cable relaxed - current pose becomes range center")
        bus.enable_torque(ids)
        run_control_loop(bus, joints, limits, joint_limits, reflex, source,
                         rate_hz=args.rate or cfg.get("rate_hz", 50), verbose=args.verbose,
                         blackbox=Blackbox(ids, rate_hz=args.rate or cfg.get("rate_hz", 50)))
        print("waypoints done")
    except KeyboardInterrupt:
        print("\nquit requested")
    except RuntimeError as e:
        print(f"\nstopped: {e}", file=sys.stderr)
    finally:
        end_session(bus, ids, limits, args.release)
        source.disconnect()


if __name__ == "__main__":
    main()
