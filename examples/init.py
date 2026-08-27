#!/usr/bin/env python3
"""초기화: 자가진단(관절별 2.6 deg 왕복) → 연속 관절 home 확정 → standby_pose로 이동 → 대기.

사용 예:
    python examples/init.py --config configs/arm.yaml            # 대기 후 Ctrl+C 로 토크 해제
    python examples/init.py --config configs/arm.yaml --hold 5   # 5초 대기 후 토크 해제
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sopo import FeetechBus, apply_safety
from sopo.config import all_motor_ids, load_arm_config, make_joint_limits, make_joints, make_limits, make_pairs
from sopo.control import Blackbox, run_control_loop
from sopo.reflex import Reflex, ReflexConfig
from sopo.safety import verify_eprom
from sopo.sources import WaypointSource
from sopo.startup import self_test


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default="configs/arm.yaml")
    parser.add_argument("--hold", type=float, default=None, help="seconds to hold standby, then torque off (default: until Ctrl+C)")
    parser.add_argument("--skip-test", action="store_true")
    args = parser.parse_args()

    cfg = load_arm_config(args.config)
    joints = make_joints(cfg)
    limits = make_limits(cfg)
    joint_limits = make_joint_limits(cfg, joints)
    ids = all_motor_ids(joints)
    reflex = Reflex(limits, make_pairs(cfg), ReflexConfig())

    bus = FeetechBus(cfg["arm"]["port"], cfg["arm"].get("baudrate", 1_000_000))
    bus.connect()
    for w in verify_eprom(bus, limits, ids):
        print(f"warn (EPROM drift): {w}", file=sys.stderr)
    try:
        apply_safety(bus, ids, limits)
        bus.enable_torque(ids)
        if not args.skip_test:
            self_test(bus, joints, limits)
        standby = {str(k): int(v) for k, v in (cfg.get("standby_pose") or {}).items()}
        source = WaypointSource([standby] if standby else [{}], dwell_s=args.hold if args.hold else 1e9)
        source.connect()
        print("standby:", standby or "(hold current pose)")
        run_control_loop(bus, joints, limits, joint_limits, reflex, source, rate_hz=cfg.get("rate_hz", 50),
                         verbose=True, status_every=1.0, blackbox=Blackbox(ids, rate_hz=cfg.get("rate_hz", 50)))
        print("init done")
    except KeyboardInterrupt:
        print("\nquit requested")
    except RuntimeError as e:
        print(f"\nstopped: {e}", file=sys.stderr)
    finally:
        print("torque off - support the arm, it may drop")
        bus.disconnect(disable_torque_ids=ids)


if __name__ == "__main__":
    main()
