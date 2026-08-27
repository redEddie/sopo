#!/usr/bin/env python3
"""키보드 조그 — 리더 암 없이 관절을 실시간으로 움직인다 (ActionSource=JogSource).

키:  ←/→ (또는 a/d) 선택 관절 -/+   ↑/↓ (또는 w/s) 관절 선택   1~6 관절 직접 선택   +/- 스텝   space(h) 홀드   q 종료
안전: 캡·스텝·리밋·브레이크 존·리플렉스는 run_control_loop가 그대로 적용. 목표는 현재 ±200틱으로 묶여
      키를 떼면 곧 멈춘다. 리플렉스가 뜨면 [r] 복구 / [q] 종료.

사용 예:
    python examples/jog.py --config configs/arm.yaml
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sopo import FeetechBus, apply_safety
from sopo.config import all_motor_ids, load_arm_config, make_joint_limits, make_joints, make_limits, make_pairs
from sopo.control import end_session, Blackbox, prompt_recover, run_control_loop
from sopo.reflex import Reflex, ReflexConfig
from sopo.safety import verify_eprom
from sopo.sources import JogSource

from sopo.keys import KeyReader


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default="configs/arm.yaml")
    parser.add_argument("--torque-limit", type=int, default=None, help="전 관절 공통 캡 오버라이드")
    parser.add_argument("--step", type=int, default=40, help="키 한 번당 틱")
    parser.add_argument("--release", action="store_true", help="drop torque at exit (default: hold, Cat 2 stop)")
    args = parser.parse_args()

    cfg = load_arm_config(args.config)
    joints = make_joints(cfg)
    limits = make_limits(cfg, args.torque_limit)
    if args.torque_limit is not None:
        limits.torque_limits = {}
    joint_limits = make_joint_limits(cfg, joints)
    ids = all_motor_ids(joints)
    reflex = Reflex(limits, make_pairs(cfg), ReflexConfig())

    bus = FeetechBus(cfg["arm"]["port"], cfg["arm"].get("baudrate", 1_000_000))
    bus.connect()
    for w in verify_eprom(bus, limits, ids):
        print(f"warn (EPROM drift): {w} -> run cookbook/10_persist_caps.py", file=sys.stderr)

    reader = KeyReader()
    source = JogSource([j.name for j in joints], reader.poll, step=args.step)

    def on_reflex(bus_, joints_, reflex_):
        reader.restore()  # 프롬프트는 일반 터미널 모드로
        try:
            return prompt_recover(bus_, joints_, reflex_)
        finally:
            reader.reenter()

    try:
        apply_safety(bus, ids, limits)
        print("torque caps: " + ", ".join(f"{i}:{limits.torque_for(i) / 10:.0f}%" for i in ids))
        print("keys: left/right (a/d) move | up/down (w/s), 1-6 select joint | +/- step | space (h) hold | q quit")
        bus.enable_torque(ids)
        with reader:
            run_control_loop(bus, joints, limits, joint_limits, reflex, source,
                             rate_hz=cfg.get("rate_hz", 50), on_reflex=on_reflex, verbose=True, status_every=0.1, status_inline=True,
                             blackbox=Blackbox(ids, rate_hz=cfg.get("rate_hz", 50)), dump_on_exit=True)
    except KeyboardInterrupt:
        print("\nquit requested")
    except RuntimeError as e:
        print(f"\nstopped: {e}", file=sys.stderr)
    finally:
        reader.restore()
        end_session(bus, ids, limits, args.release)


if __name__ == "__main__":
    main()
