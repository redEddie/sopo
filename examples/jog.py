#!/usr/bin/env python3
"""키보드 조그 — 리더 암 없이 관절을 실시간으로 움직인다 (ActionSource=JogSource).

키:  ←/→ 선택 관절 -/+   ↑/↓ 관절 선택   1~6 관절 직접 선택   +/- 스텝 증감   space 홀드   q 종료
안전: 캡·스텝·리밋·브레이크 존·리플렉스는 run_control_loop가 그대로 적용. 목표는 현재 ±200틱으로 묶여
      키를 떼면 곧 멈춘다. 리플렉스가 뜨면 [r] 복구 / [q] 종료.

사용 예:
    python examples/jog.py --config configs/arm.yaml
"""

import argparse
import select
import sys
import termios
import tty
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sopo import FeetechBus, apply_safety
from sopo.config import all_motor_ids, load_arm_config, make_joint_limits, make_joints, make_limits, make_pairs
from sopo.control import Blackbox, prompt_recover, run_control_loop
from sopo.reflex import Reflex, ReflexConfig
from sopo.safety import verify_eprom
from sopo.sources import JogSource

KEYMAP = {"\x1b[D": "left", "\x1b[C": "right", "\x1b[A": "up", "\x1b[B": "down"}


class KeyReader:
    """cbreak 모드 논블로킹 키 읽기. 화살표는 이스케이프 시퀀스로 온다."""

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.saved = None

    def __enter__(self):
        self.saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc):
        self.restore()

    def restore(self):
        if self.saved is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
            self.saved = None

    def reenter(self):
        if self.saved is None:
            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)

    def poll(self) -> list[str]:
        keys = []
        while select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch == "\x1b":
                seq = ch + sys.stdin.read(2) if select.select([sys.stdin], [], [], 0.01)[0] else ch
                keys.append(KEYMAP.get(seq, "esc"))
            else:
                keys.append(ch)
        return keys


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default="configs/arm.yaml")
    parser.add_argument("--torque-limit", type=int, default=None, help="전 관절 공통 캡 오버라이드")
    parser.add_argument("--step", type=int, default=40, help="키 한 번당 틱")
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
        print(f"경고(EPROM 드리프트): {w} — cookbook/10_persist_caps.py 실행 권장", file=sys.stderr)

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
        print(f"토크 캡: " + ", ".join(f"{i}:{limits.torque_for(i) / 10:.0f}%" for i in ids))
        print("←/→ 이동  ↑/↓·1~6 관절 선택  +/- 스텝  space 홀드  q 종료")
        bus.enable_torque(ids)
        with reader:
            run_control_loop(bus, joints, limits, joint_limits, reflex, source,
                             rate_hz=cfg.get("rate_hz", 50), on_reflex=on_reflex, verbose=True, status_every=0.2,
                             blackbox=Blackbox(ids, rate_hz=cfg.get("rate_hz", 50)))
    except KeyboardInterrupt:
        print("\n종료 요청.")
    finally:
        reader.restore()
        print("토크를 해제합니다 — 암이 내려올 수 있으니 잡아주세요.")
        bus.disconnect(disable_torque_ids=ids)


if __name__ == "__main__":
    main()
