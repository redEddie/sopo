#!/usr/bin/env python3
"""현재 자세를 읽어 arm.yaml의 standby_pose(또는 지정 키)에 기록한다.

토크를 끄고 손으로 원하는 자세를 만든 뒤 실행. 연속 관절(J1/J4)은 home_abs(2048) 기준 프레임으로
저장하므로 세션마다 바퀴 수가 달라도 같은 물리 자세를 가리킨다.

예시:
    python cookbook/13_capture_pose.py --config configs/arm.yaml                # standby_pose 갱신
    python cookbook/13_capture_pose.py --config configs/arm.yaml --key rest_pose
    python cookbook/13_capture_pose.py --config configs/arm.yaml --dry-run
"""

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import argparse

from sopo import FeetechBus
from sopo.config import all_motor_ids, load_arm_config, make_joint_limits, make_joints
from sopo.control import read_joints
from sopo.joints import ContinuousJoint


def main() -> None:
    parser = argparse.ArgumentParser(description="capture current pose into arm.yaml")
    parser.add_argument("--config", default="configs/arm.yaml")
    parser.add_argument("--key", default="standby_pose")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    cfg = load_arm_config(args.config)
    joints = make_joints(cfg)
    bus = FeetechBus(cfg["arm"]["port"], cfg["arm"].get("baudrate", 1_000_000))
    bus.connect()
    try:
        bus.torque_off_verified(all_motor_ids(joints))
        pos = read_joints(bus, joints)
    finally:
        bus.disconnect()

    joint_limits = make_joint_limits(cfg, joints)
    pose = {}
    for j in joints:
        v = pos[j.name]
        if isinstance(j, ContinuousJoint) and j.home_abs is not None:
            v -= j.home - j.home_abs  # home_abs 프레임으로
            bounds = (j.home_abs - (j.range_ticks or 2048), j.home_abs + (j.range_ticks or 2048))
        else:
            bounds = joint_limits.get(j.name)
        if bounds and not bounds[0] <= v <= bounds[1]:
            clamped = max(bounds[0], min(bounds[1], v))
            print(f"warn: {j.name} = {int(v)} is outside soft limit {bounds} (arm resting on its stop?) -> stored {clamped}", file=sys.stderr)
            v = clamped
        pose[j.name] = int(v)
    line = f"{args.key}: {{" + ", ".join(f"{k}: {v}" for k, v in pose.items()) + "}"
    print(line)
    if args.dry_run:
        return
    path = Path(args.config)
    text = path.read_text()
    pat = re.compile(rf"^{re.escape(args.key)}:.*$", re.M)
    text = pat.sub(line, text, count=1) if pat.search(text) else text.rstrip("\n") + "\n" + line + "\n"
    path.write_text(text)
    print(f"written to {path}")


if __name__ == "__main__":
    main()
