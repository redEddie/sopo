#!/usr/bin/env python3
"""모터 EPROM에 토크 상한(Max_Torque_Limit = hold/ceiling, 기본 600‰)과 위치 한계를 영구 기록한다.

운용 캡(관절별 200~400‰)은 apply_safety()가 매번 RAM Torque_Limit에 쓴다. EPROM은 그보다 높은 상한이라
결함 시 홀드(freeze)가 팔을 빳빳하게 잡을 수 있고, 전원을 켠 직후에도 상한 안에서만 움직인다.

Torque_Limit(RAM)은 전원을 켤 때 Max_Torque_Limit(EPROM)에서 복원되므로, 여기 기록해두면
어떤 스크립트가 apply_safety()를 잊거나 데몬이 죽어도 서보 자체가 캡을 지킨다 — 마지막 방어선.
Min/Max_Position_Limit(EPROM)은 펌웨어가 범위 밖 목표를 거부하게 만든다.

예시:
    python cookbook/10_persist_caps.py --config configs/arm.yaml --dry-run   # 현재값 vs 기록할 값
    python cookbook/10_persist_caps.py --config configs/arm.yaml            # 'yes' 입력 후 기록
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import argparse

from sopo import FeetechBus, persist_torque_limit
from sopo.config import all_motor_ids, load_arm_config, make_joints, make_limits
from sopo.motion.joints import ContinuousJoint


def main() -> None:
    parser = argparse.ArgumentParser(description="캡·리밋 EPROM 영구화")
    parser.add_argument("--config", default="configs/arm.yaml")
    parser.add_argument("--dry-run", action="store_true", help="쓰지 않고 비교만")
    parser.add_argument("--yes", action="store_true", help="확인 없이 기록")
    args = parser.parse_args()

    cfg = load_arm_config(args.config)
    joints = make_joints(cfg)
    limits = make_limits(cfg)
    ids = all_motor_ids(joints)
    continuous_ids = {j.motor_id for j in joints if isinstance(j, ContinuousJoint)}

    bus = FeetechBus(cfg["arm"]["port"], cfg["arm"].get("baudrate", 1_000_000))
    bus.connect()
    try:
        plan = []
        print(f"{'ID':>3} | {'Max_Torque_Limit':>17} | {'Min/Max_Position_Limit':>24}")
        for i in ids:
            cap_now = bus.read("Max_Torque_Limit", i)
            cap_want = limits.eprom_torque_ceiling  # 하드웨어 상한. 운용 캡(torque_for)은 apply_safety가 RAM에
            lim_now = (bus.read("Min_Position_Limit", i), bus.read("Max_Position_Limit", i))
            lim_want = limits.position_limits.get(i) if i not in continuous_ids else None  # 연속 관절은 펌웨어 한계 안 씀
            cap_s = f"{cap_now} -> {cap_want}" if cap_now != cap_want else f"{cap_now} (동일)"
            lim_s = "(연속 관절, 유지)" if lim_want is None and i in continuous_ids else (
                f"{lim_now} -> {tuple(lim_want)}" if lim_want and lim_now != tuple(lim_want) else f"{lim_now} (동일)" if lim_want else f"{lim_now} (캘리브레이션 없음, 유지)")
            print(f"{i:>3} | {cap_s:>17} | {lim_s:>24}")
            plan.append((i, cap_now, cap_want, lim_now, tuple(lim_want) if lim_want else None))

        changes = [x for x in plan if x[1] != x[2] or (x[4] and x[3] != x[4])]
        if args.dry_run or not changes:
            print("\n변경 없음." if not changes else "\n(dry-run) 기록하지 않았습니다.")
            return
        if not args.yes:
            ans = input(f"\n{len(changes)}개 모터의 EPROM을 기록합니다. 계속하려면 'yes': ").strip().lower()
            if ans != "yes":
                print("취소.")
                return

        for i, cap_now, cap_want, lim_now, lim_want in changes:
            if cap_now != cap_want:
                persist_torque_limit(bus, [i], cap_want)
            if lim_want and lim_now != lim_want:
                with bus.eprom_unlocked(i):
                    bus.write("Min_Position_Limit", i, lim_want[0])
                    bus.write("Max_Position_Limit", i, lim_want[1])
            got = (bus.read("Max_Torque_Limit", i), bus.read("Min_Position_Limit", i), bus.read("Max_Position_Limit", i))
            ok = got[0] == cap_want and (not lim_want or got[1:] == lim_want)
            print(f"ID{i}: 기록 {'확인' if ok else '불일치 ' + str(got)}")
        print("완료. 전원을 껐다 켜면 Torque_Limit이 이 캡으로 시작합니다.")
    finally:
        bus.disconnect()


if __name__ == "__main__":
    main()
