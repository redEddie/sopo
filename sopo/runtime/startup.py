"""시작 루틴: 자가진단(관절별 소구간 왕복) → 기준 자세 기록 → 대기 자세로 이동.

Franka가 브레이크를 풀고 FCI를 활성화하기 전에 상태를 점검하는 것에 대응한다. 제어를 시작하기 전에
통신·토크·추종·부하가 정상인지 실제로 움직여 확인하고, 연속 관절의 home을 확정한다.
"""

from __future__ import annotations

import sys
import time

from ..hal.bus import FeetechBus
from ..motion.control import command_joints, read_joints
from ..motion.joints import ContinuousJoint, DualMotorJoint, Joint
from ..safety.limits import SafetyLimits

TEST_TICKS = 40      # 자가진단 이동량 (약 3.5 deg). 목표를 한 번에 준다: 오차가 작으면 P제어 출력이 작아
                     # (오차 10틱 ~ 80‰) 무거운 관절이 움직이지 못한다.
ARRIVE = 12


def self_test(bus: FeetechBus, joints: list[Joint], limits: SafetyLimits, joint_limits: dict[str, tuple[int, int]] | None = None,
              order: list[str] | None = None) -> dict[str, dict]:
    """관절을 하나씩(기본: 말단부터) +TEST_TICKS 갔다가 되돌린다. 실패 시 RuntimeError (호출자가 토크 해제)."""
    by_name = {j.name: j for j in joints}
    names = order or [j.name for j in reversed(joints)]
    report: dict[str, dict] = {}
    present = read_joints(bus, joints)
    for j in joints:
        if isinstance(j, ContinuousJoint) and j.range_bounds():
            lo, hi = j.range_bounds()
            print(f"{j.name}: home {j.home} (abs {j.home_abs}), now {present[j.name]}, range [{lo}, {hi}]")
    for name in names:
        j = by_name[name]
        start = present[name]
        ids = list(j.motor_ids)
        result = {"ok": False, "max_load": 0, "err_back": None}
        # 범위 중심 쪽으로 움직인다 (리밋 끝에 주차된 관절도 시험 가능). 시작점이 리밋 밖이면 안쪽으로 되돌린다.
        bounds = j.range_bounds() if isinstance(j, ContinuousJoint) else (joint_limits or {}).get(name)
        if bounds:
            lo, hi = bounds
            start = max(lo, min(hi, start))
            direction = 1 if start < (lo + hi) / 2 else -1
        else:
            direction = 1
        for target in (start + direction * TEST_TICKS, start):
            t0 = time.monotonic()
            while True:
                present = read_joints(bus, joints)
                cur = present[name]
                goal = j.clamp(target) if isinstance(j, ContinuousJoint) else target
                command_joints(bus, joints, {**present, name: goal})
                loads = bus.sync_read("Present_Load", ids)
                result["max_load"] = max(result["max_load"], max(abs(v) for v in loads.values()))
                if abs(target - cur) < ARRIVE:
                    break
                if time.monotonic() - t0 > 3.0:
                    raise RuntimeError(f"self-test failed: {name} did not reach {target} (at {cur}, load {loads})")
                time.sleep(0.02)
        result["err_back"] = present[name] - start
        cap = min(limits.torque_for(i) for i in ids)
        result["ok"] = result["max_load"] < 0.9 * cap
        report[name] = result
        flag = "" if result["ok"] else f"  <-- load {result['max_load']} near cap {cap}"
        print(f"self-test {name}: ok, max load {result['max_load']}‰ / cap {cap}, return error {result['err_back']:+d}{flag}")
    bad = [n for n, r in report.items() if not r["ok"]]
    if bad:
        raise RuntimeError(f"self-test: cap headroom too small on {bad}")
    return report
