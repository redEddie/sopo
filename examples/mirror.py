#!/usr/bin/env python3
"""리더-팔로워 미러 텔레오퍼레이션.

리더 암은 토크를 끄고(사람이 자유롭게 움직임), 팔로워 암은 토크 제한을 건 채
리더의 관절각을 따라간다. Trossen xsarm_puppet(마스터 토크 OFF, 퍼펫 position 모드)과
lerobot hopejr(sync_read → clamp → sync_write, max_relative_target)의 구조를 따랐다.

관절 추상화:
  - J1은 케이블 제한 연속 관절: 랩을 넘어도 논리각 유지, 시작 자세 ± range_ticks로 목표를 잘라냄
    (전선이 풀린 자세에서 프로그램을 시작할 것 — 바퀴 수는 메모리에만 있음)
  - J2/J3는 듀얼 모터: reference_id(ID11/ID16) 기준으로 관절을 제어, 미러 모터는 K - goal

안전장치:
  - 팔로워 Torque_Limit(기본 30%) + 과부하 보호 레지스터 (apply_safety)
  - 매 사이클 팔로워 실측 위치 기준 max_relative_target 클램핑 → 점프 불가
  - 시작 시 소프트스타트: 팔로워가 리더 자세까지 천천히 수렴한 뒤 미러 시작
  - 통신 오류 연속 5회 시 토크 해제 후 종료, 종료 시 항상 토크 해제

사용 예:
    python examples/mirror.py --config configs/arm.example.yaml
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import yaml

from sopo import (
    ContinuousJoint,
    DualMotorJoint,
    FeetechBus,
    Mode,
    Reflex,
    ReflexConfig,
    SafetyLimits,
    SingleMotorJoint,
    apply_safety,
)

ARRIVAL_TICKS = 30  # 소프트스타트 수렴 판정 임계값
TICKS_PER_REV = 4096


def load_config(path: str) -> dict:
    with open(path) as f:
        cfg = yaml.safe_load(f)

    if "joints" not in cfg:
        raise ValueError("설정에 'joints' 섹션이 없습니다. configs/arm.example.yaml을 참고하세요.")

    # 실측 캘리브레이션(05_find_limits / 06_gravity_load 결과)이 같은 폴더에 있으면 덮어씌운다.
    calib_path = Path(path).parent / "calibration.yaml"
    if calib_path.exists():
        calib = yaml.safe_load(calib_path.read_text()) or {}
        safety = cfg.setdefault("safety", {})
        for key in ("position_limits", "torque_limits"):
            if key in calib:
                safety.setdefault(key, {}).update(calib[key])
        print(f"캘리브레이션 적용: {calib_path}")
    return cfg


def build_arm_joints(cfg: dict, arm_name: str):
    """설정의 joints 정의로부터 한 쪽 암의 Joint 객체 리스트를 만든다."""
    joints = []
    for jcfg in cfg["joints"]:
        name = jcfg["name"]
        jtype = jcfg["type"]
        if jtype == "single":
            joints.append(SingleMotorJoint(name, jcfg["motor_id"]))
        elif jtype == "dual":
            joints.append(DualMotorJoint(name, tuple(jcfg["ids"]), jcfg["reference_id"], jcfg["K"]))
        elif jtype == "continuous":
            joints.append(ContinuousJoint(name, jcfg["motor_id"], jcfg.get("range_ticks")))
        else:
            raise ValueError(f"알 수 없는 관절 타입: {jtype}")
    return joints


def build_pairs(cfg: dict) -> dict[str, tuple[int, int, int]]:
    pairs: dict[str, tuple[int, int, int]] = {}
    for jcfg in cfg.get("joints", []):
        if jcfg.get("type") == "dual":
            ref = jcfg["reference_id"]
            ids = tuple(jcfg["ids"])
            mirror = ids[1] if ids[0] == ref else ids[0]
            pairs[jcfg["name"]] = (ref, mirror, jcfg["K"])
    return pairs


def joint_motor_goals(joints, joint_goals: dict[str, int]) -> dict[int, int]:
    """Convert logical joint goals to per-motor goals for reflex comparison."""
    motor_goals: dict[int, int] = {}
    for j in joints:
        goal = joint_goals[j.name]
        if isinstance(j, DualMotorJoint):
            motor_goals[j.reference_id] = goal
            motor_goals[j.mirror_id] = max(0, min(TICKS_PER_REV - 1, j.K - goal))
        elif isinstance(j, ContinuousJoint):
            # Reflex compares in the logical (unwrapped) frame.
            motor_goals[j.motor_id] = goal
        else:
            motor_goals[j.motor_id] = goal
    return motor_goals


def make_limits(cfg: dict) -> SafetyLimits:
    s = cfg.get("safety", {})
    return SafetyLimits(
        torque_limit=s.get("torque_limit", 300),
        acceleration=s.get("acceleration", 30),
        max_relative_target=s.get("max_relative_target", 80),
        min_position=s.get("min_position", 200),
        max_position=s.get("max_position", 3896),
        position_limits={int(k): tuple(v) for k, v in (s.get("position_limits") or {}).items()},
        torque_limits={int(k): int(v) for k, v in (s.get("torque_limits") or {}).items()},
    )


def make_joint_limits(cfg: dict, joints) -> dict[str, tuple[int, int]]:
    """모터 ID 기준 position_limits를 관절 이름 기준으로 변환. continuous는 제외."""
    s = cfg.get("safety", {})
    motor_limits = {int(k): tuple(v) for k, v in (s.get("position_limits") or {}).items()}
    defaults = (s.get("min_position", 200), s.get("max_position", 3896))
    joint_limits = {}
    for j in joints:
        if isinstance(j, ContinuousJoint):
            continue
        if isinstance(j, DualMotorJoint):
            ref_id = j.reference_id
        else:
            ref_id = j.motor_ids[0]
        joint_limits[j.name] = motor_limits.get(ref_id, defaults)
    return joint_limits


def continuous_joints(joints) -> dict[str, ContinuousJoint]:
    return {j.name: j for j in joints if isinstance(j, ContinuousJoint)}


def read_all_joints(bus: FeetechBus, joints) -> dict[str, int]:
    return {j.name: j.read(bus) for j in joints}


def command_all_joints(bus: FeetechBus, joints, goals: dict[str, int]) -> None:
    for j in joints:
        j.command(bus, goals[j.name])


def clamp_joint_goal(
    goal: dict[str, int],
    present: dict[str, int],
    joint_limits: dict[str, tuple[int, int]],
    max_step: int,
    continuous: dict[str, ContinuousJoint],
) -> dict[str, int]:
    """관절 단위로 리밋과 스텝을 클램프. continuous 관절은 home ± range_ticks로 잘라낸다."""
    safe = {}
    for name, target in goal.items():
        if name in continuous:
            target = continuous[name].clamp(target)  # 케이블 범위: 잘라내기만, 오류 아님
        else:
            lo, hi = joint_limits.get(name, (0, TICKS_PER_REV - 1))
            target = max(lo, min(hi, target))
        if name in present:
            pos = present[name]
            target = max(pos - max_step, min(pos + max_step, target))
        safe[name] = target
    return safe


def map_leader_to_follower(
    leader_pos: dict[str, int], cfg: dict, joints
) -> dict[str, int]:
    """리더 관절값 → 팔로워 목표값. invert(축 반전)와 offset(틱 오프셋) 적용."""
    invert = set(cfg["follower"].get("invert", []))
    offsets = cfg["follower"].get("offsets", {})
    goal = {}
    for j in joints:
        pos = leader_pos[j.name]
        if j.name in invert:
            pos = (TICKS_PER_REV - 1) - pos
        goal[j.name] = pos + offsets.get(j.name, 0)
    return goal


def print_joint_table(
    leader_pos: dict[str, int],
    follower_pos: dict[str, int],
    leader_joints,
) -> None:
    """리더/팔로워 관절 위치와 오차를 표로 출력한다."""
    print(f"\n{'Joint':>4} | {'Leader':>8} | {'Follower':>8} | {'Err':>6} | Note")
    print("-" * 48)
    for j in leader_joints:
        name = j.name
        lp = leader_pos[name]
        fp = follower_pos[name]
        err = fp - lp
        note = ""
        if isinstance(j, ContinuousJoint):
            note = f"turn {j.turn_count}"
        elif isinstance(j, DualMotorJoint):
            note = f"ref={j.reference_id}"
        print(f"{name:>4} | {lp:>8} | {fp:>8} | {err:>+6} | {note}")


def soft_start(
    leader: FeetechBus,
    follower: FeetechBus,
    leader_joints,
    follower_joints,
    cfg: dict,
    limits: SafetyLimits,
    joint_limits: dict[str, tuple[int, int]],
    verbose: bool = False,
) -> None:
    """팔로워를 리더 자세까지 천천히(작은 스텝으로) 이동시킨다."""
    slow_step = 20
    continuous = continuous_joints(follower_joints)
    print("소프트스타트: 팔로워가 리더 자세로 수렴하는 중...")
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        leader_pos = read_all_joints(leader, leader_joints)
        follower_pos = read_all_joints(follower, follower_joints)
        target = map_leader_to_follower(leader_pos, cfg, follower_joints)
        goal = clamp_joint_goal(target, follower_pos, joint_limits, slow_step, continuous)
        command_all_joints(follower, follower_joints, goal)
        if verbose:
            print_joint_table(leader_pos, follower_pos, leader_joints)
        max_err = max(abs(goal[j.name] - follower_pos[j.name]) for j in follower_joints)
        if max_err < ARRIVAL_TICKS:
            print("소프트스타트 완료. 미러링 시작 (Ctrl+C로 종료)")
            return
        time.sleep(0.02)
    raise TimeoutError(
        "소프트스타트 15초 초과: 팔로워가 리더 자세에 도달하지 못했습니다. "
        "관절 제한이나 장애물, 토크 제한이 너무 낮은지 확인하세요."
    )


def wait_reflex_recover(
    reflex: Reflex,
    follower_joints,
    follower_pos: dict[str, int],
    follower_load: dict[int, int],
) -> None:
    """Block for user input while in REFLEX; 'r' tries recover, 'q' aborts."""

    def _motor_present() -> dict[int, int]:
        out: dict[int, int] = {}
        for j in follower_joints:
            pos = follower_pos[j.name]
            if isinstance(j, DualMotorJoint):
                out[j.reference_id] = pos
            elif isinstance(j, ContinuousJoint):
                out[j.motor_id] = pos
            else:
                out[j.motor_id] = pos
        return out

    while reflex.mode is Mode.REFLEX:
        try:
            key = input("리플렉스 발동. [r] 복구 시도, [q] 종료: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            raise RuntimeError("사용자가 리플렉스 상태에서 종료했습니다.")
        if key == "q":
            raise RuntimeError("사용자가 리플렉스 상태에서 종료했습니다.")
        if key == "r":
            ok, reason = reflex.recover(_motor_present(), follower_load)
            if ok:
                print("복구 성공. 미러링 재개.")
                return
            print(f"복구 불가: {reason}")


def mirror_loop(
    leader: FeetechBus,
    follower: FeetechBus,
    leader_joints,
    follower_joints,
    cfg: dict,
    limits: SafetyLimits,
    joint_limits: dict[str, tuple[int, int]],
    reflex: Reflex,
    verbose: bool = False,
    verbose_interval: float = 0.5,
) -> None:
    period = 1.0 / cfg.get("rate_hz", 50)
    follower_motor_ids = [mid for j in follower_joints for mid in j.motor_ids]
    continuous = continuous_joints(follower_joints)
    last_temp_check = 0.0
    last_verbose = 0.0

    follower_load: dict[int, int] = {}
    goal: dict[str, int] = {}
    present_raw: dict[int, int] = {}
    while True:
        cycle_start = time.monotonic()
        comm_ok = True
        try:
            leader_pos = read_all_joints(leader, leader_joints)
            follower_pos = read_all_joints(follower, follower_joints)
            follower_load = follower.sync_read("Present_Load", follower_motor_ids)
            target = map_leader_to_follower(leader_pos, cfg, follower_joints)
            goal = clamp_joint_goal(target, follower_pos, joint_limits, limits.max_relative_target, continuous)
            command_all_joints(follower, follower_joints, goal)
            # Reflex needs both reference and mirror positions for pair checks.
            present_raw = follower.sync_read("Present_Position", follower_motor_ids)
        except ConnectionError as e:
            comm_ok = False
            print(f"통신 오류: {e}", file=sys.stderr)

        motor_goal = joint_motor_goals(follower_joints, goal)
        trips = reflex.update(cycle_start, present_raw, motor_goal, follower_load, comm_ok=comm_ok)
        for trip in trips:
            print(f"[!] {trip.event.value}: {trip.detail}", file=sys.stderr)

        if reflex.mode is Mode.STOPPED:
            raise RuntimeError("STOPPED: 안전을 위해 중단합니다.")

        if reflex.mode is Mode.REFLEX:
            hold = reflex.hold_targets(present_raw)
            # Convert per-motor hold to joint goals so ContinuousJoint unwraps correctly.
            hold_joint_goals: dict[str, int] = {}
            for j in follower_joints:
                if isinstance(j, DualMotorJoint):
                    hold_joint_goals[j.name] = hold[j.reference_id]
                else:
                    hold_joint_goals[j.name] = hold[j.motor_id]
            command_all_joints(follower, follower_joints, hold_joint_goals)
            print("홀드 목표 전송.", file=sys.stderr)
            wait_reflex_recover(reflex, follower_joints, follower_pos, follower_load)
            continue

        if verbose and cycle_start - last_verbose >= verbose_interval:
            last_verbose = cycle_start
            print_joint_table(leader_pos, follower_pos, leader_joints)

        if cycle_start - last_temp_check > 2.0:
            last_temp_check = cycle_start
            temps = follower.sync_read("Present_Temperature", follower_motor_ids)
            trips = reflex.update(
                cycle_start, present_raw, motor_goal, follower_load, temps=temps, comm_ok=comm_ok
            )
            for trip in trips:
                print(f"[!] {trip.event.value}: {trip.detail}", file=sys.stderr)
            for w in reflex.warnings():
                print(f"경고: {w}", file=sys.stderr)
            if reflex.mode is Mode.STOPPED:
                raise RuntimeError("STOPPED: 과열/통신으로 안전을 위해 중단합니다.")

        elapsed = time.monotonic() - cycle_start
        if elapsed < period:
            time.sleep(period - elapsed)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default="configs/arm.example.yaml")
    parser.add_argument("--verbose", action="store_true", help="관절별 리더/팔로워 위치와 오차를 주기적으로 출력")
    parser.add_argument("--verbose-interval", type=float, default=0.5, help="verbose 출력 주기 (초)")
    args = parser.parse_args()

    cfg = load_config(args.config)
    limits = make_limits(cfg)

    leader_joints = build_arm_joints(cfg, "leader")
    follower_joints = build_arm_joints(cfg, "follower")
    follower_motor_ids = [mid for j in follower_joints for mid in j.motor_ids]
    joint_limits = make_joint_limits(cfg, follower_joints)

    pairs = build_pairs(cfg)
    reflex = Reflex(limits, pairs)

    leader = FeetechBus(cfg["leader"]["port"], cfg["leader"].get("baudrate", 1_000_000))
    follower = FeetechBus(cfg["follower"]["port"], cfg["follower"].get("baudrate", 1_000_000))
    leader.connect()
    follower.connect()

    try:
        # 리더는 자유롭게 움직여야 하므로 토크 OFF (Trossen master_modes와 동일)
        leader.disable_torque([mid for j in leader_joints for mid in j.motor_ids])
        # 팔로워는 토크를 켜기 전에 반드시 안전 제한부터 적용
        apply_safety(follower, follower_motor_ids, limits)
        follower.enable_torque(follower_motor_ids)
        caps = ", ".join(f"{i}:{limits.torque_for(i) / 10:.0f}%" for i in follower_motor_ids)
        print(f"팔로워 토크 제한: {caps} / 스텝 제한: {limits.max_relative_target} ticks")
        if continuous_joints(follower_joints):
            print("주의: 연속 관절(J1)은 전선이 풀린 자세에서 시작해야 합니다 — 지금 자세가 허용 범위의 중심이 됩니다.")

        soft_start(
            leader, follower, leader_joints, follower_joints, cfg, limits, joint_limits,
            verbose=args.verbose,
        )
        mirror_loop(
            leader, follower, leader_joints, follower_joints, cfg, limits, joint_limits,
            reflex=reflex,
            verbose=args.verbose,
            verbose_interval=args.verbose_interval,
        )
    except KeyboardInterrupt:
        print("\n종료 요청을 받았습니다.")
    finally:
        print("팔로워 토크를 해제합니다 — 암이 내려올 수 있으니 잡아주세요.")
        follower.disconnect(disable_torque_ids=follower_motor_ids)
        leader.disconnect()


if __name__ == "__main__":
    main()
