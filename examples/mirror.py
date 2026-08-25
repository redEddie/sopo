#!/usr/bin/env python3
"""리더-팔로워 미러 텔레오퍼레이션.

리더 암은 토크를 끄고(사람이 자유롭게 움직임), 팔로워 암은 토크 제한을 건 채
리더의 관절각을 따라간다. Trossen xsarm_puppet(마스터 토크 OFF, 퍼펫 position 모드)과
lerobot hopejr(sync_read → clamp → sync_write, max_relative_target)의 구조를 따랐다.

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

from sopo import FeetechBus, SafetyLimits, apply_safety, clamp_goal

MAX_COMM_ERRORS = 5
ARRIVAL_TICKS = 30  # 소프트스타트 수렴 판정 임계값
TEMP_WARN_C = 65


def load_config(path: str) -> dict:
    with open(path) as f:
        cfg = yaml.safe_load(f)
    n_leader, n_follower = len(cfg["leader"]["ids"]), len(cfg["follower"]["ids"])
    if n_leader != n_follower:
        raise ValueError(f"leader ids ({n_leader}) and follower ids ({n_follower}) must match 1:1")

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


def map_leader_to_follower(leader_pos: dict, cfg: dict) -> dict:
    """리더 관절값 → 팔로워 목표값. invert(축 반전)와 offset(틱 오프셋) 적용."""
    invert = set(cfg["follower"].get("invert", []))
    offsets = {int(k): v for k, v in cfg["follower"].get("offsets", {}).items()}
    goal = {}
    for lid, fid in zip(cfg["leader"]["ids"], cfg["follower"]["ids"]):
        pos = leader_pos[lid]
        if fid in invert:
            pos = 4095 - pos
        goal[fid] = pos + offsets.get(fid, 0)
    return goal


def soft_start(leader: FeetechBus, follower: FeetechBus, cfg: dict, limits: SafetyLimits) -> None:
    """팔로워를 리더 자세까지 천천히(작은 스텝으로) 이동시킨다."""
    slow = SafetyLimits(**{**limits.__dict__, "max_relative_target": 20})
    follower_ids = cfg["follower"]["ids"]
    print("소프트스타트: 팔로워가 리더 자세로 수렴하는 중...")
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        leader_pos = leader.sync_read("Present_Position", cfg["leader"]["ids"])
        follower_pos = follower.sync_read("Present_Position", follower_ids)
        goal = clamp_goal(map_leader_to_follower(leader_pos, cfg), follower_pos, slow)
        follower.sync_write("Goal_Position", goal)
        max_err = max(abs(goal[i] - follower_pos[i]) for i in follower_ids)
        if max_err < ARRIVAL_TICKS:
            print("소프트스타트 완료. 미러링 시작 (Ctrl+C로 종료)")
            return
        time.sleep(0.02)
    raise TimeoutError(
        "소프트스타트 15초 초과: 팔로워가 리더 자세에 도달하지 못했습니다. "
        "관절 제한이나 장애물, 토크 제한이 너무 낮은지 확인하세요."
    )


def mirror_loop(leader: FeetechBus, follower: FeetechBus, cfg: dict, limits: SafetyLimits) -> None:
    period = 1.0 / cfg.get("rate_hz", 50)
    leader_ids, follower_ids = cfg["leader"]["ids"], cfg["follower"]["ids"]
    comm_errors = 0
    last_temp_check = 0.0
    while True:
        cycle_start = time.monotonic()
        try:
            leader_pos = leader.sync_read("Present_Position", leader_ids)
            follower_pos = follower.sync_read("Present_Position", follower_ids)
            goal = clamp_goal(map_leader_to_follower(leader_pos, cfg), follower_pos, limits)
            follower.sync_write("Goal_Position", goal)
            comm_errors = 0
        except ConnectionError as e:
            comm_errors += 1
            print(f"통신 오류 ({comm_errors}/{MAX_COMM_ERRORS}): {e}", file=sys.stderr)
            if comm_errors >= MAX_COMM_ERRORS:
                raise RuntimeError("통신 오류가 연속으로 발생하여 안전을 위해 중단합니다.") from e

        if cycle_start - last_temp_check > 2.0:
            last_temp_check = cycle_start
            temps = follower.sync_read("Present_Temperature", follower_ids)
            hot = {i: t for i, t in temps.items() if t >= TEMP_WARN_C}
            if hot:
                print(f"경고: 팔로워 모터 과열 {hot} (°C)", file=sys.stderr)

        elapsed = time.monotonic() - cycle_start
        if elapsed < period:
            time.sleep(period - elapsed)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default="configs/arm.example.yaml")
    args = parser.parse_args()

    cfg = load_config(args.config)
    limits = make_limits(cfg)

    leader = FeetechBus(cfg["leader"]["port"], cfg["leader"].get("baudrate", 1_000_000))
    follower = FeetechBus(cfg["follower"]["port"], cfg["follower"].get("baudrate", 1_000_000))
    leader.connect()
    follower.connect()

    follower_ids = cfg["follower"]["ids"]
    try:
        # 리더는 자유롭게 움직여야 하므로 토크 OFF (Trossen master_modes와 동일)
        leader.disable_torque(cfg["leader"]["ids"])
        # 팔로워는 토크를 켜기 전에 반드시 안전 제한부터 적용
        apply_safety(follower, follower_ids, limits)
        follower.enable_torque(follower_ids)
        caps = ", ".join(f"{i}:{limits.torque_for(i) / 10:.0f}%" for i in follower_ids)
        print(f"팔로워 토크 제한: {caps} / 스텝 제한: {limits.max_relative_target} ticks")

        soft_start(leader, follower, cfg, limits)
        mirror_loop(leader, follower, cfg, limits)
    except KeyboardInterrupt:
        print("\n종료 요청을 받았습니다.")
    finally:
        print("팔로워 토크를 해제합니다 — 암이 내려올 수 있으니 잡아주세요.")
        follower.disconnect(disable_torque_ids=follower_ids)
        leader.disconnect()


if __name__ == "__main__":
    main()
