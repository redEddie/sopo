"""암 설정 로더: configs/arm.yaml (+ 같은 폴더의 calibration.yaml).

스키마:
    arm: {port, baudrate}
    joints: [...]            # sopo.joints.build_joints 형식
    safety: {...}            # SafetyLimits 필드
구 스키마(leader/follower 섹션)는 경고와 함께 읽어준다.

중력 보정(calibration.yaml의 gravity 섹션)은 load_gravity_cal / make_gravity_model이 담당한다.
"""

from __future__ import annotations

import sys
from pathlib import Path

import yaml

from .model.dynamics import DEFAULT_URDF, GravityCal, GravityModel, build_joint_map
from .motion.joints import ContinuousJoint, DualMotorJoint, Joint, build_joints
from .safety.limits import MODEL_STALL_TORQUE_KGCM, SafetyLimits

DEFAULT_BAUDRATE = 1_000_000


def load_arm_config(path: str | Path) -> dict:
    path = Path(path)
    cfg = yaml.safe_load(path.read_text()) or {}

    if "arm" not in cfg:
        # 구 스키마 호환: leader/follower 중 실제 존재하는 포트를 고른다 (없으면 첫 번째).
        candidates = [cfg[k] for k in ("leader", "follower") if isinstance(cfg.get(k), dict) and "port" in cfg[k]]
        if not candidates:
            raise ValueError(f"{path}: 'arm: {{port, baudrate}}' 섹션이 필요합니다 (configs/arm.yaml 참고)")
        chosen = next((c for c in candidates if Path(c["port"]).exists()), candidates[0])
        cfg["arm"] = {"port": chosen["port"], "baudrate": chosen.get("baudrate", DEFAULT_BAUDRATE)}
        print(f"warn: {path} has no 'arm:' section, using legacy leader/follower port {chosen['port']} - update to the arm.yaml schema", file=sys.stderr)

    if "joints" not in cfg:
        raise ValueError(f"{path}: 'joints' 섹션이 없습니다. configs/arm.yaml을 참고하세요.")

    calib_path = path.parent / "calibration.yaml"
    if calib_path.exists():
        calib = yaml.safe_load(calib_path.read_text()) or {}
        safety = cfg.setdefault("safety", {})
        for key in ("position_limits", "torque_limits"):
            if key in calib:
                safety.setdefault(key, {}).update(calib[key])
        print(f"calibration applied: {calib_path}")
    return cfg


def make_limits(cfg: dict, torque_limit: int | None = None) -> SafetyLimits:
    s = cfg.get("safety", {})
    return SafetyLimits(
        torque_limit=torque_limit if torque_limit is not None else s.get("torque_limit", 300),
        acceleration=s.get("acceleration", 30),
        max_relative_target=s.get("max_relative_target", 80),
        min_position=s.get("min_position", 200),
        max_position=s.get("max_position", 3896),
        position_limits={int(k): tuple(v) for k, v in (s.get("position_limits") or {}).items()},
        torque_limits={int(k): int(v) for k, v in (s.get("torque_limits") or {}).items()},
    )


def make_joints(cfg: dict) -> list[Joint]:
    return build_joints(cfg["joints"])


def make_joint_limits(cfg: dict, joints: list[Joint]) -> dict[str, tuple[int, int]]:
    """모터 ID 기준 position_limits → 관절 이름 기준. continuous는 자체 range_ticks를 쓰므로 제외."""
    s = cfg.get("safety", {})
    motor_limits = {int(k): tuple(v) for k, v in (s.get("position_limits") or {}).items()}
    defaults = (s.get("min_position", 200), s.get("max_position", 3896))
    out: dict[str, tuple[int, int]] = {}
    for j in joints:
        if isinstance(j, ContinuousJoint):
            continue
        ref = j.reference_id if isinstance(j, DualMotorJoint) else j.motor_ids[0]
        out[j.name] = motor_limits.get(ref, defaults)
    return out


def make_pairs(cfg: dict) -> dict[str, tuple[int, int, int]]:
    """듀얼 관절 → (reference_id, mirror_id, K). reflex의 PAIR_MISMATCH 검사용."""
    pairs: dict[str, tuple[int, int, int]] = {}
    for jcfg in cfg.get("joints", []):
        if jcfg.get("type") == "dual":
            ref = jcfg["reference_id"]
            ids = tuple(jcfg["ids"])
            mirror = ids[1] if ids[0] == ref else ids[0]
            pairs[jcfg["name"]] = (ref, mirror, jcfg["K"])
    return pairs


def all_motor_ids(joints: list[Joint]) -> list[int]:
    return [mid for j in joints for mid in j.motor_ids]


def load_gravity_cal(calibration_path: str | Path) -> GravityCal:
    """calibration.yaml의 gravity 섹션 → GravityCal (섹션/파일이 없으면 dir=1, scale={} 기본값)."""
    path = Path(calibration_path)
    data = yaml.safe_load(path.read_text()) if path.exists() else {}
    grav = (data or {}).get("gravity") or {}
    return GravityCal(
        zero_ticks=grav.get("zero_ticks") or {},
        dir=grav.get("dir") or {},
        scale=grav.get("scale") or {},
    )


def make_gravity_model(arm_config_path: str | Path = "configs/arm.yaml",
                       urdf_path: str | Path | None = None) -> GravityModel:
    """arm.yaml joints + 스톨 테이블로 joint_map을 유도한 GravityModel을 만든다."""
    cfg = yaml.safe_load(Path(arm_config_path).read_text()) or {}
    joint_map = build_joint_map(cfg["joints"], MODEL_STALL_TORQUE_KGCM)
    return GravityModel(urdf_path or DEFAULT_URDF, joint_map)
