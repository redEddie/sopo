"""모델 기반 중력 토크 G(q) 계산, 모터 단위 환산, 외력 토크 추정.

description/arm_no_ee.urdf (6-DOF, joint_1~6)를 pinocchio로 읽어 현재 자세에서
각 관절이 중력을 버티는 데 필요한 토크[N·m]를 계산한다.

모터 비교/추정을 위한 환산 (정지 상태, duty ≈ 토크 비율):
    예측 Present_Load[‰] ≈ G(q) / stall × 1000
    외력 토크[N·m]     ≈ measured‰ × stall / 1000 − G(q)

듀얼 모터 관절(J2/J3)은 스톨을 합산하고, 관절 토크 기여는 mount_sign으로 부호를
맞춘 모터별 부하의 합이다. per-motor 예측 ‰는 합산 ‰와 동일
(토크를 반씩 나눠 지므로 각 모터도 stall의 같은 비율로 일한다고 가정).

관절↔URDF↔모터 매핑의 단일 진실 공급원은 configs/arm.yaml이다 (build_joint_map).
"""

import math
from dataclasses import dataclass, field
from pathlib import Path

from .safety import KGCM_TO_NM

TICKS_PER_REV = 4096


def build_joint_map(joints: list[dict], stall_table: dict[str, float]) -> dict:
    """arm.yaml joints 목록에서 관절↔URDF↔모터 매핑을 유도한다 (순수 함수).

    joints 목록의 순서가 URDF 조인트 번호가 된다 (첫 관절 → joint_1).
    stall_table은 모델별 스톨 토크 [kg·cm] (safety.MODEL_STALL_TORQUE_KGCM).
    결과: {name: {"urdf", "stall" [N·m, 모터 수 합산], "ids", "mount_sign"}}.
    """
    out = {}
    for idx, j in enumerate(joints):
        ids = list(j["ids"]) if j["type"] == "dual" else [j["motor_id"]]
        out[j["name"]] = {
            "urdf": f"joint_{idx + 1}",
            "stall": stall_table[j["model"]] * KGCM_TO_NM * len(ids),
            "ids": ids,
            "mount_sign": list(j.get("mount_sign", [1] * len(ids))),
        }
    return out


# arm.yaml을 읽지 못할 때 쓰는 평백: sopo 관절 ↔ URDF 관절 ↔ 스톨 토크 [N·m @12V].
# effort가 아니라 스톨 기준: Present_Load(‰)가 스톨 대비 듀티 비율이기 때문.
_FALLBACK_JOINT_MAP = {
    "J1": {"urdf": "joint_1", "stall": 8.34},  # SM8512BL — 수직축이라 G(q)≈0
    "J2": {"urdf": "joint_2", "stall": 9.81},  # STS3250 x2 (듀얼 합산)
    "J3": {"urdf": "joint_3", "stall": 9.81},  # STS3250 x2
    "J4": {"urdf": "joint_4", "stall": 4.90},  # STS3250 — 롤축이라 G(q)≈0
    "J5": {"urdf": "joint_5", "stall": 2.94},  # STS3215 12V
    "J6": {"urdf": "joint_6", "stall": 2.94},  # STS3215 12V
}


def _default_joint_map() -> dict:
    """configs/arm.yaml에서 기본 매핑을 유도한다. 읽기 실패 시 하드코딩 값으로 평백."""
    arm_path = Path(__file__).resolve().parents[1] / "configs" / "arm.yaml"
    try:
        import yaml

        from .safety import MODEL_STALL_TORQUE_KGCM

        joints = yaml.safe_load(arm_path.read_text())["joints"]
        return build_joint_map(joints, MODEL_STALL_TORQUE_KGCM)
    except Exception:
        return {name: dict(spec) for name, spec in _FALLBACK_JOINT_MAP.items()}


JOINT_MAP = _default_joint_map()

DEFAULT_URDF = Path(__file__).resolve().parents[1] / "description" / "arm_no_ee.urdf"


def ticks_to_rad(ticks: float, zero_ticks: float, direction: int = 1) -> float:
    """모터 틱을 URDF 각도로. zero_ticks는 URDF q=0(수직 직립) 자세의 틱."""
    return direction * (ticks - zero_ticks) * (2 * math.pi / TICKS_PER_REV)


@dataclass
class GravityCal:
    """calibration.yaml gravity 섹션: URDF zero 기준 틱, 조인트 방향, ‰↔토크 스케일."""

    zero_ticks: dict[str, float] = field(default_factory=dict)
    dir: dict[str, int] = field(default_factory=dict)
    scale: dict[str, float] = field(default_factory=dict)

    def q(self, joint_name: str, ticks: float) -> float:
        """모터 틱 → URDF 각도 [rad]. dir이 없으면 +1."""
        return ticks_to_rad(ticks, self.zero_ticks[joint_name], self.dir.get(joint_name, 1))


class GravityModel:
    """pinocchio 래퍼. 스레드 안전하지 않으므로 제어 루프와 분리해 쓸 것."""

    def __init__(self, urdf_path: Path | str = DEFAULT_URDF, joint_map: dict | None = None):
        import pinocchio as pin

        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()
        self.joint_map = joint_map or JOINT_MAP
        self._jidx = {}  # sopo 관절명 -> (idx_q, idx_v, nq)
        for sopo_name, spec in self.joint_map.items():
            jid = self.model.getJointId(spec["urdf"])
            joint = self.model.joints[jid]
            self._jidx[sopo_name] = (joint.idx_q, joint.idx_v, joint.nq)

    def _build_q(self, q_rad: dict[str, float]):
        import pinocchio as pin

        q = pin.neutral(self.model)
        for sopo_name, rad in q_rad.items():
            iq, _, nq = self._jidx[sopo_name]
            if nq == 2:  # continuous (joint_6): cos/sin 쌍
                q[iq], q[iq + 1] = math.cos(rad), math.sin(rad)
            else:
                q[iq] = rad
        return q

    def gravity(self, q_rad: dict[str, float]) -> dict[str, float]:
        """q_rad: {sopo 관절명: rad} → {sopo 관절명: 중력 토크 [N·m]}"""
        import pinocchio as pin

        g = pin.computeGeneralizedGravity(self.model, self.data, self._build_q(q_rad))
        return {name: float(g[iv]) for name, (_, iv, _) in self._jidx.items()}

    def extra_load_torque(self, q_rad: dict[str, float], mass: float,
                          link: str = "flange") -> dict[str, float]:
        """링크 원점(flange 기본)에 매단 추加重량[mass kg]이 각 관절에 거는 토크 [N·m].

        스케일 보정(알려진 무게 매달기)과 페이로드 추정에 쓴다.
        """
        import numpy as np
        import pinocchio as pin

        pin.forwardKinematics(self.model, self.data, self._build_q(q_rad))
        pin.updateFramePlacements(self.model, self.data)
        point = self.data.oMf[self.model.getFrameId(link)].translation
        force = np.array([0.0, 0.0, -mass * 9.81])
        out = {}
        for sopo_name, spec in self.joint_map.items():
            jid = self.model.getJointId(spec["urdf"])
            origin = self.data.oMi[jid].translation
            axis = self.data.oMi[jid].rotation[:, 2]  # 조인트 로컬 z축의 월드 방향
            out[sopo_name] = float(np.dot(np.cross(point - origin, force), axis))
        return out

    def load_permille(self, q_rad: dict[str, float],
                      scale: dict[str, float] | None = None) -> dict[str, float]:
        """관절별 예측 Present_Load [‰]. 부호 포함 (토크 방향).

        scale은 ‰↔토크 보정 계수(관절별, 기본 1.0): 실측‰ ≈ scale × 모델‰.
        """
        g = self.gravity(q_rad)
        scale = scale or {}
        return {n: scale.get(n, 1.0) * g[n] / self.joint_map[n]["stall"] * 1000
                for n in g}

    def external_torque(self, q_rad: dict[str, float],
                        measured_permille: dict[str, float],
                        scale: dict[str, float] | None = None) -> dict[str, float]:
        """외력 토크 추정 [N·m] = 측정 토크 − 중력 토크 (정지/저속 가정).

        scale을 넣으면 측정 ‰를 실제 토크로 환산할 때 그 계수를 적용한다.
        듀얼 관절의 measured는 reference 모터의 값을 그대로 넣으면 된다.
        """
        g = self.gravity(q_rad)
        scale = scale or {}
        return {
            n: measured_permille.get(n, 0.0) * self.joint_map[n]["stall"]
            / (1000 * scale.get(n, 1.0)) - g[n]
            for n in g
        }

    def measured_torque_nm(self, loads_by_motor: dict[int, float]) -> dict[str, float]:
        """모터별 Present_Load [‰] → 관절별 측정 토크 [N·m] (정지/저속 가정).

        single: load[id] × stall / 1000.
        dual:   Σ sign_i × load[id_i] × (stall/2) / 1000 — 미러는 반전 장착이라
                mount_sign으로 부호를 맞춰 합산한다 (stall은 관절 합산 기준).
        joint_map에 ids가 없으면(구형 평백 맵) 0, mount_sign이 없으면
        reference 방식(첫 모터 값 × stall)으로 평백한다.
        """
        out = {}
        for name, spec in self.joint_map.items():
            ids = spec.get("ids") or []
            signs = spec.get("mount_sign")
            if not ids:
                out[name] = 0.0
            elif signs is None:
                out[name] = loads_by_motor.get(ids[0], 0.0) * spec["stall"] / 1000
            else:
                per_motor = spec["stall"] / len(ids)
                out[name] = sum(sg * loads_by_motor.get(i, 0.0)
                                for i, sg in zip(ids, signs)) * per_motor / 1000
        return out

    def external_torque_from_loads(self, q_rad: dict[str, float],
                                   loads_by_motor: dict[int, float],
                                   scale: dict[str, float] | None = None) -> dict[str, float]:
        """모터 부하로부터 외력 토크 [N·m] = 측정 토크 − 중력 토크 (정지/저속 가정).

        scale을 넣으면 측정 토크를 실제 토크로 환산할 때 그 계수로 나눈다.
        """
        g = self.gravity(q_rad)
        scale = scale or {}
        meas = self.measured_torque_nm(loads_by_motor)
        return {n: meas[n] / scale.get(n, 1.0) - g[n] for n in g}
