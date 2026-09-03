"""모델 기반 중력 토크 G(q) 계산, 모터 단위 환산, 외력 토크 추정.

description/arm_no_ee.urdf (6-DOF, joint_1~6)를 pinocchio로 읽어 현재 자세에서
각 관절이 중력을 버티는 데 필요한 토크[N·m]를 계산한다.

모터 비교/추정을 위한 환산 (정지 상태, duty ≈ 토크 비율):
    예측 Present_Load[‰] ≈ G(q) / stall × 1000
    외력 토크[N·m]     ≈ measured‰ × stall / 1000 − G(q)

듀얼 모터 관절(J2/J3)은 스톨을 합산한다. per-motor 예측은 합산 ‰와 동일
(토크를 반씩 나눠 지므로 각 모터도 stall의 같은 비율로 일한다고 가정).
"""

import math
from pathlib import Path

TICKS_PER_REV = 4096

# sopo 관절 ↔ URDF 관절 ↔ 스톨 토크 [N·m @12V].
# effort가 아니라 스톨 기준: Present_Load(‰)가 스톨 대비 듀티 비율이기 때문.
JOINT_MAP = {
    "J1": {"urdf": "joint_1", "stall": 8.34},  # SM8512BL — 수직축이라 G(q)≈0
    "J2": {"urdf": "joint_2", "stall": 9.81},  # STS3250 x2 (듀얼 합산)
    "J3": {"urdf": "joint_3", "stall": 9.81},  # STS3250 x2
    "J4": {"urdf": "joint_4", "stall": 4.90},  # STS3250 — 롤축이라 G(q)≈0
    "J5": {"urdf": "joint_5", "stall": 2.94},  # STS3215 12V
    "J6": {"urdf": "joint_6", "stall": 2.94},  # STS3215 12V
}

DEFAULT_URDF = Path(__file__).resolve().parents[1] / "description" / "arm_no_ee.urdf"


def ticks_to_rad(ticks: float, zero_ticks: float, direction: int = 1) -> float:
    """모터 틱을 URDF 각도로. zero_ticks는 URDF q=0(수직 직립) 자세의 틱."""
    return direction * (ticks - zero_ticks) * (2 * math.pi / TICKS_PER_REV)


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
