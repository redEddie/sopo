"""토크 기반 외력 관측기 (τ_ext).

잔차: r = LPF( τ_meas_phys − RNEA(q, q̇, 0) − τ_마찰 ).  마찰 항은 0으로 두고 deadband가 커버한다.
정지 시 RNEA는 G(q)로 축소된다. tare()는 현재 잔차를 바이어스로 저장해 이후 잔차에서 차감한다
(듀얼 preload, 마찰, 질량 오차 같은 정류 바이어스 제거용).

순수 로직 — 버스를 만지지 않는다. 매 제어 사이클 update()에 raw 센서값을 넣을 것:
- present_by_motor: 물리 raw 틱 (연속 관절 논리각이 아님 — calibration zero_ticks와 같은 프레임)
- load_by_motor:    Present_Load [‰]
- vel_by_motor:     Present_Velocity [steps/s] (None이면 v=0, 즉 정지 가정으로 평백)
"""

from __future__ import annotations

import math

from .dynamics import GravityCal, GravityModel

STEPS_S_TO_RAD_S = 2 * math.pi / 4096  # Present_Velocity [steps/s] → [rad/s]


class ExternalTorqueEstimator:
    """관절별 외력 토크 [N·m] 추정기. 스레드 안전하지 않음 (GravityModel처럼 루프 스레드에서만 쓸 것)."""

    def __init__(self, model: GravityModel, cal: GravityCal, *,
                 ref_ids: dict[str, int], ema_alpha: float = 0.24, deadband_nm: float = 0.05):
        self.model = model
        self.cal = cal
        self.ref_ids = dict(ref_ids)   # 관절명 → 기준 모터 ID (듀얼은 reference_id)
        self.ema_alpha = ema_alpha     # 1차 LPF 계수 (50 Hz에서 컷오프 ~5 Hz)
        self.deadband_nm = deadband_nm
        self._r = {n: 0.0 for n in model.joint_map}    # EMA 잔차 (바이어스/deadband 적용 전)
        self._bias = {n: 0.0 for n in model.joint_map}  # tare()가 저장한 바이어스

    @property
    def last_residual(self) -> dict[str, float]:
        """최근 EMA 잔차 [N·m] (tare 바이어스·deadband 적용 전). 디버그용."""
        return dict(self._r)

    @property
    def biases(self) -> dict[str, float]:
        """tare()로 저장된 관절별 바이어스 [N·m]."""
        return dict(self._bias)

    def update(self, present_by_motor: dict[int, int], load_by_motor: dict[int, int],
               vel_by_motor: dict[int, int] | None = None) -> dict[str, float]:
        """한 사이클 갱신 → {관절명: 외력 토크 [N·m]} (기준 모터를 읽을 수 있는 관절만)."""
        q_rad: dict[str, float] = {}
        v_rad: dict[str, float] = {}
        for name, ref in self.ref_ids.items():
            pos = present_by_motor.get(ref)
            if pos is None:
                continue
            q_rad[name] = self.cal.q(name, pos)
            if vel_by_motor is not None and ref in vel_by_motor:
                v_rad[name] = self.cal.dir.get(name, 1) * vel_by_motor[ref] * STEPS_S_TO_RAD_S
        if not q_rad:
            return {}
        meas = self.model.measured_torque_nm(load_by_motor)
        dyn = self.model.rnea(q_rad, v_rad)
        out = {}
        for name in q_rad:
            res = meas.get(name, 0.0) / self.cal.scale.get(name, 1.0) - dyn[name]
            r = self._r[name] + self.ema_alpha * (res - self._r[name])
            self._r[name] = r
            val = r - self._bias[name]
            out[name] = 0.0 if abs(val) < self.deadband_nm else val
        return out

    def tare(self) -> None:
        """현재 잔차를 바이어스로 저장 — 이후 출력은 그만큼 차감된다 (0점 맞추기)."""
        self._bias = dict(self._r)
