"""sopo/model/estimation.py 외력 관측기 테스트 — 하드웨어 불필요 (합성 센서값)."""

import math

import pytest

pin = pytest.importorskip("pinocchio", reason="pinocchio 미설치 (pip install pin)")

from sopo.model.dynamics import JOINT_MAP, GravityCal, GravityModel
from sopo.model.estimation import STEPS_S_TO_RAD_S, ExternalTorqueEstimator

NAMES = [f"J{i}" for i in range(1, 7)]
REF_IDS = {n: JOINT_MAP[n]["ids"][-1] for n in NAMES}  # 듀얼은 ids[1]이 reference


@pytest.fixture(scope="module")
def gm():
    return GravityModel()


def make_cal(**kw):
    base = dict(zero_ticks={n: 2048 for n in NAMES})
    base.update(kw)
    return GravityCal(**base)


def synth(gm, q, ext=None, scale=None):
    """자세 q에서 중력 + ext(관절별 N·m)에 해당하는 합성 raw 센서값 (위치 틱, 부하 ‰).

    듀티 ≈ scale × 물리토크 / stall 이므로 raw ‰ = sign_i × scale × (G + ext) × 1000 / stall.
    """
    ext = ext or {}
    scale = scale or {}
    g = gm.gravity(q)
    present, loads = {}, {}
    for n in NAMES:
        spec = JOINT_MAP[n]
        ticks = round(2048 + q[n] / (2 * math.pi / 4096))
        for i in spec["ids"]:
            present[i] = ticks
        m = scale.get(n, 1.0) * (g[n] + ext.get(n, 0.0)) * 1000 / spec["stall"]
        for i, s in zip(spec["ids"], spec["mount_sign"]):
            loads[i] = s * m
    return present, loads


def rest_q(**kw):
    q = {n: 0.0 for n in NAMES}
    q.update(kw)
    return q


def test_residual_extraction(gm):
    """J2에 0.5 N·m 외력 → EMA 수렴 후 그 값이 잔차로 나온다."""
    q = rest_q(J2=math.pi / 2)
    present, loads = synth(gm, q, ext={"J2": 0.5})
    est = ExternalTorqueEstimator(gm, make_cal(), ref_ids=REF_IDS)
    for _ in range(30):
        out = est.update(present, loads)
    assert out["J2"] == pytest.approx(0.5, abs=0.01)
    assert out["J5"] == pytest.approx(0.0, abs=1e-6)


def test_ema_convergence(gm):
    """1차 LPF: k 사이클 후 잔차 = 목표 × (1 − (1−α)^k)."""
    present, loads = synth(gm, rest_q(), ext={"J5": 0.3})
    est = ExternalTorqueEstimator(gm, make_cal(), ref_ids=REF_IDS)
    out = est.update(present, loads)
    assert out["J5"] == pytest.approx(0.3 * 0.24, abs=1e-9)  # 첫 사이클은 α 배
    for _ in range(29):
        out = est.update(present, loads)
    assert out["J5"] == pytest.approx(0.3 * (1 - (1 - 0.24) ** 30), abs=1e-9)


def test_deadband(gm):
    """deadband 미만 잔차는 0으로 클램프된다."""
    present, loads = synth(gm, rest_q(), ext={"J5": 0.03})
    est = ExternalTorqueEstimator(gm, make_cal(), ref_ids=REF_IDS, deadband_nm=0.05)
    for _ in range(50):
        out = est.update(present, loads)
    assert out["J5"] == 0.0


def test_tare_removes_bias(gm):
    """tare 후 같은 잔차는 0이 되고, 그 위에 얹힌 추가 외력만 보인다."""
    q = rest_q()
    present, loads = synth(gm, q, ext={"J2": 0.4})  # preload/마찰에 해당하는 정류 바이어스
    est = ExternalTorqueEstimator(gm, make_cal(), ref_ids=REF_IDS)
    for _ in range(50):
        est.update(present, loads)
    est.tare()
    assert est.update(present, loads)["J2"] == pytest.approx(0.0, abs=0.02)
    assert est.biases["J2"] == pytest.approx(0.4, abs=0.02)
    present2, loads2 = synth(gm, q, ext={"J2": 0.7})  # 바이어스 0.4 + 진짜 외력 0.3
    for _ in range(50):
        out = est.update(present2, loads2)
    assert out["J2"] == pytest.approx(0.3, abs=0.02)


def test_dual_mount_sign_composition(gm):
    """듀얼 미러는 반전 장착: 같은 부호 부하는 상쇄, 반대 부호 부하는 합산된다."""
    present, loads = synth(gm, rest_q())  # 중력만
    est = ExternalTorqueEstimator(gm, make_cal(), ref_ids=REF_IDS)
    for _ in range(50):
        out = est.update(present, loads)
    assert out["J2"] == 0.0  # 중력만 → 잔차 0 (deadband로 정확히 0)

    stall = JOINT_MAP["J2"]["stall"]
    same = dict(loads)
    same[10] += 100
    same[11] += 100  # 같은 부호 → mount_sign 합성에서 상쇄
    for _ in range(50):
        out = est.update(present, same)
    assert out["J2"] == 0.0

    opposed = dict(loads)
    opposed[10] += 100
    opposed[11] -= 100  # 반대 부호 → 합산: (100+100) × (stall/2) / 1000
    for _ in range(50):
        out = est.update(present, opposed)
    assert out["J2"] == pytest.approx(200 * (stall / 2) / 1000, rel=1e-3)


def test_scale_applied(gm):
    """scale 관절은 측정 토크를 그 계수로 나눠 물리 토크로 환산한다."""
    q = rest_q(J2=math.pi / 2)
    cal = make_cal(scale={"J2": 0.5})
    present, loads = synth(gm, q, ext={"J2": 0.5}, scale={"J2": 0.5})
    est = ExternalTorqueEstimator(gm, cal, ref_ids=REF_IDS)
    for _ in range(50):
        out = est.update(present, loads)
    assert out["J2"] == pytest.approx(0.5, abs=0.01)


def test_velocity_none_falls_back_to_rest(gm):
    """vel_by_motor=None은 v=0(정지 가정)과 동일. 회전 중엔 RNEA 속도 항이 잔차에 반영된다."""
    q = rest_q(J2=math.pi / 2)
    present, loads = synth(gm, q)  # 부하는 중력만 반영
    est = ExternalTorqueEstimator(gm, make_cal(), ref_ids=REF_IDS)
    for _ in range(50):
        out_none = est.update(present, loads, None)
    est2 = ExternalTorqueEstimator(gm, make_cal(), ref_ids=REF_IDS)
    vel0 = {i: 0 for i in present}
    for _ in range(50):
        out_zero = est2.update(present, loads, vel0)
    assert out_none == out_zero  # None 폴백은 v=0과 동일

    # J2를 돌리는 중(부하는 중력만 반영된 합성) → 코리올리 항만큼 J3에 잔차가 생겨야 한다
    steps = round(2.0 / STEPS_S_TO_RAD_S)
    vel = dict(vel0)
    vel[REF_IDS["J2"]] = steps
    est3 = ExternalTorqueEstimator(gm, make_cal(), ref_ids=REF_IDS)
    for _ in range(50):
        out = est3.update(present, loads, vel)
    v_rad = {n: 0.0 for n in NAMES} | {"J2": steps * STEPS_S_TO_RAD_S}
    g = gm.gravity(q)
    r = gm.rnea(q, v_rad)
    assert out["J3"] == pytest.approx(g["J3"] - r["J3"], abs=0.01)


def test_missing_ref_joint_is_skipped(gm):
    """기준 모터 읽기가 없는 관절은 출력에서 빠진다 (부분 데이터로 죽지 않음)."""
    present, loads = synth(gm, rest_q())
    est = ExternalTorqueEstimator(gm, make_cal(), ref_ids=REF_IDS)
    partial = {k: v for k, v in present.items() if k != REF_IDS["J2"]}
    out = est.update(partial, loads)
    assert "J2" not in out and "J5" in out
