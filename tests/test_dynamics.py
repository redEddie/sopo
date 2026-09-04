"""sopo/dynamics.py 오프라인 테스트 — 하드웨어 불필요."""

import math
from pathlib import Path

import pytest

pin = pytest.importorskip("pinocchio", reason="pinocchio 미설치 (pip install pin)")

from sopo.model.dynamics import JOINT_MAP, GravityModel, ticks_to_rad


@pytest.fixture(scope="module")
def gm():
    return GravityModel()


def q(**kwargs):
    base = {f"J{i}": 0.0 for i in range(1, 7)}
    base.update(kwargs)
    return base


def test_ticks_to_rad():
    assert ticks_to_rad(2048, 2048) == 0.0
    assert ticks_to_rad(4096, 2048) == pytest.approx(math.pi)
    assert ticks_to_rad(0, 2048) == pytest.approx(-math.pi)
    assert ticks_to_rad(4096, 2048, -1) == pytest.approx(-math.pi)


def test_zero_pose_gravity(gm):
    """zero pose에서 J1/J6는 0, 손목 캔틸레버 때문에 J2/J3는 음의 토크."""
    g = gm.gravity(q())
    assert g["J1"] == pytest.approx(0.0, abs=1e-6)
    assert g["J6"] == pytest.approx(0.0, abs=1e-6)
    assert g["J2"] < -0.5  # 수평으로 뻗은 손목 구간이 만드는 캔틸레버 토크
    assert g["J4"] == pytest.approx(0.0, abs=0.05)  # 롤축


def test_arm_horizontal(gm):
    """J2를 90° 굽히면 J2 토크가 커지고 J4/J5는 0 근처."""
    g = gm.gravity(q(J2=math.pi / 2))
    assert abs(g["J2"]) > 2.0
    assert abs(g["J2"]) < JOINT_MAP["J2"]["stall"]


def test_matches_pybullet(gm):
    """pinocchio G(q)와 pybullet ID(정지)가 자세들에 대해 일치해야 한다."""
    pybullet = pytest.importorskip("pybullet")
    urdf = Path(__file__).resolve().parents[1] / "description" / "arm_no_ee.urdf"
    pybullet.connect(pybullet.DIRECT)
    pybullet.setGravity(0, 0, -9.81)
    # base_fixed(고정) 관절을 병합해 pybullet의 ID가 6-DOF로 정렬되게 한다
    body = pybullet.loadURDF(str(urdf), useFixedBase=True,
                             flags=pybullet.URDF_MERGE_FIXED_LINKS)
    n = pybullet.getNumJoints(body)
    order = [f"J{i}" for i in range(1, 7)]
    for qdict in (q(), q(J2=0.7, J3=-0.4), q(J2=-1.2, J5=0.8), q(J3=1.4, J4=2.0)):
        g = gm.gravity(qdict)
        qv = [qdict[j] for j in order]
        pb = pybullet.calculateInverseDynamics(body, qv, [0.0] * n, [0.0] * n)
        for i, jname in enumerate(order):
            assert g[jname] == pytest.approx(pb[i], abs=1e-6), (qdict, jname)
    pybullet.disconnect()


def test_load_permille(gm):
    pm = gm.load_permille(q(J2=math.pi / 2))
    g = gm.gravity(q(J2=math.pi / 2))
    for n in g:
        assert pm[n] == pytest.approx(g[n] / JOINT_MAP[n]["stall"] * 1000)


def test_external_torque(gm):
    """측정값이 중력과 정확히 같으면 외력 0, 100‰ 초과분은 stall*0.1 만큼."""
    pose = q(J2=math.pi / 2)
    g = gm.gravity(pose)
    measured = {n: g[n] / JOINT_MAP[n]["stall"] * 1000 for n in g}
    ext = gm.external_torque(pose, measured)
    for n in ext:
        assert ext[n] == pytest.approx(0.0, abs=1e-9)
    measured["J2"] += 100
    ext = gm.external_torque(pose, measured)
    assert ext["J2"] == pytest.approx(JOINT_MAP["J2"]["stall"] * 0.1)


def test_extra_load_torque(gm):
    """J2=90°에서 플랜지에 1kg 매달면 J2에 ~4N·m (레버 약 0.4m)."""
    t = gm.extra_load_torque(q(J2=math.pi / 2), 1.0)
    assert t["J2"] == pytest.approx(3.97, abs=0.3)
    assert t["J1"] == pytest.approx(0.0, abs=1e-6)
    # zero pose(손목 수평)에서도 플랜지는 J2 축에서 0.42m 떨어져 있어 토크가 크다
    t0 = gm.extra_load_torque(q(), 1.0)
    assert t0["J2"] == pytest.approx(4.12, abs=0.3)


def test_scale_applied(gm):
    pose = q(J2=math.pi / 2)
    pm1 = gm.load_permille(pose)
    pm_half = gm.load_permille(pose, {"J2": 0.5})
    assert pm_half["J2"] == pytest.approx(pm1["J2"] * 0.5)
    assert pm_half["J3"] == pytest.approx(pm1["J3"])  # 스케일 없는 관절은 그대로

    g = gm.gravity(pose)
    measured = {n: g[n] / JOINT_MAP[n]["stall"] * 1000 for n in g}
    ext = gm.external_torque(pose, measured, {"J2": 0.5})
    # J2 측정값이 절반 스케일이면 실제 토크는 2배로 환산 → 외력 = +G
    assert ext["J2"] == pytest.approx(g["J2"], rel=1e-6)


def test_measured_torque_nm_dual_sign(gm):
    """듀얼은 mount_sign으로 부호를 맞춘 모터별 기여의 합 (stall은 관절 합산 기준)."""
    stall = JOINT_MAP["J2"]["stall"]
    loads = {10: -128.0, 11: 96.0}
    meas = gm.measured_torque_nm(loads)
    assert meas["J2"] == pytest.approx((1 * -128 + -1 * 96) * (stall / 2) / 1000)
    # single은 해당 모터 값 × stall / 1000 (없는 모터는 0)
    assert meas["J5"] == pytest.approx(0.0)
    loads[20] = 100.0
    meas = gm.measured_torque_nm(loads)
    assert meas["J5"] == pytest.approx(100 * JOINT_MAP["J5"]["stall"] / 1000)


def test_external_torque_from_loads(gm):
    """측정 부하가 중력과 정확히 맞으면 외력 0 (듀얼은 부호 맞춰 균등 분담)."""
    pose = q(J2=math.pi / 2, J3=-0.3)
    g = gm.gravity(pose)
    loads = {}
    for n, spec in JOINT_MAP.items():
        m = g[n] * 1000 / spec["stall"]
        for i, s in zip(spec["ids"], spec["mount_sign"]):
            loads[i] = s * m
    ext = gm.external_torque_from_loads(pose, loads)
    for n in ext:
        assert ext[n] == pytest.approx(0.0, abs=1e-9)
    # J2에 절반 스케일을 주면 측정 토크를 2배로 환산 → 외력 = +G
    ext = gm.external_torque_from_loads(pose, loads, {"J2": 0.5})
    assert ext["J2"] == pytest.approx(g["J2"], rel=1e-6)


def test_make_gravity_model_matches_joint_map():
    """config 로더가 만든 모델의 매핑이 모듈 JOINT_MAP(arm.yaml 유도)과 일치한다."""
    from sopo.config import make_gravity_model
    arm = Path(__file__).resolve().parents[1] / "configs" / "arm.yaml"
    gm2 = make_gravity_model(arm)
    for n, spec in JOINT_MAP.items():
        assert gm2.joint_map[n]["stall"] == pytest.approx(spec["stall"])
        assert gm2.joint_map[n]["ids"] == spec["ids"]
        assert gm2.joint_map[n]["mount_sign"] == spec["mount_sign"]
