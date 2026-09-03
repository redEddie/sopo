"""sopo/dynamics.py 오프라인 테스트 — 하드웨어 불필요."""

import math
from pathlib import Path

import pytest

pin = pytest.importorskip("pinocchio", reason="pinocchio 미설치 (pip install pin)")

from sopo.dynamics import JOINT_MAP, GravityModel, ticks_to_rad


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
