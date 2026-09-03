"""description/ 산출물 회귀 테스트 — 하드웨어/Onshape API 없이 돌아간다.

robot.urdf와 arm_no_ee.urdf가 postprocess의 기대 구조를 유지하는지 검사한다.
postprocess.py를 고칠 때마다 여기서 사고를 잡는다.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

DESC = Path(__file__).resolve().parents[1] / "description"

ARM_JOINTS = [f"joint_{i}" for i in range(1, 7)]
FINGER_JOINTS = ["right_finger_joint", "left_finger_joint"]


def parse(name):
    return ET.parse(DESC / name).getroot()


@pytest.fixture(scope="module")
def robot():
    return parse("robot.urdf")


@pytest.fixture(scope="module")
def arm():
    return parse("arm_no_ee.urdf")


def joints_of(root):
    return {j.get("name"): j for j in root.findall("joint")}


def links_of(root):
    return {l.get("name"): l for l in root.findall("link")}


def test_robot_joint_set(robot):
    assert set(joints_of(robot)) == set(ARM_JOINTS + FINGER_JOINTS)


def test_joint6_is_continuous_without_limits(robot):
    j6 = joints_of(robot)["joint_6"]
    assert j6.get("type") == "continuous"
    limit = j6.find("limit")
    assert limit is not None
    assert "lower" not in limit.attrib and "upper" not in limit.attrib


def test_joint_limits_from_cad(robot):
    joints = joints_of(robot)
    expected = {  # Onshape mate 리밋 [rad]
        "joint_1": (-3.1416, 3.1416),
        "joint_2": (-1.65806, 1.65806),
        "joint_3": (-1.5708, 1.5708),
        "joint_4": (-3.1416, 3.1416),
        "joint_5": (-1.74533, 1.74533),
    }
    for name, (lo, hi) in expected.items():
        limit = joints[name].find("limit")
        assert float(limit.get("lower")) == pytest.approx(lo, abs=1e-3), name
        assert float(limit.get("upper")) == pytest.approx(hi, abs=1e-3), name


def test_finger_mimic(robot):
    joints = joints_of(robot)
    mimic = joints["left_finger_joint"].find("mimic")
    assert mimic is not None
    assert mimic.get("joint") == "right_finger_joint"
    assert mimic.get("multiplier") == "1"
    for name in FINGER_JOINTS:  # prismatic ±42mm (CAD 기준)
        joint = joints[name]
        assert joint.get("type") == "prismatic"
        assert float(joint.find("limit").get("lower")) == pytest.approx(-0.042)


def test_motor_effort_velocity(robot):
    joints = joints_of(robot)
    specs = {  # postprocess.py JOINT_SPECS와 동기화
        "joint_1": ("2.7", "6.28"),      # SM8512BL
        "joint_2": ("3.1", "7.85"),      # STS3250 x2
        "joint_3": ("3.1", "7.85"),
        "joint_4": ("1.6", "7.85"),      # STS3250
        "joint_5": ("1.5", "5.236"),     # STS3215
        "joint_6": ("1.5", "5.236"),
    }
    for name, (effort, velocity) in specs.items():
        limit = joints[name].find("limit")
        assert limit.get("effort") == effort, name
        assert limit.get("velocity") == velocity, name


def test_gripper_has_real_mass(robot):
    links = links_of(robot)
    for name in ("ee", "right_finger", "left_finger"):
        mass = float(links[name].find("inertial/mass").get("value"))
        assert mass > 0.01, name  # 플레이스홀더(1e-9)가 아니어야 함


def test_arm_variant_structure(arm):
    joints = joints_of(arm)
    links = links_of(arm)
    assert set(joints) == set(ARM_JOINTS)
    assert "flange" in links                      # Franka link8 컨벤션
    assert joints["joint_6"].find("child").get("link") == "flange"
    for name in ("ee", "right_finger", "left_finger"):
        assert name not in links


def test_mesh_files_exist(robot, arm):
    for root in (robot, arm):
        for mesh in root.iter("mesh"):
            path = DESC / mesh.get("filename")
            assert path.exists(), path
            assert not mesh.get("filename", "").startswith("package://")


def test_collision_meshes_split(robot):
    """축소본이 있는 파트의 충돌 geom은 *_collision.stl을 가리켜야 한다 (면 수 제한 회피).
    원래 충분히 가벼운 파트(예: sopo_desktop_base)는 원본 그대로 둔다."""
    for collision in robot.iter("collision"):
        mesh = collision.find("geometry/mesh")
        assert mesh is not None
        filename = mesh.get("filename")
        assert (DESC / filename).exists(), filename
        stem = Path(filename).stem
        has_split = (DESC / "assets" / f"{stem}_collision.stl").exists()
        if has_split and not stem.endswith("_collision"):
            raise AssertionError(f"{filename}: 축소본이 있는데 원본을 충돌용으로 사용 중")


def test_viewer_xml_exists_and_matches():
    xml = DESC / "sopo_viewer.xml"
    assert xml.exists()
    root = ET.parse(xml).getroot()
    names = {j.get("name") for j in root.iter("joint")}
    assert set(ARM_JOINTS) <= names
    actuators = {a.get("name") for a in root.iter("position")}
    assert actuators == {f"act_{n}" for n in names if n}


def test_combine_masses_physics():
    """쉘+모터 합성: 질량중심 가중평균과 평행축 정리."""
    import sys
    sys.path.insert(0, str(DESC))
    import numpy as np
    from postprocess import combine_masses

    # 모터 없음: 쉘 그대로
    m, com, I = combine_masses(1.0, [0, 0, 0], np.eye(3) * 0.01, [])
    assert m == 1.0
    assert np.allclose(com, [0, 0, 0])
    assert np.allclose(I, np.eye(3) * 0.01)

    # 쉘 1kg@원점 + 모터 0.5kg@(0.1,0,0)
    m, com, I = combine_masses(1.0, [0, 0, 0], np.eye(3) * 0.01,
                               [{"mass": 0.5, "xyz": [0.1, 0, 0]}])
    assert m == 1.5
    assert np.allclose(com, [1 / 30, 0, 0])
    # x축 방향 오프셋은 iyy/izz에만 기여 (평행축 정리)
    d_shell = 1 / 30
    d_motor = 0.1 - 1 / 30
    assert I[1, 1] == pytest.approx(0.01 + 1.0 * d_shell**2 + 0.5 * d_motor**2)
    assert I[0, 0] == pytest.approx(0.01)  # ixx는 불변


def test_parts_format_masses(tmp_path, monkeypatch):
    """parts 형식: link_2(파이프+홀더+듀얼모터)에 파트별 무게를 넣으면 합성된다."""
    import sys
    sys.path.insert(0, str(DESC))
    import postprocess

    yaml_file = tmp_path / "m.yaml"
    yaml_file.write_text(
        "link_2:\n"
        "  parts: {link_2_pipe: 0.23, link_2_holder: 0.12, link_2_holder__2: 0.12,\n"
        "          joint_3_simple_sts3250: 0.0745, joint_3_simple_sts3250__2: 0.0745}\n"
    )
    monkeypatch.setattr(postprocess, "MASSES_YAML", yaml_file)

    root = parse("robot.urdf")
    msgs = postprocess.inject_link_dynamics(root)
    link = {l.get("name"): l for l in root.findall("link")}["link_2"]
    mass = float(link.find("inertial/mass").get("value"))
    assert mass == 0.23 + 0.12 + 0.12 + 0.0745 + 0.0745
    assert any("link_2" in m for m in msgs)


def test_parts_format_rejects_unknown_part(tmp_path, monkeypatch):
    import sys
    sys.path.insert(0, str(DESC))
    import postprocess

    yaml_file = tmp_path / "m.yaml"
    yaml_file.write_text("link_2:\n  parts: {nonexistent: 0.1}\n")
    monkeypatch.setattr(postprocess, "MASSES_YAML", yaml_file)
    with pytest.raises(KeyError):
        postprocess.inject_link_dynamics(parse("robot.urdf"))
