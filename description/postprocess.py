#!/usr/bin/env python3
"""onshape-to-robot export 후처리.

config.json의 post_import_commands에 등록되어 export 직후 자동 실행된다.

하는 일:
1. 6번째 축(플랜지 회전, STS3215 구동)을 continuous 타입으로 변환한다.
   onshape-to-robot은 리밋 없는 revolute를 +/-pi로 export하고 continuous를
   지원하지 않기 때문. 케이블이 지나가지 않아 무한 회전이 가능한 축이다.
2. 메시 경로의 package:// 접두사를 제거한다. ROS 패키지 컨텍스트 없이
   pybullet/MuJoCo에서도 로드되도록 상대경로로 만든다.
3. 메시 이원화: 시각용 STL은 MuJoCo 상한(200000면)을 넘을 때만 가볍게 축소하고,
   충돌용은 *_collision.stl로 크게 축소한 뒤 URDF의 collision이 그것을 가리키게 한다.
4. 핑거 구조: left_finger_joint가 right_finger_joint를 mimic하도록 추가한다
   (robonine SO-101 parallel gripper와 같은 컨셉. 두 조인트의 축이 CAD에서 이미
   반대 방향으로 잡혀 있어 multiplier는 +1). 리밋은 CAD의 +/-0.042m를 그대로 쓴다.
5. 그리퍼 링크(ee, 핑거)에 질량/관성 추정치를 넣는다. 수치는 robonine
   SO-ARM100/101 Parallel-Gripper URDF의 실측값에서 가져오되, COM/관성텐서는
   각 링크의 메시에서 재계산해 현재 프레임에 맞춘다. (실측 저울 교정 예정)
6. effort/velocity 플레이스홀더(10, 10)를 STS3215 실사양으로 교체한다.
7. ee/핑거를 잘라낸 arm_no_ee.urdf를 생성한다. 팔은 빈 flange 프레임 링크로
   끝난다 (Franka FR3의 link8 컨벤션). 제어/보상 코드는 이 파일을 기준으로 삼는다.
8. MuJoCo 뷰어용 sopo_viewer.xml을 생성한다. 중력/접촉 off, 더미 질량 보정,
   position actuator 추가, 충돌 geom 제거(저면수 충돌 메시가 렌더링에 섞여
   모델이 두껍게 보이는 문제 방지).
"""

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
URDF = HERE / "robot.urdf"
ARM_URDF = HERE / "arm_no_ee.urdf"
ASSETS = HERE / "assets"
VIEWER_XML = HERE / "sopo_viewer.xml"

# MuJoCo는 200000면 이상의 STL을 거부한다
VISUAL_MAX_FACES = 150000
COLLISION_FACES = 2000

# 6번째 축(플랜지 회전, STS3215 12V 구동). 케이블이 지나가지 않아 무한 회전 가능.
# export 소스 이름(바뀔 수 있어 모두 허용) -> 최종 이름은 joint_6.
JOINT6_SOURCES = ("joint_6", "J6", "joint_6_passive")
JOINT6_NAME = "joint_6"

# 핑거 mimic: left가 right를 따른다 (축이 반대라 multiplier +1)
FINGER_DRIVING_JOINT = "right_finger_joint"
FINGER_DRIVEN_JOINT = "left_finger_joint"

# 그리퍼 링크 질량 추정치 [kg]. robonine SO-ARM100/101 Parallel-Gripper URDF의
# 실측값(gripper_base 0.556, clamp 0.154). TODO: 실물 저울 측정으로 교체.
GRIPPER_MASS = {"ee": 0.556, "right_finger": 0.154, "left_finger": 0.154}

# 관절별 동작 사양: (effort [N·m 또는 N], velocity [rad/s 또는 m/s]).
# effort는 스톨이 아니라 정격(연속) 토크 기준. export 플레이스홀더(10/10) 대체.
#   joint_1: SM8512BL — 정격 28kg·cm, 60RPM (스톨 85kg·cm)
#   joint_2/3: STS3250 듀얼 — 정격 16kg·cm x2, 75RPM
#   joint_4: STS3250 단일
#   joint_5/6 + 그리퍼: STS3215 12V (robonine URDF 참조값)
JOINT_SPECS = {
    "joint_1": (2.7, 6.28),
    "joint_2": (3.1, 7.85),
    "joint_3": (3.1, 7.85),
    "joint_4": (1.6, 7.85),
    "joint_5": (1.5, 5.236),
    JOINT6_NAME: (1.5, 5.236),
    "right_finger_joint": (1.5, 0.05),
    "left_finger_joint": (1.5, 0.05),
}

# arm_no_ee.urdf에서 잘라낼 링크: Onshape CAD의 그리퍼 관련 파트 전부.
HAND_LINKS = ("ee", "right_finger", "left_finger")


def rpy_matrix(rpy: tuple[float, float, float]):
    import numpy as np

    rx, ry, rz = rpy
    Rx = np.array([[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]])
    Ry = np.array([[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]])
    Rz = np.array([[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx  # URDF rpy는 고정축 X->Y->Z 순


def split_mesh(stl: Path) -> tuple[list[str], str | None]:
    """시각/충돌 메시를 이원화한다. (로그 메시지들, 충돌용 파일명 또는 None)을 반환."""
    import trimesh

    mesh = trimesh.load(stl)
    faces = len(mesh.faces)
    if faces <= COLLISION_FACES:
        return [], None

    msgs = []
    collision = mesh.simplify_quadric_decimation(face_count=COLLISION_FACES)
    collision_name = f"{stl.stem}_collision.stl"
    collision.export(stl.with_name(collision_name))
    msgs.append(f"{stl.name}: collision -> {collision_name} ({len(collision.faces)} faces)")

    if faces > VISUAL_MAX_FACES:
        visual = mesh.simplify_quadric_decimation(face_count=VISUAL_MAX_FACES)
        visual.export(stl)
        msgs.append(f"{stl.name}: visual {faces} -> {len(visual.faces)} faces")
    return msgs, collision_name


def inject_gripper_dynamics(root: ET.Element) -> list[str]:
    """그리퍼 링크에 질량/관성을 넣는다. COM과 관성텐서는 visual 메시에서 계산."""
    import numpy as np
    import trimesh

    changed = []
    for link in root.findall("link"):
        name = link.get("name")
        if name not in GRIPPER_MASS:
            continue
        visual = link.find("visual")
        mesh_tag = visual.find("geometry/mesh")
        origin = visual.find("origin")
        xyz = tuple(float(v) for v in origin.get("xyz", "0 0 0").split())
        rpy = tuple(float(v) for v in origin.get("rpy", "0 0 0").split())

        mesh = trimesh.load(HERE / mesh_tag.get("filename"))
        props = mesh.mass_properties  # 밀도=1 기준
        target = GRIPPER_MASS[name]
        ratio = target / props["mass"]  # 같은 형상에서 관성은 질량에 비례

        R = rpy_matrix(rpy)
        com = R @ props["center_mass"] + np.array(xyz)  # 링크 프레임 기준 COM
        inertia = R @ (props["inertia"] * ratio) @ R.T  # COM 기준 텐서, 링크 축 정렬

        old = link.find("inertial")
        if old is not None:
            link.remove(old)
        inertial = ET.Element("inertial")
        origin_el = ET.SubElement(inertial, "origin")
        origin_el.set("xyz", " ".join(f"{v:.6g}" for v in com))
        origin_el.set("rpy", "0 0 0")
        ET.SubElement(inertial, "mass").set("value", f"{target:.6g}")
        keys = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
        values = (inertia[0, 0], inertia[0, 1], inertia[0, 2],
                  inertia[1, 1], inertia[1, 2], inertia[2, 2])
        ET.SubElement(inertial, "inertia").attrib = {
            k: f"{v:.6g}" for k, v in zip(keys, values)}
        link.insert(list(link).index(visual), inertial)
        changed.append(f"{name}: mass={target}kg, COM=({com[0]:.4f},{com[1]:.4f},{com[2]:.4f})")
    return changed


def tune_joints(root: ET.Element) -> list[str]:
    """핑거 mimic 추가 + effort/velocity를 모터별 실사양으로 교체."""
    changed = []
    joints = {j.get("name"): j for j in root.findall("joint")}

    driven = joints.get(FINGER_DRIVEN_JOINT)
    if driven is not None and driven.find("mimic") is None:
        mimic = ET.SubElement(driven, "mimic")
        mimic.set("joint", FINGER_DRIVING_JOINT)
        mimic.set("multiplier", "1")  # 축이 CAD에서 반대로 잡혀 있어 +1
        mimic.set("offset", "0")
        changed.append(f"{FINGER_DRIVEN_JOINT} now mimics {FINGER_DRIVING_JOINT} (x1)")

    for joint in root.findall("joint"):
        name = joint.get("name")
        if name not in JOINT_SPECS:
            continue
        effort, velocity = JOINT_SPECS[name]
        limit = joint.find("limit")
        if limit is not None:
            limit.set("effort", str(effort))
            limit.set("velocity", str(velocity))
            changed.append(f"{name}: effort={effort}, velocity={velocity}")
    return changed


def build_arm_only(root: ET.Element) -> str:
    """그리퍼 관련 링크를 제거한 팔 전용 URDF(6-DOF)를 만들어 파일명을 반환한다.

    joint_6의 child를 빈 flange 링크로 바꿔 팔이 플랜지 프레임에서
    끝나게 한다 (Franka 컨벤션). 핸드는 flange에 fixed joint로 결합한다.
    """
    import copy

    arm = copy.deepcopy(root)
    for joint in list(arm.findall("joint")):
        child = joint.find("child")
        if child is None or child.get("link") not in HAND_LINKS:
            continue
        if joint.get("name") == JOINT6_NAME:
            child.set("link", "flange")
        else:
            arm.remove(joint)
    for link in list(arm.findall("link")):
        if link.get("name") in HAND_LINKS:
            arm.remove(link)

    # flange: 프레임 전용 빈 링크 (질량 플레이스홀더는 export 기본값과 동일하게)
    flange = ET.SubElement(arm, "link")
    flange.set("name", "flange")
    inertial = ET.SubElement(flange, "inertial")
    ET.SubElement(inertial, "origin").set("xyz", "0 0 0")
    ET.SubElement(inertial, "mass").set("value", "1e-09")
    ET.SubElement(inertial, "inertia").attrib = {
        "ixx": "1e-09", "ixy": "0", "ixz": "0",
        "iyy": "1e-09", "iyz": "0", "izz": "1e-09",
    }

    ET.indent(arm, space="  ")
    ET.ElementTree(arm).write(ARM_URDF, encoding="utf-8", xml_declaration=True)
    return ARM_URDF.name


def build_viewer_xml() -> str:
    """MuJoCo simulate 뷰어용 MJCF를 만들어 파일명을 반환한다."""
    import mujoco

    # 접촉은 끈다 — 포징 뷰어에 자기충돌은 방해만 된다.
    # (visual geom 보존은 URDF에 심은 <mujoco> compiler discardvisual=false가 담당)
    spec = mujoco.MjSpec.from_file(str(URDF))
    spec.option.disableflags |= mujoco.mjtDisableBit.mjDSBL_GRAVITY | mujoco.mjtDisableBit.mjDSBL_CONTACT

    xml_string = spec.to_xml()
    tree = ET.ElementTree(ET.fromstring(xml_string))
    root = tree.getroot()

    # 충돌 geom 제거: 저면수 충돌 메시(얇은 벽이 두꺼워지는 부작용)가
    # simulate에 같이 렌더링되는 것을 막는다. 접촉은 이미 꺼져 있으므로 무해.
    removed = 0
    for parent in root.iter():
        for geom in list(parent.findall("geom")):
            if geom.get("contype") != "0":
                parent.remove(geom)
                removed += 1

    # URDF의 질량 플레이스홀더(1e-9)로는 물리 스텝이 불안정하니 뷰어용 더미 질량
    for inertial in root.iter("inertial"):
        if float(inertial.get("mass", "0")) < 1e-6:
            inertial.set("mass", "0.1")
            inertial.set("diaginertia", "1e-4 1e-4 1e-4")

    # position actuator를 달면 simulate의 Control 패널에 슬라이더가 생긴다
    actuator = ET.SubElement(root, "actuator")
    for joint in root.iter("joint"):
        name = joint.get("name")
        if not name:
            continue
        joint.set("damping", "1")
        position = ET.SubElement(actuator, "position")
        position.set("name", f"act_{name}")
        position.set("joint", name)
        position.set("kp", "20")
        position.set("ctrlrange", joint.get("range") or "-3.14159 3.14159")

    ET.indent(tree, space="  ")
    tree.write(VIEWER_XML, encoding="utf-8", xml_declaration=True)
    return f"{VIEWER_XML.name} (collision geom {removed}개 제거됨)"


def main() -> int:
    tree = ET.parse(URDF)
    root = tree.getroot()
    changed = []

    for joint in root.findall("joint"):
        name = joint.get("name")
        if name in JOINT6_SOURCES:
            joint.set("name", JOINT6_NAME)
            joint.set("type", "continuous")
            limit = joint.find("limit")
            if limit is not None:
                limit.attrib.pop("lower", None)
                limit.attrib.pop("upper", None)
            changed.append(f"joint {name!r} -> {JOINT6_NAME!r} (continuous, no limits)")
        elif name == JOINT6_NAME and joint.get("type") == "continuous":
            changed.append(f"joint {name!r} already continuous")  # 재실행 안전

    if not any(JOINT6_NAME in c for c in changed):
        print(f"[postprocess] ERROR: joint_6 {JOINT6_SOURCES} not found", file=sys.stderr)
        return 1

    mesh_count = 0
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename", "")
        if filename.startswith("package://"):
            mesh.set("filename", filename[len("package://"):])
            mesh_count += 1
    if mesh_count:
        changed.append(f"mesh paths: stripped package:// from {mesh_count} entries")

    # MuJoCo가 URDF의 visual geom을 버리지 않도록 확장 블록을 심는다 (다른 파서는 무시)
    if root.find("mujoco") is None:
        mujoco_ext = ET.SubElement(root, "mujoco")
        ET.SubElement(mujoco_ext, "compiler").set("discardvisual", "false")
        changed.append("added <mujoco> compiler discardvisual=false")

    collision_files = {}
    for stl in sorted(ASSETS.glob("*.stl")):
        if stl.stem.endswith("_collision"):
            continue
        msgs, collision_name = split_mesh(stl)
        changed.extend(msgs)
        if collision_name:
            collision_files[stl.stem] = collision_name

    # 충돌 태그가 축소본을 가리키게 한다
    for collision in root.iter("collision"):
        mesh = collision.find("geometry/mesh")
        if mesh is None:
            continue
        stem = Path(mesh.get("filename", "")).stem
        if stem in collision_files:
            mesh.set("filename", f"assets/{collision_files[stem]}")

    changed.extend(tune_joints(root))
    changed.extend(inject_gripper_dynamics(root))

    ET.indent(tree, space="  ")
    tree.write(URDF, encoding="utf-8", xml_declaration=True)

    changed.append(f"arm-only -> {build_arm_only(root)}")
    changed.append(f"viewer model -> {build_viewer_xml()}")

    for line in changed:
        print(f"[postprocess] {line}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
