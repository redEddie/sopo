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
5. link_masses.yaml의 실측 질량을 각 링크에 주입한다. COM/관성텐서는
   visual 메시(여러 파트면 합쳐서)를 균일 밀도로 계산한 뒤 실측 질량으로
   스케일한다. 질량이 null인 링크는 플레이스홀더 유지.
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

# Onshape mate/파트명이 갱신돼도 표준 이름으로 정규화한다.
JOINT_RENAMES = {"joint_2_main": "joint_2", "joint_3_main": "joint_3"}
LINK_RENAMES = {"link1": "link_1", "link_2_holder": "link_2", "simple_6704": "link_4"}

# REP-103 축 정규화: 이 조인트들의 축을 뒤집는다 (axis "0 0 1" -> "0 0 -1",
# 리밋 부호 교환). 물리 기하는 불변이고 +q의 방향 규약만 바뀐다.
#   joint_1: yaw 축이 아래(-z)였던 것을 위로 — +q1 = 위에서 봐서 반시계
#   joint_4/6: roll 축을 어프로치 방향으로 — +q = 어프로치 방향 오른손 법칙
#   joint_5: J2/J3과 반대였던 pitch를 통일 — +q = 전방으로 기울기
AXIS_FLIP_JOINTS = ("joint_1", "joint_4", "joint_5", "joint_6")

# 베이스 프레임: 어프로치 방향이 +y로 export되므로, -90도 z회전한 base_link를
# 새 루트로 얹어 REP-103(X=전방, Y=좌, Z=상)을 만족시킨다.
BASE_LINK = "base_link"
BASE_RPY = "0 0 -1.57079633"  # sopo_desktop_base를 base_link 기준 이 각도로
CURRENT_ROOT = "sopo_desktop_base"

# 핑거 mimic: left가 right를 따른다 (축이 반대라 multiplier +1)
FINGER_DRIVING_JOINT = "right_finger_joint"
FINGER_DRIVEN_JOINT = "left_finger_joint"

# 링크별 질량 [kg]은 link_masses.yaml이 소스. null이면 플레이스홀더 유지.
# COM/관성텐서는 메시 기하에서 균일 밀도로 자동 계산해 주입한다.
MASSES_YAML = HERE / "link_masses.yaml"

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


def combine_entries(entries):
    """(질량, COM, 관성텐서@COM 또는 None=점질량) 목록을 평행축 정리로 합성한다.

    반환: (총질량, COM, 관성텐서@COM). 순수 함수라 단위 테스트 가능.
    """
    import numpy as np

    entries = [(m, np.asarray(c), np.zeros((3, 3)) if I is None else np.asarray(I))
               for m, c, I in entries]
    total = sum(m for m, _, _ in entries)
    com = sum(m * c for m, c, _ in entries) / total
    inertia = np.zeros((3, 3))
    for m, c, I_own in entries:
        d = c - com
        inertia += I_own + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return total, com, inertia


def combine_masses(shell_mass, shell_com, shell_inertia, motors):
    """쉘(관성텐서 보유) + 모터 점질량들을 합성해 (총질량, COM, 관성텐서@COM)을 반환."""
    entries = [(shell_mass, shell_com, shell_inertia)]
    entries += [(m["mass"], m["xyz"], None) for m in motors]
    return combine_entries(entries)


def inject_link_dynamics(root: ET.Element) -> list[str]:
    """link_masses.yaml의 질량 정보를 링크에 주입한다.

    형식 (link_masses.yaml 참고):
      - 숫자: 총질량. COM/관성은 메시 균일 밀도로 계산.
      - {shell: kg, motors: [...]}: 쉘(메시 균일) + 모터(점질량) 합성.
      - {parts: {메시명: kg}, motors: [...]}: 파트별 실측 무게. 카본 파이프 +
        출력물처럼 재질이 섞인 링크에서 정확하다. 각 파트의 COM/관성은 자기
        메시에서 계산하고 실측 무게로 스케일한다.
    """
    import numpy as np
    import trimesh
    import yaml

    masses = {k: v for k, v in yaml.safe_load(MASSES_YAML.read_text()).items() if v}
    changed = []
    for link in root.findall("link"):
        name = link.get("name")
        if name not in masses:
            continue

        # 한 링크가 여러 파트(visual)를 가진다 — 파트별로 링크 프레임 변환 후 보관
        parts = []  # (메시 스템, 변환된 메시, mass_properties)
        for visual in link.findall("visual"):
            mesh_tag = visual.find("geometry/mesh")
            if mesh_tag is None:
                continue
            origin = visual.find("origin")
            xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
            rpy = tuple(float(v) for v in origin.get("rpy", "0 0 0").split())
            part = trimesh.load(HERE / mesh_tag.get("filename"))
            T = np.eye(4)
            T[:3, :3] = rpy_matrix(rpy)
            T[:3, 3] = xyz
            part.apply_transform(T)  # 링크 프레임으로
            parts.append((Path(mesh_tag.get("filename")).stem, part, part.mass_properties))
        if not parts:
            continue

        spec = masses[name]
        motors = spec.get("motors") or [] if isinstance(spec, dict) else []

        if isinstance(spec, dict) and "parts" in spec:
            # 파트별 실측: 메시 스템명으로 매칭. 오탈자 방지를 위해 엄격 검사.
            entries = [(m["mass"], m["xyz"], None) for m in motors]
            for stem, part, props in parts:
                part_mass = spec["parts"].get(stem)
                if part_mass is None:
                    print(f"[postprocess] WARNING: {name}의 파트 {stem}에 무게가 없음, 건너뜀", file=sys.stderr)
                    continue
                entries.append((float(part_mass), props["center_mass"],
                                props["inertia"] * (float(part_mass) / props["mass"])))
            unknown = set(spec["parts"]) - {stem for stem, _, _ in parts}
            if unknown:
                raise KeyError(f"{name}: yaml에 있는데 URDF에 없는 파트: {unknown}")
            target, com, inertia = combine_entries(entries)
        else:
            merged = trimesh.util.concatenate([part for _, part, _ in parts])
            props = merged.mass_properties  # 밀도=1 기준, 링크 프레임 기준
            if isinstance(spec, dict):  # 쉘 균일 + 모터
                shell_mass = float(spec["shell"])
                target, com, inertia = combine_masses(
                    shell_mass, props["center_mass"],
                    props["inertia"] * (shell_mass / props["mass"]), motors)
            else:  # 총질량만
                target = float(spec)
                inertia = props["inertia"] * (target / props["mass"])  # 관성은 질량에 비례
                com = props["center_mass"]

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
        first_visual = link.find("visual")
        link.insert(list(link).index(first_visual), inertial)
        changed.append(f"{name}: mass={target:.4g}kg, COM=({com[0]:.4f},{com[1]:.4f},{com[2]:.4f})")
    return changed


def normalize_frames(root: ET.Element) -> list[str]:
    """REP-103 규약으로 프레임을 정규화한다: 조인트 축 정규화 + base_link 루트."""
    changed = []
    for joint in root.findall("joint"):
        if joint.get("name") not in AXIS_FLIP_JOINTS:
            continue
        axis = joint.find("axis")
        if axis is None or axis.get("xyz") != "0 0 1":
            continue  # 이미 뒤집혔거나 예상과 다른 축 — 재실행 안전
        axis.set("xyz", "0 0 -1")
        limit = joint.find("limit")
        if limit is not None and limit.get("lower") is not None:
            lo = float(limit.get("lower"))
            hi = float(limit.get("upper"))
            limit.set("lower", f"{-hi:.12g}")
            limit.set("upper", f"{-lo:.12g}")
        changed.append(f"{joint.get('name')}: axis flipped (+q 방향 규약)")

    if not any(l.get("name") == BASE_LINK for l in root.findall("link")):
        base = ET.Element("link")
        base.set("name", BASE_LINK)
        inertial = ET.SubElement(base, "inertial")  # 파서 경고 방지 플레이스홀더
        ET.SubElement(inertial, "origin").set("xyz", "0 0 0")
        ET.SubElement(inertial, "mass").set("value", "1e-09")
        ET.SubElement(inertial, "inertia").attrib = {
            "ixx": "1e-09", "ixy": "0", "ixz": "0",
            "iyy": "1e-09", "iyz": "0", "izz": "1e-09",
        }
        root.insert(0, base)
        fixed = ET.SubElement(root, "joint")
        fixed.set("name", "base_fixed")
        fixed.set("type", "fixed")
        ET.SubElement(fixed, "origin").set("xyz", "0 0 0")
        fixed.find("origin").set("rpy", BASE_RPY)
        ET.SubElement(fixed, "parent").set("link", BASE_LINK)
        ET.SubElement(fixed, "child").set("link", CURRENT_ROOT)
        changed.append(f"added {BASE_LINK} root (어프로치 = +x, REP-103)")
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

    # position actuator를 달면 simulate의 Control 패널에 슬라이더가 생긴다.
    # 포징 전용이므로 실물 스펙을 버린다: 힘 제한(actuatorfrcrange/forcerange)을
    # 없애고 kp를 크게 — 그래야 슬라이더에 즉시 따라온다 (달랑거림 방지).
    actuator = ET.SubElement(root, "actuator")
    for joint in root.iter("joint"):
        name = joint.get("name")
        if not name:
            continue
        joint.attrib.pop("actuatorfrcrange", None)
        joint.set("damping", "20")
        position = ET.SubElement(actuator, "position")
        position.set("name", f"act_{name}")
        position.set("joint", name)
        position.set("kp", "2000")
        position.set("forcerange", "-100000 100000")
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

    # 조인트/링크 이름 정규화 (export 이름이 달라져도 하류 스펙/yaml 키를 안정화)
    for joint in root.findall("joint"):
        if joint.get("name") in JOINT_RENAMES:
            new = JOINT_RENAMES[joint.get("name")]
            changed.append(f"joint {joint.get('name')!r} -> {new!r}")
            joint.set("name", new)
    for link in root.findall("link"):
        if link.get("name") in LINK_RENAMES:
            new = LINK_RENAMES[link.get("name")]
            changed.append(f"link {link.get('name')!r} -> {new!r}")
            link.set("name", new)
    for tag in ("parent", "child"):
        for ref in root.iter(tag):
            if ref.get("link") in LINK_RENAMES:
                ref.set("link", LINK_RENAMES[ref.get("link")])

    changed.extend(normalize_frames(root))

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
    changed.extend(inject_link_dynamics(root))

    ET.indent(tree, space="  ")
    tree.write(URDF, encoding="utf-8", xml_declaration=True)

    changed.append(f"arm-only -> {build_arm_only(root)}")
    changed.append(f"viewer model -> {build_viewer_xml()}")

    for line in changed:
        print(f"[postprocess] {line}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
