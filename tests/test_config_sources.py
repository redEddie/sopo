import pathlib, sys, tempfile
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import yaml
from sopo.config import load_arm_config, make_joints, make_joint_limits, make_pairs
from sopo.sources import WaypointSource
from sopo.control import clamp_joint_goals
from sopo.joints import ContinuousJoint

JOINTS = [{"name": "J1", "type": "continuous", "motor_id": 1, "range_ticks": 2600},
          {"name": "J2", "type": "dual", "ids": [10, 11], "reference_id": 11, "K": 4005},
          {"name": "J4", "type": "single", "motor_id": 19}]


def _write(d, cfg, calib=None):
    (d / "arm.yaml").write_text(yaml.safe_dump(cfg))
    if calib:
        (d / "calibration.yaml").write_text(yaml.safe_dump(calib))
    return d / "arm.yaml"


def test_legacy_leader_follower_schema_is_accepted():
    d = pathlib.Path(tempfile.mkdtemp())
    path = _write(d, {"leader": {"port": "/dev/nonexistent0"}, "follower": {"port": "/dev/nonexistent1", "baudrate": 500000}, "joints": JOINTS})
    cfg = load_arm_config(path)
    assert cfg["arm"]["port"] == "/dev/nonexistent0"  # 둘 다 없으면 첫 번째(leader)


def test_new_schema_and_calibration_merge():
    d = pathlib.Path(tempfile.mkdtemp())
    path = _write(d, {"arm": {"port": "/dev/x"}, "joints": JOINTS, "safety": {"torque_limit": 300}},
                  calib={"position_limits": {19: [220, 3969]}, "torque_limits": {19: 150}})
    cfg = load_arm_config(path)
    joints = make_joints(cfg)
    assert make_joint_limits(cfg, joints) == {"J2": (200, 3896), "J4": (220, 3969)}  # J1(continuous) 제외
    assert make_pairs(cfg) == {"J2": (11, 10, 4005)}


def test_waypoint_source_sequence_and_partial_joints():
    src = WaypointSource([{"J4": 2300}, {"J4": 2000, "J2": 1500}], dwell_s=0.5)
    src.connect()
    present = {"J2": 1000, "J4": 2000}
    a = src.get_action(present, 0.0)
    assert a == {"J2": 1000, "J4": 2300}  # 미지정 관절은 현재 유지
    present = {"J2": 1000, "J4": 2290}     # 도달
    src.get_action(present, 1.0)
    assert src.index == 0
    a = src.get_action(present, 1.6)       # dwell 지나면 다음
    assert src.index == 1 and a == {"J2": 1500, "J4": 2000}
    present = {"J2": 1490, "J4": 2010}
    src.get_action(present, 2.0); src.get_action(present, 2.6)
    assert src.is_done()


def test_clamp_joint_goals_step_limits_and_continuous_range():
    joints = make_joints({"joints": JOINTS})
    j1 = next(j for j in joints if isinstance(j, ContinuousJoint))
    j1.home = 2000
    goal = clamp_joint_goals({"J1": 9000, "J2": 5000, "J4": 1000}, {"J1": 2000, "J2": 1000, "J4": 1900},
                             {"J2": (900, 3100), "J4": (220, 3969)}, 80, joints)
    assert goal == {"J1": 2080, "J2": 1080, "J4": 1820}


def test_brake_zone_slows_down_near_limits():
    joints = make_joints({"joints": JOINTS})
    lim = {"J4": (50, 4045)}
    # 리밋까지 145틱 남음, 존 200 → 스텝 80 * 145/200 = 58
    g = clamp_joint_goals({"J4": 4045}, {"J4": 3900}, lim, 80, joints, brake_zone=200, brake_min_step=10)
    assert g["J4"] == 3958
    # 리밋에서 멀면 풀 스텝, 리밋 바로 앞이면 하한 스텝
    assert clamp_joint_goals({"J4": 4045}, {"J4": 2000}, lim, 80, joints, 200, 10)["J4"] == 2080
    assert clamp_joint_goals({"J4": 4045}, {"J4": 4040}, lim, 80, joints, 200, 10)["J4"] == 4045  # 5틱 남음 → min_step 10 > 5 → 목표 도달
    # 리밋에서 멀어지는 방향은 감속 없음
    assert clamp_joint_goals({"J4": 2000}, {"J4": 4040}, lim, 80, joints, 200, 10)["J4"] == 3960


def test_jog_source_keys_and_lead_limit():
    from sopo.sources import JogSource
    keys = []
    src = JogSource(["J4", "J6"], lambda: keys.pop(0) if keys else [], step=40, lead=200)
    present = {"J4": 2000, "J6": 2000}
    assert src.get_action(present, 0.0) == present               # 첫 호출: 목표=현재
    keys.append(["right", "right", "down", "left"])
    a = src.get_action(present, 0.1)
    assert a == {"J4": 2080, "J6": 1960} and src.active_joint == "J6"
    keys.append(["right"] * 20)                                    # 키 연타해도 목표는 현재 ±lead
    assert src.get_action(present, 0.2)["J6"] == 2200
    keys.append([" "])                                            # 홀드
    assert src.get_action(present, 0.3) == present
    keys.append(["q"]); src.get_action(present, 0.4)
    assert src.is_done()


def test_verify_eprom_reports_drift():
    from sopo.safety import SafetyLimits, verify_eprom
    class FakeBus:
        regs = {(19, "Max_Torque_Limit"): 1000, (19, "Min_Position_Limit"): 0, (19, "Max_Position_Limit"): 4095,
                (21, "Max_Torque_Limit"): 150}
        def read(self, reg, mid): return self.regs[(mid, reg)]
    lim = SafetyLimits(position_limits={19: (50, 4045)}, torque_limits={19: 150, 21: 150}, eprom_torque_ceiling=600)
    problems = verify_eprom(FakeBus(), lim, [19, 21])
    # 19: ceiling 1000 != 600 and position limits; 21: ceiling 150 != 600
    assert len(problems) == 3 and problems[0].startswith("ID19") and problems[2].startswith("ID21")


def test_continuous_home_abs_picks_nearest_turn():
    from sopo.joints import ContinuousJoint
    class B:
        def __init__(self, v): self.v = v
        def read(self, reg, mid): return self.v
    j = ContinuousJoint("J1", 1, range_ticks=2048, firmware_multiturn=True, home_abs=2048)
    assert j.read(B(2300)) == 2300 and j.home == 2048                # 같은 바퀴
    j2 = ContinuousJoint("J1", 1, range_ticks=2048, firmware_multiturn=True, home_abs=2048)
    assert j2.read(B(4500)) == 4500 and j2.home == 2048 + 4096       # 다음 바퀴의 2048이 더 가까움
    assert j2.range_bounds() == (4096, 8192)


def test_reflex_joint_limit_for_continuous_range():
    from sopo import SafetyLimits
    from sopo.reflex import Reflex, ReflexConfig, Event, Mode
    r = Reflex(SafetyLimits(torque_limits={1: 200}), {}, ReflexConfig(limit_margin=30, t_limit=0.5))
    r.set_limit(1, 0, 4096)
    r.update(now=0.0, present={1: 2048}, goal={1: 2048}, load={1: 0})           # 안에서 시작 → 무장
    trips = []
    for k in range(1, 5):
        trips += r.update(now=0.3 * k, present={1: 4200}, goal={1: 4096}, load={1: 0})
    assert [t.event for t in trips] == [Event.JOINT_LIMIT] and r.mode is Mode.REFLEX


def test_stream_source_stale_hold_is_latched_not_tracking():
    from sopo.sources import StreamSource
    s = StreamSource(watchdog_s=0.5)
    s.push({"J4": 2300}, 0.0)
    assert s.get_action({"J4": 2100}, 0.1) == {"J4": 2300}
    assert s.get_action({"J4": 2250}, 1.0) == {"J4": 2250}   # stale: latch the position at that moment
    assert s.get_action({"J4": 2400}, 1.5) == {"J4": 2250}   # pushed by hand: goal stays -> arm resists


def test_build_joint_map_from_repo_arm_yaml():
    """configs/arm.yaml에서 매핑 유도: J2 듀얼 합산 스톨/ids/mount_sign 확인."""
    import pytest
    from sopo.dynamics import build_joint_map
    from sopo.safety import KGCM_TO_NM, MODEL_STALL_TORQUE_KGCM
    root = pathlib.Path(__file__).resolve().parents[1]
    joints = yaml.safe_load((root / "configs" / "arm.yaml").read_text())["joints"]
    jm = build_joint_map(joints, MODEL_STALL_TORQUE_KGCM)
    assert list(jm) == ["J1", "J2", "J3", "J4", "J5", "J6"]  # 목록 순서 = joint_1..6
    j2 = jm["J2"]
    assert j2["urdf"] == "joint_2" and jm["J1"]["urdf"] == "joint_1"
    assert j2["ids"] == [10, 11] and j2["mount_sign"] == [1, -1]
    assert j2["stall"] == pytest.approx(9.81, abs=0.01)
    assert j2["stall"] == pytest.approx(50 * KGCM_TO_NM * 2)  # sts3250 듀얼 합산
    assert jm["J3"]["ids"] == [15, 16] and jm["J3"]["mount_sign"] == [1, -1]
    # single/continuous는 ids 1개, mount_sign [1]
    assert jm["J1"]["ids"] == [1] and jm["J1"]["mount_sign"] == [1]
    assert jm["J5"]["stall"] == pytest.approx(30 * KGCM_TO_NM)  # sts3215


def test_load_gravity_cal_tmp_yaml():
    """calibration.yaml gravity 섹션 로드: zero/dir/scale + 파일 없으면 기본값."""
    import math
    import pytest
    from sopo.config import load_gravity_cal
    d = pathlib.Path(tempfile.mkdtemp())
    p = d / "calibration.yaml"
    p.write_text(yaml.safe_dump({"gravity": {"zero_ticks": {"J2": 2012}, "dir": {"J2": -1},
                                             "scale": {"J2": 0.29}}}))
    cal = load_gravity_cal(p)
    assert cal.zero_ticks == {"J2": 2012}
    assert cal.dir == {"J2": -1}
    assert cal.scale == {"J2": 0.29}
    assert cal.q("J2", 2012) == 0.0
    assert cal.q("J2", 2012 + 2048) == pytest.approx(-math.pi)  # dir -1 반영
    # 파일/섹션이 없으면 기본값 (dir은 q()에서 +1 취급)
    empty = load_gravity_cal(d / "nonexistent.yaml")
    assert empty.zero_ticks == {} and empty.dir == {} and empty.scale == {}

