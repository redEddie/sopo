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

