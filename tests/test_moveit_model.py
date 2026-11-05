"""Regression checks for the calibrated MoveIt position interface, without ROS."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2"))
from arm_model import ArmModel, JOINTS, NAMES, ROOT, Trajectory


@pytest.fixture
def model():
    return ArmModel(ROOT / "configs" / "arm.yaml")


def test_arm_limits_and_multiturn_round_trip(model):
    assert model.tick_bounds["J2"] == (952, 3105)
    assert model.tick_bounds["J3"] == (1120, 3101)
    assert model.bounds["joint_6"][0] > -3.0  # CAD continuous, real hardware bounded
    state = {"joints": {}}
    for name in JOINTS:
        offset = 4096 if name in ("J1", "J4") else 0
        state["joints"][name] = {
            "turn_offset": offset,
            "pos": model.cal.zero_ticks[name] + offset,
            "bounds": [tick + offset for tick in model.tick_bounds[name]],
        }
    assert model.radians(state, "pos") == [0.0] * 6
    assert model.ticks([0.0] * 6, state) == {name: state["joints"][name]["pos"] for name in JOINTS}
    assert model.ticks([0.1, 0, 0, 0, 0, 0], state)["J1"] < state["joints"]["J1"]["pos"]


def test_trajectory_remaps_and_enforces_timestamps(model):
    start = [0.0] * 6
    end = [0, 0, 0, 0, 0, 0.1]
    trajectory = Trajectory(model, NAMES[::-1], [(0, start), (2, end)], start)
    assert trajectory.sample(1) == pytest.approx([0.05, 0, 0, 0, 0, 0])
    with pytest.raises(ValueError, match="timestamps"):
        Trajectory(model, NAMES, [(0, start), (0, start), (2, end)], start)
    fast = Trajectory(model, NAMES, [(0, start), (0.1, end)], start)
    assert fast.sample(0.05) == pytest.approx([0, 0, 0, 0, 0, 0.05])
