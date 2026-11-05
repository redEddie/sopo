"""Shared planning limits and trajectory sampling for the SOPO MoveIt demo."""

import math
from bisect import bisect_right
from pathlib import Path
from xml.etree import ElementTree as ET

from sopo.config import load_arm_config, load_gravity_cal, make_joint_limits, make_joints
from sopo.motion.joints import ContinuousJoint

ROOT = Path(__file__).resolve().parents[1]
NAMES = tuple(f"joint_{i}" for i in range(1, 7))
JOINTS = tuple(f"J{i}" for i in range(1, 7))
MAX_SPEED = 1.0  # rad/s, MoveIt planning limit
START_ERROR = 0.02  # rad


class ArmModel:
    def __init__(self, config):
        config = Path(config)
        cfg = load_arm_config(config)
        self.cal = load_gravity_cal(config.with_name("calibration.yaml"))
        if tuple(j["name"] for j in cfg["joints"]) != JOINTS:
            raise ValueError("Expected SOPO joints J1 through J6 in URDF order")
        joints = make_joints(cfg)
        self.tick_bounds = make_joint_limits(cfg, joints)
        for joint in joints:
            if isinstance(joint, ContinuousJoint):
                home, radius = joint.home_abs, joint.range_ticks
                self.tick_bounds[joint.name] = (home - radius, home + radius)

        description = ROOT / "description"
        root = ET.parse(description / "arm_no_ee.urdf").getroot()
        self.bounds = {}
        for joint, name in zip(JOINTS, NAMES):
            element = root.find(f"./joint[@name='{name}']")
            limit = element.find("limit")
            tick_lo, tick_hi = self.tick_bounds[joint]
            if tick_lo >= tick_hi:
                raise ValueError(f"No common motor range for {joint}")
            lo, hi = sorted(self.cal.q(joint, tick) for tick in (tick_lo, tick_hi))
            lo = max(lo, float(limit.get("lower", "-inf")))
            hi = min(hi, float(limit.get("upper", "inf")))
            if not lo < hi:
                raise ValueError(f"No common planning and hardware range for {joint}")
            self.bounds[name] = (lo, hi)
            element.set("type", "revolute")
            limit.attrib.update(lower=str(lo), upper=str(hi), velocity=str(MAX_SPEED))
        for mesh in root.iter("mesh"):
            path = description / mesh.attrib["filename"]
            if not path.is_file():
                raise FileNotFoundError(path)
            mesh.set("filename", path.as_uri())
        self.urdf = ET.tostring(root, encoding="unicode")

    def radians(self, state, field):
        return [self.cal.q(j, state["joints"][j][field] - state["joints"][j]["turn_offset"])
                for j in JOINTS]

    def ticks(self, position, state):
        targets = {}
        for joint, q in zip(JOINTS, position):
            entry = state["joints"][joint]
            target = self.cal.ticks(joint, math.degrees(q)) + entry["turn_offset"]
            if entry["bounds"] is None or not entry["bounds"][0] <= target <= entry["bounds"][1]:
                raise ValueError(f"{joint} target is outside sopod's active range")
            targets[joint] = target
        return targets


class Trajectory:
    """Validate a MoveIt position trajectory, then stream linear segments at their requested times."""

    def __init__(self, model, names, points, start):
        if len(names) != len(NAMES) or set(names) != set(NAMES):
            raise ValueError("Expected all six SOPO trajectory joints")
        order = [names.index(name) for name in NAMES]
        if any(not model.bounds[name][0] <= value <= model.bounds[name][1]
               for name, value in zip(NAMES, start)):
            raise ValueError("Current position is outside the planning limits")
        self.times, self.positions = [0.0], [list(start)]
        for index, (timestamp, positions) in enumerate(points):
            if len(positions) != len(NAMES) or not math.isfinite(timestamp) or timestamp < 0:
                raise ValueError("Invalid trajectory point")
            q = [positions[i] for i in order]
            for name, value in zip(NAMES, q):
                lo, hi = model.bounds[name]
                if not lo <= value <= hi:
                    raise ValueError(f"{name} is outside the planning limits")
            if index == 0 and timestamp == 0:
                if max(abs(a - b) for a, b in zip(q, start)) > START_ERROR:
                    raise ValueError("Plan start differs from current encoder position")
                self.positions[0] = q
            else:
                if timestamp <= self.times[-1]:
                    raise ValueError("Trajectory timestamps must increase")
                self.times.append(timestamp)
                self.positions.append(q)
        if len(self.times) < 2:
            raise ValueError("Expected a nonzero-duration trajectory")

    def sample(self, elapsed):
        i = bisect_right(self.times, elapsed)
        if i == len(self.times):
            return self.positions[-1]
        fraction = (elapsed - self.times[i - 1]) / (self.times[i] - self.times[i - 1])
        return [a + fraction * (b - a) for a, b in zip(self.positions[i - 1], self.positions[i])]
