"""Launch MoveIt, RViz and the SOPO trajectory bridge directly from the checkout."""

import sys
from pathlib import Path
from xml.etree import ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from arm_model import ArmModel, MAX_SPEED, NAMES, ROOT, START_ERROR


def launch_nodes(context):
    config = Path(LaunchConfiguration("arm_config").perform(context)).expanduser().resolve()
    model = ArmModel(config)
    srdf = ET.Element("robot", name=ET.fromstring(model.urdf).attrib["name"])
    group = ET.SubElement(srdf, "group", name="arm")
    ET.SubElement(group, "chain", base_link="base_link", tip_link="flange")
    # Mechanical neighbors share mounting surfaces in the exported collision meshes.
    for joint in ET.fromstring(model.urdf).findall("joint"):
        ET.SubElement(srdf, "disable_collisions",
                      link1=joint.find("parent").attrib["link"],
                      link2=joint.find("child").attrib["link"], reason="Adjacent")

    description = {"robot_description": ParameterValue(model.urdf, value_type=str)}
    robot = {
        **description,
        "robot_description_semantic": ParameterValue(ET.tostring(srdf, encoding="unicode"), value_type=str),
        "robot_description_kinematics": {"arm": {
            "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            "kinematics_solver_timeout": 0.1,
        }},
        "robot_description_planning": {"joint_limits": {
            name: {"has_velocity_limits": True, "max_velocity": MAX_SPEED,
                   "has_acceleration_limits": True, "max_acceleration": 5.0}
            for name in NAMES
        }},
    }
    moveit = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": " ".join(
                "default_planner_request_adapters/" + adapter for adapter in (
                    "AddTimeOptimalParameterization", "ResolveConstraintFrames", "FixWorkspaceBounds",
                )
            ),
            "arm": {"planner_configs": ["RRTConnect"]},
            "planner_configs": {"RRTConnect": {"type": "geometric::RRTConnect", "range": 0.0}},
        },
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": {
            "controller_names": ["sopo_arm"],
            "sopo_arm": {"type": "FollowJointTrajectory", "action_ns": "follow_joint_trajectory",
                         "default": True, "joints": list(NAMES)},
        },
        "trajectory_execution": {"allowed_start_tolerance": START_ERROR,
                                 "execution_duration_monitoring": False},
        "publish_robot_description_semantic": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    mode = LaunchConfiguration("mode").perform(context)
    if mode not in ("mock", "hardware"):
        raise ValueError("mode must be mock or hardware")
    bridge = [sys.executable, str(HERE / "bridge.py"), "--config", str(config),
              "--logs", LaunchConfiguration("log_dir")]
    if mode == "hardware":
        bridge.append("--hardware")
    nodes = [
        ExecuteProcess(cmd=bridge, output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[description], output="screen"),
        Node(package="moveit_ros_move_group", executable="move_group",
             parameters=[robot, moveit], output="screen"),
    ]
    if LaunchConfiguration("rviz").perform(context).lower() == "true":
        nodes.append(Node(package="rviz2", executable="rviz2", arguments=["-d", str(HERE / "sopo.rviz")],
                          parameters=[robot], output="screen"))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arm_config", default_value=str(ROOT / "configs" / "arm.yaml")),
        DeclareLaunchArgument("mode", default_value="mock"),
        DeclareLaunchArgument("log_dir", default_value=str(Path.home() / ".ros" / "sopo_tracking")),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=launch_nodes),
    ])
