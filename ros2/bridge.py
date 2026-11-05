"""ROS 2 joint-state publisher and MoveIt position-trajectory endpoint for sopod."""

import argparse
import csv
import time
from pathlib import Path
from threading import Lock
from uuid import uuid4

import rclpy
import zmq
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from arm_model import ArmModel, JOINTS, NAMES, ROOT, Trajectory
from sopo.runtime.client import SopoClient

PERIOD = 0.02
STATE_TIMEOUT = 0.15
GOAL_ERROR = 0.035
LOG_FIELDS = ("desired_rad", "applied_rad", "actual_rad", "error_rad")


class Bridge(Node):
    def __init__(self, options):
        super().__init__("sopo_bridge")
        self.model = ArmModel(options.config)
        self.logs = Path(options.logs).expanduser().resolve()
        self.logs.mkdir(parents=True, exist_ok=True)
        self.client = SopoClient() if options.hardware else None
        self.state = None
        self.mock_position = [0.0] * len(NAMES)
        self.busy = Lock()
        self.joint_states = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(PERIOD, self.publish_state)
        self.action = ActionServer(
            self, FollowJointTrajectory, "/sopo_arm/follow_joint_trajectory",
            self.execute, goal_callback=self.accept_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )

    def publish_state(self):
        stamp = self.get_clock().now().to_msg()
        if self.client:
            state = self.client.state(timeout_s=0)
            if state is None:
                return
            self.state = state
            try:
                position = self.model.radians(self.fresh_state(), "pos")
            except (ValueError, KeyError, TypeError):
                return
        else:
            position = self.mock_position
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(NAMES)
        msg.position = position
        self.joint_states.publish(msg)

    def fresh_state(self):
        state = self.state
        if not state or not state.get("comm_ok") or not 0 <= time.time() - state["t"] <= STATE_TIMEOUT:
            raise ValueError("No fresh, valid sopod feedback")
        return state

    def accept_goal(self, _request):
        return GoalResponse.ACCEPT if self.busy.acquire(blocking=False) else GoalResponse.REJECT

    def execute(self, handle):
        result = FollowJointTrajectory.Result()
        client = None
        try:
            request = handle.request
            stamp = request.trajectory.header.stamp
            if stamp.sec or stamp.nanosec or request.multi_dof_trajectory.points:
                raise ValueError("Only immediate six-joint position trajectories are supported")
            if (request.path_tolerance or request.goal_tolerance
                    or request.component_path_tolerance or request.component_goal_tolerance
                    or request.goal_time_tolerance.sec or request.goal_time_tolerance.nanosec):
                raise ValueError("This commissioning bridge uses fixed position tolerances")

            points = [(p.time_from_start.sec + 1e-9 * p.time_from_start.nanosec, p.positions)
                      for p in request.trajectory.points]
            state = self.fresh_state() if self.client else None
            actual = self.model.radians(state, "pos") if state else self.mock_position
            trajectory = Trajectory(self.model, list(request.trajectory.joint_names), points, actual)
            holder = None
            if state:
                if state["mode"] != "move" or state["power_ok"] or state["entering_range"]:
                    raise ValueError("Hardware must be manually placed in MOVE with power mode inactive")
                for q in trajectory.positions:
                    self.model.ticks(q, state)
                client = SopoClient()
                client.req.setsockopt(zmq.RCVTIMEO, 300)
                holder = f"moveit-{uuid4().hex[:8]}"
                response = client.acquire(holder)
                if not response.get("ok"):
                    raise ValueError(response.get("error", "Could not acquire sopod lease"))
                deadline = time.monotonic() + STATE_TIMEOUT
                while time.monotonic() < deadline:
                    state = self.fresh_state()
                    if state["lease"] == holder:
                        break
                    time.sleep(PERIOD)
                actual = self.model.radians(state, "pos")
                trajectory = Trajectory(self.model, list(request.trajectory.joint_names), points, actual)

            mode = "hardware" if client else "mock"
            path = self.logs / f"{mode}-{time.time_ns()}.csv"
            self.get_logger().info(f"Trajectory log: {path}")
            with path.open("x", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["elapsed_s", "state_time_s"] + [
                    f"{name}_{field}" for name in NAMES for field in LOG_FIELDS
                ])
                self.run_trajectory(handle, result, trajectory, writer, client, holder)
        except (ValueError, KeyError, TypeError, OSError, zmq.ZMQError) as exc:
            if handle.is_active:
                handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
        finally:
            if client:
                if client.lease:
                    try:
                        client.release()
                    except zmq.ZMQError as exc:
                        self.get_logger().warning(f"Lease release failed, waiting for sopod watchdog: {exc}")
                for socket in (client.pub, client.req, client.sub):
                    socket.close(linger=0)
            self.busy.release()
        if result.error_string:
            self.get_logger().warning(result.error_string)
        return result

    def run_trajectory(self, handle, result, trajectory, writer, client, holder):
        start = time.monotonic()
        previous = start
        peak = [0.0] * len(NAMES)
        settled_at = None
        last_state_time = None
        saw_action = False
        while rclpy.ok():
            now = time.monotonic()
            elapsed = now - start
            if handle.is_cancel_requested:
                handle.canceled()
                return
            if now - previous > STATE_TIMEOUT:
                raise ValueError("Trajectory streaming missed its update deadline")
            previous = now
            desired = trajectory.sample(elapsed)
            if client:
                state = self.fresh_state()
                if state["mode"] != "move" or state["lease"] != holder or state["power_ok"]:
                    raise ValueError("sopod left the permitted execution state or revoked the lease")
                saw_action |= not state["stale"]
                if state["stale"] and (saw_action or elapsed > STATE_TIMEOUT):
                    raise ValueError("sopod action stream is stale")
                actual = self.model.radians(state, "pos")
                applied = self.model.radians(state, "goal")
                state_time = state["t"]
            else:
                actual = applied = self.mock_position
                state_time = time.time()

            if state_time != last_state_time:
                # Compare encoder feedback to the plan at the sample's time, rather than at receipt.
                sample_elapsed = max(0.0, elapsed - max(0.0, time.time() - state_time))
                reference = trajectory.sample(sample_elapsed)
                error = [target - measured for target, measured in zip(reference, actual)]
                writer.writerow([sample_elapsed, state_time] + [
                    value for joint in zip(reference, applied, actual, error) for value in joint
                ])
                peak = [max(old, abs(new)) for old, new in zip(peak, error)]
                last_state_time = state_time

                feedback = FollowJointTrajectory.Feedback()
                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_names = list(NAMES)
                feedback.desired.positions = reference
                feedback.actual.positions = actual
                feedback.error.positions = error
                handle.publish_feedback(feedback)

                if sample_elapsed >= trajectory.times[-1] and max(map(abs, error)) <= GOAL_ERROR:
                    if settled_at is None:
                        settled_at = sample_elapsed
                    if sample_elapsed - settled_at >= 0.2:
                        if handle.is_cancel_requested:
                            handle.canceled()
                            return
                        if client:
                            hold = self.model.ticks(trajectory.positions[-1], state)
                            reply = client.release(hold=hold)
                            if not reply.get("ok"):
                                raise ValueError(reply.get("error", "Could not hold the final target"))
                        self.get_logger().info(f"Peak absolute errors [rad] {dict(zip(NAMES, peak))}")
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        handle.succeed()
                        return
                else:
                    settled_at = None
            if client:
                client.send_action(self.model.ticks(desired, state))
            else:
                self.mock_position = desired
            time.sleep(max(0.0, PERIOD - (time.monotonic() - now)))
        if handle.is_active:
            handle.abort()
        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
        result.error_string = "ROS context shut down during execution"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(ROOT / "configs" / "arm.yaml"))
    parser.add_argument("--logs", required=True)
    parser.add_argument("--hardware", action="store_true")
    options = parser.parse_args()
    rclpy.init(args=[])
    node = Bridge(options)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
