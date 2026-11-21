"""Wave goodbye with the xArm6: raise the arm and swing it from side to side.

1. Moves to the wave pose with MoveIt (fbot_manipulator/move_joint, collision checked).
2. Sends ONE smooth trajectory to the joint trajectory controller that swings `wave_joint`
   around the wave pose (sinusoid with a soft start/stop, so it does not jerk).
3. Returns to the joint positions the arm had before (MoveIt again), unless return_to_start:=false.

Run (with manipulator.launch.py up, real arm or xarm_fake:=true):
    python3 wave_arm.py
    python3 wave_arm.py --ros-args -p cycles:=4 -p amplitude:=0.5 -p period:=1.6
"""
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message

from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from fbot_manipulator_msgs.srv import MoveJoint

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
MAX_JOINT_VELOCITY = 2.14  # rad/s, xarm_moveit_config joint_limits.yaml


def to_duration_msg(seconds):
    sec = int(seconds)
    return DurationMsg(sec=sec, nanosec=int(round((seconds - sec) * 1e9)))


class WaveArm(Node):
    def __init__(self):
        super().__init__("wave_arm")
        # Upper arm slightly forward, joint3 at -100 deg, joint5 at -67 deg (joint order joint1..joint6, rad).
        self.wave_pose = list(self.declare_parameter(
            "wave_pose", [0.0, 0.2, math.radians(-100.0), 0.0, math.radians(-67.0), 0.0]).value)
        self.wave_joint = self.declare_parameter("wave_joint", "joint1").value
        self.amplitude = float(self.declare_parameter("amplitude", 0.4).value)  # rad to each side
        self.period = float(self.declare_parameter("period", 2.0).value)  # s per side-to-side cycle
        self.cycles = int(self.declare_parameter("cycles", 3).value)
        self.rate = float(self.declare_parameter("rate", 50.0).value)  # trajectory points per second
        self.return_to_start = bool(self.declare_parameter("return_to_start", True).value)
        controller = self.declare_parameter("controller", "xarm6_traj_controller").value

        if self.wave_joint not in ARM_JOINTS:
            raise ValueError(f"wave_joint must be one of {ARM_JOINTS}, got '{self.wave_joint}'")
        if len(self.wave_pose) != len(ARM_JOINTS):
            raise ValueError(f"wave_pose needs {len(ARM_JOINTS)} values, got {len(self.wave_pose)}")
        if self.cycles < 1 or self.period <= 0.0:
            raise ValueError("cycles must be >= 1 and period > 0")
        # Peak speed of the swing is about 2*pi*A/T (a bit more during the ramps).
        peak_velocity = 1.3 * 2.0 * math.pi * self.amplitude / self.period
        if peak_velocity > 0.8 * MAX_JOINT_VELOCITY:
            raise ValueError(
                f"Swing too fast ({peak_velocity:.2f} rad/s): lower amplitude or raise period")

        self.move_joint = self.create_client(MoveJoint, "fbot_manipulator/move_joint")
        self.trajectory = ActionClient(
            self, FollowJointTrajectory, f"{controller}/follow_joint_trajectory")

    def readArmJoints(self):
        """Current arm joint positions (joint1..joint6) from /joint_states, or None."""
        for _ in range(10):  # /joint_states also carries base/neck messages without the arm joints
            ok, msg = wait_for_message(JointState, self, "/joint_states", time_to_wait=2.0)
            if not ok:
                return None
            positions = dict(zip(msg.name, msg.position))
            if all(name in positions for name in ARM_JOINTS):
                return [float(positions[name]) for name in ARM_JOINTS]
        return None

    def moveJoints(self, positions, what):
        if not self.move_joint.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service fbot_manipulator/move_joint not available")
            return False
        self.get_logger().info(f"Moving to {what} with MoveIt...")
        future = self.move_joint.call_async(MoveJoint.Request(joint_positions=positions))
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
        result = future.result()
        if result is None or not result.success:
            self.get_logger().error(f"Could not move to {what}: {result.message if result else 'timeout'}")
            return False
        return True

    def buildWaveTrajectory(self):
        """Swing wave_joint around the wave pose: x(t) = A * sin(wt) * envelope(t).

        The envelope ramps 0 -> 1 in the first half cycle and 1 -> 0 in the last one
        (smoothstep), so position and velocity start and end at the wave pose with zero speed.
        """
        index = ARM_JOINTS.index(self.wave_joint)
        omega = 2.0 * math.pi / self.period
        total = self.cycles * self.period
        ramp = 0.5 * self.period
        start_delay = 0.2  # first point a little after "now" so the controller starts from the pose
        points = []
        steps = int(math.ceil(total * self.rate))
        for k in range(steps + 1):
            t = min(k / self.rate, total)
            if t < ramp:
                u = t / ramp
                env, denv = u * u * (3.0 - 2.0 * u), 6.0 * u * (1.0 - u) / ramp
            elif t > total - ramp:
                u = (total - t) / ramp
                env, denv = u * u * (3.0 - 2.0 * u), -6.0 * u * (1.0 - u) / ramp
            else:
                env, denv = 1.0, 0.0
            offset = self.amplitude * math.sin(omega * t) * env
            speed = self.amplitude * (omega * math.cos(omega * t) * env + math.sin(omega * t) * denv)

            positions = list(self.wave_pose)
            positions[index] += offset
            velocities = [0.0] * len(ARM_JOINTS)
            velocities[index] = speed
            points.append(JointTrajectoryPoint(
                positions=positions, velocities=velocities,
                time_from_start=to_duration_msg(start_delay + t)))
        return points, start_delay + total

    def wave(self):
        if not self.trajectory.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f"Action {self.trajectory._action_name} not available (check the controller name)")
            return False

        points, duration = self.buildWaveTrajectory()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        goal.trajectory.points = points
        goal.goal_time_tolerance = Duration(seconds=1.0).to_msg()

        self.get_logger().info(
            f"Waving: {self.cycles} x {self.period:.1f} s, {self.wave_joint} +-{math.degrees(self.amplitude):.0f} deg")
        send_future = self.trajectory.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("Wave trajectory rejected by the controller")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 10.0)
        if not result_future.done():
            self.get_logger().error("Wave trajectory timed out, cancelling")
            handle.cancel_goal_async()
            return False
        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f"Wave trajectory failed ({result.error_code}): {result.error_string}")
            return False
        return True

    def run(self):
        start = self.readArmJoints()
        if start is None:
            self.get_logger().warning("Arm joints not found in /joint_states: will not return to start")

        if not self.moveJoints(self.wave_pose, "the wave pose"):
            return False
        ok = self.wave()
        if self.return_to_start and start is not None:
            ok = self.moveJoints(start, "the start pose") and ok
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = WaveArm()
    try:
        ok = node.run()
        node.get_logger().info("Done, bye!" if ok else "Wave failed")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
