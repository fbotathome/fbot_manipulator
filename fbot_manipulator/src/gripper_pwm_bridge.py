#!/usr/bin/env python3
# coding: utf-8
"""GripperCommand -> PWM bridge for the Interbotix wx200 (XL430 gripper).

The wx200 gripper motor is an XL430-W250: no current sensing, so it cannot use
current-based position control, and plain Position Control Mode overloads when it
stalls on an object. The motor is therefore run in PWM operating mode and gripped
with a bounded duty (the same +/-350 fbot_behavior uses), which holds against a
stall without tripping the overload protection.

MoveIt/MTC drives the gripper as a control_msgs/GripperCommand action (configured in
the MoveIt controllers YAML as `type: GripperCommand`, controller
`/<robot>/gripper_pwm_bridge`, action_ns `gripper_cmd`). This node serves that action
and translates each goal into a PWM command on `/<robot>/commands/joint_single` (the
same interface `robot_write_joint_command('gripper', ...)` uses). It reports success
when the fingers reach the commanded opening or stall against an object.
"""

import threading
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import GripperCommand
from interbotix_xs_msgs.msg import JointSingleCommand
from sensor_msgs.msg import JointState


class GripperPwmBridge(Node):
    def __init__(self):
        super().__init__('gripper_pwm_bridge')

        # --- parameters ---------------------------------------------------------
        self.declare_parameter('robot_name', 'wx200')
        self.declare_parameter('gripper_motor_name', 'gripper')   # xs_sdk 'single' name
        self.declare_parameter('finger_joint', 'left_finger')     # joint in /joint_states
        self.declare_parameter('pwm', 350.0)                      # grip duty magnitude
        self.declare_parameter('open_is_positive', True)          # +pwm opens (fbot_behavior)
        self.declare_parameter('stall_velocity', 0.002)           # m/s ~ fingers stopped
        self.declare_parameter('stall_time', 0.3)                 # s below thresh => settled
        self.declare_parameter('timeout', 3.0)                    # s safety cap on a move
        self.declare_parameter('position_tolerance', 0.003)       # m, open-reached tolerance
        self.declare_parameter('min_drive_time', 0.5)             # s grace before a stall counts (spin-up)
        self.declare_parameter('hold_after_grasp', True)          # keep PWM applied while gripping

        g = self.get_parameter
        self._robot = g('robot_name').value
        self._motor = g('gripper_motor_name').value
        self._finger = g('finger_joint').value
        self._pwm = float(g('pwm').value)
        self._open_is_positive = bool(g('open_is_positive').value)
        self._stall_velocity = float(g('stall_velocity').value)
        self._stall_time = float(g('stall_time').value)
        self._timeout = float(g('timeout').value)
        self._pos_tol = float(g('position_tolerance').value)
        self._min_drive_time = float(g('min_drive_time').value)
        self._hold_after_grasp = bool(g('hold_after_grasp').value)

        # --- state --------------------------------------------------------------
        self._lock = threading.Lock()
        self._finger_pos = None
        self._finger_vel = 0.0

        cb = ReentrantCallbackGroup()
        self._cmd_pub = self.create_publisher(
            JointSingleCommand, f'/{self._robot}/commands/joint_single', 10)
        self.create_subscription(
            JointState, f'/{self._robot}/joint_states',
            self._joint_state_cb, 10, callback_group=cb)
        self._server = ActionServer(
            self, GripperCommand,
            f'/{self._robot}/gripper_pwm_bridge/gripper_cmd',
            execute_callback=self._execute,
            callback_group=cb)

        self.get_logger().info(
            f"gripper_pwm_bridge ready: serving "
            f"'/{self._robot}/gripper_pwm_bridge/gripper_cmd', driving '{self._motor}' "
            f"on '/{self._robot}/commands/joint_single' at +/-{self._pwm:.0f} PWM.")

    # --------------------------------------------------------------------------
    def _joint_state_cb(self, msg: JointState):
        if self._finger in msg.name:
            i = msg.name.index(self._finger)
            vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            with self._lock:
                self._finger_pos = msg.position[i]
                self._finger_vel = vel

    def _state(self):
        with self._lock:
            return self._finger_pos, self._finger_vel

    def _send_pwm(self, value: float):
        self._cmd_pub.publish(JointSingleCommand(name=self._motor, cmd=float(value)))

    # --------------------------------------------------------------------------
    def _execute(self, goal_handle):
        target = goal_handle.request.command.position          # meters (left_finger)
        req_eff = goal_handle.request.command.max_effort
        pwm_mag = abs(req_eff) if req_eff and req_eff > 0.0 else self._pwm

        # Wait briefly for a joint state so we can decide direction.
        deadline = time.time() + 1.0
        pos, _ = self._state()
        while pos is None and time.time() < deadline:
            time.sleep(0.02)
            pos, _ = self._state()
        if pos is None:
            self.get_logger().error(f"No joint state for '{self._finger}'; aborting.")
            goal_handle.abort()
            return GripperCommand.Result()

        closing = target < pos - self._pos_tol
        opening = target > pos + self._pos_tol
        if not closing and not opening:
            self.get_logger().info(f"Gripper already at target ({target:.4f} m).")
            return self._succeed(goal_handle, pos, 0.0)

        direction = 1.0 if opening else -1.0
        if not self._open_is_positive:
            direction = -direction
        cmd = direction * pwm_mag
        self._send_pwm(cmd)
        self.get_logger().info(
            f"{'Opening' if opening else 'Closing'} gripper: target {target:.4f} m, "
            f"current {pos:.4f} m, PWM {cmd:.0f}.")

        start = time.time()
        start_pos = pos
        settled_at = None
        moved = False
        while rclpy.ok():
            elapsed = time.time() - start
            if elapsed > self._timeout:
                self.get_logger().warn("Gripper move timed out; treating as settled.")
                break
            pos, vel = self._state()
            if pos is not None:
                # reached the commanded position with nothing in the way
                if opening and pos >= target - self._pos_tol:
                    break
                if closing and pos <= target + self._pos_tol:
                    break
                if abs(pos - start_pos) > self._pos_tol:
                    moved = True
            # Stall = fingers stopped (object contact / hard stop). Only count it once the
            # gripper has actually started moving, or after a short grace period -- otherwise
            # the near-zero velocity during motor spin-up reads as a stall and the move ends
            # early (the "release only opens a little, press repeatedly" symptom).
            if (moved or elapsed > self._min_drive_time) and abs(vel) < self._stall_velocity:
                if settled_at is None:
                    settled_at = time.time()
                elif time.time() - settled_at >= self._stall_time:
                    break
            else:
                settled_at = None
            fb = GripperCommand.Feedback()
            fb.position = pos if pos is not None else target
            fb.effort = cmd
            fb.stalled = settled_at is not None
            fb.reached_goal = False
            goal_handle.publish_feedback(fb)
            time.sleep(0.02)

        # Closing: keep the PWM applied so the object stays gripped through the lift.
        # Opening: drop the duty so the motor isn't driven against the open stop.
        if opening or not self._hold_after_grasp:
            self._send_pwm(0.0)

        pos, _ = self._state()
        return self._succeed(goal_handle, pos if pos is not None else target, cmd)

    def _succeed(self, goal_handle, position, effort):
        goal_handle.succeed()
        result = GripperCommand.Result()
        result.position = float(position)
        result.effort = float(effort)
        result.stalled = True
        result.reached_goal = True
        return result


def main(args=None):
    rclpy.init(args=args)
    node = GripperPwmBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
