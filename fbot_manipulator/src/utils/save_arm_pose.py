import os
import re

import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message

from sensor_msgs.msg import JointState

class SaveArmPose(Node):
    def __init__(self):
        super().__init__("save_arm_pose")

        self.declare_parameter(
            "xacro_path",
            "/home/insider/fbot_ws/src/xarm_ros2/xarm_moveit_config/srdf/_xarm6_macro.srdf.xacro",
        )
        self.xacro_path = self.get_parameter("xacro_path").get_parameter_value().string_value

        self.subscription_ = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.publisher_ = self.create_publisher(JointState, "/saved_arm_pose", 10)
        self.save_loop_started = False
        self.done_saving = False

        self.group_name = "${prefix}xarm6"
        self.default_arm_joint_order = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.arm_joint_order = self.loadArmJointOrder()

    def loadArmJointOrder(self):
        if not os.path.exists(self.xacro_path):
            self.get_logger().warning(
                f"{self.xacro_path} not found. Using default order {self.default_arm_joint_order}."
            )
            return list(self.default_arm_joint_order)

        try:
            with open(self.xacro_path, "r", encoding="utf-8") as xacro_file:
                content = xacro_file.read()
        except OSError as exc:
            self.get_logger().warning(
                f"Could not read {self.xacro_path}: {exc}. Using default joint order."
            )
            return list(self.default_arm_joint_order)

        group_state_match = re.search(
            r'<group_state[^>]*group="\$\{prefix\}xarm6"[^>]*>(.*?)</group_state>',
            content,
            re.DOTALL,
        )
        if not group_state_match:
            self.get_logger().warning(
                f"Could not infer arm joint order from {self.xacro_path}. Using default order {self.default_arm_joint_order}."
            )
            return list(self.default_arm_joint_order)

        joint_names = re.findall(
            r'<joint[^>]*name="\$\{prefix\}([^"]+)"[^>]*/?>',
            group_state_match.group(1),
        )
        if not joint_names:
            self.get_logger().warning(
                f"No arm joints found in existing group_state. Using default order {self.default_arm_joint_order}."
            )
            return list(self.default_arm_joint_order)

        self.get_logger().info(f"Detected arm joint order: {joint_names}")
        return joint_names
    
    
    def joint_state_callback(self, msg):
        if self.done_saving:
            return
            
        self.publisher_.publish(msg)

        if not self.save_loop_started:
            self.save_loop_started = True
            self.get_logger().info("Starting pose save session...")
            self.savePose()
            self.done_saving = True
            self.get_logger().info("Pose saving complete. Shutting down...")
            rclpy.shutdown()

    def savePose(self) -> None:
        while rclpy.ok():
            arm_name = input("Move the arm to the desired pose and enter its name (e.g., 'PrePickup', 'LookToGarbage'): ").strip()
            if not arm_name:
                self.get_logger().warning("Pose name cannot be empty. Pose not saved.")
                continue

            if not re.fullmatch(r"[A-Za-z0-9_.-]+", arm_name):
                self.get_logger().warning(
                    "Invalid pose name. Use only letters, numbers, '-', '_' or '.'. Pose not saved."
                )
                continue

            success, message = wait_for_message(msg_type= JointState, node=self, topic='/joint_states', time_to_wait=2)
            if not success or message is None:
                self.get_logger().warning("Timed out waiting for /joint_states. Pose not saved.")
                continue

            joints = message.name
            values = message.position
            joint_to_value = dict(zip(joints, values))

            ordered_pose = []
            missing_joints = []
            for joint_name in self.arm_joint_order:
                joint_value = self.findJointValue(joint_to_value, joint_name)
                if joint_value is None:
                    missing_joints.append(joint_name)
                    continue
                ordered_pose.append(joint_value)

            if missing_joints:
                self.get_logger().warning(f"Missing joints in /joint_states: {missing_joints}. Pose not saved.")
                continue

            if self.writeToXacro(arm_name, ordered_pose):
                self.get_logger().info(f"Saved '{arm_name}' to {self.xacro_path}")

            save_more = input("Do you want to add more poses? (y/n): ").strip().lower()
            if save_more in ('n', 'no'):
                break
            elif save_more in ('y', 'yes'):
                continue
            else:
                self.get_logger().warning("Invalid input. Please enter 'y' or 'n'. Continuing by default.")
                continue

    def findJointValue(self, joint_to_value, joint_name):
        if joint_name in joint_to_value:
            return float(joint_to_value[joint_name])

        suffix_matches = [
            float(value)
            for name, value in joint_to_value.items()
            if name.endswith(joint_name)
        ]
        if len(suffix_matches) == 1:
            return suffix_matches[0]

        return None

    def formatGroupState(self, pose_name, pose_values):
        lines = [f'    <group_state name="{pose_name}" group="{self.group_name}">']
        for joint_name, joint_value in zip(self.arm_joint_order, pose_values):
            value_as_text = format(float(joint_value), ".17g")
            lines.append(
                f'      <joint name="${{prefix}}{joint_name}" value="{value_as_text}" />'
            )
        lines.append("    </group_state>")
        return "\n".join(lines)

    def writeToXacro(self, pose_name, pose_values):
        if not os.path.exists(self.xacro_path):
            self.get_logger().error(f"{self.xacro_path} does not exist. Pose not saved.")
            return False

        try:
            with open(self.xacro_path, "r", encoding="utf-8") as xacro_file:
                content = xacro_file.read()
        except OSError as exc:
            self.get_logger().error(f"Error reading {self.xacro_path}: {exc}")
            return False

        group_state_block = self.formatGroupState(pose_name, pose_values)

        existing_pose_pattern = re.compile(
            rf'^[ \t]*<group_state\s+name="{re.escape(pose_name)}"\s+group="\$\{{prefix\}}xarm6">.*?^[ \t]*</group_state>\s*\n?',
            re.MULTILINE | re.DOTALL,
        )

        if existing_pose_pattern.search(content):
            updated_content = existing_pose_pattern.sub(group_state_block + "\n", content, count=1)
        else:
            insertion_match = re.search(r"^[ \t]*<!--\s*gripper\s*-->.*$", content, re.MULTILINE)
            if insertion_match:
                insert_at = insertion_match.start()
                updated_content = content[:insert_at] + group_state_block + "\n\n" + content[insert_at:]
            else:
                macro_close_tag = "  </xacro:macro>"
                close_index = content.rfind(macro_close_tag)
                if close_index == -1:
                    self.get_logger().error("Could not find where to insert new <group_state> block.")
                    return False
                updated_content = content[:close_index] + group_state_block + "\n" + content[close_index:]

        try:
            with open(self.xacro_path, "w", encoding="utf-8") as xacro_file:
                xacro_file.write(updated_content)
        except OSError as exc:
            self.get_logger().error(f"Error writing {self.xacro_path}: {exc}")
            return False

        return True
       

       
def main(args=None):
    rclpy.init(args=args)
    node = SaveArmPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == "__main__":
    main()