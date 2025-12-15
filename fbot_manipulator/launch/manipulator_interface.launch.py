import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the launch description."""
    
    pkg_share = FindPackageShare(package="fbot_manipulator")
    
    arm_type_arg = DeclareLaunchArgument(
        name="arm_type",
        default_value="xarm6",
        description="Type of arm to use (e.g., 'xarm6', 'wx200')",
    )
    
    namespace_arg = DeclareLaunchArgument(
        name="namespace",
        default_value="fbot_manipulator",
        description="Namespace for the manipulator interface node",
    )
    
    # Get launch configurations
    arm_type = LaunchConfiguration("arm_type")
    namespace = LaunchConfiguration("namespace")
    
    # Create the manipulator interface node
    manipulator_interface_node = Node(
        package="fbot_manipulator",
        executable="manipulator_interface_node",
        #namespace=namespace,
        name="manipulator_interface",
        parameters=[
            {"arm_type": arm_type},
        ],
    )

    return LaunchDescription([
        arm_type_arg,
        namespace_arg,
        manipulator_interface_node,
    ])
