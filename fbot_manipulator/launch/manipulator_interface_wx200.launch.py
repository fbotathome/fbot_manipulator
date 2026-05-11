import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import yaml


# Hardcoded for the WidowX 200 (wx200) real-hardware path. Assumes the
# Interbotix MoveIt bringup (move_group + xs_sdk hardware interface) is
# launched separately. Adjust the YAML paths below to match the layout of
# the installed `interbotix_xsarm_moveit` release if needed.
ARM_TYPE = "wx200"
MOVEIT_CONFIG_PKG = "interbotix_xsarm_moveit"


def load_yaml(package_name, *path_parts):
    """Load a YAML file from a ROS 2 package share directory."""
    from ament_index_python.packages import get_package_share_directory
    full_path = os.path.join(get_package_share_directory(package_name), *path_parts)
    with open(full_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    """Generate the launch description."""

    pkg_share = FindPackageShare(package="fbot_manipulator")

    namespace_arg = DeclareLaunchArgument(
        name="namespace",
        default_value="fbot_manipulator",
        description="Namespace for the manipulator interface node",
    )

    namespace = LaunchConfiguration("namespace")

    manipulator_interface_node = Node(
        package="fbot_manipulator",
        executable="manipulator_interface_node",
        name="manipulator_interface",
        parameters=[
            {"arm_type": ARM_TYPE},
        ],
    )

    mtc_config = PathJoinSubstitution([
        pkg_share, "config", ARM_TYPE, "mtc_config.yaml"
    ])

    # ---- Load MoveIt configs from interbotix_xsarm_moveit ----
    # These are needed so the MTC node can plan and execute independently
    # without requiring a combined launch with the Interbotix MoveIt bringup.

    kinematics_yaml = load_yaml(
        MOVEIT_CONFIG_PKG, "config", "kinematics.yaml")

    ompl_planning_yaml = load_yaml(
        MOVEIT_CONFIG_PKG, "config", "ompl_planning.yaml")

    ompl_planning_pipeline_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters":
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        },
    }
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    joint_limits_yaml = load_yaml(
        MOVEIT_CONFIG_PKG, "config", "joint_limits.yaml")

    # Real-hardware controllers managed by MoveItSimpleControllerManager.
    # The controllers themselves are spawned by the Interbotix bringup; we
    # just tell MoveIt how to dispatch trajectories to them.
    controllers_yaml = load_yaml(
        MOVEIT_CONFIG_PKG, "config", "controllers.yaml")

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager":
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    manipulation_task_server = Node(
        package="fbot_manipulator",
        executable="manipulation_task_server",
        name="manipulation_task_server",
        parameters=[
            mtc_config,
            {"robot_description_kinematics": kinematics_yaml},
            {"robot_description_planning": joint_limits_yaml},
            ompl_planning_pipeline_config,
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor,
        ],
        output="screen",
    )

    return LaunchDescription([
        namespace_arg,
        manipulator_interface_node,
        manipulation_task_server,
    ])
