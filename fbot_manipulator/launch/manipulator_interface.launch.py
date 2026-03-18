import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import yaml


def load_yaml(package_name, *path_parts):
    """Load a YAML file from a ROS 2 package share directory."""
    from ament_index_python.packages import get_package_share_directory
    full_path = os.path.join(get_package_share_directory(package_name), *path_parts)
    with open(full_path, "r") as f:
        return yaml.safe_load(f)


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

    # MTC config file path
    mtc_config = PathJoinSubstitution([
        pkg_share, "config", arm_type, "mtc_config.yaml"
    ])

    # ---- Load MoveIt configs from xarm_moveit_config ----
    # These are needed so the MTC node can plan and execute independently
    # without requiring a combined launch with xarm_moveit_config.

    # Kinematics (IK solver)
    kinematics_yaml = load_yaml(
        "xarm_moveit_config", "config", "xarm6", "kinematics.yaml")

    # OMPL planning pipeline
    ompl_defaults_yaml = load_yaml(
        "xarm_moveit_config", "config", "moveit_configs", "ompl_defaults.yaml")
    ompl_planning_yaml = load_yaml(
        "xarm_moveit_config", "config", "xarm6", "ompl_planning.yaml")
    gripper_ompl_yaml = load_yaml(
        "xarm_moveit_config", "config", "xarm_gripper", "ompl_planning.yaml")
    ompl_defaults_yaml.update(ompl_planning_yaml)
    ompl_defaults_yaml.update(gripper_ompl_yaml)
    ompl_planning_yaml = ompl_defaults_yaml

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

    # Joint limits
    joint_limits_yaml = load_yaml(
        "xarm_moveit_config", "config", "xarm6", "joint_limits.yaml")
    gripper_joint_limits_yaml = load_yaml(
        "xarm_moveit_config", "config", "xarm_gripper", "joint_limits.yaml")
    joint_limits_yaml["joint_limits"].update(gripper_joint_limits_yaml["joint_limits"])

    # Controllers (fake for simulation)
    controllers_yaml = load_yaml(
        "xarm_moveit_config", "config", "xarm6", "fake_controllers.yaml")
    gripper_controllers_yaml = load_yaml(
        "xarm_moveit_config", "config", "xarm_gripper", "fake_controllers.yaml")
    for name in gripper_controllers_yaml.get("controller_names", []):
        if name not in controllers_yaml["controller_names"]:
            controllers_yaml["controller_names"].append(name)
        controllers_yaml[name] = gripper_controllers_yaml[name]

    moveit_controllers = {
        "moveit_fake_controller_manager": controllers_yaml,
        "moveit_controller_manager":
            "moveit_fake_controller_manager/MoveItFakeControllerManager",
    }

    # Trajectory execution
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    # Planning scene monitor
    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Create the manipulation task server node (MTC-based action server)
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
        arm_type_arg,
        namespace_arg,
        manipulator_interface_node,
        manipulation_task_server,
    ])
