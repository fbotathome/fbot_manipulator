"""Bring up the fbot_manipulator nodes for an Interbotix X-Series arm (default: WidowX 200).

Starts only the fbot_manipulator side:
  - manipulator_interface_node : low-level motion-primitive services
  - manipulation_task_server   : MoveIt Task Constructor action server

It deliberately does NOT start move_group / the hardware driver / RViz. Launch the
Interbotix MoveIt bringup separately first, e.g.

    ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py \\
        robot_model:=wx200 hardware_type:=actual use_moveit_rviz:=false

Notes / gotchas worth knowing:
  * Requires ``interbotix_xs_modules`` on the Python path (package
    ``interbotix_ros_toolboxes``, humble branch) -- the same dependency the
    Interbotix MoveIt bringup needs.
  * The URDF and SRDF are built here the same way ``interbotix_xsarm_moveit``
    builds them, so the model these nodes load matches the one ``move_group`` is
    running. ``robot_description_semantic`` *must* be passed as a parameter --
    nothing publishes the SRDF on a topic -- otherwise both MoveGroupInterface and
    MoveIt Task Constructor fail to load the robot model.
  * The MTC action server calls ``Task::execute()``, which dispatches to the
    ``move_group/ExecuteTaskSolution`` capability. ``xsarm_moveit.launch.py`` does
    NOT add that capability by default; add ``capabilities: move_group/ExecuteTaskSolution``
    to its ``move_group`` node (or run ``move_group`` yourself with it enabled),
    otherwise planning works but execution fails.
  * Keep these nodes in the global namespace: ``xsarm_moveit.launch.py`` runs
    ``move_group`` un-namespaced, so the MoveGroupInterface / PlanningSceneInterface
    here connect to it as-is.
"""

import os

from ament_index_python.packages import get_package_share_directory

from interbotix_xs_modules.xs_common import get_interbotix_xsarm_models
from interbotix_xs_modules.xs_launch import (
    construct_interbotix_xsarm_semantic_robot_description_command,
    declare_interbotix_xsarm_robot_description_launch_arguments,
)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import yaml

# Package that ships the MoveIt config (kinematics / OMPL / joint limits / controllers /
# SRDF) for the Interbotix X-Series arms.
MOVEIT_CONFIG_PKG = 'interbotix_xsarm_moveit'

# fbot_manipulator `arm_type` for the Interbotix X-Series arms. Deliberately equal to the
# MoveIt planning group name in the Interbotix SRDF (`interbotix_arm`), the same way the
# xArm path uses `xarm6` for both. Also selects config/<FBOT_ARM_TYPE>/ in this package.
FBOT_ARM_TYPE = 'interbotix_arm'


def _load_yaml(package_name, *path_parts):
    """Load and parse a YAML file from a ROS 2 package's share directory."""
    path = os.path.join(get_package_share_directory(package_name), *path_parts)
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def _strip_ros_params_wrapper(data):
    """Unwrap a ``/**: {ros__parameters: {...}}`` block (ROS 2 param-file style), if present."""
    if isinstance(data, dict) and 'ros__parameters' in data.get('/**', {}):
        return data['/**']['ros__parameters']
    return data


def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare(package='fbot_manipulator')

    # Resolved to a plain string -- needed to build per-model file paths below.
    robot_model = LaunchConfiguration('robot_model').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ---- Robot description (URDF) and semantic description (SRDF) ----------------
    robot_description = {
        'robot_description': LaunchConfiguration('robot_description'),
    }
    robot_description_semantic = {
        'robot_description_semantic':
            construct_interbotix_xsarm_semantic_robot_description_command(
                robot_model=robot_model,
                config_path=PathJoinSubstitution([
                    FindPackageShare(MOVEIT_CONFIG_PKG), 'config',
                ]),
            ),
    }

    # ---- Kinematics / joint limits / OMPL / controllers (from interbotix_xsarm_moveit) ----
    # interbotix_xsarm_moveit/config/kinematics.yaml is wrapped as `/**: ros__parameters: ...`,
    # so strip that before nesting it under `robot_description_kinematics`.
    kinematics_yaml = _strip_ros_params_wrapper(
        _load_yaml(MOVEIT_CONFIG_PKG, 'config', 'kinematics.yaml'))
    # The X-Series arms (wx200) are 5-DOF: full 6-DOF orientation IK leaves most grasp/Cartesian
    # poses unreachable, so by default solve for position only (the standard Interbotix workaround).
    # This affects IK done locally in these nodes (MTC ComputeIK, MoveGroupInterface Cartesian);
    # to make `move_group`'s pose-goal IK behave the same, set position_only_ik in xsarm_moveit.launch.py too.
    position_only_ik = LaunchConfiguration('position_only_ik').perform(context).lower() == 'true'
    for group_cfg in kinematics_yaml.values():
        if isinstance(group_cfg, dict) and 'kinematics_solver' in group_cfg:
            group_cfg['position_only_ik'] = position_only_ik
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    joint_limits_yaml = _load_yaml(
        MOVEIT_CONFIG_PKG, 'config', 'joint_limits', f'{robot_model}_joint_limits.yaml')
    robot_description_planning = {'robot_description_planning': joint_limits_yaml}

    ompl_planning_yaml = _load_yaml(MOVEIT_CONFIG_PKG, 'config', 'ompl_planning.yaml')
    ompl_planning_pipeline = {
        'default_planning_pipeline': 'ompl',
        'planning_pipelines': ['ompl'],
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters':
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        },
    }
    ompl_planning_pipeline['ompl'].update(ompl_planning_yaml)

    # Real-hardware controllers; the controllers themselves are spawned by the
    # Interbotix bringup, we just tell MoveIt how to dispatch trajectories to them.
    controllers_yaml = _load_yaml(
        MOVEIT_CONFIG_PKG, 'config', 'controllers', f'{robot_model}_controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controller_manager': controllers_yaml,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'trajectory_execution.execution_duration_monitoring': False,
    }

    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Parameter set shared by both nodes (MoveGroupInterface and MTC both want the lot).
    moveit_params = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        ompl_planning_pipeline,
        moveit_controllers,
        trajectory_execution,
        planning_scene_monitor,
        {'use_sim_time': use_sim_time},
    ]

    manipulator_interface_node = Node(
        package='fbot_manipulator',
        executable='manipulator_interface_node',
        name='manipulator_interface',
        parameters=[{'arm_type': FBOT_ARM_TYPE}, *moveit_params],
        output='screen',
    )

    manipulation_task_server = Node(
        package='fbot_manipulator',
        executable='manipulation_task_server',
        name='manipulation_task_server',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', FBOT_ARM_TYPE, 'mtc_config.yaml']),
            *moveit_params,
        ],
        output='screen',
    )

    return [manipulator_interface_node, manipulation_task_server]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_model',
            default_value='wx200',
            choices=get_interbotix_xsarm_models(),
            description=(
                'Interbotix X-Series arm model. Used only to locate the per-model files in '
                'interbotix_xsarm_moveit (URDF/SRDF, joint limits, controllers). The fbot '
                "config dir is config/interbotix_arm/ regardless (it's keyed by the MoveIt "
                'group name, not the model).'
            ),
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value=LaunchConfiguration('robot_model'),
            description=(
                'Robot name / tf prefix; must match the value used for the Interbotix bringup.'
            ),
        ),
        DeclareLaunchArgument(
            'external_srdf_loc',
            default_value='',
            description='Optional path to an extra SRDF snippet to merge into the semantic description.',
        ),
        DeclareLaunchArgument(
            'position_only_ik',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'Solve IK for position only. Recommended for the 5-DOF X-Series arms -- full '
                'orientation IK leaves most grasp/Cartesian poses unreachable. Set false if you '
                'specifically need orientation-constrained IK.'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=('true', 'false'),
            description='Use simulation time (set true if the Interbotix bringup is running in Gazebo).',
        ),
    ]
    # Declares `robot_description` (default = the xacro command for <robot_model>) along with
    # the base_link_frame / show_ar_tag / use_world_frame / external_urdf_loc args it needs.
    declared_arguments += declare_interbotix_xsarm_robot_description_launch_arguments(
        show_gripper_bar='true',
        show_gripper_fingers='true',
        hardware_type='actual',
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
