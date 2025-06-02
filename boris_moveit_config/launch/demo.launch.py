from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("boris", package_name="boris_moveit_config").to_moveit_configs()
    moveit_config.moveit_cpp['use_sim_time'] = True
    return generate_demo_launch(moveit_config)
