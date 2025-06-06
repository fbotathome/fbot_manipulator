cmake_minimum_required(VERSION 3.5)
project(interbotix_xsarm_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install Python executables
install(
  PROGRAMS
    demos/bartender.py
    demos/ee_cartesian_trajectory.py
    demos/ee_pose_components.py
    demos/ee_pose_matrix_control.py
    demos/gripper_control.py
    demos/joint_current_control.py
    demos/joint_position_control.py
    demos/joint_pwm_control.py
    demos/joint_trajectory_control.py
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
