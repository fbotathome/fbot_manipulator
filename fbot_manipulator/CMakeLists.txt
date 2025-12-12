cmake_minimum_required(VERSION 3.8)
project(fbot_manipulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(fbot_manipulator_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

add_executable(manipulator_interface_node
  src/manipulator_interface_node.cpp
  src/motion_primitives/motion_primitives_base.cpp
  src/motion_primitives/motion_primitives_xarm.cpp 
  src/utils.cpp
)
ament_target_dependencies(manipulator_interface_node
  rclcpp
  moveit_ros_planning_interface
  control_msgs
  trajectory_msgs
  fbot_manipulator_msgs
  xarm_msgs
  yaml-cpp
)

target_link_libraries(manipulator_interface_node yaml-cpp)

install(TARGETS
  manipulator_interface_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
