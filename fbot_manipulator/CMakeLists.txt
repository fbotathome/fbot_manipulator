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
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(fbot_manipulator_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

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
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  control_msgs
  trajectory_msgs
  fbot_manipulator_msgs
  xarm_msgs
  yaml-cpp
)

target_link_libraries(manipulator_interface_node yaml-cpp)

add_executable(manipulation_task_server
  src/manipulation_task_server.cpp
  src/mtc/mtc_task.cpp
  src/mtc/mtc_pick_task.cpp
  src/mtc/mtc_place_task.cpp
  src/mtc/mtc_pick_and_place_task.cpp
)

ament_target_dependencies(manipulation_task_server
  rclcpp
  rclcpp_action
  moveit
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  tf2_geometry_msgs
  tf2_eigen
  fbot_manipulator_msgs
)

add_executable(add_object_collision src/collision/add_object_collision.cpp)
ament_target_dependencies(add_object_collision
  rclcpp
  visualization_msgs
  geometry_msgs
  moveit_msgs
  shape_msgs
  tf2_ros
  tf2_geometry_msgs
)
add_executable(shelf_collision src/collision/shelf_collision.cpp)
ament_target_dependencies(shelf_collision
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  geometry_msgs
)
install(TARGETS
  manipulator_interface_node
  manipulation_task_server
  add_object_collision
  shelf_collision
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY src/mesh DESTINATION share/${PROJECT_NAME})

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
