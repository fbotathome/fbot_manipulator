cmake_minimum_required(VERSION 3.8)
project(fbot_manipulator_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveGripper.srv"
  "srv/MoveToNamedTarget.srv"
  "srv/MoveJoint.srv"
  "srv/MoveToPose.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# Export runtime dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs geometry_msgs sensor_msgs)

# Mark this package as an ament package
ament_package()
