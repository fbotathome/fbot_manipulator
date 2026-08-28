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

find_package(sensor_msgs REQUIRED)

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
  src/mtc/mtc_pour_task.cpp
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

# Instalação dos arquivos de inicialização e configurações
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# --- LINHAS ADICIONADAS PARA CORRIGIR O ERRO ---
# Instalação dos executáveis C++ na pasta lib/fbot_manipulator
install(TARGETS
  manipulator_interface_node
  manipulation_task_server
  DESTINATION lib/${PROJECT_NAME}
)
# -----------------------------------------------

ament_package()

