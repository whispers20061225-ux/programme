cmake_minimum_required(VERSION 3.10)
project(tactile_task_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tactile_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(sim_pick_task_node src/sim_pick_task_node.cpp)
ament_target_dependencies(
  sim_pick_task_node
  geometry_msgs
  interactive_markers
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  shape_msgs
  std_msgs
  std_srvs
  tactile_interfaces
  visualization_msgs
)

add_executable(manual_pose_debug_node src/manual_pose_debug_node.cpp)
ament_target_dependencies(
  manual_pose_debug_node
  geometry_msgs
  interactive_markers
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  shape_msgs
  std_msgs
  std_srvs
  tactile_interfaces
  visualization_msgs
)

install(TARGETS
  sim_pick_task_node
  manual_pose_debug_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
