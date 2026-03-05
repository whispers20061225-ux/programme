cmake_minimum_required(VERSION 3.10)
project(tactile_vision_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(latest_frame_relay_node src/latest_frame_relay_node.cpp)
ament_target_dependencies(latest_frame_relay_node rclcpp sensor_msgs)

install(TARGETS
  latest_frame_relay_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
