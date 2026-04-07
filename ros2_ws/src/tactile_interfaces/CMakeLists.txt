cmake_minimum_required(VERSION 3.8)
project(tactile_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectionResult.msg"
  "msg/SemanticTask.msg"
  "msg/MaskChannel.msg"
  "msg/AffordanceGroundingResult.msg"
  "msg/TaskGraspCandidate.msg"
  "msg/TaskGoal.msg"
  "msg/TaskExecutionStatus.msg"
  "msg/GraspProposal.msg"
  "msg/GraspProposalArray.msg"
  "msg/TactileRaw.msg"
  "msg/SystemHealth.msg"
  "msg/ArmState.msg"
  "msg/GripperState.msg"
  "action/MoveArmJoints.action"
  "action/ExecuteDemo.action"
  "action/ExecuteTask.action"
  "action/SearchTarget.action"
  "action/GenerateTaskGrasps.action"
  "action/VerifyGrasp.action"
  "srv/SetGripperForce.srv"
  "srv/MoveArmJoint.srv"
  "srv/MoveArmJoints.srv"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
