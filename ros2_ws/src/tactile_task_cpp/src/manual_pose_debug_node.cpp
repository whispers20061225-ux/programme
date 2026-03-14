#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

class ManualPoseDebugNode : public rclcpp::Node
{
public:
  static std::shared_ptr<ManualPoseDebugNode> create()
  {
    auto node = std::shared_ptr<ManualPoseDebugNode>(new ManualPoseDebugNode());
    node->initialize_moveit();
    node->initialize_interactive_marker();
    node->publish_target_pose();
    node->publish_current_ee_pose(false);
    return node;
  }

private:
  ManualPoseDebugNode()
  : Node("manual_pose_debug_node")
  {
    this->declare_parameter<std::string>("base_frame", "world");
    this->declare_parameter<std::string>("arm_group", "arm");
    this->declare_parameter<std::string>("ee_link", "ee_link");
    this->declare_parameter<std::string>("home_named_target", "home");
    this->declare_parameter<bool>("move_to_home_before_test", false);
    this->declare_parameter<bool>("execute_plan", false);
    this->declare_parameter<bool>("auto_plan_on_mouse_up", true);
    this->declare_parameter<bool>("plan_on_startup", false);
    this->declare_parameter<bool>("initialize_target_from_named_target_pose", true);
    this->declare_parameter<bool>("initialize_target_from_current_ee_pose", true);
    this->declare_parameter<double>("startup_delay_sec", 1.0);
    this->declare_parameter<double>("planning_time_sec", 5.0);
    this->declare_parameter<int>("planning_attempts", 10);
    this->declare_parameter<double>("velocity_scaling", 0.6);
    this->declare_parameter<double>("acceleration_scaling", 0.6);
    this->declare_parameter<double>("goal_position_tolerance_m", 0.01);
    this->declare_parameter<double>("goal_orientation_tolerance_rad", 0.10);
    this->declare_parameter<double>("interactive_marker_scale", 0.14);
    this->declare_parameter<std::string>("interactive_marker_namespace", "manual_pose_debug");
    this->declare_parameter<std::vector<double>>("target_position_xyz", {0.18, 0.0, 0.24});
    this->declare_parameter<std::vector<double>>("target_rpy_deg", {0.0, 90.0, 0.0});
    this->declare_parameter<std::string>("target_pose_topic", "/debug/manual_pose/target_pose");
    this->declare_parameter<std::string>("current_ee_pose_topic", "/debug/manual_pose/current_ee_pose");

    base_frame_ = this->get_parameter("base_frame").as_string();
    arm_group_name_ = this->get_parameter("arm_group").as_string();
    ee_link_ = this->get_parameter("ee_link").as_string();
    home_named_target_ = this->get_parameter("home_named_target").as_string();
    move_to_home_before_test_ = this->get_parameter("move_to_home_before_test").as_bool();
    execute_plan_ = this->get_parameter("execute_plan").as_bool();
    auto_plan_on_mouse_up_ = this->get_parameter("auto_plan_on_mouse_up").as_bool();
    plan_on_startup_ = this->get_parameter("plan_on_startup").as_bool();
    initialize_target_from_named_target_pose_ =
      this->get_parameter("initialize_target_from_named_target_pose").as_bool();
    initialize_target_from_current_ee_pose_ =
      this->get_parameter("initialize_target_from_current_ee_pose").as_bool();
    startup_delay_sec_ = this->get_parameter("startup_delay_sec").as_double();
    planning_time_sec_ = this->get_parameter("planning_time_sec").as_double();
    planning_attempts_ = this->get_parameter("planning_attempts").as_int();
    velocity_scaling_ = this->get_parameter("velocity_scaling").as_double();
    acceleration_scaling_ = this->get_parameter("acceleration_scaling").as_double();
    goal_position_tolerance_m_ = this->get_parameter("goal_position_tolerance_m").as_double();
    goal_orientation_tolerance_rad_ =
      this->get_parameter("goal_orientation_tolerance_rad").as_double();
    interactive_marker_scale_ = this->get_parameter("interactive_marker_scale").as_double();
    interactive_marker_namespace_ =
      this->get_parameter("interactive_marker_namespace").as_string();
    target_position_xyz_ = this->get_parameter("target_position_xyz").as_double_array();
    target_rpy_deg_ = this->get_parameter("target_rpy_deg").as_double_array();

    const auto target_pose_topic = this->get_parameter("target_pose_topic").as_string();
    const auto current_ee_pose_topic = this->get_parameter("current_ee_pose_topic").as_string();
    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    target_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(target_pose_topic, latched_qos);
    current_ee_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(current_ee_pose_topic, latched_qos);

    target_pose_msg_ = build_target_pose();

    startup_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(startup_delay_sec_)),
      [this]() {
        startup_timer_->cancel();
        if (move_to_home_before_test_) {
          move_to_named_target(home_named_target_);
        }
        if (initialize_target_from_named_target_pose_) {
          initialize_target_pose_from_named_target(home_named_target_);
        } else if (initialize_target_from_current_ee_pose_) {
          initialize_target_pose_from_current_ee_pose();
        }
        if (plan_on_startup_) {
          plan_current_target("startup");
        }
      });

    RCLCPP_INFO(
      this->get_logger(),
      "manual_pose_debug_node started: execute_plan=%s auto_plan_on_mouse_up=%s plan_on_startup=%s init_from_named=%s init_from_current=%s target=(%.3f, %.3f, %.3f) target_rpy_deg=(%.1f, %.1f, %.1f)",
      execute_plan_ ? "true" : "false",
      auto_plan_on_mouse_up_ ? "true" : "false",
      plan_on_startup_ ? "true" : "false",
      initialize_target_from_named_target_pose_ ? "true" : "false",
      initialize_target_from_current_ee_pose_ ? "true" : "false",
      target_position_xyz_[0], target_position_xyz_[1], target_position_xyz_[2],
      target_rpy_deg_[0], target_rpy_deg_[1], target_rpy_deg_[2]);
  }

  void initialize_moveit()
  {
    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), arm_group_name_);
    arm_group_->setPlanningTime(planning_time_sec_);
    arm_group_->setNumPlanningAttempts(planning_attempts_);
    arm_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    arm_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    arm_group_->setPoseReferenceFrame(base_frame_);
    arm_group_->setEndEffectorLink(ee_link_);
    arm_group_->setGoalPositionTolerance(goal_position_tolerance_m_);
    arm_group_->setGoalOrientationTolerance(goal_orientation_tolerance_rad_);
    arm_group_->allowReplanning(true);

    RCLCPP_INFO(
      this->get_logger(),
      "manual pose MoveIt ready: arm_group=%s ee_link=%s tol_pos=%.3f tol_ori=%.3f",
      arm_group_name_.c_str(), ee_link_.c_str(),
      goal_position_tolerance_m_, goal_orientation_tolerance_rad_);
  }

  void initialize_interactive_marker()
  {
    marker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      interactive_marker_namespace_, shared_from_this());
    marker_server_->insert(build_interactive_marker(),
      std::bind(&ManualPoseDebugNode::handle_marker_feedback, this, std::placeholders::_1));
    marker_server_->applyChanges();
  }

  geometry_msgs::msg::Quaternion quaternion_from_rpy_deg(
    double roll_deg,
    double pitch_deg,
    double yaw_deg) const
  {
    const double roll_rad = roll_deg * M_PI / 180.0;
    const double pitch_rad = pitch_deg * M_PI / 180.0;
    const double yaw_rad = yaw_deg * M_PI / 180.0;

    const Eigen::Quaterniond yaw_q(
      Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
    const Eigen::Quaterniond pitch_q(
      Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()));
    const Eigen::Quaterniond roll_q(
      Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX()));
    Eigen::Quaterniond result = yaw_q * pitch_q * roll_q;
    result.normalize();

    geometry_msgs::msg::Quaternion quaternion;
    quaternion.x = result.x();
    quaternion.y = result.y();
    quaternion.z = result.z();
    quaternion.w = result.w();
    return quaternion;
  }

  Eigen::Vector3d rpy_deg_from_quaternion(const geometry_msgs::msg::Quaternion & quaternion) const
  {
    const Eigen::Quaterniond q(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    const Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    return Eigen::Vector3d(
      euler.x() * 180.0 / M_PI,
      euler.y() * 180.0 / M_PI,
      euler.z() * 180.0 / M_PI);
  }

  geometry_msgs::msg::PoseStamped build_target_pose() const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = base_frame_;
    pose.pose.position.x = target_position_xyz_[0];
    pose.pose.position.y = target_position_xyz_[1];
    pose.pose.position.z = target_position_xyz_[2];
    pose.pose.orientation = quaternion_from_rpy_deg(
      target_rpy_deg_[0], target_rpy_deg_[1], target_rpy_deg_[2]);
    return pose;
  }

  visualization_msgs::msg::Marker build_visible_marker() const
  {
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.scale.x = interactive_marker_scale_ * 0.18;
    marker.scale.y = interactive_marker_scale_ * 0.05;
    marker.scale.z = interactive_marker_scale_ * 0.05;
    marker.color.r = 0.95f;
    marker.color.g = 0.45f;
    marker.color.b = 0.15f;
    marker.color.a = 0.95f;
    return marker;
  }

  visualization_msgs::msg::InteractiveMarkerControl build_control(
    const std::string & name,
    uint8_t interaction_mode,
    double ox,
    double oy,
    double oz,
    double ow = 1.0) const
  {
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.name = name;
    control.orientation.w = ow;
    control.orientation.x = ox;
    control.orientation.y = oy;
    control.orientation.z = oz;
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
    control.interaction_mode = interaction_mode;
    return control;
  }

  visualization_msgs::msg::InteractiveMarker build_interactive_marker() const
  {
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = base_frame_;
    marker.name = "manual_target_pose";
    marker.description = "Drag target pose, release mouse to plan";
    marker.scale = interactive_marker_scale_;
    marker.pose = target_pose_msg_.pose;

    visualization_msgs::msg::InteractiveMarkerControl visible_control;
    visible_control.always_visible = true;
    visible_control.markers.push_back(build_visible_marker());
    marker.controls.push_back(visible_control);

    visualization_msgs::msg::InteractiveMarkerControl move_rotate_control;
    move_rotate_control.name = "move_rotate_3d";
    move_rotate_control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;
    marker.controls.push_back(move_rotate_control);

    marker.controls.push_back(build_control("rotate_x", visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS, 1.0, 0.0, 0.0));
    marker.controls.push_back(build_control("move_x", visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS, 1.0, 0.0, 0.0));
    marker.controls.push_back(build_control("rotate_y", visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS, 0.0, 1.0, 0.0));
    marker.controls.push_back(build_control("move_y", visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS, 0.0, 1.0, 0.0));
    marker.controls.push_back(build_control("rotate_z", visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS, 0.0, 0.0, 1.0));
    marker.controls.push_back(build_control("move_z", visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS, 0.0, 0.0, 1.0));
    return marker;
  }

  void update_marker_pose()
  {
    if (!marker_server_) {
      return;
    }
    marker_server_->setPose("manual_target_pose", target_pose_msg_.pose);
    marker_server_->applyChanges();
  }

  void publish_target_pose()
  {
    auto pose = target_pose_msg_;
    pose.header.stamp = this->get_clock()->now();
    target_pose_pub_->publish(pose);
  }

  bool publish_current_ee_pose(bool verbose)
  {
    auto current_state = arm_group_->getCurrentState(0.1);
    if (!current_state) {
      if (verbose) {
        RCLCPP_WARN(this->get_logger(), "manual pose debug: failed to fetch current robot state");
      }
      return false;
    }

    auto pose = arm_group_->getCurrentPose(ee_link_);
    pose.header.frame_id = base_frame_;
    pose.header.stamp = this->get_clock()->now();
    current_ee_pose_pub_->publish(pose);
    if (verbose) {
      RCLCPP_INFO(
        this->get_logger(),
        "current ee pose: xyz=(%.3f, %.3f, %.3f)",
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    }
    return true;
  }

  std::string pose_summary(const geometry_msgs::msg::Pose & pose) const
  {
    const auto rpy_deg = rpy_deg_from_quaternion(pose.orientation);
    std::ostringstream stream;
    stream << "xyz=("
           << std::fixed << std::setprecision(3)
           << pose.position.x << ", "
           << pose.position.y << ", "
           << pose.position.z << ") rpy_deg=("
           << std::setprecision(1)
           << rpy_deg.x() << ", "
           << rpy_deg.y() << ", "
           << rpy_deg.z() << ")";
    return stream.str();
  }

  bool move_to_named_target(const std::string & target_name)
  {
    arm_group_->setStartStateToCurrentState();
    arm_group_->setNamedTarget(target_name);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(arm_group_->plan(plan));
    if (!planned) {
      RCLCPP_WARN(
        this->get_logger(),
        "manual pose debug: planning to named target %s failed",
        target_name.c_str());
      return false;
    }
    if (!execute_plan_) {
      RCLCPP_INFO(
        this->get_logger(),
        "manual pose debug: named target %s planning success (execution disabled)",
        target_name.c_str());
      return true;
    }
    const bool executed = static_cast<bool>(arm_group_->execute(plan));
    if (!executed) {
      RCLCPP_WARN(
        this->get_logger(),
        "manual pose debug: executing named target %s failed",
        target_name.c_str());
      return false;
    }
    publish_current_ee_pose(false);
    return true;
  }

  geometry_msgs::msg::Pose pose_from_eigen_transform(const Eigen::Isometry3d & transform) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();
    const Eigen::Quaterniond q(transform.rotation());
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
  }

  bool initialize_target_pose_from_named_target(const std::string & target_name)
  {
    const auto joint_values = arm_group_->getNamedTargetValues(target_name);
    if (joint_values.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "manual pose debug: named target %s not found for initial target pose",
        target_name.c_str());
      return false;
    }

    auto robot_model = arm_group_->getRobotModel();
    if (!robot_model) {
      RCLCPP_WARN(
        this->get_logger(),
        "manual pose debug: failed to fetch robot model for named target pose");
      return false;
    }

    moveit::core::RobotState state(robot_model);
    state.setToDefaultValues();
    state.setVariablePositions(joint_values);
    state.update();

    target_pose_msg_.header.frame_id = base_frame_;
    target_pose_msg_.header.stamp = this->get_clock()->now();
    target_pose_msg_.pose = pose_from_eigen_transform(state.getGlobalLinkTransform(ee_link_));
    update_marker_pose();
    publish_target_pose();

    RCLCPP_INFO(
      this->get_logger(),
      "manual pose debug: initialized target pose from named target %s as %s",
      target_name.c_str(),
      pose_summary(target_pose_msg_.pose).c_str());
    return true;
  }

  bool initialize_target_pose_from_current_ee_pose()
  {
    auto current_state = arm_group_->getCurrentState(1.0);
    if (!current_state) {
      RCLCPP_WARN(
        this->get_logger(),
        "manual pose debug: failed to fetch current state for initial target pose");
      return false;
    }

    auto current_pose = arm_group_->getCurrentPose(ee_link_);
    target_pose_msg_.header.frame_id = base_frame_;
    target_pose_msg_.header.stamp = this->get_clock()->now();
    target_pose_msg_.pose = current_pose.pose;
    update_marker_pose();
    publish_target_pose();
    publish_current_ee_pose(false);

    RCLCPP_INFO(
      this->get_logger(),
      "manual pose debug: initialized target pose from current ee pose %s",
      pose_summary(target_pose_msg_.pose).c_str());
    return true;
  }

  bool plan_current_target(const std::string & reason)
  {
    publish_target_pose();
    publish_current_ee_pose(true);

    RCLCPP_INFO(
      this->get_logger(),
      "manual pose debug [%s]: target %s",
      reason.c_str(),
      pose_summary(target_pose_msg_.pose).c_str());

    arm_group_->setStartStateToCurrentState();
    arm_group_->setPoseTarget(target_pose_msg_.pose, ee_link_);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(arm_group_->plan(plan));
    arm_group_->clearPoseTargets();

    if (!planned) {
      RCLCPP_WARN(
        this->get_logger(),
        "manual pose debug [%s]: planning failed",
        reason.c_str());
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "manual pose debug [%s]: planning success",
      reason.c_str());
    if (!execute_plan_) {
      return true;
    }

    const bool executed = static_cast<bool>(arm_group_->execute(plan));
    if (!executed) {
      RCLCPP_WARN(
        this->get_logger(),
        "manual pose debug [%s]: execution failed",
        reason.c_str());
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "manual pose debug [%s]: execution success",
      reason.c_str());
    publish_current_ee_pose(false);
    return true;
  }

  void handle_marker_feedback(
    const interactive_markers::InteractiveMarkerServer::FeedbackConstSharedPtr & feedback)
  {
    target_pose_msg_.header.frame_id = base_frame_;
    target_pose_msg_.header.stamp = this->get_clock()->now();
    target_pose_msg_.pose = feedback->pose;
    publish_target_pose();

    if (
      feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE ||
      feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "manual pose marker update: %s",
        pose_summary(target_pose_msg_.pose).c_str());
    }

    if (
      auto_plan_on_mouse_up_ &&
      feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP)
    {
      plan_current_target("marker_mouse_up");
    }
  }

  std::string base_frame_;
  std::string arm_group_name_;
  std::string ee_link_;
  std::string home_named_target_;
  bool move_to_home_before_test_{false};
  bool execute_plan_{false};
  bool auto_plan_on_mouse_up_{true};
  bool plan_on_startup_{false};
  bool initialize_target_from_named_target_pose_{true};
  bool initialize_target_from_current_ee_pose_{true};
  double startup_delay_sec_{1.0};
  double planning_time_sec_{5.0};
  int planning_attempts_{10};
  double velocity_scaling_{0.6};
  double acceleration_scaling_{0.6};
  double goal_position_tolerance_m_{0.01};
  double goal_orientation_tolerance_rad_{0.10};
  double interactive_marker_scale_{0.14};
  std::string interactive_marker_namespace_;
  std::vector<double> target_position_xyz_;
  std::vector<double> target_rpy_deg_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_ee_pose_pub_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  geometry_msgs::msg::PoseStamped target_pose_msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = ManualPoseDebugNode::create();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
