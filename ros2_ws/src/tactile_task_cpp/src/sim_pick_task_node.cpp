#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <Eigen/Geometry>
#include <iomanip>
#include <memory>
#include <mutex>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tactile_interfaces/msg/grasp_proposal_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

class SimPickTaskNode : public rclcpp::Node
{
public:
  static std::shared_ptr<SimPickTaskNode> create()
  {
    auto node = std::shared_ptr<SimPickTaskNode>(new SimPickTaskNode());
    node->initialize_moveit();
    return node;
  }

private:
  struct ApproachCandidate
  {
    std::array<double, 3> pregrasp;
    std::array<double, 3> grasp;
    geometry_msgs::msg::Quaternion pregrasp_orientation;
    geometry_msgs::msg::Quaternion grasp_orientation;
  };

  struct PregraspVariant
  {
    std::array<double, 3> pregrasp;
    std::array<double, 3> grasp;
    geometry_msgs::msg::Quaternion pregrasp_orientation;
    geometry_msgs::msg::Quaternion grasp_orientation;
    std::string label_suffix;
  };

  struct JointConstraintAttempt
  {
    double primary_joint_delta_rad{-1.0};
    double wrist_joint_delta_rad{-1.0};
    std::string label_suffix;
  };

  struct ExternalProposalMetadata
  {
    std::array<double, 3> contact_point_1{};
    std::array<double, 3> contact_point_2{};
    std::array<double, 3> grasp_center{};
    std::array<double, 3> approach_direction{};
    std::array<double, 3> closing_direction{};
    double grasp_width_m{0.0};
    double confidence_score{0.0};
    double semantic_score{0.0};
    std::size_t candidate_rank{0};
    std::string task_constraint_tag;
  };

  struct PlannedApproachOption
  {
    std::size_t candidate_index{0};
    std::array<double, 3> stage{};
    std::array<double, 3> pregrasp{};
    std::array<double, 3> grasp{};
    geometry_msgs::msg::Quaternion stage_orientation;
    geometry_msgs::msg::Quaternion pregrasp_orientation;
    geometry_msgs::msg::Quaternion grasp_orientation;
    std::string stage_label;
    std::string pregrasp_label;
    moveit::planning_interface::MoveGroupInterface::Plan stage_plan;
    moveit::planning_interface::MoveGroupInterface::Plan pregrasp_plan;
    double score{std::numeric_limits<double>::lowest()};
    double camera_penalty{0.0};
    double camera_stability_bonus{0.0};
    double primary_joint_excursion_cost{0.0};
    double secondary_joint_excursion_cost{0.0};
    double wrist_joint_excursion_cost{0.0};
    double external_semantic_score{0.0};
    double external_confidence_score{0.0};
    double external_proposal_bonus{0.0};
    std::optional<ExternalProposalMetadata> external_metadata;
    bool skip_stage{false};
  };

  struct PlannedApproachEvaluation
  {
    std::vector<PlannedApproachOption> feasible_options;
    std::size_t tested_options{0};
    std::size_t feasible_count{0};
  };

  struct ExternalEvaluationDebugInfo
  {
    std::size_t candidate_index{0};
    std::string candidate_label;
    std::string status;
    std::string variant_label;
    std::array<double, 3> stage{};
    std::array<double, 3> pregrasp{};
    std::array<double, 3> grasp{};
    bool feasible{false};
    double score{0.0};
    double semantic_score{0.0};
    double confidence_score{0.0};
    double proposal_bonus{0.0};
    std::size_t stage_plan_failures{0};
    std::size_t stage_end_state_failures{0};
    std::size_t pregrasp_plan_failures{0};
    std::size_t pregrasp_end_state_failures{0};
    std::optional<ExternalProposalMetadata> external_metadata;
  };

  SimPickTaskNode()
  : Node("sim_pick_task_node")
  {
    this->declare_parameter<std::string>("target_pose_topic", "/sim/perception/target_pose");
    this->declare_parameter<std::string>("target_locked_topic", "/sim/perception/target_locked");
    this->declare_parameter<std::string>("refresh_grasp_candidates_topic", "/grasp/refresh_request");
    this->declare_parameter<std::string>("pick_active_topic", "/sim/task/pick_active");
    this->declare_parameter<std::string>("pick_status_topic", "/sim/task/pick_status");
    this->declare_parameter<std::string>("execute_pick_service", "/task/execute_pick");
    this->declare_parameter<std::string>("reset_pick_session_service", "/task/reset_pick_session");
    this->declare_parameter<std::string>("return_home_service", "/task/return_home");
    this->declare_parameter<std::string>("return_named_target", "home");
    this->declare_parameter<std::string>("selected_pregrasp_pose_topic", "/grasp/selected_pregrasp_pose");
    this->declare_parameter<std::string>("selected_grasp_pose_topic", "/grasp/selected_grasp_pose");
    this->declare_parameter<std::string>("external_pregrasp_pose_array_topic", "/grasp/candidate_pregrasp_poses");
    this->declare_parameter<std::string>("external_grasp_pose_array_topic", "/grasp/candidate_grasp_poses");
    this->declare_parameter<std::string>("external_grasp_proposal_array_topic", "/grasp/candidate_grasp_proposals");
    this->declare_parameter<std::string>("execution_debug_markers_topic", "/grasp/execution_debug_markers");
    this->declare_parameter<std::string>("base_frame", "world");
    this->declare_parameter<std::string>("ee_link", "ee_link");
    this->declare_parameter<std::string>("arm_group", "arm");
    this->declare_parameter<std::string>("gripper_group", "gripper");
    this->declare_parameter<double>("planning_time_sec", 5.0);
    this->declare_parameter<int>("planning_attempts", 10);
    this->declare_parameter<double>("velocity_scaling", 1.0);
    this->declare_parameter<double>("acceleration_scaling", 0.9);
    this->declare_parameter<double>("gripper_velocity_scaling", 1.0);
    this->declare_parameter<double>("gripper_acceleration_scaling", 1.0);
    this->declare_parameter<bool>("enforce_upright_orientation", true);
    this->declare_parameter<bool>("lock_joint_after_target_lock", true);
    this->declare_parameter<std::string>("lock_joint_name", "arm_joint1");
    this->declare_parameter<double>("lock_joint_max_delta_deg", 15.0);
    this->declare_parameter<double>("lock_joint_relaxed_delta_deg", 25.0);
    this->declare_parameter<bool>("lock_wrist_joint_after_target_lock", true);
    this->declare_parameter<std::string>("wrist_joint_name", "arm_joint5");
    this->declare_parameter<double>("wrist_joint_max_delta_deg", 90.0);
    this->declare_parameter<double>("wrist_joint_relaxed_delta_deg", 90.0);
    this->declare_parameter<double>("orientation_constraint_tolerance_rad", 1.20);
    this->declare_parameter<bool>("stage_position_only_target", true);
    this->declare_parameter<bool>("pregrasp_position_only_target", true);
    this->declare_parameter<double>("pregrasp_backoff_m", 0.10);
    this->declare_parameter<double>("pregrasp_lift_m", 0.08);
    this->declare_parameter<double>("grasp_backoff_m", 0.04);
    this->declare_parameter<double>("grasp_lift_m", 0.03);
    this->declare_parameter<double>("lift_distance_m", 0.05);
    this->declare_parameter<bool>("debug_stop_after_pregrasp", false);
    this->declare_parameter<bool>("debug_execute_ranked_candidates", false);
    this->declare_parameter<int>("debug_ranked_candidate_limit", 12);
    this->declare_parameter<std::string>("debug_return_named_target", "home");
    this->declare_parameter<double>("grasp_height_offset_m", 0.0);
    this->declare_parameter<double>("target_radius_m", 0.025);
    this->declare_parameter<double>("target_height_m", 0.10);
    this->declare_parameter<std::string>("camera_scoring_link", "camera_depth_optical_frame");
    this->declare_parameter<double>("camera_forward_alignment_threshold", 0.60);
    this->declare_parameter<double>("camera_upright_alignment_threshold", 0.0);
    this->declare_parameter<double>("camera_forward_penalty_weight", 10.0);
    this->declare_parameter<double>("camera_upright_penalty_weight", 14.0);
    this->declare_parameter<double>("camera_trajectory_tilt_penalty_weight", 10.0);
    this->declare_parameter<double>("camera_flip_penalty_weight", 20.0);
    this->declare_parameter<double>("camera_stable_tilt_reward_deg", 5.0);
    this->declare_parameter<double>("camera_stable_reward_bonus", 2.5);
    this->declare_parameter<double>("pregrasp_score_base", 30.0);
    this->declare_parameter<double>("pregrasp_score_floor", 1.0);
    this->declare_parameter<double>("plan_duration_penalty_weight", 1.8);
    this->declare_parameter<double>("path_distance_penalty_weight", 6.0);
    this->declare_parameter<double>("approach_distance_penalty_weight", 4.0);
    this->declare_parameter<double>("primary_joint_delta_penalty_weight", 2.6);
    this->declare_parameter<double>("wrist_joint_delta_penalty_weight", 0.0);
    this->declare_parameter<double>("primary_joint_excursion_penalty_weight", 8.5);
    this->declare_parameter<std::string>("secondary_joint_name", "arm_joint2");
    this->declare_parameter<double>("secondary_joint_excursion_penalty_weight", 0.0);
    this->declare_parameter<double>("wrist_joint_excursion_penalty_weight", 0.0);
    this->declare_parameter<double>("clearance_penalty_weight", 6.0);
    this->declare_parameter<double>("grasp_base_roll_deg", 0.0);
    this->declare_parameter<double>("grasp_base_pitch_deg", 90.0);
    this->declare_parameter<double>("grasp_yaw_offset_deg", 0.0);
    this->declare_parameter<bool>("prefer_external_grasp_candidates", false);
    this->declare_parameter<bool>("require_external_grasp_candidates", false);
    this->declare_parameter<double>("external_grasp_pose_timeout_sec", 3.0);
    this->declare_parameter<double>("refresh_grasp_candidates_retry_sec", 1.5);
    this->declare_parameter<double>("external_semantic_score_weight", 0.8);
    this->declare_parameter<double>("external_confidence_score_weight", 0.6);
    this->declare_parameter<bool>("include_target_collision_object", false);
    this->declare_parameter<bool>("require_user_confirmation", false);
    this->declare_parameter<std::vector<double>>("table_size_m", {1.0, 0.8, 0.04});
    this->declare_parameter<std::vector<double>>("table_pose_xyz", {0.5, 0.0, 0.38});

    target_pose_topic_ = this->get_parameter("target_pose_topic").as_string();
    target_locked_topic_ = this->get_parameter("target_locked_topic").as_string();
    refresh_grasp_candidates_topic_ =
      this->get_parameter("refresh_grasp_candidates_topic").as_string();
    pick_active_topic_ = this->get_parameter("pick_active_topic").as_string();
    pick_status_topic_ = this->get_parameter("pick_status_topic").as_string();
    execute_pick_service_ = this->get_parameter("execute_pick_service").as_string();
    reset_pick_session_service_ = this->get_parameter("reset_pick_session_service").as_string();
    return_home_service_ = this->get_parameter("return_home_service").as_string();
    return_named_target_ = this->get_parameter("return_named_target").as_string();
    selected_pregrasp_pose_topic_ =
      this->get_parameter("selected_pregrasp_pose_topic").as_string();
    selected_grasp_pose_topic_ =
      this->get_parameter("selected_grasp_pose_topic").as_string();
    external_pregrasp_pose_array_topic_ =
      this->get_parameter("external_pregrasp_pose_array_topic").as_string();
    external_grasp_pose_array_topic_ =
      this->get_parameter("external_grasp_pose_array_topic").as_string();
    external_grasp_proposal_array_topic_ =
      this->get_parameter("external_grasp_proposal_array_topic").as_string();
    execution_debug_markers_topic_ =
      this->get_parameter("execution_debug_markers_topic").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    ee_link_ = this->get_parameter("ee_link").as_string();
    arm_group_name_ = this->get_parameter("arm_group").as_string();
    gripper_group_name_ = this->get_parameter("gripper_group").as_string();
    planning_time_sec_ = this->get_parameter("planning_time_sec").as_double();
    planning_attempts_ = this->get_parameter("planning_attempts").as_int();
    velocity_scaling_ = this->get_parameter("velocity_scaling").as_double();
    acceleration_scaling_ = this->get_parameter("acceleration_scaling").as_double();
    gripper_velocity_scaling_ = this->get_parameter("gripper_velocity_scaling").as_double();
    gripper_acceleration_scaling_ =
      this->get_parameter("gripper_acceleration_scaling").as_double();
    enforce_upright_orientation_ =
      this->get_parameter("enforce_upright_orientation").as_bool();
    lock_joint_after_target_lock_ =
      this->get_parameter("lock_joint_after_target_lock").as_bool();
    lock_joint_name_ = this->get_parameter("lock_joint_name").as_string();
    lock_joint_max_delta_deg_ = this->get_parameter("lock_joint_max_delta_deg").as_double();
    lock_joint_relaxed_delta_deg_ =
      this->get_parameter("lock_joint_relaxed_delta_deg").as_double();
    lock_wrist_joint_after_target_lock_ =
      this->get_parameter("lock_wrist_joint_after_target_lock").as_bool();
    wrist_joint_name_ = this->get_parameter("wrist_joint_name").as_string();
    wrist_joint_max_delta_deg_ =
      this->get_parameter("wrist_joint_max_delta_deg").as_double();
    wrist_joint_relaxed_delta_deg_ =
      this->get_parameter("wrist_joint_relaxed_delta_deg").as_double();
    orientation_constraint_tolerance_rad_ =
      this->get_parameter("orientation_constraint_tolerance_rad").as_double();
    stage_position_only_target_ =
      this->get_parameter("stage_position_only_target").as_bool();
    pregrasp_position_only_target_ =
      this->get_parameter("pregrasp_position_only_target").as_bool();
    pregrasp_backoff_m_ = this->get_parameter("pregrasp_backoff_m").as_double();
    pregrasp_lift_m_ = this->get_parameter("pregrasp_lift_m").as_double();
    grasp_backoff_m_ = this->get_parameter("grasp_backoff_m").as_double();
    grasp_lift_m_ = this->get_parameter("grasp_lift_m").as_double();
    lift_distance_m_ = this->get_parameter("lift_distance_m").as_double();
    debug_stop_after_pregrasp_ =
      this->get_parameter("debug_stop_after_pregrasp").as_bool();
    debug_execute_ranked_candidates_ =
      this->get_parameter("debug_execute_ranked_candidates").as_bool();
    debug_ranked_candidate_limit_ =
      this->get_parameter("debug_ranked_candidate_limit").as_int();
    debug_return_named_target_ =
      this->get_parameter("debug_return_named_target").as_string();
    grasp_height_offset_m_ = this->get_parameter("grasp_height_offset_m").as_double();
    target_radius_m_ = this->get_parameter("target_radius_m").as_double();
    target_height_m_ = this->get_parameter("target_height_m").as_double();
    camera_scoring_link_ = this->get_parameter("camera_scoring_link").as_string();
    camera_forward_alignment_threshold_ =
      this->get_parameter("camera_forward_alignment_threshold").as_double();
    camera_upright_alignment_threshold_ =
      this->get_parameter("camera_upright_alignment_threshold").as_double();
    camera_forward_penalty_weight_ =
      this->get_parameter("camera_forward_penalty_weight").as_double();
    camera_upright_penalty_weight_ =
      this->get_parameter("camera_upright_penalty_weight").as_double();
    camera_trajectory_tilt_penalty_weight_ =
      this->get_parameter("camera_trajectory_tilt_penalty_weight").as_double();
    camera_flip_penalty_weight_ =
      this->get_parameter("camera_flip_penalty_weight").as_double();
    camera_stable_tilt_reward_deg_ =
      this->get_parameter("camera_stable_tilt_reward_deg").as_double();
    camera_stable_reward_bonus_ =
      this->get_parameter("camera_stable_reward_bonus").as_double();
    pregrasp_score_base_ =
      this->get_parameter("pregrasp_score_base").as_double();
    pregrasp_score_floor_ =
      this->get_parameter("pregrasp_score_floor").as_double();
    plan_duration_penalty_weight_ =
      this->get_parameter("plan_duration_penalty_weight").as_double();
    path_distance_penalty_weight_ =
      this->get_parameter("path_distance_penalty_weight").as_double();
    approach_distance_penalty_weight_ =
      this->get_parameter("approach_distance_penalty_weight").as_double();
    primary_joint_delta_penalty_weight_ =
      this->get_parameter("primary_joint_delta_penalty_weight").as_double();
    wrist_joint_delta_penalty_weight_ =
      this->get_parameter("wrist_joint_delta_penalty_weight").as_double();
    primary_joint_excursion_penalty_weight_ =
      this->get_parameter("primary_joint_excursion_penalty_weight").as_double();
    secondary_joint_name_ = this->get_parameter("secondary_joint_name").as_string();
    secondary_joint_excursion_penalty_weight_ =
      this->get_parameter("secondary_joint_excursion_penalty_weight").as_double();
    wrist_joint_excursion_penalty_weight_ =
      this->get_parameter("wrist_joint_excursion_penalty_weight").as_double();
    clearance_penalty_weight_ =
      this->get_parameter("clearance_penalty_weight").as_double();
    grasp_base_roll_deg_ = this->get_parameter("grasp_base_roll_deg").as_double();
    grasp_base_pitch_deg_ = this->get_parameter("grasp_base_pitch_deg").as_double();
    grasp_yaw_offset_deg_ = this->get_parameter("grasp_yaw_offset_deg").as_double();
    prefer_external_grasp_candidates_ =
      this->get_parameter("prefer_external_grasp_candidates").as_bool();
    require_external_grasp_candidates_ =
      this->get_parameter("require_external_grasp_candidates").as_bool();
    external_grasp_pose_timeout_sec_ =
      this->get_parameter("external_grasp_pose_timeout_sec").as_double();
    refresh_grasp_candidates_retry_sec_ =
      this->get_parameter("refresh_grasp_candidates_retry_sec").as_double();
    external_semantic_score_weight_ =
      this->get_parameter("external_semantic_score_weight").as_double();
    external_confidence_score_weight_ =
      this->get_parameter("external_confidence_score_weight").as_double();
    include_target_collision_object_ =
      this->get_parameter("include_target_collision_object").as_bool();
    require_user_confirmation_ =
      this->get_parameter("require_user_confirmation").as_bool();
    table_size_m_ = this->get_parameter("table_size_m").as_double_array();
    table_pose_xyz_ = this->get_parameter("table_pose_xyz").as_double_array();

    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      target_pose_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::scoped_lock<std::mutex> lock(target_mutex_);
        latest_target_pose_ = *msg;
        has_target_pose_ = true;
        if (target_locked_ && !pick_armed_.load() && !executing_.load() && !completed_.load()) {
          latched_target_pose_ = latest_target_pose_;
          has_latched_target_pose_ = true;
        }
      });

    target_locked_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      target_locked_topic_, 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        target_locked_ = msg->data;
        if (target_locked_) {
          if (suppress_target_relock_) {
            RCLCPP_INFO_THROTTLE(
              this->get_logger(),
              *this->get_clock(),
              3000,
              "target re-lock ignored while failure/manual home hold is active");
            return;
          }
          try_arm_pick_session();
        } else if (!pick_armed_.load() && !executing_.load() && !completed_.load()) {
          pick_armed_ = false;
          {
            std::scoped_lock<std::mutex> lock(target_mutex_);
            has_latched_target_pose_ = false;
          }
          publish_pick_active();
          publish_pick_status("idle", "target lock released");
        } else if (pick_armed_.load() && !executing_.load() && !completed_.load()) {
          publish_pick_status(
            "planning",
            "target lock released after execute request; keeping latched target pose");
          RCLCPP_INFO(
            this->get_logger(),
            "target lock released after execute request; preserving latched target pose");
        }
      });

    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    selected_pregrasp_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      selected_pregrasp_pose_topic_, latched_qos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::scoped_lock<std::mutex> lock(grasp_candidate_mutex_);
        latest_selected_pregrasp_pose_ = *msg;
        has_selected_pregrasp_pose_ = !msg->header.frame_id.empty();
        latest_external_grasp_update_sec_ = this->get_clock()->now().seconds();
      });

    selected_grasp_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      selected_grasp_pose_topic_, latched_qos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::scoped_lock<std::mutex> lock(grasp_candidate_mutex_);
        latest_selected_grasp_pose_ = *msg;
        has_selected_grasp_pose_ = !msg->header.frame_id.empty();
        latest_external_grasp_update_sec_ = this->get_clock()->now().seconds();
      });

    external_pregrasp_pose_array_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseArray>(
      external_pregrasp_pose_array_topic_, latched_qos,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        std::scoped_lock<std::mutex> lock(grasp_candidate_mutex_);
        latest_external_pregrasp_pose_array_ = *msg;
        has_external_pregrasp_pose_array_ = !msg->poses.empty();
        latest_external_grasp_update_sec_ = this->get_clock()->now().seconds();
      });

    external_grasp_pose_array_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseArray>(
      external_grasp_pose_array_topic_, latched_qos,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        std::scoped_lock<std::mutex> lock(grasp_candidate_mutex_);
        latest_external_grasp_pose_array_ = *msg;
        has_external_grasp_pose_array_ = !msg->poses.empty();
        latest_external_grasp_update_sec_ = this->get_clock()->now().seconds();
      });

    external_grasp_proposal_array_sub_ =
      this->create_subscription<tactile_interfaces::msg::GraspProposalArray>(
      external_grasp_proposal_array_topic_, latched_qos,
      [this](const tactile_interfaces::msg::GraspProposalArray::SharedPtr msg) {
        std::scoped_lock<std::mutex> lock(grasp_candidate_mutex_);
        latest_external_grasp_proposal_array_ = *msg;
        has_external_grasp_proposal_array_ = !msg->proposals.empty();
        latest_external_grasp_update_sec_ = this->get_clock()->now().seconds();
      });

    pick_active_pub_ = this->create_publisher<std_msgs::msg::Bool>(pick_active_topic_, 10);
    refresh_grasp_candidates_pub_ =
      this->create_publisher<std_msgs::msg::Bool>(refresh_grasp_candidates_topic_, 10);
    pick_status_pub_ = this->create_publisher<std_msgs::msg::String>(pick_status_topic_, latched_qos);
    execution_debug_markers_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
      execution_debug_markers_topic_, latched_qos);
    execute_pick_srv_ = this->create_service<std_srvs::srv::Trigger>(
      execute_pick_service_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        if (executing_.load()) {
          response->success = false;
          response->message = "pick sequence is already executing";
          return;
        }
        if (pick_armed_.load()) {
          response->success = true;
          response->message = "pick sequence is already armed";
          return;
        }
        if (!target_locked_) {
          response->success = false;
          response->message = "target is not locked";
          publish_pick_status("idle", response->message);
          return;
        }

        geometry_msgs::msg::PoseStamped target_pose;
        {
          std::scoped_lock<std::mutex> lock(target_mutex_);
          if (!has_target_pose_) {
            response->success = false;
            response->message = "target pose is unavailable";
            publish_pick_status("idle", response->message);
            return;
          }
          latched_target_pose_ = latest_target_pose_;
          has_latched_target_pose_ = true;
          target_pose = latched_target_pose_;
        }

        suppress_target_relock_ = false;
        pick_armed_ = true;
        completed_ = false;
        publish_pick_active();
        publish_pick_status(
          "planning",
          "execute requested; waiting for external candidates and planning");
        request_grasp_candidate_refresh("execute requested", true);
        response->success = true;
        std::ostringstream message;
        message << "pick armed from latched target pose ("
                << std::fixed << std::setprecision(3)
                << target_pose.pose.position.x << ", "
                << target_pose.pose.position.y << ", "
                << target_pose.pose.position.z << ")";
        response->message = message.str();
      });
    reset_pick_session_srv_ = this->create_service<std_srvs::srv::Trigger>(
      reset_pick_session_service_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        clear_locked_joint_references();
        suppress_target_relock_ = false;
        pick_armed_ = false;
        executing_ = false;
        completed_ = false;
        {
          std::scoped_lock<std::mutex> lock(target_mutex_);
          has_latched_target_pose_ = false;
        }
        publish_pick_active();
        if (target_locked_) {
          try_arm_pick_session();
          response->message = "pick session reset; waiting for explicit execute";
        } else {
          publish_pick_status("idle", "pick session reset");
          response->message = "pick session reset";
        }
        response->success = true;
      });
    return_home_srv_ = this->create_service<std_srvs::srv::Trigger>(
      return_home_service_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        suppress_target_relock_ = true;
        clear_locked_joint_references();
        pick_armed_ = false;
        executing_ = false;
        completed_ = false;
        {
          std::scoped_lock<std::mutex> lock(target_mutex_);
          has_latched_target_pose_ = false;
        }
        publish_pick_active();
        (void)execute_named_target(*gripper_group_, "open", "open gripper before return home");
        const bool ok = execute_named_target(
          *arm_group_, return_named_target_, "return arm to named home pose");
        publish_pick_status(
          ok ? "idle" : "error",
          ok ? "arm returned to named home pose" : "failed to return arm to named home pose");
        response->success = ok;
        response->message = ok ? "arm returned to named home pose" : "failed to return arm to named home pose";
      });
    publish_pick_active();
    publish_pick_status(
      "idle",
      "waiting for target lock and explicit execute");

    check_timer_ = this->create_wall_timer(
      300ms,
      [this]() {
        if (
          moveit_ready_ && pick_armed_.load() && has_latched_target_pose_ &&
          !executing_.load() && !completed_.load())
        {
          if (require_external_grasp_candidates_) {
            double age_sec = -1.0;
            std::size_t candidate_count = 0;
            if (!has_fresh_external_grasp_candidates(&age_sec, &candidate_count)) {
              request_grasp_candidate_refresh("planning waiting for fresh external grasp candidates");
              RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 3000,
                "waiting for external grasp candidates: count=%zu age=%.2fs timeout=%.2fs",
                candidate_count, age_sec, external_grasp_pose_timeout_sec_);
              return;
            }
          }
          pick_armed_ = false;
          executing_ = true;
          publish_pick_active();
          auto self = shared_from_this();
          std::thread([self]() {
            std::static_pointer_cast<SimPickTaskNode>(self)->run_pick_sequence();
          }).detach();
        }
      });

    RCLCPP_INFO(
      this->get_logger(),
      "sim_pick_task_node started: pose=%s locked=%s",
      target_pose_topic_.c_str(), target_locked_topic_.c_str());
  }

  void initialize_moveit()
  {
    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), arm_group_name_);
    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), gripper_group_name_);
    planning_scene_interface_ =
      std::make_unique<moveit::planning_interface::PlanningSceneInterface>();

    arm_group_->setPlanningTime(planning_time_sec_);
    arm_group_->setNumPlanningAttempts(planning_attempts_);
    arm_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    arm_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    arm_group_->setPoseReferenceFrame(base_frame_);
    arm_group_->setEndEffectorLink(ee_link_);
    arm_group_->setGoalPositionTolerance(0.02);
    arm_group_->setGoalJointTolerance(0.02);
    arm_group_->setGoalOrientationTolerance(orientation_constraint_tolerance_rad_);
    arm_group_->allowReplanning(true);

    gripper_group_->setPlanningTime(3.0);
    gripper_group_->setNumPlanningAttempts(5);
    gripper_group_->setMaxVelocityScalingFactor(gripper_velocity_scaling_);
    gripper_group_->setMaxAccelerationScalingFactor(gripper_acceleration_scaling_);
    gripper_group_->setGoalJointTolerance(0.02);
    gripper_group_->allowReplanning(true);

    moveit_ready_ = true;
    RCLCPP_DEBUG(
      this->get_logger(),
      "MoveIt clients ready: arm_group=%s gripper_group=%s ee_link=%s vel=%.2f acc=%.2f keep_orientation=%s tol=%.2f debug_stop_after_pregrasp=%s lock_joint=%s enabled=%s delta=%.1f relaxed=%.1f wrist_joint=%s enabled=%s delta=%.1f relaxed=%.1f",
      arm_group_name_.c_str(), gripper_group_name_.c_str(), ee_link_.c_str(),
      velocity_scaling_, acceleration_scaling_,
      enforce_upright_orientation_ ? "true" : "false", orientation_constraint_tolerance_rad_,
      debug_stop_after_pregrasp_ ? "true" : "false",
      lock_joint_name_.c_str(),
      lock_joint_after_target_lock_ ? "true" : "false",
      lock_joint_max_delta_deg_, lock_joint_relaxed_delta_deg_,
      wrist_joint_name_.c_str(),
      lock_wrist_joint_after_target_lock_ ? "true" : "false",
      wrist_joint_max_delta_deg_, wrist_joint_relaxed_delta_deg_);
    RCLCPP_INFO(
      this->get_logger(),
      "planning target modes: stage_position_only=%s pregrasp_position_only=%s",
      stage_position_only_target_ ? "true" : "false",
      pregrasp_position_only_target_ ? "true" : "false");
  }

  static std::string escape_json_string(const std::string & value)
  {
    std::ostringstream escaped;
    for (const char ch : value) {
      switch (ch) {
        case '\\':
          escaped << "\\\\";
          break;
        case '"':
          escaped << "\\\"";
          break;
        case '\n':
          escaped << "\\n";
          break;
        case '\r':
          escaped << "\\r";
          break;
        case '\t':
          escaped << "\\t";
          break;
        default:
          escaped << ch;
          break;
      }
    }
    return escaped.str();
  }

  void publish_pick_active()
  {
    std_msgs::msg::Bool msg;
    msg.data = pick_armed_.load() || executing_.load() || completed_.load();
    pick_active_pub_->publish(msg);
  }

  void publish_pick_status(
    const std::string & phase,
    const std::string & message,
    int progress_current = -1,
    int progress_total = -1,
    const std::string & progress_stage = "")
  {
    std_msgs::msg::String msg;
    std::ostringstream payload;
    payload << "{"
            << "\"phase\":\"" << escape_json_string(phase) << "\","
            << "\"message\":\"" << escape_json_string(message) << "\","
            << "\"require_user_confirmation\":"
            << (require_user_confirmation_ ? "true" : "false") << ","
            << "\"target_locked\":" << (target_locked_ ? "true" : "false") << ","
            << "\"pick_armed\":" << (pick_armed_.load() ? "true" : "false") << ","
            << "\"executing\":" << (executing_.load() ? "true" : "false") << ","
            << "\"completed\":" << (completed_.load() ? "true" : "false") << ","
            << "\"has_latched_target_pose\":"
            << (has_latched_target_pose_ ? "true" : "false");
    if (progress_total > 0 && progress_current >= 0) {
      const int bounded_current = std::max(0, std::min(progress_current, progress_total));
      const int progress_percent = static_cast<int>(
        std::lround(100.0 * static_cast<double>(bounded_current) /
        static_cast<double>(std::max(1, progress_total))));
      payload << ",\"progress_current\":" << bounded_current
              << ",\"progress_total\":" << progress_total
              << ",\"progress_percent\":" << progress_percent
              << ",\"progress_stage\":\"" << escape_json_string(progress_stage) << "\"";
    }
    payload << "}";
    msg.data = payload.str();
    pick_status_pub_->publish(msg);
  }

  void run_pick_sequence()
  {
    geometry_msgs::msg::PoseStamped target_pose;
    if (!consume_latched_target_pose(target_pose)) {
      return fail_sequence("missing latched target pose");
    }

    RCLCPP_INFO(
      this->get_logger(),
      "pick sequence starting from target pose (%.3f, %.3f, %.3f)",
      target_pose.pose.position.x,
      target_pose.pose.position.y,
      target_pose.pose.position.z);
    publish_pick_status("planning", "planning grasp sequence from latched target pose");

    capture_locked_joint_references();

    apply_scene(target_pose, include_target_collision_object_);
    std::this_thread::sleep_for(300ms);

    const auto current_pose = arm_group_->getCurrentPose(ee_link_).pose;
    RCLCPP_DEBUG(
      this->get_logger(),
      "current ee pose before pick: (%.3f, %.3f, %.3f)",
      current_pose.position.x,
      current_pose.position.y,
      current_pose.position.z);

    const auto target_point = std::array<double, 3>{
      target_pose.pose.position.x,
      target_pose.pose.position.y,
      target_pose.pose.position.z + grasp_height_offset_m_,
    };

    if (debug_execute_ranked_candidates_) {
      run_ranked_pregrasp_debug_sequence(current_pose, target_point);
      return;
    }

    const auto selected_option = select_pregrasp_option(current_pose, target_point);
    if (!selected_option.has_value()) {
      return fail_sequence("failed to find a feasible pregrasp candidate");
    }

    const auto & pregrasp_option = selected_option.value();
    RCLCPP_INFO(
      this->get_logger(),
      "selected candidate %zu stage=(%.3f, %.3f, %.3f) pregrasp=(%.3f, %.3f, %.3f) grasp=(%.3f, %.3f, %.3f) score=%.2f semantic=%.2f confidence=%.2f bonus=%.2f",
      pregrasp_option.candidate_index,
      pregrasp_option.stage[0], pregrasp_option.stage[1], pregrasp_option.stage[2],
      pregrasp_option.pregrasp[0], pregrasp_option.pregrasp[1], pregrasp_option.pregrasp[2],
      pregrasp_option.grasp[0], pregrasp_option.grasp[1], pregrasp_option.grasp[2],
      pregrasp_option.score,
      pregrasp_option.external_semantic_score,
      pregrasp_option.external_confidence_score,
      pregrasp_option.external_proposal_bonus);
    RCLCPP_DEBUG(
      this->get_logger(),
      "selected candidate %zu penalties: camera=%.2f camera_bonus=%.2f joint1_excursion=%.2f joint2_excursion=%.2f wrist_excursion=%.2f",
      pregrasp_option.candidate_index,
      pregrasp_option.camera_penalty,
      pregrasp_option.camera_stability_bonus,
      pregrasp_option.primary_joint_excursion_cost,
      pregrasp_option.secondary_joint_excursion_cost,
      pregrasp_option.wrist_joint_excursion_cost);
    if (pregrasp_option.external_metadata.has_value()) {
      const auto & metadata = pregrasp_option.external_metadata.value();
      RCLCPP_DEBUG(
        this->get_logger(),
        "selected candidate %zu contact_pair: p1=(%.3f, %.3f, %.3f) p2=(%.3f, %.3f, %.3f) width=%.3f task_tag=%s",
        pregrasp_option.candidate_index,
        metadata.contact_point_1[0], metadata.contact_point_1[1], metadata.contact_point_1[2],
        metadata.contact_point_2[0], metadata.contact_point_2[1], metadata.contact_point_2[2],
        metadata.grasp_width_m,
        metadata.task_constraint_tag.c_str());
    }

    RCLCPP_INFO(
      this->get_logger(),
      "executing selected candidate %zu with live replanning between phases",
      pregrasp_option.candidate_index);
    if (pregrasp_option.skip_stage) {
      publish_pick_status(
        "executing",
        "executing direct pregrasp, grasp, retreat, and lift");
      if (!execute_preplanned_arm_plan(
            pregrasp_option.pregrasp_plan,
            pregrasp_option.pregrasp_label,
            true))
      {
        return fail_sequence("failed to execute direct pregrasp plan");
      }
    } else {
      publish_pick_status(
        "executing",
        "executing selected stage, pregrasp, grasp, retreat, and lift");

      if (!execute_stage_position_target_with_gripper_open(
            pregrasp_option.stage, pregrasp_option.stage_orientation, pregrasp_option.stage_label))
      {
        return fail_sequence("failed to execute selected stage plan");
      }

      const auto post_stage_pose = arm_group_->getCurrentPose(ee_link_).pose;
      RCLCPP_DEBUG(
        this->get_logger(),
        "post-stage ee pose: (%.3f, %.3f, %.3f)",
        post_stage_pose.position.x,
        post_stage_pose.position.y,
        post_stage_pose.position.z);

      if (!execute_position_target(
            pregrasp_option.pregrasp,
            pregrasp_option.pregrasp_orientation,
            pregrasp_option.pregrasp_label))
      {
        return fail_sequence("failed to execute selected pregrasp plan");
      }
    }

    if (debug_stop_after_pregrasp_) {
      clear_locked_joint_references();
      executing_ = false;
      completed_ = true;
      publish_pick_active();
      publish_pick_status("completed", "debug stop enabled at pregrasp");
      RCLCPP_WARN(
        this->get_logger(),
        "debug stop enabled: holding at pregrasp and skipping grasp/close/retreat/lift");
      return;
    }

    remove_target_from_scene();
    std::this_thread::sleep_for(150ms);

    const std::array<double, 3> retreat_point = pregrasp_option.pregrasp;
    const std::array<double, 3> lift_point = {
      pregrasp_option.pregrasp[0],
      pregrasp_option.pregrasp[1],
      pregrasp_option.pregrasp[2] + lift_distance_m_,
    };

    RCLCPP_DEBUG(
      this->get_logger(),
      "selected pregrasp=(%.3f, %.3f, %.3f) grasp=(%.3f, %.3f, %.3f) retreat=(%.3f, %.3f, %.3f) lift=(%.3f, %.3f, %.3f)",
      pregrasp_option.pregrasp[0], pregrasp_option.pregrasp[1], pregrasp_option.pregrasp[2],
      pregrasp_option.grasp[0], pregrasp_option.grasp[1], pregrasp_option.grasp[2],
      retreat_point[0], retreat_point[1], retreat_point[2],
      lift_point[0], lift_point[1], lift_point[2]);

    if (!execute_cartesian_segment(
          pregrasp_option.grasp,
          pregrasp_option.grasp_orientation,
          "move to grasp"))
    {
      return fail_sequence("failed to reach grasp");
    }

    if (!execute_named_target(*gripper_group_, "closed", "close gripper")) {
      return fail_sequence("failed to close gripper");
    }

    if (!execute_position_target(
          retreat_point, pregrasp_option.pregrasp_orientation, "retreat to pregrasp with object"))
    {
      return fail_sequence("failed to retreat after closing gripper");
    }

    if (!execute_position_target(
          lift_point, pregrasp_option.pregrasp_orientation, "lift object"))
    {
      return fail_sequence("failed to lift after retreat");
    }

    clear_locked_joint_references();
    completed_ = true;
    executing_ = false;
    publish_pick_active();
    publish_pick_status("completed", "pick sequence completed");
    RCLCPP_INFO(this->get_logger(), "pick sequence completed");
  }

  void run_ranked_pregrasp_debug_sequence(
    const geometry_msgs::msg::Pose & current_pose,
    const std::array<double, 3> & target_point)
  {
    const auto evaluation = evaluate_internal_pregrasp_options(current_pose, target_point);
    if (evaluation.feasible_options.empty()) {
      return fail_sequence("ranked pregrasp debug found no feasible internal candidates");
    }

    const std::size_t candidate_limit = std::min(
      static_cast<std::size_t>(std::max(debug_ranked_candidate_limit_, 1)),
      evaluation.feasible_options.size());
    RCLCPP_INFO(
      this->get_logger(),
      "ranked pregrasp debug enabled: executing top %zu of %zu feasible internal candidates (tested=%zu)",
      candidate_limit,
      evaluation.feasible_options.size(),
      evaluation.tested_options);

    std::size_t stage_success_count = 0;
    std::size_t pregrasp_success_count = 0;

    for (std::size_t rank_index = 0; rank_index < candidate_limit; ++rank_index) {
      const auto & option = evaluation.feasible_options[rank_index];
      const std::size_t rank = rank_index + 1;

      if (!execute_named_target(
            *arm_group_,
            debug_return_named_target_,
            "debug return to " + debug_return_named_target_ + " before ranked candidate " +
            std::to_string(rank)))
      {
        return fail_sequence("failed to return to debug home before ranked candidate execution");
      }
      std::this_thread::sleep_for(150ms);

      const auto start_pose = arm_group_->getCurrentPose(ee_link_).pose;
      RCLCPP_INFO(
        this->get_logger(),
        "ranked candidate %zu/%zu: base_candidate=%zu score=%.2f stage=(%.3f, %.3f, %.3f) pregrasp=(%.3f, %.3f, %.3f) grasp=(%.3f, %.3f, %.3f)",
        rank,
        candidate_limit,
        option.candidate_index,
        option.score,
        option.stage[0], option.stage[1], option.stage[2],
        option.pregrasp[0], option.pregrasp[1], option.pregrasp[2],
        option.grasp[0], option.grasp[1], option.grasp[2]);
      RCLCPP_INFO(
        this->get_logger(),
        "ranked candidate %zu penalties: camera=%.2f camera_bonus=%.2f joint1_excursion=%.2f joint2_excursion=%.2f wrist_excursion=%.2f",
        rank,
        option.camera_penalty,
        option.camera_stability_bonus,
        option.primary_joint_excursion_cost,
        option.secondary_joint_excursion_cost,
        option.wrist_joint_excursion_cost);

      if (!execute_stage_position_target_with_gripper_open(
            option.stage,
            option.stage_orientation,
            "ranked stage candidate " + std::to_string(rank)))
      {
        RCLCPP_WARN(
          this->get_logger(),
          "ranked candidate %zu: stage execution failed",
          rank);
        continue;
      }
      ++stage_success_count;

      const auto post_stage_pose = arm_group_->getCurrentPose(ee_link_).pose;
      RCLCPP_DEBUG(
        this->get_logger(),
        "ranked candidate %zu post-stage ee pose: (%.3f, %.3f, %.3f)",
        rank,
        post_stage_pose.position.x,
        post_stage_pose.position.y,
        post_stage_pose.position.z);

      if (!execute_position_target(
            option.pregrasp,
            option.pregrasp_orientation,
            "ranked pregrasp candidate " + std::to_string(rank)))
      {
        RCLCPP_WARN(
          this->get_logger(),
          "ranked candidate %zu: pregrasp execution failed",
          rank);
        continue;
      }
      ++pregrasp_success_count;

      RCLCPP_INFO(
        this->get_logger(),
        "ranked candidate %zu: pregrasp reached successfully; returning to %s",
        rank,
        debug_return_named_target_.c_str());
    }

    if (!execute_named_target(
          *arm_group_,
          debug_return_named_target_,
          "debug return to " + debug_return_named_target_ + " after ranked candidate sweep"))
    {
      return fail_sequence("failed to return to debug home after ranked candidate sweep");
    }

    clear_locked_joint_references();
    completed_ = true;
    executing_ = false;
    publish_pick_active();
    RCLCPP_WARN(
      this->get_logger(),
      "ranked pregrasp debug completed: requested=%zu stage_success=%zu pregrasp_success=%zu; holding at %s",
      candidate_limit,
      stage_success_count,
      pregrasp_success_count,
      debug_return_named_target_.c_str());
  }

  bool execute_position_target(
    const std::array<double, 3> & target_point,
    const geometry_msgs::msg::Quaternion & target_orientation,
    const std::string & label)
  {
    const auto joint_lock_attempts = get_locked_joint_attempts();
    if (has_locked_joint_constraints() && !joint_lock_attempts.empty()) {
      for (std::size_t index = 0; index < joint_lock_attempts.size(); ++index) {
        const auto & attempt = joint_lock_attempts[index];
        const std::string attempt_label =
          index == 0 ? label : label + attempt.label_suffix;
        if (
          execute_position_target_attempt(
            target_point,
            target_orientation,
            attempt_label,
            attempt,
            pregrasp_position_only_target_))
        {
          return true;
        }
      }
      return false;
    }

    return execute_position_target_attempt(
      target_point,
      target_orientation,
      label,
      JointConstraintAttempt{},
      pregrasp_position_only_target_);
  }

  bool execute_stage_position_target_with_gripper_open(
    const std::array<double, 3> & target_point,
    const geometry_msgs::msg::Quaternion & target_orientation,
    const std::string & label)
  {
    const auto joint_lock_attempts = get_locked_joint_attempts();
    if (has_locked_joint_constraints() && !joint_lock_attempts.empty()) {
      for (std::size_t index = 0; index < joint_lock_attempts.size(); ++index) {
        const auto & attempt = joint_lock_attempts[index];
        const std::string attempt_label =
          index == 0 ? label : label + attempt.label_suffix;
        if (execute_stage_position_target_attempt(
            target_point,
            target_orientation,
            attempt_label,
            attempt,
            stage_position_only_target_))
        {
          return true;
        }
      }
      return false;
    }

    return execute_stage_position_target_attempt(
      target_point,
      target_orientation,
      label,
      JointConstraintAttempt{},
      stage_position_only_target_);
  }

  bool execute_cartesian_segment(
    const std::array<double, 3> & target_point,
    const geometry_msgs::msg::Quaternion & target_orientation,
    const std::string & label)
  {
    set_bounded_start_state(*arm_group_);

    geometry_msgs::msg::Pose waypoint = arm_group_->getCurrentPose(ee_link_).pose;
    waypoint.position.x = target_point[0];
    waypoint.position.y = target_point[1];
    waypoint.position.z = target_point[2];
    if (enforce_upright_orientation_) {
      waypoint.orientation = target_orientation;
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    const double fraction = arm_group_->computeCartesianPath(
      {waypoint}, 0.01, trajectory, true, &error_code);

    if (fraction < 0.98) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "%s: cartesian path fraction %.2f, falling back to sampled planning",
        label.c_str(), fraction);
      return execute_position_target(target_point, target_orientation, label + " fallback");
    }

    const bool executed = static_cast<bool>(arm_group_->execute(trajectory));
    if (!executed) {
      RCLCPP_DEBUG(this->get_logger(), "%s: cartesian execution failed", label.c_str());
      return execute_position_target(target_point, target_orientation, label + " fallback");
    }

    RCLCPP_DEBUG(this->get_logger(), "%s: cartesian success", label.c_str());
    return true;
  }

  bool execute_preplanned_arm_plan(
    moveit::planning_interface::MoveGroupInterface::Plan plan,
    const std::string & label,
    bool augment_with_open_gripper = false)
  {
    if (augment_with_open_gripper) {
      augment_stage_plan_with_open_gripper(plan);
    }

    const bool executed = static_cast<bool>(arm_group_->execute(plan));
    if (!executed) {
      RCLCPP_DEBUG(this->get_logger(), "%s: execution failed", label.c_str());
      return false;
    }

    RCLCPP_DEBUG(this->get_logger(), "%s: success", label.c_str());
    return true;
  }

  geometry_msgs::msg::Quaternion normalize_quaternion(
    const geometry_msgs::msg::Quaternion & quaternion) const
  {
    Eigen::Quaterniond eigen_quaternion(
      quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    if (eigen_quaternion.norm() < 1e-9) {
      eigen_quaternion = Eigen::Quaterniond::Identity();
    } else {
      eigen_quaternion.normalize();
    }

    geometry_msgs::msg::Quaternion normalized;
    normalized.x = eigen_quaternion.x();
    normalized.y = eigen_quaternion.y();
    normalized.z = eigen_quaternion.z();
    normalized.w = eigen_quaternion.w();
    return normalized;
  }

  geometry_msgs::msg::Quaternion rotate_orientation_about_world_z(
    const geometry_msgs::msg::Quaternion & orientation,
    double yaw_deg) const
  {
    const auto normalized = normalize_quaternion(orientation);
    Eigen::Quaterniond base(
      normalized.w, normalized.x, normalized.y, normalized.z);
    const Eigen::Quaterniond yaw_rotation(
      Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond rotated = yaw_rotation * base;
    rotated.normalize();

    geometry_msgs::msg::Quaternion result;
    result.x = rotated.x();
    result.y = rotated.y();
    result.z = rotated.z();
    result.w = rotated.w();
    return result;
  }

  geometry_msgs::msg::Quaternion make_semantic_grasp_orientation(
    const std::array<double, 2> & backoff_direction) const
  {
    const double yaw_rad =
      std::atan2(backoff_direction[1], backoff_direction[0]) +
      (grasp_yaw_offset_deg_ * M_PI / 180.0);
    const double pitch_rad = grasp_base_pitch_deg_ * M_PI / 180.0;
    const double roll_rad = grasp_base_roll_deg_ * M_PI / 180.0;

    const Eigen::Quaterniond yaw_rotation(
      Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
    const Eigen::Quaterniond pitch_rotation(
      Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()));
    const Eigen::Quaterniond roll_rotation(
      Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX()));

    Eigen::Quaterniond orientation = yaw_rotation * pitch_rotation * roll_rotation;
    orientation.normalize();

    geometry_msgs::msg::Quaternion result;
    result.x = orientation.x();
    result.y = orientation.y();
    result.z = orientation.z();
    result.w = orientation.w();
    return result;
  }

  bool execute_named_target(
    moveit::planning_interface::MoveGroupInterface & group,
    const std::string & target_name,
    const std::string & label)
  {
    set_bounded_start_state(group);
    group.setNamedTarget(target_name);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(group.plan(plan));
    if (!planned) {
      RCLCPP_DEBUG(this->get_logger(), "%s: planning failed", label.c_str());
      return false;
    }
    const bool executed = static_cast<bool>(group.execute(plan));
    if (!executed) {
      RCLCPP_DEBUG(this->get_logger(), "%s: execution failed", label.c_str());
      return false;
    }
    RCLCPP_DEBUG(this->get_logger(), "%s: success", label.c_str());
    return true;
  }

  void apply_scene(const geometry_msgs::msg::PoseStamped & target_pose, bool include_target)
  {
    std::vector<moveit_msgs::msg::CollisionObject> objects;

    moveit_msgs::msg::CollisionObject table;
    table.id = "sim_table";
    table.header.frame_id = base_frame_;
    table.operation = moveit_msgs::msg::CollisionObject::ADD;
    shape_msgs::msg::SolidPrimitive table_primitive;
    table_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    table_primitive.dimensions = {
      table_size_m_[0],
      table_size_m_[1],
      table_size_m_[2],
    };
    geometry_msgs::msg::Pose table_pose;
    table_pose.orientation.w = 1.0;
    if (table_pose_xyz_.size() >= 3) {
      table_pose.position.x = table_pose_xyz_[0];
      table_pose.position.y = table_pose_xyz_[1];
      table_pose.position.z = table_pose_xyz_[2];
    } else {
      const double table_top_z =
        target_pose.pose.position.z - (target_height_m_ * 0.5);
      table_pose.position.x = target_pose.pose.position.x + 0.05;
      table_pose.position.y = target_pose.pose.position.y;
      table_pose.position.z = table_top_z - (table_size_m_[2] * 0.5);
    }
    table.primitives.push_back(table_primitive);
    table.primitive_poses.push_back(table_pose);
    objects.push_back(table);

    if (include_target) {
      moveit_msgs::msg::CollisionObject cylinder;
      cylinder.id = "sim_target";
      cylinder.header.frame_id = base_frame_;
      cylinder.operation = moveit_msgs::msg::CollisionObject::ADD;
      shape_msgs::msg::SolidPrimitive cylinder_primitive;
      cylinder_primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      cylinder_primitive.dimensions = {
        target_height_m_,
        target_radius_m_,
      };
      auto cylinder_pose = target_pose.pose;
      cylinder_pose.orientation.w = 1.0;
      cylinder.primitives.push_back(cylinder_primitive);
      cylinder.primitive_poses.push_back(cylinder_pose);
      objects.push_back(cylinder);
    }

    planning_scene_interface_->applyCollisionObjects(objects);
    RCLCPP_DEBUG(
      this->get_logger(),
      "planning scene updated: table=(%.3f, %.3f, %.3f) include_target=%s",
      table_pose.position.x,
      table_pose.position.y,
      table_pose.position.z,
      include_target ? "true" : "false");
  }

  bool has_fresh_external_grasp_candidates(
    double * age_sec_out = nullptr,
    std::size_t * candidate_count_out = nullptr)
  {
    std::scoped_lock<std::mutex> lock(grasp_candidate_mutex_);
    const double age_sec =
      this->get_clock()->now().seconds() - latest_external_grasp_update_sec_;
    std::size_t candidate_count = 0;
    if (has_external_grasp_proposal_array_) {
      candidate_count += latest_external_grasp_proposal_array_.proposals.size();
    }
    if (has_external_pregrasp_pose_array_ && has_external_grasp_pose_array_) {
      candidate_count += std::min(
        latest_external_pregrasp_pose_array_.poses.size(),
        latest_external_grasp_pose_array_.poses.size());
    }
    if (has_selected_pregrasp_pose_ && has_selected_grasp_pose_) {
      candidate_count += 1U;
    }
    if (age_sec_out != nullptr) {
      *age_sec_out = age_sec;
    }
    if (candidate_count_out != nullptr) {
      *candidate_count_out = candidate_count;
    }
    return candidate_count > 0U && age_sec <= external_grasp_pose_timeout_sec_;
  }

  void remove_target_from_scene()
  {
    planning_scene_interface_->removeCollisionObjects({"sim_target"});
    RCLCPP_DEBUG(this->get_logger(), "removed target collision object to allow contact");
  }

  void set_bounded_start_state(moveit::planning_interface::MoveGroupInterface & group)
  {
    auto current_state = group.getCurrentState(1.0);
    if (!current_state) {
      group.setStartStateToCurrentState();
      return;
    }

    const auto * joint_model_group =
      current_state->getJointModelGroup(group.getName());
    if (joint_model_group == nullptr) {
      group.setStartStateToCurrentState();
      return;
    }

    current_state->enforceBounds(joint_model_group);
    group.setStartState(*current_state);
  }

  void fail_sequence(const std::string & reason)
  {
    suppress_target_relock_ = true;
    clear_locked_joint_references();
    pick_armed_ = false;
    executing_ = false;
    completed_ = false;
    has_latched_target_pose_ = false;
    publish_pick_active();
    publish_pick_status("error", reason);
    RCLCPP_WARN(this->get_logger(), "pick sequence aborted: %s", reason.c_str());
    (void)execute_named_target(*gripper_group_, "open", "open gripper after failure");
    const bool returned_home = execute_named_target(
      *arm_group_, return_named_target_, "return arm to named home pose after failure");
    if (returned_home) {
      RCLCPP_WARN(this->get_logger(), "arm returned to named home pose after failure");
    } else {
      RCLCPP_WARN(this->get_logger(), "failed to return arm to named home pose after failure");
    }
  }

  bool execute_position_target_attempt(
    const std::array<double, 3> & target_point,
    const geometry_msgs::msg::Quaternion & target_orientation,
    const std::string & label,
    const JointConstraintAttempt & joint_attempt,
    bool position_only_target)
  {
    constexpr int kExecutionAttempts = 2;
    for (int execution_attempt = 0; execution_attempt < kExecutionAttempts; ++execution_attempt) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const std::string attempt_label =
        execution_attempt == 0 ? label : label + " replanned";
      if (
        !plan_pose_target_attempt(
          target_point,
          target_orientation,
          joint_attempt,
          plan,
          nullptr,
          position_only_target))
      {
        RCLCPP_WARN(this->get_logger(), "%s: planning failed", attempt_label.c_str());
        continue;
      }

      if (execute_preplanned_arm_plan(std::move(plan), attempt_label)) {
        return true;
      }

      if (execution_attempt + 1 < kExecutionAttempts) {
        RCLCPP_WARN(
          this->get_logger(),
          "%s: execution failed, replanning from refreshed current state",
          label.c_str());
      }
    }

    return false;
  }

  bool execute_stage_position_target_attempt(
    const std::array<double, 3> & target_point,
    const geometry_msgs::msg::Quaternion & target_orientation,
    const std::string & label,
    const JointConstraintAttempt & joint_attempt,
    bool position_only_target)
  {
    constexpr int kExecutionAttempts = 2;
    for (int execution_attempt = 0; execution_attempt < kExecutionAttempts; ++execution_attempt) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const std::string attempt_label =
        execution_attempt == 0 ? label : label + " replanned";
      if (
        !plan_pose_target_attempt(
          target_point,
          target_orientation,
          joint_attempt,
          plan,
          nullptr,
          position_only_target))
      {
        RCLCPP_WARN(this->get_logger(), "%s: planning failed", attempt_label.c_str());
        continue;
      }

      if (execute_preplanned_arm_plan(std::move(plan), attempt_label, true)) {
        return true;
      }

      if (execution_attempt + 1 < kExecutionAttempts) {
        RCLCPP_WARN(
          this->get_logger(),
          "%s: execution failed, replanning from refreshed current state",
          label.c_str());
      }
    }

    return false;
  }

  void augment_stage_plan_with_open_gripper(
    moveit::planning_interface::MoveGroupInterface::Plan & plan)
  {
    auto & trajectory = plan.trajectory.joint_trajectory;
    if (trajectory.points.empty()) {
      return;
    }

    const std::string grip_joint_name = "grip_joint";
    if (std::find(
        trajectory.joint_names.begin(),
        trajectory.joint_names.end(),
        grip_joint_name) != trajectory.joint_names.end())
    {
      return;
    }

    double current_grip_position = 0.0;
    const auto grip_joint_values = gripper_group_->getCurrentJointValues();
    if (!grip_joint_values.empty()) {
      current_grip_position = grip_joint_values.front();
    }
    constexpr double open_grip_position = 0.0;

    trajectory.joint_names.push_back(grip_joint_name);
    const std::size_t point_count = trajectory.points.size();
    for (std::size_t index = 0; index < point_count; ++index) {
      auto & point = trajectory.points[index];
      const double ratio =
        point_count > 1 ? static_cast<double>(index) / static_cast<double>(point_count - 1) : 1.0;
      const double grip_position =
        current_grip_position + ratio * (open_grip_position - current_grip_position);
      point.positions.push_back(grip_position);
      if (!point.velocities.empty()) {
        point.velocities.push_back(0.0);
      }
      if (!point.accelerations.empty()) {
        point.accelerations.push_back(0.0);
      }
      if (!point.effort.empty()) {
        point.effort.push_back(0.0);
      }
    }

    RCLCPP_INFO(
      this->get_logger(),
      "stage trajectory augmented with grip_joint open interpolation: start=%.4f target=%.4f points=%zu",
      current_grip_position,
      open_grip_position,
      point_count);
  }

  std::vector<PregraspVariant> build_local_pregrasp_variants(
    const ApproachCandidate & candidate) const
  {
    const std::vector<PregraspVariant> variants = {
      {
        candidate.pregrasp,
        candidate.grasp,
        candidate.pregrasp_orientation,
        candidate.grasp_orientation,
        "",
      },
      {{
        candidate.pregrasp[0],
        candidate.pregrasp[1],
        candidate.pregrasp[2] + 0.02,
      }, {
        candidate.grasp[0],
        candidate.grasp[1],
        candidate.grasp[2] + 0.02,
      },
      candidate.pregrasp_orientation,
      candidate.grasp_orientation,
      " lifted"},
      {{
        candidate.pregrasp[0] - 0.015,
        candidate.pregrasp[1],
        candidate.pregrasp[2] + 0.015,
      }, {
        candidate.grasp[0] - 0.015,
        candidate.grasp[1],
        candidate.grasp[2] + 0.015,
      },
      candidate.pregrasp_orientation,
      candidate.grasp_orientation,
      " backed_off"},
    };
    return variants;
  }

  std::vector<ApproachCandidate> build_approach_candidates(
    const geometry_msgs::msg::Pose & current_pose,
    const std::array<double, 3> & target_point) const
  {
    std::array<double, 2> approach_xy = {
      current_pose.position.x - target_point[0],
      current_pose.position.y - target_point[1],
    };
    double approach_norm = std::hypot(approach_xy[0], approach_xy[1]);
    if (approach_norm < 1e-3) {
      approach_xy = {-1.0, 0.0};
      approach_norm = 1.0;
    }
    approach_xy[0] /= approach_norm;
    approach_xy[1] /= approach_norm;

    const auto rotate_xy = [](const std::array<double, 2> & direction, double yaw_deg) {
        const double yaw_rad = yaw_deg * M_PI / 180.0;
        const double cos_yaw = std::cos(yaw_rad);
        const double sin_yaw = std::sin(yaw_rad);
        return std::array<double, 2>{
          direction[0] * cos_yaw - direction[1] * sin_yaw,
          direction[0] * sin_yaw + direction[1] * cos_yaw,
        };
      };

    const auto make_point =
      [&](const std::array<double, 2> & direction, double backoff_m, double lift_m)
        -> std::array<double, 3>
      {
        return {
          target_point[0] + direction[0] * backoff_m,
          target_point[1] + direction[1] * backoff_m,
          target_point[2] + lift_m,
        };
      };

    const double nominal_pregrasp_backoff = std::max(pregrasp_backoff_m_, 0.09);
    const double nominal_grasp_backoff = std::max(grasp_backoff_m_, 0.04);
    const double wide_pregrasp_backoff = nominal_pregrasp_backoff + 0.03;
    const double wide_grasp_backoff = nominal_grasp_backoff + 0.02;
    const auto left_approach_xy = rotate_xy(approach_xy, 18.0);
    const auto right_approach_xy = rotate_xy(approach_xy, -18.0);
    const auto base_orientation = make_semantic_grasp_orientation(approach_xy);
    const auto left_orientation = make_semantic_grasp_orientation(left_approach_xy);
    const auto right_orientation = make_semantic_grasp_orientation(right_approach_xy);

    const std::vector<ApproachCandidate> candidates = {
      {
        make_point(approach_xy, nominal_pregrasp_backoff, pregrasp_lift_m_),
        make_point(approach_xy, nominal_grasp_backoff, grasp_lift_m_),
        base_orientation,
        base_orientation,
      },
      {
        make_point(approach_xy, wide_pregrasp_backoff, pregrasp_lift_m_ + 0.025),
        make_point(approach_xy, wide_grasp_backoff, grasp_lift_m_ + 0.015),
        base_orientation,
        base_orientation,
      },
      {
        make_point(left_approach_xy, nominal_pregrasp_backoff + 0.01, pregrasp_lift_m_ + 0.015),
        make_point(left_approach_xy, nominal_grasp_backoff + 0.005, grasp_lift_m_ + 0.01),
        left_orientation,
        left_orientation,
      },
      {
        make_point(right_approach_xy, nominal_pregrasp_backoff + 0.01, pregrasp_lift_m_ + 0.015),
        make_point(right_approach_xy, nominal_grasp_backoff + 0.005, grasp_lift_m_ + 0.01),
        right_orientation,
        right_orientation,
      },
    };

    RCLCPP_INFO(
      this->get_logger(),
      "approach direction from current ee to target: dx=%.3f dy=%.3f semantic_rpy=(%.1f, %.1f, %.1f)",
      approach_xy[0], approach_xy[1],
      grasp_base_roll_deg_, grasp_base_pitch_deg_, grasp_yaw_offset_deg_);
    return candidates;
  }

  bool consume_latched_target_pose(geometry_msgs::msg::PoseStamped & target_pose)
  {
    std::scoped_lock<std::mutex> lock(target_mutex_);
    if (has_latched_target_pose_) {
      target_pose = latched_target_pose_;
      return true;
    }
    if (has_target_pose_) {
      target_pose = latest_target_pose_;
      return true;
    }
    return false;
  }

  bool request_grasp_candidate_refresh(const std::string & reason, bool force = false)
  {
    if (!refresh_grasp_candidates_pub_) {
      return false;
    }
    if (!require_external_grasp_candidates_ && !prefer_external_grasp_candidates_) {
      return false;
    }

    const double now_sec = this->get_clock()->now().seconds();
    if (
      !force && last_grasp_refresh_request_sec_ > 0.0 &&
      (now_sec - last_grasp_refresh_request_sec_) < refresh_grasp_candidates_retry_sec_)
    {
      return false;
    }

    std_msgs::msg::Bool refresh_msg;
    refresh_msg.data = true;
    refresh_grasp_candidates_pub_->publish(refresh_msg);
    last_grasp_refresh_request_sec_ = now_sec;
    RCLCPP_INFO(this->get_logger(), "requested grasp candidate refresh: %s", reason.c_str());
    return true;
  }

  void try_arm_pick_session()
  {
    if (pick_armed_.load() || executing_.load() || completed_.load() || !target_locked_) {
      return;
    }

    geometry_msgs::msg::PoseStamped target_pose;
    {
      std::scoped_lock<std::mutex> lock(target_mutex_);
      if (!has_target_pose_) {
        return;
      }
      latched_target_pose_ = latest_target_pose_;
      has_latched_target_pose_ = true;
      target_pose = latched_target_pose_;
    }

    publish_pick_active();
    if (!require_user_confirmation_) {
      RCLCPP_WARN_ONCE(
        this->get_logger(),
        "automatic pick arming on target lock is disabled; waiting for explicit execute command");
    }
    publish_pick_status(
      "waiting_execute",
      "target locked; waiting for explicit execute command");
    RCLCPP_INFO(
      this->get_logger(),
      "latched target pose for manual-confirm pick session: (%.3f, %.3f, %.3f)",
      target_pose.pose.position.x,
      target_pose.pose.position.y,
      target_pose.pose.position.z);
  }

  void capture_locked_joint_references()
  {
    clear_locked_joint_references();
    capture_locked_joint_reference(
      lock_joint_after_target_lock_,
      lock_joint_name_,
      lock_joint_max_delta_deg_,
      lock_joint_relaxed_delta_deg_,
      locked_joint_reference_rad_,
      locked_joint_reference_valid_);
    capture_locked_joint_reference(
      lock_wrist_joint_after_target_lock_,
      wrist_joint_name_,
      wrist_joint_max_delta_deg_,
      wrist_joint_relaxed_delta_deg_,
      locked_wrist_joint_reference_rad_,
      locked_wrist_joint_reference_valid_);
  }

  void capture_locked_joint_reference(
    bool enabled,
    const std::string & joint_name,
    double max_delta_deg,
    double relaxed_delta_deg,
    double & reference_rad,
    bool & reference_valid)
  {
    if (!enabled) {
      return;
    }

    const auto joint_names = arm_group_->getJoints();
    const auto joint_values = arm_group_->getCurrentJointValues();
    const std::size_t count = std::min(joint_names.size(), joint_values.size());
    for (std::size_t index = 0; index < count; ++index) {
      if (joint_names[index] != joint_name) {
        continue;
      }

      reference_rad = joint_values[index];
      reference_valid = true;
      RCLCPP_INFO(
        this->get_logger(),
        "locked %s at %.1fdeg for pick planning (delta %.1fdeg, relaxed %.1fdeg)",
        joint_name.c_str(),
        reference_rad * 180.0 / M_PI,
        max_delta_deg,
        relaxed_delta_deg);
      return;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "failed to capture %s from current arm joint state; pick planning will run unconstrained",
      joint_name.c_str());
  }

  void clear_locked_joint_references()
  {
    locked_joint_reference_valid_ = false;
    locked_joint_reference_rad_ = 0.0;
    locked_wrist_joint_reference_valid_ = false;
    locked_wrist_joint_reference_rad_ = 0.0;
    arm_group_->clearPathConstraints();
  }

  bool has_locked_joint_constraints() const
  {
    return locked_joint_reference_valid_ || locked_wrist_joint_reference_valid_;
  }

  moveit_msgs::msg::Constraints make_locked_joint_constraints(
    const JointConstraintAttempt & attempt) const
  {
    moveit_msgs::msg::Constraints constraints;
    constraints.name = "locked_joint_after_target_lock";

    if (locked_joint_reference_valid_ && attempt.primary_joint_delta_rad > 0.0) {
      moveit_msgs::msg::JointConstraint joint_constraint;
      joint_constraint.joint_name = lock_joint_name_;
      joint_constraint.position = locked_joint_reference_rad_;
      joint_constraint.tolerance_above = attempt.primary_joint_delta_rad;
      joint_constraint.tolerance_below = attempt.primary_joint_delta_rad;
      joint_constraint.weight = 1.0;
      constraints.joint_constraints.push_back(joint_constraint);
    }

    if (locked_wrist_joint_reference_valid_ && attempt.wrist_joint_delta_rad > 0.0) {
      moveit_msgs::msg::JointConstraint joint_constraint;
      joint_constraint.joint_name = wrist_joint_name_;
      joint_constraint.position = locked_wrist_joint_reference_rad_;
      joint_constraint.tolerance_above = attempt.wrist_joint_delta_rad;
      joint_constraint.tolerance_below = attempt.wrist_joint_delta_rad;
      joint_constraint.weight = 1.0;
      constraints.joint_constraints.push_back(joint_constraint);
    }

    return constraints;
  }

  bool plan_pose_target_attempt(
    const std::array<double, 3> & target_point,
    const geometry_msgs::msg::Quaternion & target_orientation,
    const JointConstraintAttempt & joint_attempt,
    moveit::planning_interface::MoveGroupInterface::Plan & plan,
    const moveit::core::RobotStatePtr & start_state = nullptr,
    bool position_only_target = false)
  {
    if (start_state) {
      arm_group_->setStartState(*start_state);
    } else {
      set_bounded_start_state(*arm_group_);
    }

    const auto constraints = make_locked_joint_constraints(joint_attempt);
    if (!constraints.joint_constraints.empty()) {
      arm_group_->setPathConstraints(constraints);
    } else {
      arm_group_->clearPathConstraints();
    }

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = target_point[0];
    target_pose.position.y = target_point[1];
    target_pose.position.z = target_point[2];
    target_pose.orientation = normalize_quaternion(target_orientation);
    if (position_only_target) {
      arm_group_->setPositionTarget(
        target_pose.position.x,
        target_pose.position.y,
        target_pose.position.z,
        ee_link_);
    } else {
      arm_group_->setPoseTarget(target_pose, ee_link_);
    }
    const bool planned = static_cast<bool>(arm_group_->plan(plan));
    arm_group_->clearPoseTargets();
    arm_group_->clearPathConstraints();
    arm_group_->setStartStateToCurrentState();
    return planned;
  }

  moveit::core::RobotStatePtr build_plan_end_state(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan)
  {
    auto state = arm_group_->getCurrentState(1.0);
    if (!state) {
      return nullptr;
    }

    const auto * joint_model_group = state->getJointModelGroup(arm_group_name_);
    if (joint_model_group == nullptr) {
      return nullptr;
    }

    const auto & trajectory = plan.trajectory.joint_trajectory;
    if (trajectory.points.empty()) {
      return state;
    }

    std::vector<double> joint_positions;
    state->copyJointGroupPositions(joint_model_group, joint_positions);
    const auto & variable_names = joint_model_group->getVariableNames();
    const auto & last_point = trajectory.points.back();
    for (std::size_t index = 0; index < variable_names.size(); ++index) {
      const auto joint_it = std::find(
        trajectory.joint_names.begin(),
        trajectory.joint_names.end(),
        variable_names[index]);
      if (joint_it == trajectory.joint_names.end()) {
        continue;
      }
      const auto joint_index = static_cast<std::size_t>(
        std::distance(trajectory.joint_names.begin(), joint_it));
      if (joint_index >= last_point.positions.size()) {
        continue;
      }
      joint_positions[index] = last_point.positions[joint_index];
    }

    state->setJointGroupPositions(joint_model_group, joint_positions);
    state->update();
    return state;
  }

  double compute_point_distance(
    const std::array<double, 3> & point_a,
    const std::array<double, 3> & point_b) const
  {
    const double dx = point_a[0] - point_b[0];
    const double dy = point_a[1] - point_b[1];
    const double dz = point_a[2] - point_b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  double compute_point_distance(
    const geometry_msgs::msg::Point & point_a,
    const std::array<double, 3> & point_b) const
  {
    return compute_point_distance(
      {point_a.x, point_a.y, point_a.z},
      point_b);
  }

  static std::array<double, 3> point_to_array(const geometry_msgs::msg::Point & point)
  {
    return {point.x, point.y, point.z};
  }

  static geometry_msgs::msg::Point array_to_point(const std::array<double, 3> & point)
  {
    geometry_msgs::msg::Point point_msg;
    point_msg.x = point[0];
    point_msg.y = point[1];
    point_msg.z = point[2];
    return point_msg;
  }

  static std::array<double, 3> vector_to_array(const geometry_msgs::msg::Vector3 & vector)
  {
    return {vector.x, vector.y, vector.z};
  }

  static bool points_are_close(
    const std::array<double, 3> & point_a,
    const std::array<double, 3> & point_b,
    double tolerance = 1e-4)
  {
    return
      std::abs(point_a[0] - point_b[0]) <= tolerance &&
      std::abs(point_a[1] - point_b[1]) <= tolerance &&
      std::abs(point_a[2] - point_b[2]) <= tolerance;
  }

  void publish_external_execution_markers(
    const std::vector<ExternalEvaluationDebugInfo> & debug_entries,
    const std::optional<PlannedApproachOption> & best_option)
  {
    if (!execution_debug_markers_pub_) {
      return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    const builtin_interfaces::msg::Time now = this->get_clock()->now();

    int marker_id = 0;
    auto append_sphere =
      [&](const std::string & marker_ns,
        const std::array<double, 3> & point,
        double scale,
        double r, double g, double b, double a)
      {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = base_frame_;
        marker.header.stamp = now;
        marker.ns = marker_ns;
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.pose.position = array_to_point(point);
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;
        marker.color.r = static_cast<float>(r);
        marker.color.g = static_cast<float>(g);
        marker.color.b = static_cast<float>(b);
        marker.color.a = static_cast<float>(a);
        marker_array.markers.push_back(marker);
      };
    auto append_line_strip =
      [&](const std::string & marker_ns,
        const std::vector<std::array<double, 3>> & points,
        double width,
        double r, double g, double b, double a)
      {
        if (points.size() < 2) {
          return;
        }
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = base_frame_;
        marker.header.stamp = now;
        marker.ns = marker_ns;
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = width;
        marker.color.r = static_cast<float>(r);
        marker.color.g = static_cast<float>(g);
        marker.color.b = static_cast<float>(b);
        marker.color.a = static_cast<float>(a);
        for (const auto & point : points) {
          marker.points.push_back(array_to_point(point));
        }
        marker_array.markers.push_back(marker);
      };
    auto append_arrow =
      [&](const std::string & marker_ns,
        const std::array<double, 3> & point_a,
        const std::array<double, 3> & point_b,
        double shaft_diameter,
        double head_diameter,
        double head_length,
        double r, double g, double b, double a)
      {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = base_frame_;
        marker.header.stamp = now;
        marker.ns = marker_ns;
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = shaft_diameter;
        marker.scale.y = head_diameter;
        marker.scale.z = head_length;
        marker.color.r = static_cast<float>(r);
        marker.color.g = static_cast<float>(g);
        marker.color.b = static_cast<float>(b);
        marker.color.a = static_cast<float>(a);
        marker.points.push_back(array_to_point(point_a));
        marker.points.push_back(array_to_point(point_b));
        marker_array.markers.push_back(marker);
      };
    auto append_text =
      [&](const std::string & marker_ns,
        const std::array<double, 3> & point,
        const std::string & text,
        double scale_z,
        double r, double g, double b, double a)
      {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = base_frame_;
        marker.header.stamp = now;
        marker.ns = marker_ns;
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.pose.position = array_to_point(point);
        marker.scale.z = scale_z;
        marker.color.r = static_cast<float>(r);
        marker.color.g = static_cast<float>(g);
        marker.color.b = static_cast<float>(b);
        marker.color.a = static_cast<float>(a);
        marker.text = text;
        marker_array.markers.push_back(marker);
      };

    for (std::size_t index = 0; index < debug_entries.size(); ++index) {
      const auto & entry = debug_entries[index];
      const bool is_selected =
        best_option.has_value() &&
        points_are_close(entry.pregrasp, best_option->pregrasp) &&
        points_are_close(entry.grasp, best_option->grasp);
      double r = 0.55;
      double g = 0.55;
      double b = 0.55;
      double a = 0.45;
      if (is_selected || entry.feasible) {
        r = 0.10;
        g = 0.90;
        b = 0.20;
        a = is_selected ? 0.95 : 0.75;
      } else if (entry.pregrasp_plan_failures > 0 || entry.pregrasp_end_state_failures > 0) {
        r = 1.00;
        g = 0.55;
        b = 0.10;
        a = 0.80;
      } else if (entry.stage_plan_failures > 0 || entry.stage_end_state_failures > 0) {
        r = 1.00;
        g = 0.15;
        b = 0.15;
        a = 0.82;
      }

      append_line_strip(
        "execution_path",
        {entry.stage, entry.pregrasp, entry.grasp},
        is_selected ? 0.0055 : 0.004,
        r, g, b, a);
      append_sphere("stage_point", entry.stage, is_selected ? 0.016 : 0.013, 0.95, 0.95, 0.95, 0.80);
      append_sphere("pregrasp_point", entry.pregrasp, is_selected ? 0.017 : 0.014, 0.35, 0.70, 1.00, 0.85);
      append_sphere("grasp_point", entry.grasp, is_selected ? 0.018 : 0.015, r, g, b, 0.92);

      if (entry.external_metadata.has_value()) {
        const auto & metadata = entry.external_metadata.value();
        const std::array<double, 3> display_offset{
          metadata.approach_direction[0] * 0.006,
          metadata.approach_direction[1] * 0.006,
          metadata.approach_direction[2] * 0.006 + 0.003,
        };
        const std::array<double, 3> contact_point_1_display{
          metadata.contact_point_1[0] + display_offset[0],
          metadata.contact_point_1[1] + display_offset[1],
          metadata.contact_point_1[2] + display_offset[2],
        };
        const std::array<double, 3> contact_point_2_display{
          metadata.contact_point_2[0] + display_offset[0],
          metadata.contact_point_2[1] + display_offset[1],
          metadata.contact_point_2[2] + display_offset[2],
        };
        const std::array<double, 3> grasp_center_display{
          metadata.grasp_center[0] + display_offset[0],
          metadata.grasp_center[1] + display_offset[1],
          metadata.grasp_center[2] + display_offset[2],
        };
        append_sphere("contact_point_1", contact_point_1_display, 0.028, 1.00, 0.00, 0.00, 1.00);
        append_sphere("contact_point_2", contact_point_2_display, 0.028, 0.00, 0.35, 1.00, 1.00);
        append_sphere("grasp_center", grasp_center_display, 0.020, 1.00, 1.00, 0.00, 0.95);
        append_line_strip(
          "contact_span",
          {contact_point_1_display, contact_point_2_display},
          0.007,
          1.00, 0.92, 0.15, 0.92);

        const double approach_axis_length = 0.045;
        const std::array<double, 3> approach_tip{
          grasp_center_display[0] + metadata.approach_direction[0] * approach_axis_length,
          grasp_center_display[1] + metadata.approach_direction[1] * approach_axis_length,
          grasp_center_display[2] + metadata.approach_direction[2] * approach_axis_length,
        };
        append_arrow(
          "approach_axis",
          grasp_center_display,
          approach_tip,
          0.0035,
          0.007,
          0.010,
          0.10, 0.85, 1.00, 0.85);

        const double closing_half_span = std::max(0.012, 0.5 * metadata.grasp_width_m);
        const std::array<double, 3> closing_start{
          grasp_center_display[0] - metadata.closing_direction[0] * closing_half_span,
          grasp_center_display[1] - metadata.closing_direction[1] * closing_half_span,
          grasp_center_display[2] - metadata.closing_direction[2] * closing_half_span,
        };
        const std::array<double, 3> closing_end{
          grasp_center_display[0] + metadata.closing_direction[0] * closing_half_span,
          grasp_center_display[1] + metadata.closing_direction[1] * closing_half_span,
          grasp_center_display[2] + metadata.closing_direction[2] * closing_half_span,
        };
        append_arrow(
          "closing_axis",
          closing_start,
          closing_end,
          0.0025,
          0.006,
          0.008,
          1.00, 0.20, 0.85, 0.80);
      }

      const double top_z = std::max(entry.stage[2], std::max(entry.pregrasp[2], entry.grasp[2]));
      std::array<double, 3> text_point{entry.stage[0], entry.stage[1], top_z + 0.040 + 0.012 * index};
      const std::array<double, 3> leader_target =
        entry.external_metadata.has_value() ? std::array<double, 3>{
          entry.external_metadata->grasp_center[0] + entry.external_metadata->approach_direction[0] * 0.006,
          entry.external_metadata->grasp_center[1] + entry.external_metadata->approach_direction[1] * 0.006,
          entry.external_metadata->grasp_center[2] + entry.external_metadata->approach_direction[2] * 0.006 + 0.003,
        } : entry.grasp;
      append_line_strip(
        "execution_label_link",
        {text_point, leader_target},
        is_selected ? 0.003 : 0.0025,
        1.00, 1.00, 0.00, is_selected ? 0.95 : 0.75);
      std::ostringstream text_stream;
      text_stream << entry.candidate_label << " | " << entry.status;
      if (entry.feasible) {
        text_stream << " | score=" << std::fixed << std::setprecision(2) << entry.score;
      }
      if (!entry.variant_label.empty()) {
        text_stream << " | " << entry.variant_label;
      }
      text_stream
        << " | s=" << entry.stage_plan_failures
        << "/" << entry.stage_end_state_failures
        << " p=" << entry.pregrasp_plan_failures
        << "/" << entry.pregrasp_end_state_failures;
      append_text(
        "execution_label",
        text_point,
        text_stream.str(),
        is_selected ? 0.028 : 0.022,
        1.00, 1.00, 1.00, 0.95);
    }

    visualization_msgs::msg::MarkerArray clear_array;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = base_frame_;
    clear_marker.header.stamp = now;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    clear_array.markers.push_back(clear_marker);
    execution_debug_markers_pub_->publish(clear_array);
    if (!marker_array.markers.empty()) {
      execution_debug_markers_pub_->publish(marker_array);
    }
  }

  double compute_plan_duration_sec(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan) const
  {
    const auto & trajectory = plan.trajectory.joint_trajectory;
    if (trajectory.points.empty()) {
      return 0.0;
    }

    const auto & last_point = trajectory.points.back();
    return static_cast<double>(last_point.time_from_start.sec) +
      static_cast<double>(last_point.time_from_start.nanosec) / 1e9;
  }

  double compute_focus_joint_delta_rad(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan,
    const std::string & joint_name) const
  {
    if (joint_name.empty()) {
      return 0.0;
    }

    const auto & trajectory = plan.trajectory.joint_trajectory;
    if (trajectory.points.empty()) {
      return 0.0;
    }

    const auto joint_it = std::find(
      trajectory.joint_names.begin(), trajectory.joint_names.end(), joint_name);
    if (joint_it == trajectory.joint_names.end()) {
      return 0.0;
    }

    const auto joint_index = static_cast<std::size_t>(
      std::distance(trajectory.joint_names.begin(), joint_it));
    const auto & first_point = trajectory.points.front();
    const auto & last_point = trajectory.points.back();
    if (
      joint_index >= first_point.positions.size() ||
      joint_index >= last_point.positions.size())
    {
      return 0.0;
    }

    return std::abs(last_point.positions[joint_index] - first_point.positions[joint_index]);
  }

  double compute_focus_joint_excursion_rad(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan,
    const std::string & joint_name) const
  {
    if (joint_name.empty()) {
      return 0.0;
    }

    const auto & trajectory = plan.trajectory.joint_trajectory;
    if (trajectory.points.empty()) {
      return 0.0;
    }

    const auto joint_it = std::find(
      trajectory.joint_names.begin(), trajectory.joint_names.end(), joint_name);
    if (joint_it == trajectory.joint_names.end()) {
      return 0.0;
    }

    const auto joint_index = static_cast<std::size_t>(
      std::distance(trajectory.joint_names.begin(), joint_it));
    double min_position = std::numeric_limits<double>::infinity();
    double max_position = -std::numeric_limits<double>::infinity();
    for (const auto & point : trajectory.points) {
      if (joint_index >= point.positions.size()) {
        continue;
      }
      min_position = std::min(min_position, point.positions[joint_index]);
      max_position = std::max(max_position, point.positions[joint_index]);
    }

    if (!std::isfinite(min_position) || !std::isfinite(max_position)) {
      return 0.0;
    }
    return max_position - min_position;
  }

  std::array<double, 3> compute_stage_point(
    const geometry_msgs::msg::Pose & start_pose,
    const std::array<double, 3> & pregrasp_point) const
  {
    return {
      start_pose.position.x + 0.5 * (pregrasp_point[0] - start_pose.position.x),
      start_pose.position.y + 0.5 * (pregrasp_point[1] - start_pose.position.y),
      std::max(start_pose.position.z, pregrasp_point[2]) + 0.01,
    };
  }

  double compute_camera_pose_penalty(
    const moveit::core::RobotStatePtr & state,
    const std::array<double, 3> & target_point) const
  {
    if (!state || camera_scoring_link_.empty()) {
      return 0.0;
    }

    const auto * link_model = state->getLinkModel(camera_scoring_link_);
    if (link_model == nullptr) {
      return 0.0;
    }

    const Eigen::Isometry3d camera_transform = state->getGlobalLinkTransform(camera_scoring_link_);
    Eigen::Vector3d to_target(
      target_point[0] - camera_transform.translation().x(),
      target_point[1] - camera_transform.translation().y(),
      target_point[2] - camera_transform.translation().z());
    const double target_distance = to_target.norm();
    if (target_distance < 1e-6) {
      return 0.0;
    }
    to_target.normalize();

    const Eigen::Vector3d camera_forward = camera_transform.linear().col(2).normalized();
    const Eigen::Vector3d image_up = (-camera_transform.linear().col(1)).normalized();
    const double forward_alignment =
      std::clamp(camera_forward.dot(to_target), -1.0, 1.0);
    const double upright_alignment =
      std::clamp(image_up.dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0);

    const double forward_penalty = std::max(
      0.0, camera_forward_alignment_threshold_ - forward_alignment);
    const double upright_penalty = std::max(
      0.0, camera_upright_alignment_threshold_ - upright_alignment);

    return camera_forward_penalty_weight_ * forward_penalty +
      camera_upright_penalty_weight_ * upright_penalty;
  }

  double compute_camera_upright_tilt_rad(
    const moveit::core::RobotStatePtr & state) const
  {
    if (!state || camera_scoring_link_.empty()) {
      return M_PI;
    }

    const auto * link_model = state->getLinkModel(camera_scoring_link_);
    if (link_model == nullptr) {
      return M_PI;
    }

    const Eigen::Isometry3d camera_transform = state->getGlobalLinkTransform(camera_scoring_link_);
    const Eigen::Vector3d image_up = (-camera_transform.linear().col(1)).normalized();
    const double upright_alignment =
      std::clamp(image_up.dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0);
    return std::acos(upright_alignment);
  }

  double compute_camera_trajectory_penalty(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan,
    const moveit::core::RobotStatePtr & start_state,
    double * max_tilt_rad_out = nullptr) const
  {
    if (!start_state) {
      if (max_tilt_rad_out != nullptr) {
        *max_tilt_rad_out = M_PI;
      }
      return camera_flip_penalty_weight_;
    }

    const auto * joint_model_group = start_state->getJointModelGroup(arm_group_name_);
    if (joint_model_group == nullptr) {
      if (max_tilt_rad_out != nullptr) {
        *max_tilt_rad_out = M_PI;
      }
      return camera_flip_penalty_weight_;
    }

    auto state = std::make_shared<moveit::core::RobotState>(*start_state);
    const auto & variable_names = joint_model_group->getVariableNames();
    const auto & trajectory = plan.trajectory.joint_trajectory;
    double max_tilt_rad = compute_camera_upright_tilt_rad(state);

    for (const auto & point : trajectory.points) {
      std::vector<double> joint_positions;
      state->copyJointGroupPositions(joint_model_group, joint_positions);
      for (std::size_t index = 0; index < variable_names.size(); ++index) {
        const auto joint_it = std::find(
          trajectory.joint_names.begin(), trajectory.joint_names.end(), variable_names[index]);
        if (joint_it == trajectory.joint_names.end()) {
          continue;
        }
        const auto joint_index = static_cast<std::size_t>(
          std::distance(trajectory.joint_names.begin(), joint_it));
        if (joint_index >= point.positions.size()) {
          continue;
        }
        joint_positions[index] = point.positions[joint_index];
      }
      state->setJointGroupPositions(joint_model_group, joint_positions);
      state->update();
      max_tilt_rad = std::max(max_tilt_rad, compute_camera_upright_tilt_rad(state));
    }

    if (max_tilt_rad_out != nullptr) {
      *max_tilt_rad_out = max_tilt_rad;
    }

    const double stable_threshold_rad = camera_stable_tilt_reward_deg_ * M_PI / 180.0;
    const double tilt_penalty = camera_trajectory_tilt_penalty_weight_ * std::max(
      0.0, max_tilt_rad - stable_threshold_rad);
    const double flip_penalty = camera_flip_penalty_weight_ * std::max(
      0.0, max_tilt_rad - M_PI_2);
    return tilt_penalty + flip_penalty;
  }

  double compute_pregrasp_option_score(
    const geometry_msgs::msg::Pose & current_pose,
    const std::array<double, 3> & target_point,
    const std::array<double, 3> & stage_point,
    const PregraspVariant & variant,
    const moveit::planning_interface::MoveGroupInterface::Plan & stage_plan,
    const moveit::planning_interface::MoveGroupInterface::Plan & pregrasp_plan,
    const moveit::core::RobotStatePtr & stage_end_state,
    const moveit::core::RobotStatePtr & pregrasp_end_state,
    double * camera_penalty_out = nullptr,
    double * camera_stability_bonus_out = nullptr,
    double * primary_joint_excursion_cost_out = nullptr,
    double * secondary_joint_excursion_cost_out = nullptr,
    double * wrist_joint_excursion_cost_out = nullptr) const
  {
    const double total_duration_sec =
      compute_plan_duration_sec(stage_plan) + compute_plan_duration_sec(pregrasp_plan);
    const double path_distance_m =
      compute_point_distance(current_pose.position, stage_point) +
      compute_point_distance(stage_point, variant.pregrasp);
    const double approach_distance_m = compute_point_distance(variant.pregrasp, variant.grasp);
    const double primary_joint_cost =
      compute_focus_joint_delta_rad(stage_plan, lock_joint_name_) +
      compute_focus_joint_delta_rad(pregrasp_plan, lock_joint_name_);
    const double wrist_joint_cost =
      compute_focus_joint_delta_rad(stage_plan, wrist_joint_name_) +
      compute_focus_joint_delta_rad(pregrasp_plan, wrist_joint_name_);
    const double primary_joint_excursion_cost =
      compute_focus_joint_excursion_rad(stage_plan, lock_joint_name_) +
      compute_focus_joint_excursion_rad(pregrasp_plan, lock_joint_name_);
    const double secondary_joint_excursion_cost =
      compute_focus_joint_excursion_rad(stage_plan, secondary_joint_name_) +
      compute_focus_joint_excursion_rad(pregrasp_plan, secondary_joint_name_);
    const double wrist_joint_excursion_cost =
      compute_focus_joint_excursion_rad(stage_plan, wrist_joint_name_) +
      compute_focus_joint_excursion_rad(pregrasp_plan, wrist_joint_name_);
    const double endpoint_camera_penalty =
      0.4 * compute_camera_pose_penalty(stage_end_state, target_point) +
      1.0 * compute_camera_pose_penalty(pregrasp_end_state, target_point);
    const auto stage_start_state = arm_group_->getCurrentState(0.5);
    double stage_max_tilt_rad = M_PI;
    double pregrasp_max_tilt_rad = M_PI;
    const double trajectory_camera_penalty =
      0.6 * compute_camera_trajectory_penalty(stage_plan, stage_start_state, &stage_max_tilt_rad) +
      1.0 * compute_camera_trajectory_penalty(
      pregrasp_plan, stage_end_state, &pregrasp_max_tilt_rad);
    const double camera_penalty = endpoint_camera_penalty + trajectory_camera_penalty;
    const double stable_threshold_rad = camera_stable_tilt_reward_deg_ * M_PI / 180.0;
    const double camera_stability_bonus =
      stage_max_tilt_rad <= stable_threshold_rad &&
      pregrasp_max_tilt_rad <= stable_threshold_rad ? camera_stable_reward_bonus_ : 0.0;
    const double clearance_penalty =
      std::max(0.0, variant.pregrasp[2] - (variant.grasp[2] + 0.08));
    const double duration_penalty = plan_duration_penalty_weight_ * total_duration_sec;
    const double path_penalty = path_distance_penalty_weight_ * path_distance_m;
    const double approach_penalty = approach_distance_penalty_weight_ * approach_distance_m;
    const double primary_joint_delta_penalty =
      primary_joint_delta_penalty_weight_ * primary_joint_cost;
    const double wrist_joint_delta_penalty =
      wrist_joint_delta_penalty_weight_ * wrist_joint_cost;
    const double primary_joint_excursion_penalty =
      primary_joint_excursion_penalty_weight_ * primary_joint_excursion_cost;
    const double secondary_joint_excursion_penalty =
      secondary_joint_excursion_penalty_weight_ * secondary_joint_excursion_cost;
    const double wrist_joint_excursion_penalty =
      wrist_joint_excursion_penalty_weight_ * wrist_joint_excursion_cost;
    const double clearance_cost = clearance_penalty_weight_ * clearance_penalty;
    const double raw_score =
      pregrasp_score_base_
      - duration_penalty
      - path_penalty
      - approach_penalty
      - primary_joint_delta_penalty
      - wrist_joint_delta_penalty
      - primary_joint_excursion_penalty
      - secondary_joint_excursion_penalty
      - wrist_joint_excursion_penalty
      - camera_penalty
      - clearance_cost
      + camera_stability_bonus;

    if (camera_penalty_out != nullptr) {
      *camera_penalty_out = camera_penalty;
    }
    if (camera_stability_bonus_out != nullptr) {
      *camera_stability_bonus_out = camera_stability_bonus;
    }
    if (primary_joint_excursion_cost_out != nullptr) {
      *primary_joint_excursion_cost_out = primary_joint_excursion_cost;
    }
    if (secondary_joint_excursion_cost_out != nullptr) {
      *secondary_joint_excursion_cost_out = secondary_joint_excursion_cost;
    }
    if (wrist_joint_excursion_cost_out != nullptr) {
      *wrist_joint_excursion_cost_out = wrist_joint_excursion_cost;
    }

    return std::max(pregrasp_score_floor_, raw_score);
  }

  double compute_direct_pregrasp_option_score(
    const geometry_msgs::msg::Pose & current_pose,
    const std::array<double, 3> & target_point,
    const PregraspVariant & variant,
    const moveit::planning_interface::MoveGroupInterface::Plan & pregrasp_plan,
    const moveit::core::RobotStatePtr & pregrasp_end_state,
    double * camera_penalty_out = nullptr,
    double * camera_stability_bonus_out = nullptr,
    double * primary_joint_excursion_cost_out = nullptr,
    double * secondary_joint_excursion_cost_out = nullptr,
    double * wrist_joint_excursion_cost_out = nullptr) const
  {
    const double total_duration_sec = compute_plan_duration_sec(pregrasp_plan);
    const double path_distance_m = compute_point_distance(current_pose.position, variant.pregrasp);
    const double approach_distance_m = compute_point_distance(variant.pregrasp, variant.grasp);
    const double primary_joint_cost =
      compute_focus_joint_delta_rad(pregrasp_plan, lock_joint_name_);
    const double wrist_joint_cost =
      compute_focus_joint_delta_rad(pregrasp_plan, wrist_joint_name_);
    const double primary_joint_excursion_cost =
      compute_focus_joint_excursion_rad(pregrasp_plan, lock_joint_name_);
    const double secondary_joint_excursion_cost =
      compute_focus_joint_excursion_rad(pregrasp_plan, secondary_joint_name_);
    const double wrist_joint_excursion_cost =
      compute_focus_joint_excursion_rad(pregrasp_plan, wrist_joint_name_);
    const double endpoint_camera_penalty =
      1.0 * compute_camera_pose_penalty(pregrasp_end_state, target_point);
    const auto pregrasp_start_state = arm_group_->getCurrentState(0.5);
    double pregrasp_max_tilt_rad = M_PI;
    const double trajectory_camera_penalty =
      compute_camera_trajectory_penalty(
      pregrasp_plan, pregrasp_start_state, &pregrasp_max_tilt_rad);
    const double camera_penalty = endpoint_camera_penalty + trajectory_camera_penalty;
    const double stable_threshold_rad = camera_stable_tilt_reward_deg_ * M_PI / 180.0;
    const double camera_stability_bonus =
      pregrasp_max_tilt_rad <= stable_threshold_rad ? camera_stable_reward_bonus_ : 0.0;
    const double clearance_penalty =
      std::max(0.0, variant.pregrasp[2] - (variant.grasp[2] + 0.08));
    const double duration_penalty = plan_duration_penalty_weight_ * total_duration_sec;
    const double path_penalty = path_distance_penalty_weight_ * path_distance_m;
    const double approach_penalty = approach_distance_penalty_weight_ * approach_distance_m;
    const double primary_joint_delta_penalty =
      primary_joint_delta_penalty_weight_ * primary_joint_cost;
    const double wrist_joint_delta_penalty =
      wrist_joint_delta_penalty_weight_ * wrist_joint_cost;
    const double primary_joint_excursion_penalty =
      primary_joint_excursion_penalty_weight_ * primary_joint_excursion_cost;
    const double secondary_joint_excursion_penalty =
      secondary_joint_excursion_penalty_weight_ * secondary_joint_excursion_cost;
    const double wrist_joint_excursion_penalty =
      wrist_joint_excursion_penalty_weight_ * wrist_joint_excursion_cost;
    const double clearance_cost = clearance_penalty_weight_ * clearance_penalty;
    const double raw_score =
      pregrasp_score_base_
      - duration_penalty
      - path_penalty
      - approach_penalty
      - primary_joint_delta_penalty
      - wrist_joint_delta_penalty
      - primary_joint_excursion_penalty
      - secondary_joint_excursion_penalty
      - wrist_joint_excursion_penalty
      - camera_penalty
      - clearance_cost
      + camera_stability_bonus;

    if (camera_penalty_out != nullptr) {
      *camera_penalty_out = camera_penalty;
    }
    if (camera_stability_bonus_out != nullptr) {
      *camera_stability_bonus_out = camera_stability_bonus;
    }
    if (primary_joint_excursion_cost_out != nullptr) {
      *primary_joint_excursion_cost_out = primary_joint_excursion_cost;
    }
    if (secondary_joint_excursion_cost_out != nullptr) {
      *secondary_joint_excursion_cost_out = secondary_joint_excursion_cost;
    }
    if (wrist_joint_excursion_cost_out != nullptr) {
      *wrist_joint_excursion_cost_out = wrist_joint_excursion_cost;
    }

    return std::max(pregrasp_score_floor_, raw_score);
  }

  ExternalProposalMetadata build_external_proposal_metadata(
    const tactile_interfaces::msg::GraspProposal & proposal) const
  {
    ExternalProposalMetadata metadata;
    metadata.contact_point_1 = point_to_array(proposal.contact_point_1);
    metadata.contact_point_2 = point_to_array(proposal.contact_point_2);
    metadata.grasp_center = point_to_array(proposal.grasp_center);
    metadata.approach_direction = vector_to_array(proposal.approach_direction);
    metadata.closing_direction = vector_to_array(proposal.closing_direction);
    metadata.grasp_width_m = proposal.grasp_width_m;
    metadata.confidence_score = proposal.confidence_score;
    metadata.semantic_score = proposal.semantic_score;
    metadata.candidate_rank = proposal.candidate_rank;
    metadata.task_constraint_tag = proposal.task_constraint_tag;
    return metadata;
  }

  double compute_external_proposal_bonus(
    const std::optional<ExternalProposalMetadata> & metadata,
    double * semantic_score_out = nullptr,
    double * confidence_score_out = nullptr) const
  {
    if (!metadata.has_value()) {
      if (semantic_score_out != nullptr) {
        *semantic_score_out = 0.0;
      }
      if (confidence_score_out != nullptr) {
        *confidence_score_out = 0.0;
      }
      return 0.0;
    }

    const double semantic_score = std::max(0.0, metadata->semantic_score);
    const double confidence_score = std::clamp(metadata->confidence_score, 0.0, 1.0);
    if (semantic_score_out != nullptr) {
      *semantic_score_out = semantic_score;
    }
    if (confidence_score_out != nullptr) {
      *confidence_score_out = confidence_score;
    }
    return
      external_semantic_score_weight_ * semantic_score +
      external_confidence_score_weight_ * confidence_score;
  }

  std::string format_xyz(const std::array<double, 3> & point) const
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "(" << point[0] << ", " << point[1] << ", " << point[2] << ")";
    return stream.str();
  }

  std::string format_external_debug_entry(
    const ExternalEvaluationDebugInfo & entry) const
  {
    const double input_semantic_score =
      entry.external_metadata.has_value() ? entry.external_metadata->semantic_score :
      entry.semantic_score;
    const double input_confidence_score =
      entry.external_metadata.has_value() ? entry.external_metadata->confidence_score :
      entry.confidence_score;

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "candidate=" << entry.candidate_index
           << " label=" << entry.candidate_label
           << " status=" << entry.status;
    if (!entry.variant_label.empty()) {
      stream << " variant=" << entry.variant_label;
    }
    stream << " stage=" << format_xyz(entry.stage)
           << " pregrasp=" << format_xyz(entry.pregrasp)
           << " grasp=" << format_xyz(entry.grasp)
           << " score=" << entry.score
           << " semantic=" << input_semantic_score
           << " confidence=" << input_confidence_score
           << " bonus=" << entry.proposal_bonus
           << " fail[s_plan=" << entry.stage_plan_failures
           << " s_state=" << entry.stage_end_state_failures
           << " p_plan=" << entry.pregrasp_plan_failures
           << " p_state=" << entry.pregrasp_end_state_failures
           << "]";
    return stream.str();
  }

  std::string format_progress_bar(
    std::size_t completed,
    std::size_t total,
    std::size_t width = 10) const
  {
    if (width == 0) {
      width = 10;
    }
    const std::size_t safe_total = std::max<std::size_t>(1, total);
    const double ratio =
      std::clamp(static_cast<double>(completed) / static_cast<double>(safe_total), 0.0, 1.0);
    const std::size_t filled =
      std::min(width, static_cast<std::size_t>(std::llround(ratio * static_cast<double>(width))));
    return "[" + std::string(filled, '#') + std::string(width - filled, '-') + "]";
  }

  void log_planning_progress(
    const std::string & stage,
    std::size_t completed,
    std::size_t total,
    std::size_t feasible,
    const std::string & label,
    const std::string & status)
  {
    const std::size_t safe_total = std::max<std::size_t>(1, total);
    const int percent = static_cast<int>(
      std::llround(
        100.0 * static_cast<double>(std::min(completed, safe_total)) /
        static_cast<double>(safe_total)));
    RCLCPP_INFO(
      this->get_logger(),
      "%s progress %s %zu/%zu (%d%%) feasible=%zu last=%s status=%s",
      stage.c_str(),
      format_progress_bar(completed, total).c_str(),
      completed,
      total,
      percent,
      feasible,
      label.c_str(),
      status.c_str());
    if (total > 0) {
      std::ostringstream message;
      message << stage << " " << format_progress_bar(completed, total)
              << " " << completed << "/" << total
              << " feasible=" << feasible
              << " last=" << label
              << " status=" << status;
      publish_pick_status(
        "planning",
        message.str(),
        static_cast<int>(completed),
        static_cast<int>(total),
        stage);
    }
  }

  std::optional<PlannedApproachOption> select_external_pregrasp_option(
    const geometry_msgs::msg::Pose & current_pose,
    const std::array<double, 3> & target_point)
  {
    if (!prefer_external_grasp_candidates_) {
      publish_external_execution_markers({}, std::nullopt);
      return std::nullopt;
    }

    geometry_msgs::msg::PoseStamped selected_pregrasp_pose;
    geometry_msgs::msg::PoseStamped selected_grasp_pose;
    geometry_msgs::msg::PoseArray external_pregrasp_pose_array;
    geometry_msgs::msg::PoseArray external_grasp_pose_array;
    tactile_interfaces::msg::GraspProposalArray external_grasp_proposal_array;
    bool have_pose_arrays = false;
    bool have_proposal_array = false;
    bool have_single_candidate = false;
    std::size_t proposal_candidate_count = 0;
    std::size_t pose_candidate_count = 0;
    {
      std::scoped_lock<std::mutex> lock(grasp_candidate_mutex_);
      const bool has_single_candidate_local =
        has_selected_pregrasp_pose_ && has_selected_grasp_pose_;
      const bool has_pose_candidate_arrays =
        has_external_pregrasp_pose_array_ && has_external_grasp_pose_array_;
      const bool has_proposal_array_local = has_external_grasp_proposal_array_;
      if (!has_single_candidate_local && !has_pose_candidate_arrays && !has_proposal_array_local) {
        if (require_external_grasp_candidates_) {
          RCLCPP_WARN(
            this->get_logger(),
            "external grasp candidates required but none have been received yet");
        }
        publish_external_execution_markers({}, std::nullopt);
        return std::nullopt;
      }

      const double age_sec =
        this->get_clock()->now().seconds() - latest_external_grasp_update_sec_;
      if (age_sec > external_grasp_pose_timeout_sec_) {
        if (require_external_grasp_candidates_) {
          RCLCPP_WARN(
            this->get_logger(),
            "external grasp candidates required but latest update is stale: age=%.2fs timeout=%.2fs",
            age_sec,
            external_grasp_pose_timeout_sec_);
        }
        publish_external_execution_markers({}, std::nullopt);
        return std::nullopt;
      }

      if (has_proposal_array_local) {
        external_grasp_proposal_array = latest_external_grasp_proposal_array_;
        have_proposal_array = true;
        proposal_candidate_count = external_grasp_proposal_array.proposals.size();
      }
      if (has_pose_candidate_arrays) {
        external_pregrasp_pose_array = latest_external_pregrasp_pose_array_;
        external_grasp_pose_array = latest_external_grasp_pose_array_;
        have_pose_arrays = true;
        pose_candidate_count = std::min(
          external_pregrasp_pose_array.poses.size(),
          external_grasp_pose_array.poses.size());
      }
      if (has_single_candidate_local) {
        selected_pregrasp_pose = latest_selected_pregrasp_pose_;
        selected_grasp_pose = latest_selected_grasp_pose_;
        have_single_candidate = true;
      }
    }

    const auto joint_lock_attempts = get_locked_joint_attempts();
    const std::vector<JointConstraintAttempt> attempts =
      joint_lock_attempts.empty() ? std::vector<JointConstraintAttempt>{JointConstraintAttempt{}} :
      joint_lock_attempts;
    std::optional<PlannedApproachOption> best_option;
    std::size_t candidates_received = proposal_candidate_count + pose_candidate_count +
      (have_single_candidate ? 1U : 0U);
    std::size_t candidates_tested = 0;
    std::size_t candidates_feasible = 0;
    std::size_t stage_plan_failures = 0;
    std::size_t stage_end_state_failures = 0;
    std::size_t pregrasp_plan_failures = 0;
    std::size_t pregrasp_end_state_failures = 0;
    std::vector<ExternalEvaluationDebugInfo> debug_entries;

    RCLCPP_INFO(
      this->get_logger(),
      "external grasp evaluation start: proposals=%zu pose_pairs=%zu single=%s joint_attempts=%zu require_external=%s",
      proposal_candidate_count,
      pose_candidate_count,
      have_single_candidate ? "true" : "false",
      attempts.size(),
      require_external_grasp_candidates_ ? "true" : "false");

    auto evaluate_external_candidate =
      [&](const ApproachCandidate & candidate,
        const std::string & candidate_label,
        std::size_t candidate_index,
        const std::optional<ExternalProposalMetadata> & external_metadata = std::nullopt)
      {
        ++candidates_tested;
        std::size_t candidate_stage_plan_failures = 0;
        std::size_t candidate_stage_end_state_failures = 0;
        std::size_t candidate_pregrasp_plan_failures = 0;
        std::size_t candidate_pregrasp_end_state_failures = 0;
        bool candidate_feasible = false;
        const auto variants = build_local_pregrasp_variants(candidate);
        ExternalEvaluationDebugInfo debug_info;
        debug_info.candidate_index = candidate_index;
        debug_info.candidate_label = candidate_label;
        debug_info.status = "rejected";
        debug_info.stage = compute_stage_point(current_pose, variants.front().pregrasp);
        debug_info.pregrasp = variants.front().pregrasp;
        debug_info.grasp = variants.front().grasp;
        debug_info.external_metadata = external_metadata;

        for (const auto & variant : variants) {
          const std::string pregrasp_label =
            "move directly to pregrasp " + candidate_label + variant.label_suffix;

          for (const auto & attempt : attempts) {
            moveit::planning_interface::MoveGroupInterface::Plan pregrasp_plan;
            if (!plan_pose_target_attempt(
                variant.pregrasp,
                variant.pregrasp_orientation,
                attempt,
                pregrasp_plan,
                nullptr,
                pregrasp_position_only_target_))
            {
              ++candidate_pregrasp_plan_failures;
              ++pregrasp_plan_failures;
              continue;
            }

            const auto pregrasp_end_state = build_plan_end_state(pregrasp_plan);
            if (!pregrasp_end_state) {
              ++candidate_pregrasp_end_state_failures;
              ++pregrasp_end_state_failures;
              continue;
            }

            double camera_penalty = 0.0;
            double camera_stability_bonus = 0.0;
            double primary_joint_excursion_cost = 0.0;
            double secondary_joint_excursion_cost = 0.0;
            double wrist_joint_excursion_cost = 0.0;
            const double base_score = compute_direct_pregrasp_option_score(
              current_pose,
              target_point,
              variant,
              pregrasp_plan,
              pregrasp_end_state,
              &camera_penalty,
              &camera_stability_bonus,
              &primary_joint_excursion_cost,
              &secondary_joint_excursion_cost,
              &wrist_joint_excursion_cost);
            double external_semantic_score = 0.0;
            double external_confidence_score = 0.0;
            const double external_proposal_bonus = compute_external_proposal_bonus(
              external_metadata,
              &external_semantic_score,
              &external_confidence_score);
            const double score = base_score + external_proposal_bonus;
            candidate_feasible = true;
            ++candidates_feasible;
            debug_info.feasible = true;
            debug_info.status = "feasible";
            debug_info.variant_label = variant.label_suffix.empty() ? "base" : variant.label_suffix;
            debug_info.stage = variant.pregrasp;
            debug_info.pregrasp = variant.pregrasp;
            debug_info.grasp = variant.grasp;
            debug_info.score = score;
            debug_info.semantic_score = external_semantic_score;
            debug_info.confidence_score = external_confidence_score;
            debug_info.proposal_bonus = external_proposal_bonus;

            if (!best_option.has_value() || score > best_option->score) {
              best_option = PlannedApproachOption{
                candidate_index,
                variant.pregrasp,
                variant.pregrasp,
                variant.grasp,
                variant.pregrasp_orientation,
                variant.pregrasp_orientation,
                variant.grasp_orientation,
                "skip stage (direct to pregrasp) " + candidate_label,
                pregrasp_label,
                moveit::planning_interface::MoveGroupInterface::Plan{},
                pregrasp_plan,
                score,
                camera_penalty,
                camera_stability_bonus,
                primary_joint_excursion_cost,
                secondary_joint_excursion_cost,
                wrist_joint_excursion_cost,
                external_semantic_score,
                external_confidence_score,
                external_proposal_bonus,
                external_metadata,
                true,
              };
            }

            RCLCPP_DEBUG(
              this->get_logger(),
              "%s feasible direct-pregrasp: variant=%s score=%.2f base=%.2f semantic=%.2f confidence=%.2f bonus=%.2f pregrasp=(%.3f, %.3f, %.3f) grasp=(%.3f, %.3f, %.3f)",
              candidate_label.c_str(),
              variant.label_suffix.empty() ? "base" : variant.label_suffix.c_str(),
              score,
              base_score,
              external_semantic_score,
              external_confidence_score,
              external_proposal_bonus,
              variant.pregrasp[0], variant.pregrasp[1], variant.pregrasp[2],
              variant.grasp[0], variant.grasp[1], variant.grasp[2]);
            break;
          }
          if (candidate_feasible) {
            break;
          }
        }

        debug_info.stage_plan_failures = candidate_stage_plan_failures;
        debug_info.stage_end_state_failures = candidate_stage_end_state_failures;
        debug_info.pregrasp_plan_failures = candidate_pregrasp_plan_failures;
        debug_info.pregrasp_end_state_failures = candidate_pregrasp_end_state_failures;

        if (!candidate_feasible) {
          if (candidate_pregrasp_end_state_failures > 0) {
            debug_info.status = "pregrasp end-state fail";
          } else if (candidate_pregrasp_plan_failures > 0) {
            debug_info.status = "pregrasp plan fail";
          } else {
            debug_info.status = "not executable";
          }
          RCLCPP_DEBUG(
            this->get_logger(),
            "%s rejected direct-pregrasp: pregrasp_plan_fail=%zu pregrasp_end_state_fail=%zu",
            candidate_label.c_str(),
            candidate_pregrasp_plan_failures,
            candidate_pregrasp_end_state_failures);
        }
        debug_entries.push_back(debug_info);
        publish_external_execution_markers(debug_entries, best_option);
        log_planning_progress(
          "planning/external",
          candidates_tested,
          candidates_received,
          candidates_feasible,
          candidate_label,
          debug_info.status);
      };

    if (have_proposal_array) {
      if (
        !external_grasp_proposal_array.header.frame_id.empty() &&
        external_grasp_proposal_array.header.frame_id != base_frame_)
      {
        RCLCPP_WARN(
          this->get_logger(),
          "external grasp proposal array ignored: frame_id=%s expected=%s",
          external_grasp_proposal_array.header.frame_id.c_str(),
          base_frame_.c_str());
      } else {
        for (std::size_t index = 0; index < external_grasp_proposal_array.proposals.size(); ++index) {
          const auto & proposal = external_grasp_proposal_array.proposals[index];
          const std::string proposal_frame_id =
            proposal.header.frame_id.empty() ? external_grasp_proposal_array.header.frame_id :
            proposal.header.frame_id;
          if (!proposal_frame_id.empty() && proposal_frame_id != base_frame_) {
            RCLCPP_WARN(
              this->get_logger(),
              "external grasp proposal %zu ignored: frame_id=%s expected=%s",
              index + 1,
              proposal_frame_id.c_str(),
              base_frame_.c_str());
            continue;
          }

          const auto metadata = build_external_proposal_metadata(proposal);
          const std::size_t resolved_candidate_index =
            metadata.candidate_rank > 0 ? metadata.candidate_rank : index + 1;
          evaluate_external_candidate(
            ApproachCandidate{
              {
                proposal.pregrasp_pose.position.x,
                proposal.pregrasp_pose.position.y,
                proposal.pregrasp_pose.position.z,
              },
              {
                proposal.grasp_pose.position.x,
                proposal.grasp_pose.position.y,
                proposal.grasp_pose.position.z,
              },
              proposal.pregrasp_pose.orientation,
              proposal.grasp_pose.orientation,
            },
            "external proposal " + std::to_string(resolved_candidate_index),
            resolved_candidate_index,
            metadata);
        }
      }
    }

    if (have_pose_arrays) {
      if (
        !external_pregrasp_pose_array.header.frame_id.empty() &&
        external_pregrasp_pose_array.header.frame_id != base_frame_)
      {
        RCLCPP_WARN(
          this->get_logger(),
          "external pregrasp candidate array ignored: frame_id=%s expected=%s",
          external_pregrasp_pose_array.header.frame_id.c_str(),
          base_frame_.c_str());
      } else if (
        !external_grasp_pose_array.header.frame_id.empty() &&
        external_grasp_pose_array.header.frame_id != base_frame_)
      {
        RCLCPP_WARN(
          this->get_logger(),
          "external grasp candidate array ignored: frame_id=%s expected=%s",
          external_grasp_pose_array.header.frame_id.c_str(),
          base_frame_.c_str());
      } else {
        const std::size_t candidate_count = std::min(
          external_pregrasp_pose_array.poses.size(),
          external_grasp_pose_array.poses.size());
        for (std::size_t index = 0; index < candidate_count; ++index) {
          const auto & pregrasp_pose = external_pregrasp_pose_array.poses[index];
          const auto & grasp_pose = external_grasp_pose_array.poses[index];
          evaluate_external_candidate(
            ApproachCandidate{
              {pregrasp_pose.position.x, pregrasp_pose.position.y, pregrasp_pose.position.z},
              {grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z},
              pregrasp_pose.orientation,
              grasp_pose.orientation,
            },
            "external candidate " + std::to_string(index + 1),
            index + 1);
        }
      }
    }

    if (
      !best_option.has_value() &&
      !have_proposal_array &&
      !have_pose_arrays &&
      !selected_pregrasp_pose.header.frame_id.empty() &&
      selected_pregrasp_pose.header.frame_id != base_frame_)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "external pregrasp candidate ignored: frame_id=%s expected=%s",
        selected_pregrasp_pose.header.frame_id.c_str(),
        base_frame_.c_str());
      publish_external_execution_markers({}, std::nullopt);
      return std::nullopt;
    }
    if (
      !best_option.has_value() &&
      !have_proposal_array &&
      !have_pose_arrays &&
      !selected_grasp_pose.header.frame_id.empty() &&
      selected_grasp_pose.header.frame_id != base_frame_)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "external grasp candidate ignored: frame_id=%s expected=%s",
        selected_grasp_pose.header.frame_id.c_str(),
        base_frame_.c_str());
      publish_external_execution_markers({}, std::nullopt);
      return std::nullopt;
    }
    if (!best_option.has_value() && !have_proposal_array && !have_pose_arrays && have_single_candidate) {
      evaluate_external_candidate(
        ApproachCandidate{
          {
            selected_pregrasp_pose.pose.position.x,
            selected_pregrasp_pose.pose.position.y,
            selected_pregrasp_pose.pose.position.z,
          },
          {
            selected_grasp_pose.pose.position.x,
            selected_grasp_pose.pose.position.y,
            selected_grasp_pose.pose.position.z,
          },
          selected_pregrasp_pose.pose.orientation,
          selected_grasp_pose.pose.orientation,
        },
        "external candidate",
        1);
    }

    publish_external_execution_markers(debug_entries, best_option);

    if (best_option.has_value()) {
      RCLCPP_INFO(
        this->get_logger(),
        "external grasp candidate accepted: tested=%zu/%zu feasible=%zu stage_fail=%zu stage_state_fail=%zu pregrasp_fail=%zu pregrasp_state_fail=%zu pregrasp=(%.3f, %.3f, %.3f) grasp=(%.3f, %.3f, %.3f) score=%.2f semantic=%.2f confidence=%.2f bonus=%.2f",
        candidates_tested,
        candidates_received,
        candidates_feasible,
        stage_plan_failures,
        stage_end_state_failures,
        pregrasp_plan_failures,
        pregrasp_end_state_failures,
        best_option->pregrasp[0], best_option->pregrasp[1], best_option->pregrasp[2],
        best_option->grasp[0], best_option->grasp[1], best_option->grasp[2],
        best_option->score,
        best_option->external_semantic_score,
        best_option->external_confidence_score,
        best_option->external_proposal_bonus);
      return best_option;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "external grasp candidate available but not executable: received=%zu tested=%zu feasible=%zu stage_fail=%zu stage_state_fail=%zu pregrasp_fail=%zu pregrasp_state_fail=%zu; falling back to internal templates",
      candidates_received,
      candidates_tested,
      candidates_feasible,
      stage_plan_failures,
      stage_end_state_failures,
      pregrasp_plan_failures,
      pregrasp_end_state_failures);
    for (const auto & entry : debug_entries) {
      RCLCPP_WARN(
        this->get_logger(),
        "external grasp evaluation detail: %s",
        format_external_debug_entry(entry).c_str());
    }
    return std::nullopt;
  }

  PlannedApproachEvaluation evaluate_internal_pregrasp_options(
    const geometry_msgs::msg::Pose & current_pose,
    const std::array<double, 3> & target_point)
  {
    PlannedApproachEvaluation evaluation;
    const auto candidates = build_approach_candidates(current_pose, target_point);
    const auto joint_lock_attempts = get_locked_joint_attempts();
    const std::vector<JointConstraintAttempt> attempts =
      joint_lock_attempts.empty() ? std::vector<JointConstraintAttempt>{JointConstraintAttempt{}} :
      joint_lock_attempts;

    for (std::size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index) {
      const auto & candidate = candidates[candidate_index];
      const auto pregrasp_variants = build_local_pregrasp_variants(candidate);
      bool candidate_feasible = false;

      for (const auto & variant : pregrasp_variants) {
        ++evaluation.tested_options;
        const auto stage_point = compute_stage_point(current_pose, variant.pregrasp);
        const std::string stage_label =
          "move to stage candidate " + std::to_string(candidate_index + 1);
        const std::string pregrasp_label =
          "move to pregrasp candidate " + std::to_string(candidate_index + 1) +
          variant.label_suffix;

        for (const auto & attempt : attempts) {
          moveit::planning_interface::MoveGroupInterface::Plan stage_plan;
          if (!plan_pose_target_attempt(
              stage_point,
              variant.pregrasp_orientation,
              attempt,
              stage_plan,
              nullptr,
              stage_position_only_target_))
          {
            continue;
          }

          const auto stage_end_state = build_plan_end_state(stage_plan);
          if (!stage_end_state) {
            continue;
          }

          moveit::planning_interface::MoveGroupInterface::Plan pregrasp_plan;
          if (!plan_pose_target_attempt(
              variant.pregrasp,
              variant.pregrasp_orientation,
              attempt,
              pregrasp_plan,
              stage_end_state,
              pregrasp_position_only_target_))
          {
            continue;
          }

          const auto pregrasp_end_state = build_plan_end_state(pregrasp_plan);
          if (!pregrasp_end_state) {
            continue;
          }

          double camera_penalty = 0.0;
          double camera_stability_bonus = 0.0;
          double primary_joint_excursion_cost = 0.0;
          double secondary_joint_excursion_cost = 0.0;
          double wrist_joint_excursion_cost = 0.0;
          const double score = compute_pregrasp_option_score(
            current_pose,
            target_point,
            stage_point,
            variant,
            stage_plan,
            pregrasp_plan,
            stage_end_state,
            pregrasp_end_state,
            &camera_penalty,
              &camera_stability_bonus,
              &primary_joint_excursion_cost,
              &secondary_joint_excursion_cost,
              &wrist_joint_excursion_cost);
          ++evaluation.feasible_count;
          evaluation.feasible_options.push_back(PlannedApproachOption{
            candidate_index + 1,
            stage_point,
            variant.pregrasp,
            variant.grasp,
            variant.pregrasp_orientation,
            variant.pregrasp_orientation,
            variant.grasp_orientation,
            stage_label,
            pregrasp_label,
            stage_plan,
            pregrasp_plan,
            score,
            camera_penalty,
            camera_stability_bonus,
            primary_joint_excursion_cost,
            secondary_joint_excursion_cost,
            wrist_joint_excursion_cost,
            0.0,
            0.0,
            0.0,
            std::nullopt,
          });
          candidate_feasible = true;
          break;
        }
      }

      log_planning_progress(
        "planning/internal",
        candidate_index + 1,
        candidates.size(),
        evaluation.feasible_count,
        "candidate " + std::to_string(candidate_index + 1),
        candidate_feasible ? "feasible" : "rejected");
    }

    std::sort(
      evaluation.feasible_options.begin(),
      evaluation.feasible_options.end(),
      [](const PlannedApproachOption & lhs, const PlannedApproachOption & rhs) {
        return lhs.score > rhs.score;
      });

    return evaluation;
  }

  std::optional<PlannedApproachOption> select_best_pregrasp_option(
    const geometry_msgs::msg::Pose & current_pose,
    const std::array<double, 3> & target_point)
  {
    const auto evaluation = evaluate_internal_pregrasp_options(current_pose, target_point);

    if (evaluation.feasible_options.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "pregrasp evaluation: no feasible option (tested=%zu)",
        evaluation.tested_options);
      return std::nullopt;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "pregrasp evaluation: tested=%zu feasible=%zu best=candidate %zu score=%.2f",
      evaluation.tested_options,
      evaluation.feasible_count,
      evaluation.feasible_options.front().candidate_index,
      evaluation.feasible_options.front().score);
    return evaluation.feasible_options.front();
  }

  std::optional<PlannedApproachOption> select_pregrasp_option(
    const geometry_msgs::msg::Pose & current_pose,
    const std::array<double, 3> & target_point)
  {
    if (const auto external_option = select_external_pregrasp_option(current_pose, target_point)) {
      return external_option;
    }
    if (require_external_grasp_candidates_) {
      RCLCPP_WARN(
        this->get_logger(),
        "external grasp candidates are required; aborting without internal fallback");
      return std::nullopt;
    }
    return select_best_pregrasp_option(current_pose, target_point);
  }

  std::vector<JointConstraintAttempt> get_locked_joint_attempts() const
  {
    if (!has_locked_joint_constraints()) {
      return {};
    }

    const auto to_rad = [](double delta_deg) {
      return delta_deg > 0.0 ? delta_deg * M_PI / 180.0 : -1.0;
    };

    std::vector<JointConstraintAttempt> attempts;
    const JointConstraintAttempt primary_attempt{
      to_rad(lock_joint_max_delta_deg_),
      to_rad(wrist_joint_max_delta_deg_),
      "",
    };
    if (make_locked_joint_constraints(primary_attempt).joint_constraints.size() > 0) {
      attempts.push_back(primary_attempt);
    }

    const JointConstraintAttempt relaxed_attempt{
      to_rad(lock_joint_relaxed_delta_deg_),
      to_rad(wrist_joint_relaxed_delta_deg_),
      " relaxed",
    };
    const bool same_as_primary =
      std::abs(relaxed_attempt.primary_joint_delta_rad - primary_attempt.primary_joint_delta_rad) <
        1e-6 &&
      std::abs(relaxed_attempt.wrist_joint_delta_rad - primary_attempt.wrist_joint_delta_rad) <
        1e-6;
    if (!same_as_primary &&
      make_locked_joint_constraints(relaxed_attempt).joint_constraints.size() > 0)
    {
      attempts.push_back(relaxed_attempt);
    }

    return attempts;
  }

  std::string target_pose_topic_;
  std::string target_locked_topic_;
  std::string refresh_grasp_candidates_topic_;
  std::string pick_active_topic_;
  std::string pick_status_topic_;
  std::string execute_pick_service_;
  std::string reset_pick_session_service_;
  std::string return_home_service_;
  std::string return_named_target_{"home"};
  std::string selected_pregrasp_pose_topic_;
  std::string selected_grasp_pose_topic_;
  std::string external_pregrasp_pose_array_topic_;
  std::string external_grasp_pose_array_topic_;
  std::string external_grasp_proposal_array_topic_;
  std::string execution_debug_markers_topic_;
  std::string base_frame_;
  std::string ee_link_;
  std::string arm_group_name_;
  std::string gripper_group_name_;
  double planning_time_sec_{5.0};
  int planning_attempts_{10};
  double velocity_scaling_{1.0};
  double acceleration_scaling_{0.9};
  double gripper_velocity_scaling_{1.0};
  double gripper_acceleration_scaling_{1.0};
  bool enforce_upright_orientation_{true};
  bool lock_joint_after_target_lock_{true};
  std::string lock_joint_name_{"arm_joint1"};
  double lock_joint_max_delta_deg_{15.0};
  double lock_joint_relaxed_delta_deg_{25.0};
  bool lock_wrist_joint_after_target_lock_{true};
  std::string wrist_joint_name_{"arm_joint5"};
  double wrist_joint_max_delta_deg_{90.0};
  double wrist_joint_relaxed_delta_deg_{90.0};
  double orientation_constraint_tolerance_rad_{1.20};
  bool stage_position_only_target_{true};
  bool pregrasp_position_only_target_{true};
  double pregrasp_backoff_m_{0.10};
  double pregrasp_lift_m_{0.08};
  double grasp_backoff_m_{0.04};
  double grasp_lift_m_{0.03};
  double lift_distance_m_{0.05};
  bool debug_stop_after_pregrasp_{false};
  bool debug_execute_ranked_candidates_{false};
  int debug_ranked_candidate_limit_{12};
  std::string debug_return_named_target_{"home"};
  double grasp_height_offset_m_{0.0};
  double target_radius_m_{0.025};
  double target_height_m_{0.10};
  std::string camera_scoring_link_{"camera_depth_optical_frame"};
  double camera_forward_alignment_threshold_{0.60};
  double camera_upright_alignment_threshold_{0.0};
  double camera_forward_penalty_weight_{10.0};
  double camera_upright_penalty_weight_{14.0};
  double camera_trajectory_tilt_penalty_weight_{10.0};
  double camera_flip_penalty_weight_{20.0};
  double camera_stable_tilt_reward_deg_{5.0};
  double camera_stable_reward_bonus_{2.5};
  double pregrasp_score_base_{30.0};
  double pregrasp_score_floor_{1.0};
  double plan_duration_penalty_weight_{1.8};
  double path_distance_penalty_weight_{6.0};
  double approach_distance_penalty_weight_{4.0};
  double primary_joint_delta_penalty_weight_{2.6};
  double wrist_joint_delta_penalty_weight_{0.0};
  double primary_joint_excursion_penalty_weight_{8.5};
  std::string secondary_joint_name_{"arm_joint2"};
  double secondary_joint_excursion_penalty_weight_{0.0};
  double wrist_joint_excursion_penalty_weight_{0.0};
  double clearance_penalty_weight_{6.0};
  double grasp_base_roll_deg_{0.0};
  double grasp_base_pitch_deg_{90.0};
  double grasp_yaw_offset_deg_{0.0};
  bool prefer_external_grasp_candidates_{false};
  bool require_external_grasp_candidates_{false};
  double external_grasp_pose_timeout_sec_{3.0};
  double refresh_grasp_candidates_retry_sec_{1.5};
  double external_semantic_score_weight_{0.8};
  double external_confidence_score_weight_{0.6};
  bool include_target_collision_object_{false};
  bool require_user_confirmation_{false};
  std::vector<double> table_size_m_;
  std::vector<double> table_pose_xyz_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_locked_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr selected_pregrasp_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr selected_grasp_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr external_pregrasp_pose_array_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr external_grasp_pose_array_sub_;
  rclcpp::Subscription<tactile_interfaces::msg::GraspProposalArray>::SharedPtr
    external_grasp_proposal_array_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pick_active_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr refresh_grasp_candidates_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pick_status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr execution_debug_markers_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_pick_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_pick_session_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr return_home_srv_;
  rclcpp::TimerBase::SharedPtr check_timer_;

  std::mutex target_mutex_;
  std::mutex grasp_candidate_mutex_;
  geometry_msgs::msg::PoseStamped latest_target_pose_;
  geometry_msgs::msg::PoseStamped latched_target_pose_;
  geometry_msgs::msg::PoseStamped latest_selected_pregrasp_pose_;
  geometry_msgs::msg::PoseStamped latest_selected_grasp_pose_;
  geometry_msgs::msg::PoseArray latest_external_pregrasp_pose_array_;
  geometry_msgs::msg::PoseArray latest_external_grasp_pose_array_;
  tactile_interfaces::msg::GraspProposalArray latest_external_grasp_proposal_array_;
  bool has_target_pose_{false};
  bool has_latched_target_pose_{false};
  bool has_selected_pregrasp_pose_{false};
  bool has_selected_grasp_pose_{false};
  bool has_external_pregrasp_pose_array_{false};
  bool has_external_grasp_pose_array_{false};
  bool has_external_grasp_proposal_array_{false};
  bool target_locked_{false};
  bool moveit_ready_{false};
  bool locked_joint_reference_valid_{false};
  double locked_joint_reference_rad_{0.0};
  bool locked_wrist_joint_reference_valid_{false};
  double locked_wrist_joint_reference_rad_{0.0};
  std::atomic_bool pick_armed_{false};
  std::atomic_bool executing_{false};
  std::atomic_bool completed_{false};
  bool suppress_target_relock_{false};
  double latest_external_grasp_update_sec_{0.0};
  double last_grasp_refresh_request_sec_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = SimPickTaskNode::create();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
