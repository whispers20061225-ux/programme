#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <Eigen/Geometry>
#include <future>
#include <functional>
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
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tactile_interfaces/msg/grasp_proposal_array.hpp>
#include <tactile_interfaces/srv/move_arm_joints.hpp>
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

  struct StageVariant
  {
    std::array<double, 3> stage;
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

  enum class ReducedOrientationFreeAxis
  {
    X,
    Y,
    Z,
  };

  struct ReducedOrientationConstraint
  {
    bool enabled{false};
    ReducedOrientationFreeAxis free_axis{ReducedOrientationFreeAxis::Z};
    double free_axis_tolerance_rad{M_PI};
    double constrained_y_tolerance_rad{M_PI};
    double constrained_z_tolerance_rad{M_PI};
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
    ReducedOrientationConstraint reduced_orientation_constraint;
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
    int quick_ik_hits{0};
    int quick_ik_samples{0};
    std::optional<ExternalProposalMetadata> external_metadata;
  };

  struct PlanningWorker
  {
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group;
  };

  struct ExternalCandidateJob
  {
    ApproachCandidate candidate;
    std::string candidate_label;
    std::size_t candidate_index{0};
    std::optional<ExternalProposalMetadata> external_metadata;
    double first_round_score{0.0};
    double second_round_score{0.0};
    int second_round_hits{0};
    int second_round_samples{0};
    bool second_round_has_hit{false};
    std::vector<PregraspVariant> second_round_variants;
  };

  struct ExternalQuickIkRegionSample
  {
    PregraspVariant variant;
    std::size_t seed_index{0};
    int cluster_id{0};
    double intrinsic_quality{0.0};
  };

  struct ExternalSecondRoundScore
  {
    double total_score{0.0};
    double region_score{0.0};
    double deviation_score{0.0};
    double joint_score{0.0};
    double pregrasp_score{0.0};
    double best_sample_quality{0.0};
    int hits{0};
    int samples{0};
    bool has_hit{false};
    std::vector<int> hits_per_cluster;
    std::vector<std::size_t> top_hit_sample_indices;
  };

  struct ExternalCandidateEvaluationResult
  {
    std::optional<PlannedApproachOption> best_option;
    ExternalEvaluationDebugInfo debug_info;
    std::size_t tested_options{0};
    std::size_t feasible_options{0};
    std::size_t stage_plan_failures{0};
    std::size_t stage_end_state_failures{0};
    std::size_t pregrasp_plan_failures{0};
    std::size_t pregrasp_end_state_failures{0};
  };

  struct InternalCandidateJob
  {
    ApproachCandidate candidate;
    std::size_t candidate_index{0};
  };

  struct InternalCandidateEvaluationResult
  {
    std::optional<PlannedApproachOption> best_option;
    std::size_t tested_options{0};
    bool quick_ik_prefilter_rejected{false};
    int quick_ik_hits{0};
    int quick_ik_samples{0};
  };

  struct QuickIkPrefilterResult
  {
    bool passed{true};
    int hits{0};
    int samples{0};
  };

  SimPickTaskNode()
  : Node("sim_pick_task_node")
  {
    this->declare_parameter<std::string>("target_pose_topic", "/sim/perception/target_pose");
    this->declare_parameter<std::string>("target_locked_topic", "/sim/perception/target_locked");
    this->declare_parameter<std::string>(
      "target_candidate_visible_topic", "/sim/perception/target_candidate_visible");
    this->declare_parameter<std::string>("refresh_grasp_candidates_topic", "/grasp/refresh_request");
    this->declare_parameter<std::string>("pick_active_topic", "/sim/task/pick_active");
    this->declare_parameter<std::string>("pick_status_topic", "/sim/task/pick_status");
    this->declare_parameter<std::string>("execute_pick_service", "/task/execute_pick");
    this->declare_parameter<std::string>("reset_pick_session_service", "/task/reset_pick_session");
    this->declare_parameter<std::string>("return_home_service", "/task/return_home");
    this->declare_parameter<std::string>("return_named_target", "home");
    this->declare_parameter<std::string>("home_gripper_named_target", "open");
    this->declare_parameter<bool>("apply_home_gripper_named_target_on_startup", true);
    this->declare_parameter<bool>("return_home_allow_named_target_fallback", false);
    this->declare_parameter<double>("manual_home_hold_sec", 30.0);
    this->declare_parameter<double>("failure_rearm_hold_sec", 4.0);
    this->declare_parameter<std::vector<int64_t>>(
      "return_home_joint_ids", {1, 2, 3, 4, 5, 6});
    this->declare_parameter<std::vector<double>>(
        "return_home_joint_angles_deg", {3.0, -58.0, 89.405064, 96.0, 0.0, -68.754936});
    this->declare_parameter<int>("return_home_duration_ms", 3500);
    this->declare_parameter<bool>("return_home_wait_for_completion", true);
    this->declare_parameter<int>("return_home_request_timeout_ms", 12000);
    this->declare_parameter<std::string>(
      "control_arm_move_joints_service",
      "/control/arm/move_joints");
    this->declare_parameter<int>("control_gripper_joint_id", 6);
    this->declare_parameter<int>("control_gripper_duration_ms", 1500);
    this->declare_parameter<bool>("control_gripper_wait", true);
    this->declare_parameter<std::string>("selected_pregrasp_pose_topic", "/grasp/selected_pregrasp_pose");
    this->declare_parameter<std::string>("selected_grasp_pose_topic", "/grasp/selected_grasp_pose");
    this->declare_parameter<std::string>("external_pregrasp_pose_array_topic", "/grasp/candidate_pregrasp_poses");
    this->declare_parameter<std::string>("external_grasp_pose_array_topic", "/grasp/candidate_grasp_poses");
    this->declare_parameter<std::string>("external_grasp_proposal_array_topic", "/grasp/candidate_grasp_proposals");
    this->declare_parameter<std::string>("execution_debug_markers_topic", "/grasp/execution_debug_markers");
    this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
    this->declare_parameter<std::string>("base_frame", "world");
    this->declare_parameter<std::string>("ee_link", "ee_link");
    this->declare_parameter<std::string>("arm_group", "arm");
    this->declare_parameter<std::string>("gripper_group", "gripper");
    this->declare_parameter<double>("planning_time_sec", 5.0);
    this->declare_parameter<int>("planning_attempts", 10);
    this->declare_parameter<double>("candidate_screening_planning_time_sec", 0.3);
    this->declare_parameter<int>("candidate_screening_planning_attempts", 1);
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
    this->declare_parameter<bool>("planning_seed_bias_enabled", true);
    this->declare_parameter<std::string>("planning_seed_joint3_name", "arm_joint3");
    this->declare_parameter<std::string>("planning_seed_joint4_name", "arm_joint4");
    this->declare_parameter<double>("planning_seed_joint3_bias_rad", -0.15);
    this->declare_parameter<double>("planning_seed_joint4_bias_rad", 0.0);
    this->declare_parameter<bool>("stage_position_only_target", true);
    this->declare_parameter<bool>("pregrasp_position_only_target", true);
    this->declare_parameter<bool>("external_orientation_sampling_enabled", true);
    this->declare_parameter<int>("external_orientation_sample_count", 11);
    this->declare_parameter<double>("external_orientation_sample_max_deg", 30.0);
    this->declare_parameter<bool>("external_reduced_orientation_constraint_enabled", true);
    this->declare_parameter<bool>("external_stage_enabled", true);
    this->declare_parameter<double>("external_reduced_free_axis_tolerance_deg", 180.0);
    this->declare_parameter<double>("external_reduced_constrained_axis_tolerance_deg", 20.0);
    this->declare_parameter<int>("external_screening_candidate_limit", 6);
    this->declare_parameter<bool>("external_first_round_rerank_enabled", true);
    this->declare_parameter<double>("external_first_round_network_weight", 0.20);
    this->declare_parameter<double>("external_first_round_table_weight", 0.20);
    this->declare_parameter<double>("external_first_round_contact_weight", 0.20);
    this->declare_parameter<double>("external_first_round_com_weight", 0.20);
    this->declare_parameter<double>("external_first_round_width_weight", 0.10);
    this->declare_parameter<double>("external_first_round_surface_weight", 0.10);
    this->declare_parameter<bool>("external_second_round_rerank_enabled", true);
    this->declare_parameter<int>("external_second_round_candidate_limit", 3);
    this->declare_parameter<bool>("external_second_round_keep_geometry_fallback", true);
    this->declare_parameter<int>("external_second_round_geometry_fallback_count", 1);
    this->declare_parameter<double>("external_second_round_standoff_delta_m", 0.020);
    this->declare_parameter<double>("external_second_round_lateral_delta_m", 0.018);
    this->declare_parameter<double>("external_second_round_vertical_delta_m", 0.010);
    this->declare_parameter<double>("external_second_round_roll_slack_deg", 10.0);
    this->declare_parameter<double>("external_second_round_pitch_slack_deg", 8.0);
    this->declare_parameter<bool>("external_second_round_refine_enabled", true);
    this->declare_parameter<int>("external_second_round_refine_top_seed_count", 1);
    this->declare_parameter<double>("external_second_round_refine_scale", 0.5);
    this->declare_parameter<int>("external_second_round_planning_variant_limit", 4);
    this->declare_parameter<double>("external_second_round_region_weight", 0.45);
    this->declare_parameter<double>("external_second_round_deviation_weight", 0.25);
    this->declare_parameter<double>("external_second_round_joint_weight", 0.15);
    this->declare_parameter<double>("external_second_round_pregrasp_weight", 0.15);
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
    this->declare_parameter<bool>("allow_internal_fallback_after_external_timeout", false);
    this->declare_parameter<double>("internal_fallback_after_external_timeout_sec", 3.5);
    this->declare_parameter<double>("soft_prep_target_pose_timeout_sec", 1.5);
    this->declare_parameter<double>("refresh_grasp_candidates_retry_sec", 1.5);
    this->declare_parameter<double>("external_semantic_score_weight", 0.8);
    this->declare_parameter<double>("external_confidence_score_weight", 0.6);
    this->declare_parameter<bool>("include_table_collision_object", true);
    this->declare_parameter<bool>("include_target_collision_object", false);
    this->declare_parameter<bool>("require_user_confirmation", false);
    this->declare_parameter<bool>("allow_soft_lock_pregrasp_prep", true);
    this->declare_parameter<double>("soft_lock_final_lock_timeout_sec", 1.0);
    this->declare_parameter<bool>("parallel_candidate_evaluation_enabled", true);
    this->declare_parameter<int>("parallel_candidate_worker_count", 3);
    this->declare_parameter<int>("parallel_candidate_min_jobs", 2);
    this->declare_parameter<bool>("quick_ik_prefilter_enabled", true);
    this->declare_parameter<bool>("quick_ik_prefilter_parallel_enabled", true);
    this->declare_parameter<double>("quick_ik_prefilter_timeout_sec", 0.015);
    this->declare_parameter<int>("quick_ik_prefilter_required_hits", 1);
    this->declare_parameter<std::vector<double>>("table_size_m", {1.0, 0.8, 0.04});
    this->declare_parameter<std::vector<double>>("table_pose_xyz", {0.5, 0.0, 0.38});

    target_pose_topic_ = this->get_parameter("target_pose_topic").as_string();
    target_locked_topic_ = this->get_parameter("target_locked_topic").as_string();
    target_candidate_visible_topic_ =
      this->get_parameter("target_candidate_visible_topic").as_string();
    refresh_grasp_candidates_topic_ =
      this->get_parameter("refresh_grasp_candidates_topic").as_string();
    pick_active_topic_ = this->get_parameter("pick_active_topic").as_string();
    pick_status_topic_ = this->get_parameter("pick_status_topic").as_string();
    execute_pick_service_ = this->get_parameter("execute_pick_service").as_string();
    reset_pick_session_service_ = this->get_parameter("reset_pick_session_service").as_string();
    return_home_service_ = this->get_parameter("return_home_service").as_string();
    return_named_target_ = this->get_parameter("return_named_target").as_string();
    home_gripper_named_target_ = this->get_parameter("home_gripper_named_target").as_string();
    apply_home_gripper_named_target_on_startup_ =
      this->get_parameter("apply_home_gripper_named_target_on_startup").as_bool();
    return_home_allow_named_target_fallback_ =
      this->get_parameter("return_home_allow_named_target_fallback").as_bool();
    manual_home_hold_sec_ = std::max(
      0.0, this->get_parameter("manual_home_hold_sec").as_double());
    failure_rearm_hold_sec_ = std::max(
      0.0, this->get_parameter("failure_rearm_hold_sec").as_double());
    return_home_joint_ids_ = this->get_parameter("return_home_joint_ids").as_integer_array();
    return_home_joint_angles_deg_ =
      this->get_parameter("return_home_joint_angles_deg").as_double_array();
    return_home_duration_ms_ = std::max(
      100, static_cast<int>(this->get_parameter("return_home_duration_ms").as_int()));
    return_home_wait_for_completion_ =
      this->get_parameter("return_home_wait_for_completion").as_bool();
    return_home_request_timeout_ms_ = std::max(
      3000, static_cast<int>(this->get_parameter("return_home_request_timeout_ms").as_int()));
    const int control_gripper_joint_id =
      static_cast<int>(this->get_parameter("control_gripper_joint_id").as_int());
    const int control_gripper_duration_ms =
      static_cast<int>(this->get_parameter("control_gripper_duration_ms").as_int());
    control_arm_move_joints_service_ =
      this->get_parameter("control_arm_move_joints_service").as_string();
    control_gripper_joint_id_ = std::max(1, control_gripper_joint_id);
    control_gripper_duration_ms_ = std::max(100, control_gripper_duration_ms);
    control_gripper_wait_ =
      this->get_parameter("control_gripper_wait").as_bool();
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
    joint_states_topic_ = this->get_parameter("joint_states_topic").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    ee_link_ = this->get_parameter("ee_link").as_string();
    arm_group_name_ = this->get_parameter("arm_group").as_string();
    gripper_group_name_ = this->get_parameter("gripper_group").as_string();
    planning_time_sec_ = this->get_parameter("planning_time_sec").as_double();
    planning_attempts_ = this->get_parameter("planning_attempts").as_int();
    candidate_screening_planning_time_sec_ =
      std::max(0.05, this->get_parameter("candidate_screening_planning_time_sec").as_double());
    candidate_screening_planning_attempts_ =
      std::max<int>(1, static_cast<int>(
        this->get_parameter("candidate_screening_planning_attempts").as_int()));
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
    planning_seed_bias_enabled_ =
      this->get_parameter("planning_seed_bias_enabled").as_bool();
    planning_seed_joint3_name_ = this->get_parameter("planning_seed_joint3_name").as_string();
    planning_seed_joint4_name_ = this->get_parameter("planning_seed_joint4_name").as_string();
    planning_seed_joint3_bias_rad_ =
      this->get_parameter("planning_seed_joint3_bias_rad").as_double();
    planning_seed_joint4_bias_rad_ =
      this->get_parameter("planning_seed_joint4_bias_rad").as_double();
    stage_position_only_target_ =
      this->get_parameter("stage_position_only_target").as_bool();
    pregrasp_position_only_target_ =
      this->get_parameter("pregrasp_position_only_target").as_bool();
    external_orientation_sampling_enabled_ =
      this->get_parameter("external_orientation_sampling_enabled").as_bool();
    external_orientation_sample_count_ =
      this->get_parameter("external_orientation_sample_count").as_int();
    external_orientation_sample_max_deg_ =
      this->get_parameter("external_orientation_sample_max_deg").as_double();
    external_reduced_orientation_constraint_enabled_ =
      this->get_parameter("external_reduced_orientation_constraint_enabled").as_bool();
    external_stage_enabled_ =
      this->get_parameter("external_stage_enabled").as_bool();
    external_reduced_free_axis_tolerance_deg_ =
      this->get_parameter("external_reduced_free_axis_tolerance_deg").as_double();
    external_reduced_constrained_axis_tolerance_deg_ =
      this->get_parameter("external_reduced_constrained_axis_tolerance_deg").as_double();
    external_screening_candidate_limit_ = std::max(
      0,
      static_cast<int>(this->get_parameter("external_screening_candidate_limit").as_int()));
    external_first_round_rerank_enabled_ =
      this->get_parameter("external_first_round_rerank_enabled").as_bool();
    external_first_round_network_weight_ =
      this->get_parameter("external_first_round_network_weight").as_double();
    external_first_round_table_weight_ =
      this->get_parameter("external_first_round_table_weight").as_double();
    external_first_round_contact_weight_ =
      this->get_parameter("external_first_round_contact_weight").as_double();
    external_first_round_com_weight_ =
      this->get_parameter("external_first_round_com_weight").as_double();
    external_first_round_width_weight_ =
      this->get_parameter("external_first_round_width_weight").as_double();
    external_first_round_surface_weight_ =
      this->get_parameter("external_first_round_surface_weight").as_double();
    external_second_round_rerank_enabled_ =
      this->get_parameter("external_second_round_rerank_enabled").as_bool();
    external_second_round_candidate_limit_ = std::max(
      0,
      static_cast<int>(this->get_parameter("external_second_round_candidate_limit").as_int()));
    external_second_round_keep_geometry_fallback_ =
      this->get_parameter("external_second_round_keep_geometry_fallback").as_bool();
    external_second_round_geometry_fallback_count_ = std::max(
      0,
      static_cast<int>(
        this->get_parameter("external_second_round_geometry_fallback_count").as_int()));
    external_second_round_standoff_delta_m_ = std::max(
      0.0, this->get_parameter("external_second_round_standoff_delta_m").as_double());
    external_second_round_lateral_delta_m_ = std::max(
      0.0, this->get_parameter("external_second_round_lateral_delta_m").as_double());
    external_second_round_vertical_delta_m_ = std::max(
      0.0, this->get_parameter("external_second_round_vertical_delta_m").as_double());
    external_second_round_roll_slack_deg_ = std::max(
      0.0, this->get_parameter("external_second_round_roll_slack_deg").as_double());
    external_second_round_pitch_slack_deg_ = std::max(
      0.0, this->get_parameter("external_second_round_pitch_slack_deg").as_double());
    external_second_round_refine_enabled_ =
      this->get_parameter("external_second_round_refine_enabled").as_bool();
    external_second_round_refine_top_seed_count_ = std::max(
      1,
      static_cast<int>(
        this->get_parameter("external_second_round_refine_top_seed_count").as_int()));
    external_second_round_refine_scale_ = std::clamp(
      this->get_parameter("external_second_round_refine_scale").as_double(),
      0.1,
      1.0);
    external_second_round_planning_variant_limit_ = std::max(
      1,
      static_cast<int>(
        this->get_parameter("external_second_round_planning_variant_limit").as_int()));
    external_second_round_region_weight_ =
      this->get_parameter("external_second_round_region_weight").as_double();
    external_second_round_deviation_weight_ =
      this->get_parameter("external_second_round_deviation_weight").as_double();
    external_second_round_joint_weight_ =
      this->get_parameter("external_second_round_joint_weight").as_double();
    external_second_round_pregrasp_weight_ =
      this->get_parameter("external_second_round_pregrasp_weight").as_double();
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
    allow_internal_fallback_after_external_timeout_ =
      this->get_parameter("allow_internal_fallback_after_external_timeout").as_bool();
    internal_fallback_after_external_timeout_sec_ = std::max(
      0.0,
      this->get_parameter("internal_fallback_after_external_timeout_sec").as_double());
    refresh_grasp_candidates_retry_sec_ =
      this->get_parameter("refresh_grasp_candidates_retry_sec").as_double();
    external_semantic_score_weight_ =
      this->get_parameter("external_semantic_score_weight").as_double();
    external_confidence_score_weight_ =
      this->get_parameter("external_confidence_score_weight").as_double();
    include_table_collision_object_ =
      this->get_parameter("include_table_collision_object").as_bool();
    include_target_collision_object_ =
      this->get_parameter("include_target_collision_object").as_bool();
    require_user_confirmation_ =
      this->get_parameter("require_user_confirmation").as_bool();
    allow_soft_lock_pregrasp_prep_ =
      this->get_parameter("allow_soft_lock_pregrasp_prep").as_bool();
    soft_lock_final_lock_timeout_sec_ = std::max(
      0.0, this->get_parameter("soft_lock_final_lock_timeout_sec").as_double());
    parallel_candidate_evaluation_enabled_ =
      this->get_parameter("parallel_candidate_evaluation_enabled").as_bool();
    parallel_candidate_worker_count_ = std::max(
      1,
      static_cast<int>(this->get_parameter("parallel_candidate_worker_count").as_int()));
    parallel_candidate_min_jobs_ = std::max(
      1,
      static_cast<int>(this->get_parameter("parallel_candidate_min_jobs").as_int()));
    quick_ik_prefilter_enabled_ =
      this->get_parameter("quick_ik_prefilter_enabled").as_bool();
    quick_ik_prefilter_parallel_enabled_ =
      this->get_parameter("quick_ik_prefilter_parallel_enabled").as_bool();
    quick_ik_prefilter_timeout_sec_ = std::max(
      0.001,
      this->get_parameter("quick_ik_prefilter_timeout_sec").as_double());
    quick_ik_prefilter_required_hits_ = std::max(
      1,
      static_cast<int>(this->get_parameter("quick_ik_prefilter_required_hits").as_int()));
    soft_prep_target_pose_timeout_sec_ = std::max(
      0.0, this->get_parameter("soft_prep_target_pose_timeout_sec").as_double());
    table_size_m_ = this->get_parameter("table_size_m").as_double_array();
    table_pose_xyz_ = this->get_parameter("table_pose_xyz").as_double_array();
    if (!control_arm_move_joints_service_.empty()) {
      service_client_callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      control_arm_move_joints_client_ =
        this->create_client<tactile_interfaces::srv::MoveArmJoints>(
        control_arm_move_joints_service_,
        rmw_qos_profile_services_default,
        service_client_callback_group_);
    }

    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      target_pose_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::scoped_lock<std::mutex> lock(target_mutex_);
        latest_target_pose_ = *msg;
        has_target_pose_ = true;
        latest_target_pose_update_sec_ = this->get_clock()->now().seconds();
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
    target_candidate_visible_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      target_candidate_visible_topic_, 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        target_candidate_visible_ = msg->data;
      });

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_,
      rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::scoped_lock<std::mutex> lock(joint_state_mutex_);
        latest_joint_state_ = *msg;
        has_latest_joint_state_ = true;
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
        const bool soft_prep_allowed =
          allow_soft_lock_pregrasp_prep_ && !target_locked_ && target_candidate_visible_;
        if (!target_locked_ && !soft_prep_allowed) {
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
          if (soft_prep_allowed && soft_prep_target_pose_timeout_sec_ > 0.0) {
            const double pose_age_sec =
              latest_target_pose_update_sec_ > 0.0 ?
              (this->get_clock()->now().seconds() - latest_target_pose_update_sec_) :
              std::numeric_limits<double>::infinity();
            if (pose_age_sec > soft_prep_target_pose_timeout_sec_) {
              std::ostringstream stale_message;
              stale_message << "target pose is stale (age="
                            << std::fixed << std::setprecision(2)
                            << pose_age_sec << "s)";
              response->success = false;
              response->message = stale_message.str();
              publish_pick_status("idle", response->message);
              return;
            }
          }
          latched_target_pose_ = latest_target_pose_;
          has_latched_target_pose_ = true;
          target_pose = latched_target_pose_;
        }

        failure_rearm_hold_until_sec_ = 0.0;
        manual_home_hold_until_sec_ = 0.0;
        clear_external_grasp_candidate_cache();
        armed_sequence_token_.store(begin_execution_sequence_token());
        pick_armed_ = true;
        completed_ = false;
        pick_request_started_sec_ = this->get_clock()->now().seconds();
        allow_internal_fallback_for_current_pick_ = false;
        allow_stale_external_candidates_for_current_pick_ = false;
        waiting_for_final_lock_ = soft_prep_allowed;
        waiting_for_final_lock_started_sec_ =
          soft_prep_allowed ? this->get_clock()->now().seconds() : 0.0;
        publish_pick_active();
        if (soft_prep_allowed) {
          publish_pick_status(
            "planning",
            "execute requested; preparing pregrasp while waiting for final target lock");
        } else {
          publish_pick_status(
            "planning",
            "execute requested; waiting for external candidates and planning");
        }
        request_grasp_candidate_refresh("execute requested", true);
        response->success = true;
        std::ostringstream message;
        message << "pick armed from latched target pose ("
                << std::fixed << std::setprecision(3)
                << target_pose.pose.position.x << ", "
                << target_pose.pose.position.y << ", "
                << target_pose.pose.position.z << ")";
        if (soft_prep_allowed) {
          message << "; waiting for final target lock";
        }
        response->message = message.str();
      });
    reset_pick_session_srv_ = this->create_service<std_srvs::srv::Trigger>(
      reset_pick_session_service_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        interrupt_active_sequence("pick session reset", true);
        clear_locked_joint_references();
        manual_home_hold_until_sec_ = 0.0;
        failure_rearm_hold_until_sec_ = 0.0;
        pick_armed_ = false;
        pick_request_started_sec_ = 0.0;
        allow_internal_fallback_for_current_pick_ = false;
        waiting_for_final_lock_ = false;
        waiting_for_final_lock_started_sec_ = 0.0;
        allow_stale_external_candidates_for_current_pick_ = false;
        executing_ = false;
        completed_ = false;
        clear_cached_target_pose();
        clear_external_grasp_candidate_cache();
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
        const double now_sec = this->get_clock()->now().seconds();
        manual_home_hold_until_sec_ = now_sec + manual_home_hold_sec_;
        failure_rearm_hold_until_sec_ = 0.0;
        if (last_return_home_request_sec_ > 0.0 && (now_sec - last_return_home_request_sec_) <= 5.0)
        {
          publish_pick_active();
          publish_pick_status("idle", "arm return home already in progress");
          response->success = true;
          response->message = "arm return home already in progress";
          return;
        }
        last_return_home_request_sec_ = now_sec;
        interrupt_active_sequence("manual return home", true);
        clear_locked_joint_references();
        pick_armed_ = false;
        pick_request_started_sec_ = 0.0;
        allow_internal_fallback_for_current_pick_ = false;
        waiting_for_final_lock_ = false;
        waiting_for_final_lock_started_sec_ = 0.0;
        allow_stale_external_candidates_for_current_pick_ = false;
        executing_ = false;
        completed_ = false;
        clear_cached_target_pose();
        clear_external_grasp_candidate_cache();
        publish_pick_active();
        std::string home_message;
        const bool ok = request_return_home_motion(
          "return arm to home pose",
          &home_message);
        publish_pick_status(
          ok ? "idle" : "error",
          ok ? "arm returned to home pose" : "failed to return arm to home pose");
        response->success = ok;
        response->message = !home_message.empty() ?
          home_message :
          (ok ? "arm returned to home pose" : "failed to return arm to home pose");
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
          if (waiting_for_final_lock_ && !target_locked_) {
            const bool soft_ready =
              allow_soft_lock_pregrasp_prep_ && target_candidate_visible_;
            const double now_sec = this->get_clock()->now().seconds();
            const bool soft_lock_wait_timeout =
              soft_lock_final_lock_timeout_sec_ > 0.0 &&
              waiting_for_final_lock_started_sec_ > 0.0 &&
              (now_sec - waiting_for_final_lock_started_sec_) >= soft_lock_final_lock_timeout_sec_;
            const bool fallback_timeout_reached =
              allow_internal_fallback_after_external_timeout_ &&
              internal_fallback_after_external_timeout_sec_ > 0.0 &&
              pick_request_started_sec_ > 0.0 &&
              (now_sec - pick_request_started_sec_) >=
              internal_fallback_after_external_timeout_sec_;
            if (require_external_grasp_candidates_) {
              double age_sec = -1.0;
              std::size_t candidate_count = 0;
              const bool has_fresh_candidates =
                has_fresh_external_grasp_candidates(&age_sec, &candidate_count);
              const bool has_any_candidates =
                has_any_external_grasp_candidates(&age_sec, &candidate_count);
              const bool soft_timeout_ready = soft_lock_wait_timeout && has_any_candidates;
              const bool internal_fallback_ready =
                fallback_timeout_reached && !has_fresh_candidates;
              if (!((soft_ready && has_fresh_candidates) || soft_timeout_ready || internal_fallback_ready)) {
                request_grasp_candidate_refresh(
                  "planning preparation waiting for final target lock");
                RCLCPP_INFO_THROTTLE(
                  this->get_logger(), *this->get_clock(), 3000,
                  "waiting for final target lock while preparing pregrasp candidates");
                return;
              }
              if (internal_fallback_ready) {
                allow_internal_fallback_for_current_pick_ = true;
                allow_stale_external_candidates_for_current_pick_ = false;
                RCLCPP_WARN_THROTTLE(
                  this->get_logger(), *this->get_clock(), 3000,
                  "final hard lock not reached and no fresh external grasp candidates are available within %.2fs (count=%zu age=%.2fs); falling back to internal planning from latched target pose",
                  internal_fallback_after_external_timeout_sec_,
                  candidate_count,
                  age_sec);
              } else if (soft_timeout_ready && !target_locked_) {
                allow_stale_external_candidates_for_current_pick_ = true;
                RCLCPP_WARN_THROTTLE(
                  this->get_logger(), *this->get_clock(), 3000,
                  "final hard lock not reached within %.2fs; continuing with latched target pose and cached grasp candidates",
                  soft_lock_final_lock_timeout_sec_);
              }
            } else if (!(soft_ready || soft_lock_wait_timeout)) {
              RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 3000,
                "waiting for final target lock before execution");
              return;
            }
            publish_pick_status(
              "planning",
              "soft lock preparation complete; executing from latched target pose");
          }
          if (require_external_grasp_candidates_ && !allow_internal_fallback_for_current_pick_) {
            double age_sec = -1.0;
            std::size_t candidate_count = 0;
            const bool has_fresh_candidates =
              has_fresh_external_grasp_candidates(&age_sec, &candidate_count);
            const bool has_any_candidates =
              has_any_external_grasp_candidates(&age_sec, &candidate_count);
            const bool fallback_timeout_reached =
              allow_internal_fallback_after_external_timeout_ &&
              internal_fallback_after_external_timeout_sec_ > 0.0 &&
              pick_request_started_sec_ > 0.0 &&
              (this->get_clock()->now().seconds() - pick_request_started_sec_) >=
              internal_fallback_after_external_timeout_sec_;
            if (!(has_fresh_candidates || (allow_stale_external_candidates_for_current_pick_ && has_any_candidates))) {
              if (fallback_timeout_reached) {
                allow_internal_fallback_for_current_pick_ = true;
                RCLCPP_WARN_THROTTLE(
                  this->get_logger(), *this->get_clock(), 3000,
                  "external grasp candidates timed out after %.2fs; falling back to internal planning",
                  internal_fallback_after_external_timeout_sec_);
              } else {
                request_grasp_candidate_refresh("planning waiting for fresh external grasp candidates");
                RCLCPP_INFO_THROTTLE(
                  this->get_logger(), *this->get_clock(), 3000,
                  "waiting for external grasp candidates: count=%zu age=%.2fs timeout=%.2fs",
                  candidate_count, age_sec, external_grasp_pose_timeout_sec_);
                return;
              }
            }
            if (!has_fresh_candidates && allow_stale_external_candidates_for_current_pick_) {
              RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 3000,
                "continuing with cached external grasp candidates: count=%zu age=%.2fs timeout=%.2fs",
                candidate_count, age_sec, external_grasp_pose_timeout_sec_);
            }
          }
          waiting_for_final_lock_ = false;
          waiting_for_final_lock_started_sec_ = 0.0;
          pick_armed_ = false;
          executing_ = true;
          publish_pick_active();
          const std::uint64_t sequence_token = armed_sequence_token_.load();
          auto self = shared_from_this();
          std::thread([self, sequence_token]() {
            std::static_pointer_cast<SimPickTaskNode>(self)->run_pick_sequence(sequence_token);
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
    configure_arm_planning_group(*arm_group_);

    gripper_group_->setPlanningTime(3.0);
    gripper_group_->setNumPlanningAttempts(5);
    gripper_group_->setMaxVelocityScalingFactor(gripper_velocity_scaling_);
    gripper_group_->setMaxAccelerationScalingFactor(gripper_acceleration_scaling_);
    gripper_group_->setGoalJointTolerance(0.02);
    gripper_group_->allowReplanning(true);

    moveit_ready_ = true;
    initialize_parallel_planning_workers();
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
    RCLCPP_INFO(
      this->get_logger(),
      "planning seed bias: enabled=%s %s=%.3frad %s=%.3frad",
      planning_seed_bias_enabled_ ? "true" : "false",
      planning_seed_joint3_name_.c_str(),
      planning_seed_joint3_bias_rad_,
      planning_seed_joint4_name_.c_str(),
      planning_seed_joint4_bias_rad_);
    RCLCPP_INFO(
      this->get_logger(),
      "parallel candidate evaluation: enabled=%s workers=%d min_jobs=%d",
      parallel_candidate_evaluation_enabled_ ? "true" : "false",
      parallel_candidate_worker_count_,
      parallel_candidate_min_jobs_);
    RCLCPP_INFO(
      this->get_logger(),
      "quick IK prefilter: enabled=%s parallel=%s timeout=%.3fs required_hits=%d",
      quick_ik_prefilter_enabled_ ? "true" : "false",
      quick_ik_prefilter_parallel_enabled_ ? "true" : "false",
      quick_ik_prefilter_timeout_sec_,
      quick_ik_prefilter_required_hits_);
    if (apply_home_gripper_named_target_on_startup_) {
      if (ensure_home_gripper_named_target("set home gripper target on startup")) {
        RCLCPP_INFO(
          this->get_logger(),
          "home gripper target applied on startup: %s",
          home_gripper_named_target_.c_str());
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "failed to apply home gripper target on startup: %s",
          home_gripper_named_target_.c_str());
      }
    }
  }

  void configure_arm_planning_group(moveit::planning_interface::MoveGroupInterface & group)
  {
    group.setPlanningTime(planning_time_sec_);
    group.setNumPlanningAttempts(planning_attempts_);
    group.setMaxVelocityScalingFactor(velocity_scaling_);
    group.setMaxAccelerationScalingFactor(acceleration_scaling_);
    group.setPoseReferenceFrame(base_frame_);
    group.setEndEffectorLink(ee_link_);
    group.setGoalPositionTolerance(0.02);
    group.setGoalJointTolerance(0.02);
    group.setGoalOrientationTolerance(orientation_constraint_tolerance_rad_);
    group.allowReplanning(true);
  }

  void initialize_parallel_planning_workers()
  {
    planning_workers_.clear();
    if (!parallel_candidate_evaluation_enabled_) {
      return;
    }

    for (int index = 0; index < parallel_candidate_worker_count_; ++index) {
      auto worker_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), arm_group_name_);
      configure_arm_planning_group(*worker_group);
      planning_workers_.push_back(PlanningWorker{worker_group});
    }
  }

  bool ensure_home_gripper_named_target(const std::string & context)
  {
    if (!gripper_group_) {
      return false;
    }
    if (home_gripper_named_target_.empty()) {
      return true;
    }
    return execute_gripper_named_target(home_gripper_named_target_, context);
  }

  bool resolve_gripper_named_target_position(
    const std::string & target_name,
    double * target_position_rad_out)
  {
    if (!gripper_group_ || target_position_rad_out == nullptr || target_name.empty()) {
      return false;
    }

    const auto robot_model = gripper_group_->getRobotModel();
    const auto * joint_model_group =
      robot_model ? robot_model->getJointModelGroup(gripper_group_name_) : nullptr;
    if (!robot_model || !joint_model_group) {
      return false;
    }

    moveit::core::RobotState target_state(robot_model);
    target_state.setToDefaultValues(joint_model_group, target_name);
    std::vector<double> target_values;
    target_state.copyJointGroupPositions(joint_model_group, target_values);
    if (target_values.empty()) {
      return false;
    }

    *target_position_rad_out = target_values.front();
    return true;
  }

  bool execute_gripper_named_target(const std::string & target_name, const std::string & label)
  {
    if (!gripper_group_ || target_name.empty()) {
      return false;
    }

    if (execute_named_target(*gripper_group_, target_name, label)) {
      return true;
    }

    if (!control_arm_move_joints_client_) {
      return false;
    }

    double target_position_rad = 0.0;
    if (!resolve_gripper_named_target_position(target_name, &target_position_rad)) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s: failed to resolve gripper named target '%s' for service fallback",
        label.c_str(),
        target_name.c_str());
      return false;
    }

    if (!control_arm_move_joints_client_->wait_for_service(250ms)) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s: gripper fallback service unavailable: %s",
        label.c_str(),
        control_arm_move_joints_service_.c_str());
      return false;
    }

    auto request = std::make_shared<tactile_interfaces::srv::MoveArmJoints::Request>();
    request->joint_ids = {
      static_cast<uint8_t>(std::max(1, std::min(control_gripper_joint_id_, 255)))
    };
    request->angles_deg = {
      static_cast<float>(target_position_rad * 180.0 / M_PI)
    };
    request->duration_ms = control_gripper_duration_ms_;
    request->wait = control_gripper_wait_;

    auto future = control_arm_move_joints_client_->async_send_request(request);
    const auto wait_status = future.wait_for(
      std::chrono::milliseconds(control_gripper_duration_ms_ + 2500));
    if (wait_status != std::future_status::ready) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s: gripper service fallback timed out after %dms",
        label.c_str(),
        control_gripper_duration_ms_ + 2500);
      return false;
    }

    const auto response = future.get();
    if (!response || !response->success) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s: gripper service fallback failed: %s",
        label.c_str(),
        response ? response->message.c_str() : "<null response>");
      return false;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "%s: MoveIt named target failed; service fallback commanded grip_joint to %.1fdeg",
      label.c_str(),
      request->angles_deg.front());
    return true;
  }

  std::uint64_t begin_execution_sequence_token()
  {
    {
      std::scoped_lock<std::mutex> lock(sequence_mutex_);
      interrupted_sequence_reason_.clear();
    }
    return sequence_token_.fetch_add(1) + 1;
  }

  void interrupt_active_sequence(const std::string & reason, bool stop_motion)
  {
    {
      std::scoped_lock<std::mutex> lock(sequence_mutex_);
      interrupted_sequence_reason_ = reason;
    }
    armed_sequence_token_.store(0);
    (void)sequence_token_.fetch_add(1);
    if (stop_motion) {
      stop_active_motion(reason);
    }
  }

  bool was_sequence_interrupted(std::uint64_t sequence_token, std::string * reason_out = nullptr)
  {
    if (sequence_token == 0 || sequence_token == sequence_token_.load()) {
      return false;
    }
    if (reason_out != nullptr) {
      std::scoped_lock<std::mutex> lock(sequence_mutex_);
      *reason_out = interrupted_sequence_reason_;
    }
    return true;
  }

  void stop_active_motion(const std::string & context)
  {
    try {
      if (arm_group_) {
        arm_group_->stop();
        arm_group_->clearPoseTargets();
      }
    } catch (const std::exception & exc) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s: arm stop failed: %s",
        context.c_str(),
        exc.what());
    }
    try {
      if (gripper_group_) {
        gripper_group_->stop();
      }
    } catch (const std::exception & exc) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s: gripper stop failed: %s",
        context.c_str(),
        exc.what());
    }
  }

  bool execute_joint_motion_request(
    const std::vector<int64_t> & joint_ids,
    const std::vector<double> & angles_deg,
    int duration_ms,
    bool wait,
    const std::string & label,
    std::string * detail_out = nullptr,
    int timeout_override_ms = -1)
  {
    if (joint_ids.empty() || joint_ids.size() != angles_deg.size()) {
      if (detail_out != nullptr) {
        *detail_out = "joint motion config is invalid";
      }
      return false;
    }
    if (!control_arm_move_joints_client_) {
      if (detail_out != nullptr) {
        *detail_out = "move joints service client is unavailable";
      }
      return false;
    }
    if (!control_arm_move_joints_client_->wait_for_service(500ms)) {
      if (detail_out != nullptr) {
        *detail_out = "move joints service is unavailable";
      }
      RCLCPP_WARN(
        this->get_logger(),
        "%s: move joints service unavailable: %s",
        label.c_str(),
        control_arm_move_joints_service_.c_str());
      return false;
    }

    auto request = std::make_shared<tactile_interfaces::srv::MoveArmJoints::Request>();
    request->joint_ids.reserve(joint_ids.size());
    request->angles_deg.reserve(angles_deg.size());
    for (std::size_t index = 0; index < joint_ids.size(); ++index) {
      request->joint_ids.push_back(
        static_cast<uint8_t>(std::max<int64_t>(1, std::min<int64_t>(joint_ids[index], 255))));
      request->angles_deg.push_back(static_cast<float>(angles_deg[index]));
    }
    request->duration_ms = std::max(100, duration_ms);
    request->wait = wait;

    auto future = control_arm_move_joints_client_->async_send_request(request);
    const int timeout_ms = timeout_override_ms > 0 ?
      timeout_override_ms :
      std::max(request->duration_ms + 2500, 3000);
    const auto wait_status = future.wait_for(std::chrono::milliseconds(timeout_ms));
    if (wait_status != std::future_status::ready) {
      if (detail_out != nullptr) {
        *detail_out = "move joints service timed out";
      }
      RCLCPP_WARN(
        this->get_logger(),
        "%s: move joints service timed out after %dms",
        label.c_str(),
        timeout_ms);
      return false;
    }

    const auto response = future.get();
    if (!response || !response->success) {
      if (detail_out != nullptr) {
        *detail_out = response ? response->message : "move joints service failed";
      }
      RCLCPP_WARN(
        this->get_logger(),
        "%s: move joints service failed: %s",
        label.c_str(),
        response ? response->message.c_str() : "<null response>");
      return false;
    }

    if (detail_out != nullptr) {
      *detail_out = std::string(response->message);
    }
    return true;
  }

  bool request_return_home_motion(const std::string & label, std::string * detail_out = nullptr)
  {
    const bool has_explicit_home =
      !return_home_joint_ids_.empty() &&
      return_home_joint_ids_.size() == return_home_joint_angles_deg_.size();
    if (has_explicit_home) {
      std::string explicit_message;
      if (
        execute_joint_motion_request(
          return_home_joint_ids_,
          return_home_joint_angles_deg_,
          return_home_duration_ms_,
          return_home_wait_for_completion_,
          label + " via explicit joints",
          &explicit_message,
          return_home_request_timeout_ms_))
      {
        if (detail_out != nullptr) {
          *detail_out = explicit_message.empty() ?
            "arm returned to configured home joints" :
            explicit_message;
        }
        return true;
      }
      if (!return_home_allow_named_target_fallback_) {
        if (detail_out != nullptr) {
          *detail_out = explicit_message.empty() ?
            "configured home joint motion failed" :
            explicit_message;
        }
        RCLCPP_WARN(
          this->get_logger(),
          "%s: explicit home joint motion failed; named target fallback disabled: %s",
          label.c_str(),
          explicit_message.c_str());
        return false;
      }
      RCLCPP_WARN(
        this->get_logger(),
        "%s: explicit home joint motion failed, falling back to named target: %s",
        label.c_str(),
        explicit_message.c_str());
    }

    (void)ensure_home_gripper_named_target("set home gripper target before named home return");
    const bool ok = execute_named_target(*arm_group_, return_named_target_, label + " via named target");
    if (detail_out != nullptr) {
      *detail_out = ok ?
        "arm returned to named home pose" :
        "failed to return arm to named home pose";
    }
    return ok;
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

  void run_pick_sequence(std::uint64_t sequence_token)
  {
    geometry_msgs::msg::PoseStamped target_pose;
    if (!consume_latched_target_pose(target_pose)) {
      return fail_sequence("missing latched target pose", sequence_token);
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
      run_ranked_pregrasp_debug_sequence(current_pose, target_point, sequence_token);
      return;
    }

    const auto selected_option = select_pregrasp_option(current_pose, target_point);
    if (!selected_option.has_value()) {
      return fail_sequence("failed to find a feasible pregrasp candidate", sequence_token);
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
      "executing selected candidate %zu with latched proposal execution",
      pregrasp_option.candidate_index);
    if (pregrasp_option.skip_stage) {
      publish_pick_status(
        "executing",
        "executing direct pregrasp, grasp, retreat, and lift");
      if (!ensure_home_gripper_named_target("open gripper before direct pregrasp")) {
        return fail_sequence("failed to open gripper before direct pregrasp", sequence_token);
      }
      if (!execute_position_target_with_mode(
            pregrasp_option.pregrasp,
            pregrasp_option.pregrasp_orientation,
            pregrasp_option.pregrasp_label,
            true,
            make_no_reduced_orientation_constraint()))
      {
        return fail_sequence("failed to execute direct pregrasp plan", sequence_token);
      }
    } else {
      publish_pick_status(
        "executing",
        "executing selected stage, pregrasp, grasp, retreat, and lift");

      if (!execute_stage_position_target_with_gripper_open(
            pregrasp_option.stage, pregrasp_option.stage_orientation, pregrasp_option.stage_label))
      {
        return fail_sequence("failed to execute selected stage plan", sequence_token);
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
        return fail_sequence("failed to execute selected pregrasp plan", sequence_token);
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

    if (!execute_cartesian_segment_with_mode(
          pregrasp_option.grasp,
          pregrasp_option.grasp_orientation,
          "move to grasp",
          !(pregrasp_option.skip_stage && pregrasp_option.reduced_orientation_constraint.enabled),
          pregrasp_option.reduced_orientation_constraint))
    {
      return fail_sequence("failed to reach grasp", sequence_token);
    }

    if (!execute_gripper_named_target("closed", "close gripper")) {
      return fail_sequence("failed to close gripper", sequence_token);
    }

    if (pregrasp_option.skip_stage && pregrasp_option.reduced_orientation_constraint.enabled) {
      if (!execute_position_target_with_mode(
            retreat_point,
            pregrasp_option.pregrasp_orientation,
            "retreat to pregrasp with object",
            false,
            pregrasp_option.reduced_orientation_constraint))
      {
        return fail_sequence("failed to retreat after closing gripper", sequence_token);
      }

      if (!execute_position_target_with_mode(
            lift_point,
            pregrasp_option.pregrasp_orientation,
            "lift object",
            false,
            pregrasp_option.reduced_orientation_constraint))
      {
        return fail_sequence("failed to lift after retreat", sequence_token);
      }
    } else {
      if (!execute_position_target(
            retreat_point, pregrasp_option.pregrasp_orientation, "retreat to pregrasp with object"))
      {
        return fail_sequence("failed to retreat after closing gripper", sequence_token);
      }

      if (!execute_position_target(
            lift_point, pregrasp_option.pregrasp_orientation, "lift object"))
      {
        return fail_sequence("failed to lift after retreat", sequence_token);
      }
    }

    std::string interrupted_reason;
    if (was_sequence_interrupted(sequence_token, &interrupted_reason)) {
      return fail_sequence(
        interrupted_reason.empty() ? "pick sequence interrupted" : interrupted_reason,
        sequence_token);
    }

    clear_locked_joint_references();
    pick_request_started_sec_ = 0.0;
    allow_internal_fallback_for_current_pick_ = false;
    completed_ = true;
    executing_ = false;
    publish_pick_active();
    publish_pick_status("completed", "pick sequence completed");
    RCLCPP_INFO(this->get_logger(), "pick sequence completed");
  }

  void run_ranked_pregrasp_debug_sequence(
    const geometry_msgs::msg::Pose & current_pose,
    const std::array<double, 3> & target_point,
    std::uint64_t sequence_token)
  {
    const auto evaluation = evaluate_internal_pregrasp_options(current_pose, target_point);
    if (evaluation.feasible_options.empty()) {
      return fail_sequence(
        "ranked pregrasp debug found no feasible internal candidates",
        sequence_token);
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
        return fail_sequence(
          "failed to return to debug home before ranked candidate execution",
          sequence_token);
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
      return fail_sequence(
        "failed to return to debug home after ranked candidate sweep",
        sequence_token);
    }

    clear_locked_joint_references();
    pick_request_started_sec_ = 0.0;
    allow_internal_fallback_for_current_pick_ = false;
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

  bool execute_position_target_with_mode(
    const std::array<double, 3> & target_point,
    const geometry_msgs::msg::Quaternion & target_orientation,
    const std::string & label,
    bool position_only_target,
    const ReducedOrientationConstraint & reduced_orientation_constraint)
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
            position_only_target,
            reduced_orientation_constraint))
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
      position_only_target,
      reduced_orientation_constraint);
  }

  bool execute_position_target(
    const std::array<double, 3> & target_point,
    const geometry_msgs::msg::Quaternion & target_orientation,
    const std::string & label)
  {
    return execute_position_target_with_mode(
      target_point,
      target_orientation,
      label,
      pregrasp_position_only_target_,
      make_no_reduced_orientation_constraint());
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
    return execute_cartesian_segment_with_mode(
      target_point,
      target_orientation,
      label,
      true,
      make_no_reduced_orientation_constraint());
  }

  bool execute_cartesian_segment_with_mode(
    const std::array<double, 3> & target_point,
    const geometry_msgs::msg::Quaternion & target_orientation,
    const std::string & label,
    bool enforce_orientation,
    const ReducedOrientationConstraint & reduced_orientation_constraint)
  {
    set_bounded_start_state(*arm_group_);

    geometry_msgs::msg::Pose waypoint = arm_group_->getCurrentPose(ee_link_).pose;
    waypoint.position.x = target_point[0];
    waypoint.position.y = target_point[1];
    waypoint.position.z = target_point[2];
    if (enforce_upright_orientation_ && enforce_orientation) {
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
      if (reduced_orientation_constraint.enabled) {
        return execute_position_target_with_mode(
          target_point,
          target_orientation,
          label + " fallback",
          false,
          reduced_orientation_constraint);
      }
      return execute_position_target(target_point, target_orientation, label + " fallback");
    }

    const bool executed = static_cast<bool>(arm_group_->execute(trajectory));
    if (!executed) {
      RCLCPP_DEBUG(this->get_logger(), "%s: cartesian execution failed", label.c_str());
      if (reduced_orientation_constraint.enabled) {
        return execute_position_target_with_mode(
          target_point,
          target_orientation,
          label + " fallback",
          false,
          reduced_orientation_constraint);
      }
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

  geometry_msgs::msg::Quaternion rotate_orientation_about_local_x(
    const geometry_msgs::msg::Quaternion & orientation,
    double roll_deg) const
  {
    const auto normalized = normalize_quaternion(orientation);
    Eigen::Quaterniond base(
      normalized.w, normalized.x, normalized.y, normalized.z);
    const Eigen::Quaterniond roll_rotation(
      Eigen::AngleAxisd(roll_deg * M_PI / 180.0, Eigen::Vector3d::UnitX()));
    Eigen::Quaterniond rotated = base * roll_rotation;
    rotated.normalize();

    geometry_msgs::msg::Quaternion result;
    result.x = rotated.x();
    result.y = rotated.y();
    result.z = rotated.z();
    result.w = rotated.w();
    return result;
  }

  geometry_msgs::msg::Quaternion rotate_orientation_about_local_y(
    const geometry_msgs::msg::Quaternion & orientation,
    double pitch_deg) const
  {
    const auto normalized = normalize_quaternion(orientation);
    Eigen::Quaterniond base(
      normalized.w, normalized.x, normalized.y, normalized.z);
    const Eigen::Quaterniond pitch_rotation(
      Eigen::AngleAxisd(pitch_deg * M_PI / 180.0, Eigen::Vector3d::UnitY()));
    Eigen::Quaterniond rotated = base * pitch_rotation;
    rotated.normalize();

    geometry_msgs::msg::Quaternion result;
    result.x = rotated.x();
    result.y = rotated.y();
    result.z = rotated.z();
    result.w = rotated.w();
    return result;
  }

  static Eigen::Vector3d array_to_eigen(const std::array<double, 3> & values)
  {
    return Eigen::Vector3d(values[0], values[1], values[2]);
  }

  static std::array<double, 3> eigen_to_array(const Eigen::Vector3d & values)
  {
    return {values.x(), values.y(), values.z()};
  }

  static double clamp01(double value)
  {
    return std::clamp(value, 0.0, 1.0);
  }

  static double safe_axis_alignment_score(
    const Eigen::Vector3d & actual,
    const Eigen::Vector3d & target,
    double max_angle_deg,
    bool absolute_alignment = false)
  {
    if (actual.norm() < 1e-6 || target.norm() < 1e-6 || max_angle_deg <= 1e-6) {
      return 0.0;
    }
    double dot = actual.normalized().dot(target.normalized());
    if (absolute_alignment) {
      dot = std::abs(dot);
    }
    dot = std::clamp(dot, -1.0, 1.0);
    const double angle_rad = std::acos(dot);
    const double max_angle_rad = max_angle_deg * M_PI / 180.0;
    return clamp01(1.0 - (angle_rad / max_angle_rad));
  }

  static Eigen::Vector3d project_onto_plane(
    const Eigen::Vector3d & vector,
    const Eigen::Vector3d & normal)
  {
    return vector - (vector.dot(normal) * normal);
  }

  static bool append_unique_direction(
    std::vector<std::array<double, 3>> & directions,
    const Eigen::Vector3d & candidate)
  {
    if (candidate.norm() < 1e-6) {
      return false;
    }
    const Eigen::Vector3d normalized = candidate.normalized();
    for (const auto & existing : directions) {
      const Eigen::Vector3d existing_vector = array_to_eigen(existing);
      if (existing_vector.norm() > 1e-6 && normalized.dot(existing_vector.normalized()) > 0.985) {
        return false;
      }
    }
    directions.push_back(eigen_to_array(normalized));
    return true;
  }

  geometry_msgs::msg::Quaternion make_axis_aligned_orientation(
    const std::array<double, 3> & closing_direction,
    const std::array<double, 3> & approach_direction) const
  {
    Eigen::Vector3d x_axis = array_to_eigen(closing_direction);
    if (x_axis.norm() < 1e-6) {
      return normalize_quaternion(geometry_msgs::msg::Quaternion{});
    }
    x_axis.normalize();

    Eigen::Vector3d z_axis = project_onto_plane(array_to_eigen(approach_direction), x_axis);
    if (z_axis.norm() < 1e-6) {
      z_axis = project_onto_plane(Eigen::Vector3d::UnitZ(), x_axis);
    }
    if (z_axis.norm() < 1e-6) {
      z_axis = project_onto_plane(Eigen::Vector3d::UnitY(), x_axis);
    }
    if (z_axis.norm() < 1e-6) {
      z_axis = project_onto_plane(Eigen::Vector3d::UnitX(), x_axis);
    }
    if (z_axis.norm() < 1e-6) {
      return normalize_quaternion(geometry_msgs::msg::Quaternion{});
    }
    z_axis.normalize();

    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    if (y_axis.norm() < 1e-6) {
      return normalize_quaternion(geometry_msgs::msg::Quaternion{});
    }
    y_axis.normalize();
    z_axis = x_axis.cross(y_axis).normalized();

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix.col(0) = x_axis;
    rotation_matrix.col(1) = y_axis;
    rotation_matrix.col(2) = z_axis;
    const Eigen::Quaterniond orientation(rotation_matrix);

    geometry_msgs::msg::Quaternion result;
    result.x = orientation.x();
    result.y = orientation.y();
    result.z = orientation.z();
    result.w = orientation.w();
    return normalize_quaternion(result);
  }

  std::vector<std::array<double, 3>> build_external_approach_direction_candidates(
    const geometry_msgs::msg::Pose & current_pose,
    const ExternalProposalMetadata & metadata) const
  {
    std::vector<std::array<double, 3>> directions;
    Eigen::Vector3d closing_axis = array_to_eigen(metadata.closing_direction);
    if (closing_axis.norm() < 1e-6) {
      return directions;
    }
    closing_axis.normalize();

    const Eigen::Vector3d grasp_center = array_to_eigen(metadata.grasp_center);
    const Eigen::Vector3d current_position(
      current_pose.position.x,
      current_pose.position.y,
      current_pose.position.z);
    const Eigen::Vector3d current_to_grasp =
      project_onto_plane(grasp_center - current_position, closing_axis);
    const Eigen::Vector3d horizontal_current_to_grasp =
      project_onto_plane(
      Eigen::Vector3d(current_to_grasp.x(), current_to_grasp.y(), 0.0),
      closing_axis);
    const Eigen::Vector3d world_up = Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d table_lateral =
      project_onto_plane(world_up.cross(closing_axis), closing_axis);
    const Eigen::Vector3d backend_approach =
      project_onto_plane(array_to_eigen(metadata.approach_direction), closing_axis);

    append_unique_direction(directions, horizontal_current_to_grasp);
    append_unique_direction(directions, -horizontal_current_to_grasp);
    append_unique_direction(directions, table_lateral);
    append_unique_direction(directions, -table_lateral);
    append_unique_direction(directions, current_to_grasp);
    append_unique_direction(directions, -current_to_grasp);
    append_unique_direction(directions, backend_approach);
    append_unique_direction(directions, -backend_approach);

    if (directions.empty()) {
      append_unique_direction(
        directions,
        project_onto_plane(Eigen::Vector3d::UnitX(), closing_axis));
      append_unique_direction(
        directions,
        project_onto_plane(Eigen::Vector3d::UnitY(), closing_axis));
    }
    return directions;
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

    if (include_table_collision_object_) {
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
      table.primitives.push_back(table_primitive);
      table.primitive_poses.push_back(table_pose);
      objects.push_back(table);
    } else {
      planning_scene_interface_->removeCollisionObjects({"sim_table"});
    }

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

  bool has_any_external_grasp_candidates(
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
    return candidate_count > 0U;
  }

  void remove_target_from_scene()
  {
    planning_scene_interface_->removeCollisionObjects({"sim_target"});
    RCLCPP_DEBUG(this->get_logger(), "removed target collision object to allow contact");
  }

  void set_bounded_start_state(
    moveit::planning_interface::MoveGroupInterface & group,
    bool apply_seed_bias = false)
  {
    auto current_state = group.getCurrentState(1.0);
    if (!current_state) {
      current_state = build_fallback_current_state(group);
      if (!current_state) {
        group.setStartStateToCurrentState();
        return;
      }
    }

    const auto * joint_model_group =
      current_state->getJointModelGroup(group.getName());
    if (joint_model_group == nullptr) {
      group.setStartStateToCurrentState();
      return;
    }

    if (apply_seed_bias) {
      apply_planning_seed_bias(*current_state);
    } else {
      current_state->enforceBounds(joint_model_group);
    }
    group.setStartState(*current_state);
  }

  void apply_planning_seed_bias(moveit::core::RobotState & state) const
  {
    if (!planning_seed_bias_enabled_) {
      return;
    }

    const auto * joint_model_group = state.getJointModelGroup(arm_group_name_);
    if (joint_model_group == nullptr) {
      return;
    }

    const auto apply_joint_bias = [&](const std::string & joint_name, double bias_rad) {
      if (joint_name.empty() || std::abs(bias_rad) < 1e-6) {
        return;
      }
      const auto & variable_names = state.getVariableNames();
      if (std::find(variable_names.begin(), variable_names.end(), joint_name) == variable_names.end()) {
        return;
      }
      state.setVariablePosition(joint_name, state.getVariablePosition(joint_name) + bias_rad);
    };

    apply_joint_bias(planning_seed_joint3_name_, planning_seed_joint3_bias_rad_);
    apply_joint_bias(planning_seed_joint4_name_, planning_seed_joint4_bias_rad_);
    state.enforceBounds(joint_model_group);
    state.update();
  }

  void fail_sequence(const std::string & reason, std::uint64_t sequence_token = 0)
  {
    std::string interrupted_reason;
    const bool interrupted = was_sequence_interrupted(sequence_token, &interrupted_reason);
    const double now_sec = this->get_clock()->now().seconds();
    failure_rearm_hold_until_sec_ = now_sec + failure_rearm_hold_sec_;
    manual_home_hold_until_sec_ = 0.0;
    clear_locked_joint_references();
    pick_armed_ = false;
    pick_request_started_sec_ = 0.0;
    allow_internal_fallback_for_current_pick_ = false;
    waiting_for_final_lock_ = false;
    waiting_for_final_lock_started_sec_ = 0.0;
    allow_stale_external_candidates_for_current_pick_ = false;
    executing_ = false;
    completed_ = false;
    clear_cached_target_pose();
    clear_external_grasp_candidate_cache();
    publish_pick_active();
    const std::string final_reason =
      interrupted && !interrupted_reason.empty() ? interrupted_reason : reason;
    if (interrupted) {
      publish_pick_status("idle", final_reason.empty() ? "pick sequence interrupted" : final_reason);
      RCLCPP_WARN(
        this->get_logger(),
        "pick sequence interrupted: %s",
        final_reason.empty() ? "<unspecified>" : final_reason.c_str());
      return;
    }

    publish_pick_status("error", final_reason);
    RCLCPP_WARN(this->get_logger(), "pick sequence aborted: %s", final_reason.c_str());
    std::string home_message;
    const bool returned_home = request_return_home_motion(
      "return arm to home pose after failure",
      &home_message);
    if (returned_home) {
      RCLCPP_WARN(
        this->get_logger(),
        "arm returned to home pose after failure: %s",
        home_message.c_str());
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "failed to return arm to home pose after failure: %s",
        home_message.c_str());
    }
    if (target_locked_) {
      publish_pick_status(
        "waiting_execute",
        "target locked; waiting for explicit execute command");
    } else {
      publish_pick_status("idle", "pick sequence reset after failure");
    }
    publish_pick_active();
  }

  bool execute_position_target_attempt(
    const std::array<double, 3> & target_point,
    const geometry_msgs::msg::Quaternion & target_orientation,
    const std::string & label,
    const JointConstraintAttempt & joint_attempt,
    bool position_only_target,
    const ReducedOrientationConstraint & reduced_orientation_constraint)
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
          position_only_target,
          0.0,
          0,
          reduced_orientation_constraint))
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
          position_only_target,
          0.0,
          0,
          make_no_reduced_orientation_constraint()))
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
    double open_grip_position = current_grip_position;
    if (gripper_group_ && !home_gripper_named_target_.empty()) {
      const auto robot_model = gripper_group_->getRobotModel();
      const auto * joint_model_group =
        robot_model ? robot_model->getJointModelGroup(gripper_group_name_) : nullptr;
      if (robot_model && joint_model_group) {
        moveit::core::RobotState target_state(robot_model);
        target_state.setToDefaultValues(joint_model_group, home_gripper_named_target_);
        std::vector<double> open_values;
        target_state.copyJointGroupPositions(joint_model_group, open_values);
        if (!open_values.empty()) {
          open_grip_position = open_values.front();
        }
      }
    }

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

  std::vector<StageVariant> build_stage_variants(
    const geometry_msgs::msg::Pose & current_pose,
    const std::array<double, 3> & target_point) const
  {
    constexpr std::array<double, 4> kStageFractions{0.38, 0.46, 0.30, 0.54};
    constexpr std::array<double, 4> kStageLiftOffsets{0.010, 0.012, 0.008, 0.015};

    const Eigen::Vector3d current_position(
      current_pose.position.x,
      current_pose.position.y,
      current_pose.position.z);
    Eigen::Vector3d corridor_direction(
      target_point[0] - current_position.x(),
      target_point[1] - current_position.y(),
      target_point[2] - current_position.z());
    if (corridor_direction.norm() < 1e-6) {
      corridor_direction = Eigen::Vector3d(-0.12, 0.0, 0.0);
    }

    std::vector<StageVariant> variants;
    variants.reserve(kStageFractions.size());
    for (std::size_t index = 0; index < kStageFractions.size(); ++index) {
      const Eigen::Vector3d stage_position =
        current_position + corridor_direction * kStageFractions[index] +
        Eigen::Vector3d(0.0, 0.0, kStageLiftOffsets[index]);
      std::ostringstream suffix;
      suffix << " f";
      suffix << std::fixed << std::setprecision(2) << kStageFractions[index];
      variants.push_back(StageVariant{
        {
          stage_position.x(),
          stage_position.y(),
          stage_position.z(),
        },
        suffix.str(),
      });
    }

    return variants;
  }

  std::vector<PregraspVariant> build_external_pregrasp_variants(
    const ApproachCandidate & candidate) const
  {
    if (external_reduced_orientation_constraint_enabled_) {
      return {
        {
          candidate.pregrasp,
          candidate.grasp,
          candidate.pregrasp_orientation,
          candidate.grasp_orientation,
          "",
        }
      };
    }

    if (!external_orientation_sampling_enabled_) {
      return {
        {
          candidate.pregrasp,
          candidate.grasp,
          candidate.pregrasp_orientation,
          candidate.grasp_orientation,
          "",
        }
      };
    }

    const int sample_count = std::max(1, external_orientation_sample_count_);
    const double max_deg = std::max(0.0, external_orientation_sample_max_deg_);
    if (sample_count == 1 || max_deg <= 1e-6) {
      return {
        {
          candidate.pregrasp,
          candidate.grasp,
          candidate.pregrasp_orientation,
          candidate.grasp_orientation,
          "",
        }
      };
    }

    std::vector<PregraspVariant> variants;
    variants.reserve(static_cast<std::size_t>(sample_count));
    const double denominator = static_cast<double>(sample_count - 1);
    for (int sample_index = 0; sample_index < sample_count; ++sample_index) {
      const double ratio = static_cast<double>(sample_index) / denominator;
      const double roll_deg = -max_deg + (2.0 * max_deg * ratio);
      std::ostringstream suffix;
      suffix << " roll";
      if (roll_deg >= 0.0) {
        suffix << "+";
      }
      suffix << std::fixed << std::setprecision(0) << roll_deg;
      variants.push_back(
        PregraspVariant{
          candidate.pregrasp,
          candidate.grasp,
          rotate_orientation_about_local_x(candidate.pregrasp_orientation, roll_deg),
          rotate_orientation_about_local_x(candidate.grasp_orientation, roll_deg),
          std::abs(roll_deg) < 1e-6 ? "" : suffix.str(),
        });
    }
    return variants;
  }

  std::vector<PregraspVariant> build_external_pregrasp_variants(
    const geometry_msgs::msg::Pose & current_pose,
    const ApproachCandidate & candidate,
    const std::optional<ExternalProposalMetadata> & external_metadata) const
  {
    if (!external_metadata.has_value()) {
      return build_external_pregrasp_variants(candidate);
    }

    const auto approach_directions =
      build_external_approach_direction_candidates(current_pose, external_metadata.value());
    const double nominal_pregrasp_offset = std::max(
      0.01,
      std::sqrt(
        std::pow(candidate.pregrasp[0] - candidate.grasp[0], 2) +
        std::pow(candidate.pregrasp[1] - candidate.grasp[1], 2) +
        std::pow(candidate.pregrasp[2] - candidate.grasp[2], 2)));
    constexpr std::size_t kMaxExternalPregraspVariants = 2U;

    const Eigen::Vector3d closing_axis = array_to_eigen(external_metadata->closing_direction);
    if (closing_axis.norm() < 1e-6) {
      return build_external_pregrasp_variants(candidate);
    }

    const Eigen::Vector3d grasp_center = array_to_eigen(external_metadata->grasp_center);
    const Eigen::Vector3d current_position(
      current_pose.position.x,
      current_pose.position.y,
      current_pose.position.z);
    const Eigen::Vector3d nominal_pregrasp = array_to_eigen(candidate.pregrasp);
    const Eigen::Vector3d nominal_grasp = array_to_eigen(candidate.grasp);

    Eigen::Vector3d nominal_approach_direction = nominal_pregrasp - nominal_grasp;
    if (nominal_approach_direction.norm() < 1e-6) {
      nominal_approach_direction = array_to_eigen(external_metadata->approach_direction);
    }
    if (nominal_approach_direction.norm() < 1e-6) {
      nominal_approach_direction = grasp_center - current_position;
    }
    nominal_approach_direction = project_onto_plane(nominal_approach_direction, closing_axis);
    if (nominal_approach_direction.norm() < 1e-6 && !approach_directions.empty()) {
      nominal_approach_direction =
        project_onto_plane(array_to_eigen(approach_directions.front()), closing_axis);
    }
    if (nominal_approach_direction.norm() < 1e-6) {
      return build_external_pregrasp_variants(candidate);
    }
    nominal_approach_direction.normalize();

    const double grasp_z = external_metadata->grasp_center[2];
    const double current_z = current_pose.position.z;
    const double table_top_z =
      (table_pose_xyz_.size() >= 3 ? table_pose_xyz_[2] : 0.38) +
      0.5 * (table_size_m_.size() >= 3 ? table_size_m_[2] : 0.04);
    const bool table_supported_target = (grasp_z - table_top_z) <= 0.12;
    const double nominal_standoff = std::clamp(nominal_pregrasp_offset, 0.07, 0.16);
    const double safe_standoff = std::clamp(
      std::max(nominal_standoff * 1.20, nominal_standoff + 0.025),
      nominal_standoff + 0.01,
      0.19);
    const double nominal_min_z = table_supported_target ?
      std::max(grasp_z + 0.030, table_top_z + 0.020) :
      (grasp_z - 0.030);
    const double nominal_max_z = table_supported_target ?
      (grasp_z + 0.070) :
      (grasp_z + 0.110);
    const double safe_vertical_clearance = std::max(
      {
        candidate.pregrasp[2] + 0.015,
        grasp_z + 0.038,
        current_z - 0.005,
      });

    const auto safe_orientation =
      make_axis_aligned_orientation(
      external_metadata->closing_direction,
      eigen_to_array(nominal_approach_direction));

    std::vector<PregraspVariant> variants;
    variants.reserve(kMaxExternalPregraspVariants);
    Eigen::Vector3d nominal_seed =
      grasp_center + nominal_approach_direction * nominal_standoff;
    nominal_seed.z() = std::clamp(candidate.pregrasp[2], nominal_min_z, nominal_max_z);
    variants.push_back(
      PregraspVariant{
        eigen_to_array(nominal_seed),
        candidate.grasp,
        candidate.pregrasp_orientation,
        candidate.grasp_orientation,
        " nominal",
      });
    variants.push_back(
      PregraspVariant{
        {
          grasp_center.x() + nominal_approach_direction.x() * safe_standoff,
          grasp_center.y() + nominal_approach_direction.y() * safe_standoff,
          std::clamp(safe_vertical_clearance, nominal_min_z + 0.008, nominal_max_z + 0.020),
        },
        candidate.grasp,
        safe_orientation,
        safe_orientation,
        " safe_lifted",
      });

    return variants;
  }

  std::vector<ExternalQuickIkRegionSample> build_external_second_round_region_samples(
    const std::vector<PregraspVariant> & seed_variants,
    const ExternalProposalMetadata & metadata,
    double spatial_scale,
    double angular_scale,
    const std::vector<std::size_t> & selected_seed_indices = {}) const
  {
    std::vector<ExternalQuickIkRegionSample> samples;
    if (seed_variants.empty()) {
      return samples;
    }

    std::vector<std::size_t> seed_indices = selected_seed_indices;
    if (seed_indices.empty()) {
      seed_indices.reserve(seed_variants.size());
      for (std::size_t index = 0; index < seed_variants.size(); ++index) {
        seed_indices.push_back(index);
      }
    }

    const Eigen::Vector3d world_up = Eigen::Vector3d::UnitZ();
    const double table_top_z =
      (table_pose_xyz_.size() >= 3 ? table_pose_xyz_[2] : 0.38) +
      0.5 * (table_size_m_.size() >= 3 ? table_size_m_[2] : 0.04);
    const bool table_supported_target = (metadata.grasp_center[2] - table_top_z) <= 0.12;
    Eigen::Vector3d closing_axis = array_to_eigen(metadata.closing_direction);
    if (closing_axis.norm() > 1e-6) {
      closing_axis.normalize();
    }
    Eigen::Vector3d metadata_approach = array_to_eigen(metadata.approach_direction);
    if (metadata_approach.norm() > 1e-6) {
      metadata_approach.normalize();
    }

    const double standoff_delta = external_second_round_standoff_delta_m_ * spatial_scale;
    const double lateral_delta = external_second_round_lateral_delta_m_ * spatial_scale;
    const double vertical_delta = external_second_round_vertical_delta_m_ * spatial_scale;
    const double roll_delta_deg = external_second_round_roll_slack_deg_ * angular_scale;
    const double pitch_delta_deg = external_second_round_pitch_slack_deg_ * angular_scale;

    struct RegionPattern
    {
      double standoff_ratio;
      double lateral_ratio;
      double vertical_ratio;
      double roll_ratio;
      double pitch_ratio;
      int cluster_id;
    };

    const std::vector<RegionPattern> patterns = {
      {0.0, 0.0, 0.0, 0.0, 0.0, 0},
      {1.0, 0.0, 0.0, 0.0, 0.0, 1},
      {-1.0, 0.0, 0.0, 0.0, 0.0, 2},
      {0.0, 1.0, 0.0, 0.0, 0.0, 3},
      {0.0, -1.0, 0.0, 0.0, 0.0, 4},
      {0.0, 0.0, 1.0, 0.0, 0.0, 5},
      {0.0, 0.0, -1.0, 0.0, 0.0, 6},
      {0.0, 0.0, 0.0, 1.0, 0.0, 7},
      {0.0, 0.0, 0.0, -1.0, 0.0, 8},
      {0.0, 0.0, 0.0, 0.0, 1.0, 9},
      {0.0, 0.0, 0.0, 0.0, -1.0, 10},
      {1.0, 0.0, 1.0, 0.0, 0.0, 11},
      {1.0, 1.0, 0.0, 0.0, 0.0, 12},
      {1.0, -1.0, 0.0, 0.0, 0.0, 13},
      {1.0, 0.0, 1.0, 1.0, 0.0, 14},
      {1.0, 0.0, 1.0, -1.0, 0.0, 15},
      {1.0, 0.0, 0.0, 0.0, 1.0, 16},
      {1.0, 0.0, 0.0, 0.0, -1.0, 17},
    };

    const auto push_sample =
      [&](std::size_t seed_index,
      const PregraspVariant & variant,
      const RegionPattern & pattern)
      {
        const double penalty =
          0.28 * std::abs(pattern.standoff_ratio) +
          0.18 * std::abs(pattern.lateral_ratio) +
          0.14 * std::abs(pattern.vertical_ratio) +
          0.12 * std::abs(pattern.roll_ratio) +
          0.10 * std::abs(pattern.pitch_ratio);
        samples.push_back(ExternalQuickIkRegionSample{
          variant,
          seed_index,
          pattern.cluster_id,
          clamp01(1.0 - penalty),
        });
      };

    for (const std::size_t seed_index : seed_indices) {
      if (seed_index >= seed_variants.size()) {
        continue;
      }

      const auto & seed = seed_variants[seed_index];
      const Eigen::Vector3d grasp = array_to_eigen(seed.grasp);
      const Eigen::Vector3d pregrasp = array_to_eigen(seed.pregrasp);

      Eigen::Vector3d approach_direction = pregrasp - grasp;
      if (approach_direction.norm() < 1e-6) {
        approach_direction = metadata_approach;
      }
      if (approach_direction.norm() < 1e-6) {
        approach_direction = Eigen::Vector3d(-1.0, 0.0, 0.0);
      }
      approach_direction.normalize();

      Eigen::Vector3d lateral_direction = closing_axis;
      if (lateral_direction.norm() < 1e-6) {
        lateral_direction = project_onto_plane(world_up.cross(approach_direction), approach_direction);
      }
      if (lateral_direction.norm() > 1e-6) {
        lateral_direction.normalize();
      }

      for (const auto & pattern : patterns) {
        auto sample = seed;
        Eigen::Vector3d adjusted_pregrasp = pregrasp;
        const double effective_vertical_ratio =
          table_supported_target ? std::max(0.0, pattern.vertical_ratio) : pattern.vertical_ratio;
        if (standoff_delta > 1e-6) {
          adjusted_pregrasp += approach_direction * (pattern.standoff_ratio * standoff_delta);
        }
        if (lateral_delta > 1e-6 && lateral_direction.norm() > 1e-6) {
          adjusted_pregrasp += lateral_direction * (pattern.lateral_ratio * lateral_delta);
        }
        if (vertical_delta > 1e-6) {
          adjusted_pregrasp += world_up * (effective_vertical_ratio * vertical_delta);
        }
        if (table_supported_target) {
          const double min_pregrasp_z = std::max(grasp.z() + 0.030, table_top_z + 0.020);
          const double max_pregrasp_z = grasp.z() + 0.075;
          adjusted_pregrasp.z() = std::clamp(adjusted_pregrasp.z(), min_pregrasp_z, max_pregrasp_z);
        }
        sample.pregrasp = eigen_to_array(adjusted_pregrasp);
        if (roll_delta_deg > 1e-6 && std::abs(pattern.roll_ratio) > 1e-6) {
          sample.pregrasp_orientation = rotate_orientation_about_local_x(
            sample.pregrasp_orientation, pattern.roll_ratio * roll_delta_deg);
          sample.grasp_orientation = rotate_orientation_about_local_x(
            sample.grasp_orientation, pattern.roll_ratio * roll_delta_deg);
        }
        if (pitch_delta_deg > 1e-6 && std::abs(pattern.pitch_ratio) > 1e-6) {
          sample.pregrasp_orientation = rotate_orientation_about_local_y(
            sample.pregrasp_orientation, pattern.pitch_ratio * pitch_delta_deg);
          sample.grasp_orientation = rotate_orientation_about_local_y(
            sample.grasp_orientation, pattern.pitch_ratio * pitch_delta_deg);
        }
        if (table_supported_target && effective_vertical_ratio != pattern.vertical_ratio) {
          sample.label_suffix += " table_clamped";
        }
        push_sample(seed_index, sample, pattern);
      }
    }

    return samples;
  }

  ReducedOrientationConstraint build_external_reduced_orientation_constraint() const
  {
    if (!external_reduced_orientation_constraint_enabled_) {
      return make_no_reduced_orientation_constraint();
    }

    ReducedOrientationConstraint constraint;
    constraint.enabled = true;
    constraint.free_axis = ReducedOrientationFreeAxis::X;
    constraint.free_axis_tolerance_rad =
      std::clamp(external_reduced_free_axis_tolerance_deg_, 0.0, 180.0) * M_PI / 180.0;
    const double constrained_axis_tolerance_rad =
      std::clamp(external_reduced_constrained_axis_tolerance_deg_, 0.5, 180.0) * M_PI / 180.0;
    constraint.constrained_y_tolerance_rad = constrained_axis_tolerance_rad;
    constraint.constrained_z_tolerance_rad = constrained_axis_tolerance_rad;
    return constraint;
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

  void clear_cached_target_pose()
  {
    std::scoped_lock<std::mutex> lock(target_mutex_);
    latest_target_pose_ = geometry_msgs::msg::PoseStamped{};
    latched_target_pose_ = geometry_msgs::msg::PoseStamped{};
    has_target_pose_ = false;
    has_latched_target_pose_ = false;
    latest_target_pose_update_sec_ = 0.0;
  }

  void clear_external_grasp_candidate_cache()
  {
    {
      std::scoped_lock<std::mutex> lock(grasp_candidate_mutex_);
      latest_selected_pregrasp_pose_ = geometry_msgs::msg::PoseStamped{};
      latest_selected_grasp_pose_ = geometry_msgs::msg::PoseStamped{};
      latest_external_pregrasp_pose_array_ = geometry_msgs::msg::PoseArray{};
      latest_external_grasp_pose_array_ = geometry_msgs::msg::PoseArray{};
      latest_external_grasp_proposal_array_ = tactile_interfaces::msg::GraspProposalArray{};
      has_selected_pregrasp_pose_ = false;
      has_selected_grasp_pose_ = false;
      has_external_pregrasp_pose_array_ = false;
      has_external_grasp_pose_array_ = false;
      has_external_grasp_proposal_array_ = false;
      latest_external_grasp_update_sec_ = 0.0;
    }
    last_grasp_refresh_request_sec_ = 0.0;
    publish_external_execution_markers({}, std::nullopt);
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
    const double now_sec = this->get_clock()->now().seconds();
    if (manual_home_hold_until_sec_ > now_sec) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "target re-lock ignored while manual home hold is active");
      return;
    }
    if (failure_rearm_hold_until_sec_ > now_sec) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "target re-lock ignored while failure re-arm hold is active");
      return;
    }
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

  moveit_msgs::msg::Constraints make_path_constraints(
    const JointConstraintAttempt & attempt,
    const geometry_msgs::msg::Quaternion & target_orientation,
    const ReducedOrientationConstraint & reduced_orientation_constraint) const
  {
    auto constraints = make_locked_joint_constraints(attempt);
    if (!reduced_orientation_constraint.enabled) {
      return constraints;
    }

    moveit_msgs::msg::OrientationConstraint orientation_constraint;
    orientation_constraint.header.frame_id = base_frame_;
    orientation_constraint.link_name = ee_link_;
    orientation_constraint.orientation = normalize_quaternion(target_orientation);
    double x_tolerance = std::max(1e-3, reduced_orientation_constraint.constrained_y_tolerance_rad);
    double y_tolerance = std::max(1e-3, reduced_orientation_constraint.constrained_z_tolerance_rad);
    double z_tolerance = std::max(1e-3, reduced_orientation_constraint.constrained_z_tolerance_rad);
    switch (reduced_orientation_constraint.free_axis) {
      case ReducedOrientationFreeAxis::X:
        x_tolerance = std::max(1e-3, reduced_orientation_constraint.free_axis_tolerance_rad);
        break;
      case ReducedOrientationFreeAxis::Y:
        y_tolerance = std::max(1e-3, reduced_orientation_constraint.free_axis_tolerance_rad);
        break;
      case ReducedOrientationFreeAxis::Z:
      default:
        z_tolerance = std::max(1e-3, reduced_orientation_constraint.free_axis_tolerance_rad);
        break;
    }
    orientation_constraint.absolute_x_axis_tolerance = x_tolerance;
    orientation_constraint.absolute_y_axis_tolerance = y_tolerance;
    orientation_constraint.absolute_z_axis_tolerance = z_tolerance;
    orientation_constraint.weight = 1.0;
    constraints.orientation_constraints.push_back(orientation_constraint);
    return constraints;
  }

  ReducedOrientationConstraint make_no_reduced_orientation_constraint() const
  {
    ReducedOrientationConstraint constraint;
    return constraint;
  }

  static double wrapped_angle_delta(double from_rad, double to_rad)
  {
    return std::remainder(to_rad - from_rad, 2.0 * M_PI);
  }

  bool satisfies_joint_lock_attempt(
    const moveit::core::RobotState & state,
    const JointConstraintAttempt & attempt) const
  {
    if (locked_joint_reference_valid_ && attempt.primary_joint_delta_rad > 0.0 &&
      !lock_joint_name_.empty())
    {
      const double joint_value = state.getVariablePosition(lock_joint_name_);
      if (std::abs(wrapped_angle_delta(locked_joint_reference_rad_, joint_value)) >
        attempt.primary_joint_delta_rad + 1e-6)
      {
        return false;
      }
    }

    if (locked_wrist_joint_reference_valid_ && attempt.wrist_joint_delta_rad > 0.0 &&
      !wrist_joint_name_.empty())
    {
      const double joint_value = state.getVariablePosition(wrist_joint_name_);
      if (std::abs(wrapped_angle_delta(locked_wrist_joint_reference_rad_, joint_value)) >
        attempt.wrist_joint_delta_rad + 1e-6)
      {
        return false;
      }
    }

    return true;
  }

  bool solve_quick_ik_pose(
    const moveit::core::RobotModelConstPtr & robot_model,
    const moveit::core::JointModelGroup * joint_model_group,
    const moveit::core::RobotStatePtr & seed_state,
    const std::array<double, 3> & target_point,
    const geometry_msgs::msg::Quaternion & target_orientation,
    const JointConstraintAttempt & joint_attempt,
    bool apply_seed_bias,
    moveit::core::RobotStatePtr * solved_state_out) const
  {
    if (!robot_model || joint_model_group == nullptr) {
      return false;
    }

    auto state = seed_state ? std::make_shared<moveit::core::RobotState>(*seed_state) :
      std::make_shared<moveit::core::RobotState>(robot_model);
    if (!state) {
      return false;
    }
    if (!seed_state) {
      state->setToDefaultValues();
    }
    if (apply_seed_bias) {
      apply_planning_seed_bias(*state);
    } else {
      state->enforceBounds(joint_model_group);
    }

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = target_point[0];
    target_pose.position.y = target_point[1];
    target_pose.position.z = target_point[2];
    target_pose.orientation = normalize_quaternion(target_orientation);

    kinematics::KinematicsQueryOptions query_options;
    query_options.return_approximate_solution = true;

    const bool solved = state->setFromIK(
      joint_model_group,
      target_pose,
      ee_link_,
      quick_ik_prefilter_timeout_sec_,
      moveit::core::GroupStateValidityCallbackFn(),
      query_options);
    if (!solved) {
      return false;
    }

    state->update();
    if (!satisfies_joint_lock_attempt(*state, joint_attempt)) {
      return false;
    }

    if (solved_state_out != nullptr) {
      *solved_state_out = state;
    }
    return true;
  }

  std::size_t effective_quick_ik_worker_count(std::size_t job_count) const
  {
    if (!quick_ik_prefilter_parallel_enabled_ || job_count < 2U) {
      return 1U;
    }
    const std::size_t configured_workers = std::max<int>(
      1,
      parallel_candidate_evaluation_enabled_ ? parallel_candidate_worker_count_ : 2);
    return std::max<std::size_t>(
      1U,
      std::min<std::size_t>(job_count, configured_workers));
  }

  QuickIkPrefilterResult run_quick_ik_prefilter_jobs(
    std::size_t job_count,
    const std::function<bool(std::size_t)> & evaluate_job) const
  {
    QuickIkPrefilterResult result;
    if (!quick_ik_prefilter_enabled_ || job_count == 0U) {
      return result;
    }

    const int required_hits = std::max(
      1,
      std::min<int>(quick_ik_prefilter_required_hits_, static_cast<int>(job_count)));
    std::atomic_size_t next_job{0};
    std::atomic_int hits{0};
    std::atomic_int tested_jobs{0};

    const auto worker_fn = [&]() {
      while (hits.load(std::memory_order_relaxed) < required_hits) {
        const std::size_t job_index = next_job.fetch_add(1, std::memory_order_relaxed);
        if (job_index >= job_count) {
          break;
        }
        tested_jobs.fetch_add(1, std::memory_order_relaxed);
        if (evaluate_job(job_index)) {
          const int updated_hits = hits.fetch_add(1, std::memory_order_relaxed) + 1;
          if (updated_hits >= required_hits) {
            break;
          }
        }
      }
    };

    const std::size_t worker_count = effective_quick_ik_worker_count(job_count);
    if (worker_count > 1U) {
      std::vector<std::future<void>> futures;
      futures.reserve(worker_count);
      for (std::size_t worker_index = 0; worker_index < worker_count; ++worker_index) {
        futures.push_back(std::async(std::launch::async, worker_fn));
      }
      for (auto & future : futures) {
        future.get();
      }
    } else {
      worker_fn();
    }

    result.hits = hits.load(std::memory_order_relaxed);
    result.samples = tested_jobs.load(std::memory_order_relaxed);
    result.passed = result.hits >= required_hits;
    return result;
  }

  bool plan_pose_target_attempt(
    moveit::planning_interface::MoveGroupInterface & planning_group,
    const std::array<double, 3> & target_point,
    const geometry_msgs::msg::Quaternion & target_orientation,
    const JointConstraintAttempt & joint_attempt,
    moveit::planning_interface::MoveGroupInterface::Plan & plan,
    const moveit::core::RobotStatePtr & start_state,
    bool position_only_target,
    double planning_time_override_sec,
    int planning_attempts_override,
    const ReducedOrientationConstraint & reduced_orientation_constraint)
  {
    const bool override_planning_time = planning_time_override_sec > 0.0;
    const bool override_planning_attempts = planning_attempts_override > 0;
    if (override_planning_time) {
      planning_group.setPlanningTime(planning_time_override_sec);
    }
    if (override_planning_attempts) {
      planning_group.setNumPlanningAttempts(planning_attempts_override);
    }

    if (start_state) {
      auto planning_start_state =
        std::make_shared<moveit::core::RobotState>(*start_state);
      apply_planning_seed_bias(*planning_start_state);
      planning_group.setStartState(*planning_start_state);
    } else {
      set_bounded_start_state(planning_group, true);
    }

    const auto constraints =
      make_path_constraints(joint_attempt, target_orientation, reduced_orientation_constraint);
    if (!constraints.joint_constraints.empty() || !constraints.orientation_constraints.empty()) {
      planning_group.setPathConstraints(constraints);
    } else {
      planning_group.clearPathConstraints();
    }

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = target_point[0];
    target_pose.position.y = target_point[1];
    target_pose.position.z = target_point[2];
    target_pose.orientation = normalize_quaternion(target_orientation);
    const bool use_position_target =
      position_only_target || reduced_orientation_constraint.enabled;
    if (use_position_target) {
      planning_group.setPositionTarget(
        target_pose.position.x,
        target_pose.position.y,
        target_pose.position.z,
        ee_link_);
    } else {
      planning_group.setPoseTarget(target_pose, ee_link_);
    }
    const bool planned = static_cast<bool>(planning_group.plan(plan));
    planning_group.clearPoseTargets();
    planning_group.clearPathConstraints();
    if (override_planning_time) {
      planning_group.setPlanningTime(planning_time_sec_);
    }
    if (override_planning_attempts) {
      planning_group.setNumPlanningAttempts(planning_attempts_);
    }
    auto fallback_state = build_fallback_current_state(planning_group);
    if (fallback_state) {
      planning_group.setStartState(*fallback_state);
    } else {
      planning_group.setStartStateToCurrentState();
    }
    return planned;
  }

  bool plan_pose_target_attempt(
    const std::array<double, 3> & target_point,
    const geometry_msgs::msg::Quaternion & target_orientation,
    const JointConstraintAttempt & joint_attempt,
    moveit::planning_interface::MoveGroupInterface::Plan & plan,
    const moveit::core::RobotStatePtr & start_state,
    bool position_only_target,
    double planning_time_override_sec,
    int planning_attempts_override,
    const ReducedOrientationConstraint & reduced_orientation_constraint)
  {
    return plan_pose_target_attempt(
      *arm_group_,
      target_point,
      target_orientation,
      joint_attempt,
      plan,
      start_state,
      position_only_target,
      planning_time_override_sec,
      planning_attempts_override,
      reduced_orientation_constraint);
  }

  moveit::core::RobotStatePtr build_plan_end_state(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan,
    const moveit::core::RobotStatePtr & seed_state)
  {
    auto state = seed_state ? std::make_shared<moveit::core::RobotState>(*seed_state) : nullptr;
    if (!state) {
      state = arm_group_->getCurrentState(1.0);
      if (!state) {
        state = build_fallback_current_state(*arm_group_);
        if (!state) {
          return nullptr;
        }
      }
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

  moveit::core::RobotStatePtr build_plan_end_state(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan)
  {
    return build_plan_end_state(plan, nullptr);
  }

  moveit::core::RobotStatePtr build_fallback_current_state(
    moveit::planning_interface::MoveGroupInterface & group)
  {
    std::scoped_lock<std::mutex> lock(joint_state_mutex_);
    if (!has_latest_joint_state_) {
      return nullptr;
    }

    auto robot_model = group.getRobotModel();
    if (!robot_model) {
      return nullptr;
    }

    auto state = std::make_shared<moveit::core::RobotState>(robot_model);
    state->setToDefaultValues();
    const auto & variable_names = state->getVariableNames();
    const std::size_t count = std::min(latest_joint_state_.name.size(), latest_joint_state_.position.size());
    for (std::size_t index = 0; index < count; ++index) {
      if (
        std::find(
          variable_names.begin(),
          variable_names.end(),
          latest_joint_state_.name[index]) == variable_names.end())
      {
        continue;
      }
      state->setVariablePosition(latest_joint_state_.name[index], latest_joint_state_.position[index]);
    }
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
    const moveit::core::RobotStatePtr & stage_start_state,
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
    double * wrist_joint_excursion_cost_out = nullptr)
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
    double * wrist_joint_excursion_cost_out = nullptr)
  {
    auto stage_start_state = arm_group_->getCurrentState(0.5);
    if (!stage_start_state) {
      stage_start_state = build_fallback_current_state(*arm_group_);
    }
    return compute_pregrasp_option_score(
      stage_start_state,
      current_pose,
      target_point,
      stage_point,
      variant,
      stage_plan,
      pregrasp_plan,
      stage_end_state,
      pregrasp_end_state,
      camera_penalty_out,
      camera_stability_bonus_out,
      primary_joint_excursion_cost_out,
      secondary_joint_excursion_cost_out,
      wrist_joint_excursion_cost_out);
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

  double compute_external_first_round_score(
    const ExternalProposalMetadata & metadata,
    const std::array<double, 3> & target_point) const
  {
    const double max_width_m = 0.06;
    if (metadata.grasp_width_m > max_width_m + 1e-6 || metadata.grasp_width_m <= 1e-6) {
      return -1.0;
    }

    const Eigen::Vector3d contact_point_1 = array_to_eigen(metadata.contact_point_1);
    const Eigen::Vector3d contact_point_2 = array_to_eigen(metadata.contact_point_2);
    const Eigen::Vector3d grasp_center = array_to_eigen(metadata.grasp_center);
    const Eigen::Vector3d midpoint = 0.5 * (contact_point_1 + contact_point_2);
    Eigen::Vector3d closing_direction = array_to_eigen(metadata.closing_direction);
    if (closing_direction.norm() > 1e-6) {
      closing_direction.normalize();
    }

    const double semantic_score = clamp01(metadata.semantic_score);
    const double confidence_score = clamp01(metadata.confidence_score);
    const double network_score = clamp01(0.55 * semantic_score + 0.45 * confidence_score);

    const double table_top_z =
      (table_pose_xyz_.size() >= 3 ? table_pose_xyz_[2] : 0.38) +
      0.5 * (table_size_m_.size() >= 3 ? table_size_m_[2] : 0.04);
    const double target_center_z = target_point[2];
    const bool table_supported_target =
      (target_center_z - table_top_z) <= std::max(0.04, (target_height_m_ * 0.5) + 0.02);
    const double line_horizontal_score =
      closing_direction.norm() > 1e-6 ? 1.0 - std::abs(closing_direction.dot(Eigen::Vector3d::UnitZ())) : 0.5;
    double table_clearance_score = 1.0;
    if (table_supported_target) {
      const double min_contact_z = std::min(contact_point_1.z(), contact_point_2.z());
      table_clearance_score = clamp01((min_contact_z - (table_top_z + 0.006)) / 0.020);
    }
    const double table_score = clamp01(0.6 * line_horizontal_score + 0.4 * table_clearance_score);

    const double span_distance_m = (contact_point_1 - contact_point_2).norm();
    const double midpoint_error_m = (midpoint - grasp_center).norm();
    const double contact_symmetry_m =
      std::abs((contact_point_1 - grasp_center).norm() - (contact_point_2 - grasp_center).norm());
    const double contact_span_error_m = std::abs(span_distance_m - metadata.grasp_width_m);
    const double midpoint_score = clamp01(1.0 - midpoint_error_m / 0.015);
    const double symmetry_score = clamp01(1.0 - contact_symmetry_m / 0.015);
    const double span_score = clamp01(1.0 - contact_span_error_m / 0.015);
    const double contact_score = clamp01((midpoint_score + symmetry_score + span_score) / 3.0);

    const Eigen::Vector2d grasp_xy(grasp_center.x(), grasp_center.y());
    const Eigen::Vector2d target_xy(target_point[0], target_point[1]);
    const double xy_offset_m = (grasp_xy - target_xy).norm();
    const double xy_score = clamp01(1.0 - xy_offset_m / std::max(0.020, target_radius_m_ * 1.2));
    const double object_bottom_z = target_center_z - (0.5 * target_height_m_);
    const double z_fraction = target_height_m_ > 1e-6 ?
      (grasp_center.z() - object_bottom_z) / target_height_m_ : 0.5;
    const double z_score = clamp01(1.0 - std::abs(z_fraction - 0.55) / 0.55);
    const double com_score = clamp01(0.55 * xy_score + 0.45 * z_score);

    const double width_ratio = metadata.grasp_width_m / max_width_m;
    const double width_score = clamp01(1.0 - std::abs(width_ratio - 0.55) / 0.55);

    const double target_radius_proxy = std::max(0.010, target_radius_m_);
    const double target_height_proxy = std::max(0.04, target_height_m_);
    const double radial_support_score =
      clamp01(1.0 - std::abs(0.5 * span_distance_m - target_radius_proxy) / std::max(0.010, target_radius_proxy));
    const double height_support_score =
      clamp01(1.0 - std::abs(grasp_center.z() - target_center_z) / std::max(0.025, 0.5 * target_height_proxy));
    const double surface_score = clamp01(0.6 * radial_support_score + 0.4 * height_support_score);

    const double weighted_score =
      external_first_round_network_weight_ * network_score +
      external_first_round_table_weight_ * table_score +
      external_first_round_contact_weight_ * contact_score +
      external_first_round_com_weight_ * com_score +
      external_first_round_width_weight_ * width_score +
      external_first_round_surface_weight_ * surface_score;
    const double total_weight =
      external_first_round_network_weight_ +
      external_first_round_table_weight_ +
      external_first_round_contact_weight_ +
      external_first_round_com_weight_ +
      external_first_round_width_weight_ +
      external_first_round_surface_weight_;
    if (total_weight <= 1e-6) {
      return weighted_score;
    }
    return weighted_score / total_weight;
  }

  double compute_external_second_round_joint_score(
    const moveit::core::RobotState & current_start_state,
    const moveit::core::RobotState & pregrasp_state,
    const moveit::core::RobotState & grasp_state) const
  {
    auto compute_joint_delta_score =
      [&](const std::string & joint_name, double tolerance_rad, double default_score) -> double
      {
        if (joint_name.empty() || tolerance_rad <= 1e-6) {
          return default_score;
        }
        const double current_value = current_start_state.getVariablePosition(joint_name);
        const double pregrasp_value = pregrasp_state.getVariablePosition(joint_name);
        const double grasp_value = grasp_state.getVariablePosition(joint_name);
        const double delta_rad = std::max(
          std::abs(wrapped_angle_delta(current_value, pregrasp_value)),
          std::abs(wrapped_angle_delta(current_value, grasp_value)));
        return clamp01(1.0 - (delta_rad / tolerance_rad));
      };

    const auto * joint_model_group = current_start_state.getJointModelGroup(arm_group_name_);
    double mean_joint_score = 0.6;
    if (joint_model_group != nullptr) {
      std::vector<double> current_joint_values;
      std::vector<double> pregrasp_joint_values;
      std::vector<double> grasp_joint_values;
      current_start_state.copyJointGroupPositions(joint_model_group, current_joint_values);
      pregrasp_state.copyJointGroupPositions(joint_model_group, pregrasp_joint_values);
      grasp_state.copyJointGroupPositions(joint_model_group, grasp_joint_values);
      const std::size_t joint_count = std::min(
        current_joint_values.size(),
        std::min(pregrasp_joint_values.size(), grasp_joint_values.size()));
      if (joint_count > 0U) {
        double mean_abs_delta = 0.0;
        for (std::size_t index = 0; index < joint_count; ++index) {
          const double pregrasp_delta = std::abs(
            wrapped_angle_delta(current_joint_values[index], pregrasp_joint_values[index]));
          const double grasp_delta = std::abs(
            wrapped_angle_delta(current_joint_values[index], grasp_joint_values[index]));
          mean_abs_delta += std::max(pregrasp_delta, grasp_delta);
        }
        mean_abs_delta /= static_cast<double>(joint_count);
        mean_joint_score = clamp01(1.0 - (mean_abs_delta / 0.75));
      }
    }

    const double primary_joint_score = compute_joint_delta_score(lock_joint_name_, 1.20, 0.7);
    const double wrist_joint_score = compute_joint_delta_score(wrist_joint_name_, 1.55, 0.7);
    return clamp01(
      0.40 * primary_joint_score +
      0.25 * wrist_joint_score +
      0.35 * mean_joint_score);
  }

  double compute_external_second_round_deviation_score(
    const moveit::core::RobotState & grasp_state,
    const ExternalProposalMetadata & metadata) const
  {
    const Eigen::Isometry3d & ee_transform = grasp_state.getGlobalLinkTransform(ee_link_);
    const Eigen::Vector3d actual_center = ee_transform.translation();
    const Eigen::Vector3d actual_closing = ee_transform.rotation().col(0).normalized();
    const Eigen::Vector3d actual_approach = ee_transform.rotation().col(2).normalized();
    const Eigen::Vector3d target_center = array_to_eigen(metadata.grasp_center);
    const Eigen::Vector3d target_contact_1 = array_to_eigen(metadata.contact_point_1);
    const Eigen::Vector3d target_contact_2 = array_to_eigen(metadata.contact_point_2);
    Eigen::Vector3d target_closing = array_to_eigen(metadata.closing_direction);
    if (target_closing.norm() > 1e-6) {
      target_closing.normalize();
    }
    Eigen::Vector3d target_approach = array_to_eigen(metadata.approach_direction);
    if (target_approach.norm() > 1e-6) {
      target_approach.normalize();
    }

    const double d_center_m = (actual_center - target_center).norm();
    const double center_score =
      clamp01(1.0 - (d_center_m / std::max(0.018, target_radius_m_ * 1.4)));

    const Eigen::Vector3d predicted_contact_1 =
      actual_center - (actual_closing * (0.5 * metadata.grasp_width_m));
    const Eigen::Vector3d predicted_contact_2 =
      actual_center + (actual_closing * (0.5 * metadata.grasp_width_m));
    const double contact_error_direct =
      0.5 * (
      (predicted_contact_1 - target_contact_1).norm() +
      (predicted_contact_2 - target_contact_2).norm());
    const double contact_error_swapped =
      0.5 * (
      (predicted_contact_1 - target_contact_2).norm() +
      (predicted_contact_2 - target_contact_1).norm());
    const double d_contact_m = std::min(contact_error_direct, contact_error_swapped);
    const double contact_score =
      clamp01(1.0 - (d_contact_m / std::max(0.018, target_radius_m_ * 1.2)));

    const double closing_alignment_score =
      safe_axis_alignment_score(actual_closing, target_closing, 45.0, true);
    const double approach_alignment_score =
      target_approach.norm() > 1e-6 ?
      safe_axis_alignment_score(actual_approach, target_approach, 35.0, false) :
      1.0;

    return clamp01(
      0.30 * center_score +
      0.25 * contact_score +
      0.25 * closing_alignment_score +
      0.20 * approach_alignment_score);
  }

  double compute_external_second_round_pregrasp_score(
    const PregraspVariant & variant,
    const ExternalProposalMetadata & metadata) const
  {
    const Eigen::Vector3d pregrasp = array_to_eigen(variant.pregrasp);
    const Eigen::Vector3d grasp = array_to_eigen(variant.grasp);
    const double standoff_m = (pregrasp - grasp).norm();
    const double standoff_score = clamp01(1.0 - (std::abs(standoff_m - 0.12) / 0.08));

    const double vertical_offset_m = variant.pregrasp[2] - metadata.grasp_center[2];
    const double vertical_score = clamp01(1.0 - (std::abs(vertical_offset_m - 0.035) / 0.04));

    Eigen::Vector3d approach_direction = pregrasp - grasp;
    if (approach_direction.norm() > 1e-6) {
      approach_direction.normalize();
    }
    Eigen::Vector3d target_approach = array_to_eigen(metadata.approach_direction);
    if (target_approach.norm() > 1e-6) {
      target_approach.normalize();
    }
    const double approach_score =
      target_approach.norm() > 1e-6 ?
      safe_axis_alignment_score(approach_direction, target_approach, 50.0, false) :
      1.0;

    return clamp01(
      0.40 * standoff_score +
      0.25 * vertical_score +
      0.35 * approach_score);
  }

  ExternalSecondRoundScore combine_external_second_round_scores(
    const ExternalSecondRoundScore & coarse,
    const ExternalSecondRoundScore & refine) const
  {
    ExternalSecondRoundScore combined;
    combined.samples = coarse.samples + refine.samples;
    combined.hits = coarse.hits + refine.hits;
    combined.has_hit = coarse.has_hit || refine.has_hit;
    combined.best_sample_quality = std::max(coarse.best_sample_quality, refine.best_sample_quality);
    const std::size_t hit_weight = static_cast<std::size_t>(combined.hits);
    auto weighted_average = [&](double lhs, int lhs_hits, double rhs, int rhs_hits) {
        const int total_hits = lhs_hits + rhs_hits;
        if (total_hits <= 0) {
          return std::max(lhs, rhs);
        }
        return
          ((lhs * static_cast<double>(lhs_hits)) +
          (rhs * static_cast<double>(rhs_hits))) /
          static_cast<double>(total_hits);
      };
    combined.deviation_score = weighted_average(
      coarse.deviation_score, coarse.hits, refine.deviation_score, refine.hits);
    combined.joint_score = weighted_average(
      coarse.joint_score, coarse.hits, refine.joint_score, refine.hits);
    combined.pregrasp_score = weighted_average(
      coarse.pregrasp_score, coarse.hits, refine.pregrasp_score, refine.hits);

    combined.hits_per_cluster = coarse.hits_per_cluster;
    if (combined.hits_per_cluster.size() < refine.hits_per_cluster.size()) {
      combined.hits_per_cluster.resize(refine.hits_per_cluster.size(), 0);
    }
    for (std::size_t index = 0; index < refine.hits_per_cluster.size(); ++index) {
      combined.hits_per_cluster[index] += refine.hits_per_cluster[index];
    }

    const double hit_ratio =
      combined.samples > 0 ? static_cast<double>(combined.hits) / static_cast<double>(combined.samples) : 0.0;
    int densest_cluster_hits = 0;
    for (const int cluster_hits : combined.hits_per_cluster) {
      densest_cluster_hits = std::max(densest_cluster_hits, cluster_hits);
    }
    const double cluster_density =
      combined.hits > 0 ? static_cast<double>(densest_cluster_hits) / static_cast<double>(combined.hits) : 0.0;
    combined.region_score = clamp01(
      0.55 * hit_ratio +
      0.25 * cluster_density +
      0.20 * combined.best_sample_quality);
    const double total_weight =
      std::max(
      1e-6,
      external_second_round_region_weight_ +
      external_second_round_deviation_weight_ +
      external_second_round_joint_weight_ +
      external_second_round_pregrasp_weight_);
    combined.total_score =
      (
      external_second_round_region_weight_ * combined.region_score +
      external_second_round_deviation_weight_ * combined.deviation_score +
      external_second_round_joint_weight_ * combined.joint_score +
      external_second_round_pregrasp_weight_ * combined.pregrasp_score) /
      total_weight;
    (void)hit_weight;
    return combined;
  }

  ExternalSecondRoundScore compute_external_second_round_score(
    moveit::planning_interface::MoveGroupInterface & planning_group,
    const moveit::core::RobotStatePtr & current_start_state,
    const std::vector<JointConstraintAttempt> & attempts,
    const ExternalProposalMetadata & metadata,
    const std::vector<ExternalQuickIkRegionSample> & samples) const
  {
    ExternalSecondRoundScore score;
    if (samples.empty() || attempts.empty()) {
      return score;
    }

    const auto robot_model = planning_group.getRobotModel();
    const auto * joint_model_group =
      robot_model ? robot_model->getJointModelGroup(arm_group_name_) : nullptr;
    if (!robot_model || joint_model_group == nullptr) {
      return score;
    }

    struct SampleAccum
    {
      bool hit{false};
      double quality{0.0};
      double deviation{0.0};
      double joint{0.0};
      double pregrasp{0.0};
      std::size_t seed_index{0};
      int cluster_id{0};
    };

    std::vector<SampleAccum> sample_results(samples.size());
    for (std::size_t index = 0; index < samples.size(); ++index) {
      sample_results[index].seed_index = samples[index].seed_index;
      sample_results[index].cluster_id = samples[index].cluster_id;
    }
    std::vector<std::mutex> sample_mutexes(samples.size());

    const std::size_t total_jobs = samples.size() * attempts.size();
    std::size_t worker_count = 1U;
    if (quick_ik_prefilter_parallel_enabled_) {
      worker_count = std::max<std::size_t>(
        1U,
        std::min<std::size_t>(
          static_cast<std::size_t>(std::max(parallel_candidate_worker_count_, 1)),
          total_jobs));
    }

    auto evaluate_job = [&](std::size_t job_index) {
        const std::size_t sample_index = job_index % samples.size();
        const std::size_t attempt_index = (job_index / samples.size()) % attempts.size();
        const auto & sample = samples[sample_index];
        const auto & attempt = attempts[attempt_index];

        moveit::core::RobotStatePtr pregrasp_state;
        if (!solve_quick_ik_pose(
              robot_model,
              joint_model_group,
              current_start_state,
              sample.variant.pregrasp,
              sample.variant.pregrasp_orientation,
              attempt,
              true,
              &pregrasp_state))
        {
          return;
        }

        moveit::core::RobotStatePtr grasp_state;
        if (!solve_quick_ik_pose(
              robot_model,
              joint_model_group,
              pregrasp_state,
              sample.variant.grasp,
              sample.variant.grasp_orientation,
              attempt,
              false,
              &grasp_state))
        {
          return;
        }

        const double deviation_score =
          compute_external_second_round_deviation_score(*grasp_state, metadata);
        const double joint_score =
          compute_external_second_round_joint_score(*current_start_state, *pregrasp_state, *grasp_state);
        const double pregrasp_score =
          compute_external_second_round_pregrasp_score(sample.variant, metadata);
        const double best_sample_quality = clamp01(
          0.35 * sample.intrinsic_quality +
          0.30 * deviation_score +
          0.20 * joint_score +
          0.15 * pregrasp_score);

        std::scoped_lock<std::mutex> lock(sample_mutexes[sample_index]);
        auto & result = sample_results[sample_index];
        if (!result.hit || best_sample_quality > result.quality) {
          result.hit = true;
          result.quality = best_sample_quality;
          result.deviation = deviation_score;
          result.joint = joint_score;
          result.pregrasp = pregrasp_score;
        }
      };

    if (worker_count > 1U) {
      std::atomic_size_t next_job{0};
      std::vector<std::future<void>> futures;
      futures.reserve(worker_count);
      for (std::size_t worker_index = 0; worker_index < worker_count; ++worker_index) {
        futures.push_back(std::async(
            std::launch::async,
            [&]() {
              for (;;) {
                const std::size_t job_index = next_job.fetch_add(1);
                if (job_index >= total_jobs) {
                  break;
                }
                evaluate_job(job_index);
              }
            }));
      }
      for (auto & future : futures) {
        future.get();
      }
    } else {
      for (std::size_t job_index = 0; job_index < total_jobs; ++job_index) {
        evaluate_job(job_index);
      }
    }

    score.samples = static_cast<int>(samples.size());
    int max_cluster_id = 0;
    for (const auto & sample : samples) {
      max_cluster_id = std::max(max_cluster_id, sample.cluster_id);
    }
    score.hits_per_cluster.resize(static_cast<std::size_t>(max_cluster_id + 1), 0);
    double deviation_sum = 0.0;
    double joint_sum = 0.0;
    double pregrasp_sum = 0.0;
    for (const auto & sample_result : sample_results) {
      if (!sample_result.hit) {
        continue;
      }
      score.has_hit = true;
      ++score.hits;
      deviation_sum += sample_result.deviation;
      joint_sum += sample_result.joint;
      pregrasp_sum += sample_result.pregrasp;
      score.best_sample_quality = std::max(score.best_sample_quality, sample_result.quality);
      if (sample_result.cluster_id >= 0) {
        const auto cluster_index = static_cast<std::size_t>(sample_result.cluster_id);
        if (cluster_index < score.hits_per_cluster.size()) {
          ++score.hits_per_cluster[cluster_index];
        }
      }
    }
    if (score.hits > 0) {
      score.deviation_score = deviation_sum / static_cast<double>(score.hits);
      score.joint_score = joint_sum / static_cast<double>(score.hits);
      score.pregrasp_score = pregrasp_sum / static_cast<double>(score.hits);
    }

    std::vector<std::pair<double, std::size_t>> hit_ranking;
    hit_ranking.reserve(sample_results.size());
    for (std::size_t sample_index = 0; sample_index < sample_results.size(); ++sample_index) {
      if (!sample_results[sample_index].hit) {
        continue;
      }
      hit_ranking.emplace_back(sample_results[sample_index].quality, sample_index);
    }
    std::sort(
      hit_ranking.begin(),
      hit_ranking.end(),
      [](const auto & lhs, const auto & rhs) {
        return lhs.first > rhs.first;
      });
    for (const auto & [quality, sample_index] : hit_ranking) {
      (void)quality;
      score.top_hit_sample_indices.push_back(sample_index);
    }

    const double hit_ratio =
      score.samples > 0 ? static_cast<double>(score.hits) / static_cast<double>(score.samples) : 0.0;
    int densest_cluster_hits = 0;
    for (const int cluster_hits : score.hits_per_cluster) {
      densest_cluster_hits = std::max(densest_cluster_hits, cluster_hits);
    }
    const double cluster_density =
      score.hits > 0 ? static_cast<double>(densest_cluster_hits) / static_cast<double>(score.hits) : 0.0;
    score.region_score = clamp01(
      0.55 * hit_ratio +
      0.25 * cluster_density +
      0.20 * score.best_sample_quality);

    const double total_weight =
      std::max(
      1e-6,
      external_second_round_region_weight_ +
      external_second_round_deviation_weight_ +
      external_second_round_joint_weight_ +
      external_second_round_pregrasp_weight_);
    score.total_score =
      (
      external_second_round_region_weight_ * score.region_score +
      external_second_round_deviation_weight_ * score.deviation_score +
      external_second_round_joint_weight_ * score.joint_score +
      external_second_round_pregrasp_weight_ * score.pregrasp_score) /
      total_weight;
    return score;
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
           << " ik[hits=" << entry.quick_ik_hits
           << " samples=" << entry.quick_ik_samples
           << "]"
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
        if (!allow_stale_external_candidates_for_current_pick_) {
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
        RCLCPP_WARN(
          this->get_logger(),
          "continuing with stale cached external grasp candidates after timeout: age=%.2fs timeout=%.2fs",
          age_sec,
          external_grasp_pose_timeout_sec_);
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

    const std::size_t original_proposal_candidate_count = proposal_candidate_count;
    if (
      external_screening_candidate_limit_ > 0 &&
      pose_candidate_count > static_cast<std::size_t>(external_screening_candidate_limit_))
    {
      pose_candidate_count = static_cast<std::size_t>(external_screening_candidate_limit_);
    }

    const auto joint_lock_attempts = get_locked_joint_attempts();
    const std::vector<JointConstraintAttempt> attempts =
      joint_lock_attempts.empty() ? std::vector<JointConstraintAttempt>{JointConstraintAttempt{}} :
      joint_lock_attempts;
    RCLCPP_INFO(
      this->get_logger(),
      "external grasp evaluation start: proposals=%zu pose_pairs=%zu single=%s joint_attempts=%zu require_external=%s",
      original_proposal_candidate_count,
      pose_candidate_count,
      have_single_candidate ? "true" : "false",
      attempts.size(),
      require_external_grasp_candidates_ ? "true" : "false");
    auto current_start_state = arm_group_->getCurrentState(0.5);
    if (!current_start_state) {
      current_start_state = build_fallback_current_state(*arm_group_);
    }
    std::vector<ExternalCandidateJob> jobs;
    jobs.reserve(proposal_candidate_count + pose_candidate_count + (have_single_candidate ? 1U : 0U));

    std::vector<ExternalCandidateJob> proposal_jobs;
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
        proposal_jobs.reserve(proposal_candidate_count);
        for (std::size_t index = 0; index < proposal_candidate_count; ++index) {
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
          ExternalCandidateJob job{
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
            metadata,
          };
          if (external_first_round_rerank_enabled_) {
            job.first_round_score = compute_external_first_round_score(metadata, target_point);
          }
          proposal_jobs.push_back(std::move(job));
        }
      }
    }

    if (!proposal_jobs.empty() && external_first_round_rerank_enabled_) {
      std::stable_sort(
        proposal_jobs.begin(),
        proposal_jobs.end(),
        [](const ExternalCandidateJob & lhs, const ExternalCandidateJob & rhs) {
          return lhs.first_round_score > rhs.first_round_score;
        });
      const std::size_t preview_count = std::min<std::size_t>(3U, proposal_jobs.size());
      std::ostringstream score_stream;
      score_stream << "external first-round rerank:";
      for (std::size_t index = 0; index < preview_count; ++index) {
        score_stream << " [" << proposal_jobs[index].candidate_index <<
          " score=" << std::fixed << std::setprecision(2) << proposal_jobs[index].first_round_score << "]";
      }
      RCLCPP_INFO(this->get_logger(), "%s", score_stream.str().c_str());
    }

    if (
      external_screening_candidate_limit_ > 0 &&
      proposal_jobs.size() > static_cast<std::size_t>(external_screening_candidate_limit_))
    {
      RCLCPP_INFO(
        this->get_logger(),
        "limiting external grasp proposal screening to top %d of %zu reranked candidates",
        external_screening_candidate_limit_,
        proposal_jobs.size());
      proposal_jobs.resize(static_cast<std::size_t>(external_screening_candidate_limit_));
    }

    jobs.insert(jobs.end(), proposal_jobs.begin(), proposal_jobs.end());

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
        for (std::size_t index = 0; index < pose_candidate_count; ++index) {
          const auto & pregrasp_pose = external_pregrasp_pose_array.poses[index];
          const auto & grasp_pose = external_grasp_pose_array.poses[index];
          jobs.push_back(ExternalCandidateJob{
            ApproachCandidate{
              {pregrasp_pose.position.x, pregrasp_pose.position.y, pregrasp_pose.position.z},
              {grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z},
              pregrasp_pose.orientation,
              grasp_pose.orientation,
            },
            "external candidate " + std::to_string(index + 1),
            index + 1,
            std::nullopt,
          });
        }
      }
    }

    if (
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
    if (!have_proposal_array && !have_pose_arrays && have_single_candidate) {
      jobs.push_back(ExternalCandidateJob{
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
        1,
        std::nullopt,
        0.0,
      });
    }

    const std::size_t candidates_received = jobs.size();

    return evaluate_external_jobs(
      current_pose,
      target_point,
      attempts,
      current_start_state,
      candidates_received,
      std::move(jobs));
  }

  std::size_t effective_parallel_worker_count(std::size_t job_count) const
  {
    if (
      !parallel_candidate_evaluation_enabled_ ||
      job_count < static_cast<std::size_t>(std::max(parallel_candidate_min_jobs_, 1)))
    {
      return 1U;
    }
    return std::max<std::size_t>(
      1U,
      std::min<std::size_t>(
        static_cast<std::size_t>(std::max(parallel_candidate_worker_count_, 1)),
      std::min(job_count, planning_workers_.size())));
  }

  QuickIkPrefilterResult run_external_quick_ik_prefilter(
    moveit::planning_interface::MoveGroupInterface & planning_group,
    const moveit::core::RobotStatePtr & current_start_state,
    const std::vector<StageVariant> & stage_variants,
    const std::vector<PregraspVariant> & variants,
    const std::vector<JointConstraintAttempt> & attempts) const
  {
    if (!quick_ik_prefilter_enabled_ || stage_variants.empty() || variants.empty() || attempts.empty()) {
      return QuickIkPrefilterResult{};
    }

    const auto robot_model = planning_group.getRobotModel();
    const auto * joint_model_group =
      robot_model ? robot_model->getJointModelGroup(arm_group_name_) : nullptr;
    if (!robot_model || joint_model_group == nullptr) {
      return QuickIkPrefilterResult{};
    }

    if (!external_stage_enabled_) {
      const std::size_t total_jobs = variants.size() * attempts.size();
      return run_quick_ik_prefilter_jobs(
        total_jobs,
        [&](std::size_t job_index) -> bool {
          const std::size_t variant_index = job_index % variants.size();
          job_index /= variants.size();
          const std::size_t attempt_index = job_index % attempts.size();

          const auto & variant = variants[variant_index];
          const auto & attempt = attempts[attempt_index];

          moveit::core::RobotStatePtr pregrasp_state;
          if (!solve_quick_ik_pose(
                robot_model,
                joint_model_group,
                current_start_state,
                variant.pregrasp,
                variant.pregrasp_orientation,
                attempt,
                true,
                &pregrasp_state))
          {
            return false;
          }

          return solve_quick_ik_pose(
            robot_model,
            joint_model_group,
            pregrasp_state,
            variant.grasp,
            variant.grasp_orientation,
            attempt,
            false,
            nullptr);
        });
    }

    const std::size_t total_jobs =
      stage_variants.size() * variants.size() * attempts.size();
    return run_quick_ik_prefilter_jobs(
      total_jobs,
      [&](std::size_t job_index) -> bool {
        const std::size_t stage_index = job_index % stage_variants.size();
        job_index /= stage_variants.size();
        const std::size_t variant_index = job_index % variants.size();
        job_index /= variants.size();
        const std::size_t attempt_index = job_index % attempts.size();

        const auto & stage_variant = stage_variants[stage_index];
        const auto & variant = variants[variant_index];
        const auto & attempt = attempts[attempt_index];

        moveit::core::RobotStatePtr stage_state;
        if (!solve_quick_ik_pose(
              robot_model,
              joint_model_group,
              current_start_state,
              stage_variant.stage,
              variant.pregrasp_orientation,
              attempt,
              true,
              &stage_state))
        {
          return false;
        }

        moveit::core::RobotStatePtr pregrasp_state;
        if (!solve_quick_ik_pose(
              robot_model,
              joint_model_group,
              stage_state,
              variant.pregrasp,
              variant.pregrasp_orientation,
              attempt,
              false,
              &pregrasp_state))
        {
          return false;
        }

        return solve_quick_ik_pose(
          robot_model,
          joint_model_group,
          pregrasp_state,
          variant.grasp,
          variant.grasp_orientation,
          attempt,
          false,
          nullptr);
      });
  }

  QuickIkPrefilterResult run_internal_quick_ik_prefilter(
    moveit::planning_interface::MoveGroupInterface & planning_group,
    const moveit::core::RobotStatePtr & current_start_state,
    const std::vector<StageVariant> & stage_variants,
    const std::vector<PregraspVariant> & pregrasp_variants,
    const std::vector<JointConstraintAttempt> & attempts) const
  {
    if (
      !quick_ik_prefilter_enabled_ || stage_variants.empty() || pregrasp_variants.empty() ||
      attempts.empty())
    {
      return QuickIkPrefilterResult{};
    }

    const auto robot_model = planning_group.getRobotModel();
    const auto * joint_model_group =
      robot_model ? robot_model->getJointModelGroup(arm_group_name_) : nullptr;
    if (!robot_model || joint_model_group == nullptr) {
      return QuickIkPrefilterResult{};
    }

    const std::size_t total_jobs =
      stage_variants.size() * pregrasp_variants.size() * attempts.size();
    return run_quick_ik_prefilter_jobs(
      total_jobs,
      [&](std::size_t job_index) -> bool {
        const std::size_t stage_index = job_index % stage_variants.size();
        job_index /= stage_variants.size();
        const std::size_t variant_index = job_index % pregrasp_variants.size();
        job_index /= pregrasp_variants.size();
        const std::size_t attempt_index = job_index % attempts.size();

        const auto & stage_variant = stage_variants[stage_index];
        const auto & variant = pregrasp_variants[variant_index];
        const auto & attempt = attempts[attempt_index];

        moveit::core::RobotStatePtr stage_state;
        if (!solve_quick_ik_pose(
              robot_model,
              joint_model_group,
              current_start_state,
              stage_variant.stage,
              variant.pregrasp_orientation,
              attempt,
              true,
              &stage_state))
        {
          return false;
        }

        moveit::core::RobotStatePtr pregrasp_state;
        if (!solve_quick_ik_pose(
              robot_model,
              joint_model_group,
              stage_state,
              variant.pregrasp,
              variant.pregrasp_orientation,
              attempt,
              false,
              &pregrasp_state))
        {
          return false;
        }

        return solve_quick_ik_pose(
          robot_model,
          joint_model_group,
          pregrasp_state,
          variant.grasp,
          variant.grasp_orientation,
          attempt,
          false,
          nullptr);
      });
  }

  ExternalCandidateEvaluationResult evaluate_external_candidate_with_group(
    moveit::planning_interface::MoveGroupInterface & planning_group,
    const moveit::core::RobotStatePtr & current_start_state,
    const geometry_msgs::msg::Pose & current_pose,
    const std::array<double, 3> & target_point,
    const std::vector<JointConstraintAttempt> & attempts,
    const ExternalCandidateJob & job)
  {
    ExternalCandidateEvaluationResult result;
    const auto seed_variants = build_external_pregrasp_variants(
      current_pose,
      job.candidate,
      job.external_metadata);
    const auto & variants =
      job.second_round_variants.empty() ? seed_variants : job.second_round_variants;
    const auto stage_variants = build_stage_variants(current_pose, target_point);
    const bool skip_stage = !external_stage_enabled_;
    const ReducedOrientationConstraint reduced_orientation_constraint =
      job.external_metadata.has_value() ? build_external_reduced_orientation_constraint() :
      ReducedOrientationConstraint{};
    const double reduced_pregrasp_planning_time_sec =
      reduced_orientation_constraint.enabled ?
      std::max(candidate_screening_planning_time_sec_, 0.35) :
      candidate_screening_planning_time_sec_;
    const int reduced_pregrasp_planning_attempts =
      reduced_orientation_constraint.enabled ?
      std::max(candidate_screening_planning_attempts_, 2) :
      candidate_screening_planning_attempts_;
    const double stage_planning_time_sec = candidate_screening_planning_time_sec_;
    const int stage_planning_attempts = candidate_screening_planning_attempts_;

    result.debug_info.candidate_index = job.candidate_index;
    result.debug_info.candidate_label = job.candidate_label;
    result.debug_info.status = "rejected";
    result.debug_info.stage =
      (skip_stage || stage_variants.empty()) ? variants.front().pregrasp : stage_variants.front().stage;
    result.debug_info.pregrasp = variants.front().pregrasp;
    result.debug_info.grasp = variants.front().grasp;
    result.debug_info.external_metadata = job.external_metadata;

    const QuickIkPrefilterResult quick_ik_result = run_external_quick_ik_prefilter(
      planning_group,
      current_start_state,
      stage_variants,
      variants,
      attempts);
    result.debug_info.quick_ik_hits = quick_ik_result.hits;
    result.debug_info.quick_ik_samples = quick_ik_result.samples;
    if (!quick_ik_result.passed) {
      result.debug_info.status = "ik prefilter fail";
      return result;
    }

    for (const auto & variant : variants) {
      const std::string pregrasp_label =
        "move to pregrasp " + job.candidate_label + variant.label_suffix;

      const auto evaluate_attempt = [&](const StageVariant * stage_variant_ptr, const JointConstraintAttempt & attempt)
        -> bool {
          const std::array<double, 3> stage_point =
            stage_variant_ptr ? stage_variant_ptr->stage : variant.pregrasp;
          const std::string stage_label =
            stage_variant_ptr ?
            ("move to stage " + job.candidate_label + stage_variant_ptr->label_suffix) :
            ("skip stage " + job.candidate_label + variant.label_suffix);

          ++result.tested_options;
          moveit::planning_interface::MoveGroupInterface::Plan stage_plan;
          moveit::core::RobotStatePtr stage_end_state = current_start_state;
          if (stage_variant_ptr != nullptr) {
            if (!plan_pose_target_attempt(
                planning_group,
                stage_variant_ptr->stage,
                variant.pregrasp_orientation,
                attempt,
                stage_plan,
                nullptr,
                stage_position_only_target_,
                stage_planning_time_sec,
                stage_planning_attempts,
                make_no_reduced_orientation_constraint()))
            {
              ++result.stage_plan_failures;
              return false;
            }

            stage_end_state = build_plan_end_state(stage_plan, current_start_state);
            if (!stage_end_state) {
              ++result.stage_end_state_failures;
              return false;
            }
          }

          moveit::planning_interface::MoveGroupInterface::Plan pregrasp_plan;
          if (!plan_pose_target_attempt(
              planning_group,
              variant.pregrasp,
              variant.pregrasp_orientation,
              attempt,
              pregrasp_plan,
              stage_end_state,
              pregrasp_position_only_target_,
              reduced_pregrasp_planning_time_sec,
              reduced_pregrasp_planning_attempts,
              reduced_orientation_constraint))
          {
            ++result.pregrasp_plan_failures;
            return false;
          }

          const auto pregrasp_end_state = build_plan_end_state(pregrasp_plan, stage_end_state);
          if (!pregrasp_end_state) {
            ++result.pregrasp_end_state_failures;
            return false;
          }

          double camera_penalty = 0.0;
          double camera_stability_bonus = 0.0;
          double primary_joint_excursion_cost = 0.0;
          double secondary_joint_excursion_cost = 0.0;
          double wrist_joint_excursion_cost = 0.0;
          const double base_score = compute_pregrasp_option_score(
            current_start_state,
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
          double external_semantic_score = 0.0;
          double external_confidence_score = 0.0;
          const double external_proposal_bonus = compute_external_proposal_bonus(
            job.external_metadata,
            &external_semantic_score,
            &external_confidence_score);
          const double score = base_score + external_proposal_bonus;

          ++result.feasible_options;
          result.debug_info.feasible = true;
          result.debug_info.status = "feasible";
          result.debug_info.variant_label =
            variant.label_suffix.empty() ? "base" : variant.label_suffix;
          result.debug_info.stage = stage_point;
          result.debug_info.pregrasp = variant.pregrasp;
          result.debug_info.grasp = variant.grasp;
          result.debug_info.score = score;
          result.debug_info.semantic_score = external_semantic_score;
          result.debug_info.confidence_score = external_confidence_score;
          result.debug_info.proposal_bonus = external_proposal_bonus;

          result.best_option = PlannedApproachOption{
            job.candidate_index,
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
            external_semantic_score,
            external_confidence_score,
            external_proposal_bonus,
            job.external_metadata,
            skip_stage,
            reduced_orientation_constraint,
          };
          return true;
        };

      if (skip_stage) {
        for (const auto & attempt : attempts) {
          if (evaluate_attempt(nullptr, attempt)) {
            break;
          }
        }
      } else {
        for (const auto & stage_variant : stage_variants) {
          for (const auto & attempt : attempts) {
            if (evaluate_attempt(&stage_variant, attempt)) {
              break;
            }
          }
          if (result.best_option.has_value()) {
            break;
          }
        }
      }
      if (result.best_option.has_value()) {
        break;
      }
    }

    result.debug_info.stage_plan_failures = result.stage_plan_failures;
    result.debug_info.stage_end_state_failures = result.stage_end_state_failures;
    result.debug_info.pregrasp_plan_failures = result.pregrasp_plan_failures;
    result.debug_info.pregrasp_end_state_failures = result.pregrasp_end_state_failures;
    if (!result.best_option.has_value()) {
      if (result.stage_end_state_failures > 0) {
        result.debug_info.status = "stage end-state fail";
      } else if (result.stage_plan_failures > 0) {
        result.debug_info.status = "stage plan fail";
      } else if (result.pregrasp_end_state_failures > 0) {
        result.debug_info.status = "pregrasp end-state fail";
      } else if (result.pregrasp_plan_failures > 0) {
        result.debug_info.status = "pregrasp plan fail";
      } else {
        result.debug_info.status = "not executable";
      }
    }

    return result;
  }

  InternalCandidateEvaluationResult evaluate_internal_candidate_with_group(
    moveit::planning_interface::MoveGroupInterface & planning_group,
    const moveit::core::RobotStatePtr & current_start_state,
    const geometry_msgs::msg::Pose & current_pose,
    const std::array<double, 3> & target_point,
    const std::vector<StageVariant> & stage_variants,
    const std::vector<JointConstraintAttempt> & attempts,
    const InternalCandidateJob & job)
  {
    InternalCandidateEvaluationResult result;
    const auto pregrasp_variants = build_local_pregrasp_variants(job.candidate);
    const double stage_planning_time_sec = candidate_screening_planning_time_sec_;
    const int stage_planning_attempts = candidate_screening_planning_attempts_;
    const double pregrasp_planning_time_sec = candidate_screening_planning_time_sec_;
    const int pregrasp_planning_attempts = candidate_screening_planning_attempts_;

    const QuickIkPrefilterResult quick_ik_result = run_internal_quick_ik_prefilter(
      planning_group,
      current_start_state,
      stage_variants,
      pregrasp_variants,
      attempts);
    result.quick_ik_hits = quick_ik_result.hits;
    result.quick_ik_samples = quick_ik_result.samples;
    if (!quick_ik_result.passed) {
      result.quick_ik_prefilter_rejected = true;
      return result;
    }

    for (const auto & variant : pregrasp_variants) {
      const std::string pregrasp_label =
        "move to pregrasp candidate " + std::to_string(job.candidate_index) +
        variant.label_suffix;

      for (const auto & stage_variant : stage_variants) {
        const std::string stage_label =
          "move to stage candidate " + std::to_string(job.candidate_index) +
          stage_variant.label_suffix;

        for (const auto & attempt : attempts) {
          ++result.tested_options;
          moveit::planning_interface::MoveGroupInterface::Plan stage_plan;
          if (!plan_pose_target_attempt(
              planning_group,
              stage_variant.stage,
              variant.pregrasp_orientation,
              attempt,
              stage_plan,
              nullptr,
              stage_position_only_target_,
              stage_planning_time_sec,
              stage_planning_attempts,
              make_no_reduced_orientation_constraint()))
          {
            continue;
          }

          const auto stage_end_state = build_plan_end_state(stage_plan, current_start_state);
          if (!stage_end_state) {
            continue;
          }

          moveit::planning_interface::MoveGroupInterface::Plan pregrasp_plan;
          if (!plan_pose_target_attempt(
              planning_group,
              variant.pregrasp,
              variant.pregrasp_orientation,
              attempt,
              pregrasp_plan,
              stage_end_state,
              pregrasp_position_only_target_,
              pregrasp_planning_time_sec,
              pregrasp_planning_attempts,
              make_no_reduced_orientation_constraint()))
          {
            continue;
          }

          const auto pregrasp_end_state = build_plan_end_state(pregrasp_plan, stage_end_state);
          if (!pregrasp_end_state) {
            continue;
          }

          double camera_penalty = 0.0;
          double camera_stability_bonus = 0.0;
          double primary_joint_excursion_cost = 0.0;
          double secondary_joint_excursion_cost = 0.0;
          double wrist_joint_excursion_cost = 0.0;
          const double score = compute_pregrasp_option_score(
            current_start_state,
            current_pose,
            target_point,
            stage_variant.stage,
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
          result.best_option = PlannedApproachOption{
            job.candidate_index,
            stage_variant.stage,
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
            false,
            make_no_reduced_orientation_constraint(),
          };
          return result;
        }
      }
    }

    return result;
  }

  std::optional<PlannedApproachOption> evaluate_external_jobs(
    const geometry_msgs::msg::Pose & current_pose,
    const std::array<double, 3> & target_point,
    const std::vector<JointConstraintAttempt> & attempts,
    const moveit::core::RobotStatePtr & current_start_state,
    std::size_t candidates_received,
    std::vector<ExternalCandidateJob> jobs)
  {
    std::optional<PlannedApproachOption> best_option;
    std::size_t candidates_tested = 0;
    std::size_t candidates_feasible = 0;
    std::size_t stage_plan_failures = 0;
    std::size_t stage_end_state_failures = 0;
    std::size_t pregrasp_plan_failures = 0;
    std::size_t pregrasp_end_state_failures = 0;
    std::vector<ExternalEvaluationDebugInfo> debug_entries;

    if (!jobs.empty()) {
      const auto job_key = [](const ExternalCandidateJob & job) {
          return std::make_pair(job.candidate_index, job.candidate_label);
        };

      if (external_second_round_rerank_enabled_) {
        const std::vector<ExternalCandidateJob> geometry_ranked_jobs = jobs;
        std::vector<ExternalSecondRoundScore> second_round_scores(jobs.size());
        const std::size_t worker_count = effective_parallel_worker_count(jobs.size());
        auto score_job = [&](std::size_t job_index, moveit::planning_interface::MoveGroupInterface & planning_group) {
            auto & job = jobs[job_index];
            if (!job.external_metadata.has_value()) {
              job.second_round_score = job.first_round_score;
              second_round_scores[job_index].total_score = job.second_round_score;
              return;
            }

            const auto seed_variants = build_external_pregrasp_variants(
              current_pose,
              job.candidate,
              job.external_metadata);
            std::vector<PregraspVariant> selected_planning_variants;
            const auto append_unique_variants =
              [&](const std::vector<ExternalQuickIkRegionSample> & source_samples,
              const ExternalSecondRoundScore & source_score)
              {
                for (const std::size_t sample_index : source_score.top_hit_sample_indices) {
                  if (sample_index >= source_samples.size()) {
                    continue;
                  }
                  const auto & candidate_variant = source_samples[sample_index].variant;
                  const auto duplicate_it = std::find_if(
                    selected_planning_variants.begin(),
                    selected_planning_variants.end(),
                    [&](const PregraspVariant & existing_variant) {
                      const double pregrasp_distance = std::sqrt(
                        std::pow(existing_variant.pregrasp[0] - candidate_variant.pregrasp[0], 2) +
                        std::pow(existing_variant.pregrasp[1] - candidate_variant.pregrasp[1], 2) +
                        std::pow(existing_variant.pregrasp[2] - candidate_variant.pregrasp[2], 2));
                      const double grasp_distance = std::sqrt(
                        std::pow(existing_variant.grasp[0] - candidate_variant.grasp[0], 2) +
                        std::pow(existing_variant.grasp[1] - candidate_variant.grasp[1], 2) +
                        std::pow(existing_variant.grasp[2] - candidate_variant.grasp[2], 2));
                      return pregrasp_distance < 0.004 && grasp_distance < 0.002;
                    });
                  if (duplicate_it != selected_planning_variants.end()) {
                    continue;
                  }
                  selected_planning_variants.push_back(candidate_variant);
                  if (
                    selected_planning_variants.size() >=
                    static_cast<std::size_t>(std::max(external_second_round_planning_variant_limit_, 1)))
                  {
                    break;
                  }
                }
              };
            const auto coarse_samples = build_external_second_round_region_samples(
              seed_variants,
              job.external_metadata.value(),
              1.0,
              1.0);
            const auto coarse_score = compute_external_second_round_score(
              planning_group,
              current_start_state,
              attempts,
              job.external_metadata.value(),
              coarse_samples);
            auto combined_score = coarse_score;
            append_unique_variants(coarse_samples, coarse_score);

            if (external_second_round_refine_enabled_ && combined_score.has_hit) {
              std::vector<PregraspVariant> refine_seed_variants;
              refine_seed_variants.reserve(
                static_cast<std::size_t>(std::max(external_second_round_refine_top_seed_count_, 1)));
              for (const std::size_t sample_index : combined_score.top_hit_sample_indices) {
                if (sample_index >= coarse_samples.size()) {
                  continue;
                }
                refine_seed_variants.push_back(coarse_samples[sample_index].variant);
                if (
                  refine_seed_variants.size() >=
                  static_cast<std::size_t>(std::max(external_second_round_refine_top_seed_count_, 1)))
                {
                  break;
                }
              }
              if (!refine_seed_variants.empty()) {
                const auto refine_samples = build_external_second_round_region_samples(
                  refine_seed_variants,
                  job.external_metadata.value(),
                  external_second_round_refine_scale_,
                  external_second_round_refine_scale_);
                const auto refine_score = compute_external_second_round_score(
                  planning_group,
                  current_start_state,
                  attempts,
                  job.external_metadata.value(),
                  refine_samples);
                append_unique_variants(refine_samples, refine_score);
                combined_score = combine_external_second_round_scores(combined_score, refine_score);
              }
            }

            second_round_scores[job_index] = combined_score;
            job.second_round_score = combined_score.total_score;
            job.second_round_hits = combined_score.hits;
            job.second_round_samples = combined_score.samples;
            job.second_round_has_hit = combined_score.has_hit;
            if (!selected_planning_variants.empty()) {
              for (const auto & seed_variant : seed_variants) {
                if (
                  selected_planning_variants.size() >=
                  static_cast<std::size_t>(std::max(external_second_round_planning_variant_limit_, 1)))
                {
                  break;
                }
                const auto duplicate_it = std::find_if(
                  selected_planning_variants.begin(),
                  selected_planning_variants.end(),
                  [&](const PregraspVariant & existing_variant) {
                    const double pregrasp_distance = std::sqrt(
                      std::pow(existing_variant.pregrasp[0] - seed_variant.pregrasp[0], 2) +
                      std::pow(existing_variant.pregrasp[1] - seed_variant.pregrasp[1], 2) +
                      std::pow(existing_variant.pregrasp[2] - seed_variant.pregrasp[2], 2));
                    const double grasp_distance = std::sqrt(
                      std::pow(existing_variant.grasp[0] - seed_variant.grasp[0], 2) +
                      std::pow(existing_variant.grasp[1] - seed_variant.grasp[1], 2) +
                      std::pow(existing_variant.grasp[2] - seed_variant.grasp[2], 2));
                    return pregrasp_distance < 0.004 && grasp_distance < 0.002;
                  });
                if (duplicate_it != selected_planning_variants.end()) {
                  continue;
                }
                selected_planning_variants.push_back(seed_variant);
              }
            }
            job.second_round_variants = std::move(selected_planning_variants);
          };

        if (worker_count > 1U) {
          std::atomic_size_t next_job{0};
          std::vector<std::future<void>> futures;
          futures.reserve(worker_count);
          for (std::size_t worker_index = 0; worker_index < worker_count; ++worker_index) {
            futures.push_back(std::async(
                std::launch::async,
                [&, worker_index]() {
                  auto & planning_group = *planning_workers_[worker_index].arm_group;
                  for (;;) {
                    const std::size_t job_index = next_job.fetch_add(1);
                    if (job_index >= jobs.size()) {
                      break;
                    }
                    score_job(job_index, planning_group);
                  }
                }));
          }
          for (auto & future : futures) {
            future.get();
          }
        } else {
          for (std::size_t job_index = 0; job_index < jobs.size(); ++job_index) {
            score_job(job_index, *arm_group_);
          }
        }

        std::stable_sort(
          jobs.begin(),
          jobs.end(),
          [](const ExternalCandidateJob & lhs, const ExternalCandidateJob & rhs) {
            return lhs.second_round_score > rhs.second_round_score;
          });

        const std::size_t preview_count = std::min<std::size_t>(3U, jobs.size());
        std::ostringstream score_stream;
        score_stream << "external second-round rerank:";
        for (std::size_t index = 0; index < preview_count; ++index) {
          score_stream << " [" << jobs[index].candidate_index
                       << " score=" << std::fixed << std::setprecision(2) << jobs[index].second_round_score
                       << " hits=" << jobs[index].second_round_hits << "/" << jobs[index].second_round_samples
                       << "]";
        }
        RCLCPP_INFO(this->get_logger(), "%s", score_stream.str().c_str());

        if (external_second_round_candidate_limit_ > 0) {
          std::vector<ExternalCandidateJob> selected_jobs;
          const std::size_t primary_limit = std::min<std::size_t>(
            static_cast<std::size_t>(external_second_round_candidate_limit_),
            jobs.size());
          selected_jobs.insert(
            selected_jobs.end(),
            jobs.begin(),
            jobs.begin() + static_cast<std::ptrdiff_t>(primary_limit));

          if (external_second_round_keep_geometry_fallback_) {
            int fallbacks_added = 0;
            for (const auto & geometry_job : geometry_ranked_jobs) {
              if (fallbacks_added >= external_second_round_geometry_fallback_count_) {
                break;
              }
              const auto selected_it = std::find_if(
                selected_jobs.begin(),
                selected_jobs.end(),
                [&](const ExternalCandidateJob & selected_job) {
                  return job_key(selected_job) == job_key(geometry_job);
                });
              if (selected_it != selected_jobs.end()) {
                continue;
              }
              const auto scored_it = std::find_if(
                jobs.begin(),
                jobs.end(),
                [&](const ExternalCandidateJob & scored_job) {
                  return job_key(scored_job) == job_key(geometry_job);
                });
              if (scored_it != jobs.end() && scored_it->second_round_has_hit) {
                continue;
              }
              selected_jobs.push_back(geometry_job);
              ++fallbacks_added;
            }
          }

          jobs = std::move(selected_jobs);
        }
      }

      std::vector<ExternalCandidateEvaluationResult> results(jobs.size());
      const std::size_t worker_count = effective_parallel_worker_count(jobs.size());
      if (worker_count > 1) {
        std::atomic_size_t next_job{0};
        std::vector<std::future<void>> futures;
        futures.reserve(worker_count);
        for (std::size_t worker_index = 0; worker_index < worker_count; ++worker_index) {
          futures.push_back(std::async(
              std::launch::async,
              [&, worker_index]() {
                auto & planning_group = *planning_workers_[worker_index].arm_group;
                for (;;) {
                  const std::size_t job_index = next_job.fetch_add(1);
                  if (job_index >= jobs.size()) {
                    break;
                  }
                  results[job_index] = evaluate_external_candidate_with_group(
                    planning_group,
                    current_start_state,
                    current_pose,
                    target_point,
                    attempts,
                    jobs[job_index]);
                }
              }));
        }
        for (auto & future : futures) {
          future.get();
        }
      } else {
        for (std::size_t job_index = 0; job_index < jobs.size(); ++job_index) {
          results[job_index] = evaluate_external_candidate_with_group(
            *arm_group_,
            current_start_state,
            current_pose,
            target_point,
            attempts,
            jobs[job_index]);
        }
      }

      for (std::size_t job_index = 0; job_index < jobs.size(); ++job_index) {
        const auto & result = results[job_index];
        candidates_tested += result.tested_options;
        candidates_feasible += result.best_option.has_value() ? 1U : 0U;
        stage_plan_failures += result.stage_plan_failures;
        stage_end_state_failures += result.stage_end_state_failures;
        pregrasp_plan_failures += result.pregrasp_plan_failures;
        pregrasp_end_state_failures += result.pregrasp_end_state_failures;
        debug_entries.push_back(result.debug_info);
        if (!best_option.has_value() ||
          (result.best_option.has_value() && result.best_option->score > best_option->score))
        {
          best_option = result.best_option;
        }
        log_planning_progress(
          "planning/external",
          job_index + 1,
          jobs.size(),
          candidates_feasible,
          jobs[job_index].candidate_label,
          result.debug_info.status);
      }
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
    const auto stage_variants = build_stage_variants(current_pose, target_point);
    const auto joint_lock_attempts = get_locked_joint_attempts();
    const std::vector<JointConstraintAttempt> attempts =
      joint_lock_attempts.empty() ? std::vector<JointConstraintAttempt>{JointConstraintAttempt{}} :
      joint_lock_attempts;
    auto current_start_state = arm_group_->getCurrentState(0.5);
    if (!current_start_state) {
      current_start_state = build_fallback_current_state(*arm_group_);
    }

    std::vector<InternalCandidateJob> jobs;
    jobs.reserve(candidates.size());
    for (std::size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index) {
      jobs.push_back(InternalCandidateJob{candidates[candidate_index], candidate_index + 1});
    }

    std::vector<InternalCandidateEvaluationResult> results(jobs.size());
    const std::size_t worker_count = effective_parallel_worker_count(jobs.size());
    if (worker_count > 1) {
      std::atomic_size_t next_job{0};
      std::vector<std::future<void>> futures;
      futures.reserve(worker_count);
      for (std::size_t worker_index = 0; worker_index < worker_count; ++worker_index) {
        futures.push_back(std::async(
            std::launch::async,
            [&, worker_index]() {
              auto & planning_group = *planning_workers_[worker_index].arm_group;
              for (;;) {
                const std::size_t job_index = next_job.fetch_add(1);
                if (job_index >= jobs.size()) {
                  break;
                }
                results[job_index] = evaluate_internal_candidate_with_group(
                  planning_group,
                  current_start_state,
                  current_pose,
                  target_point,
                  stage_variants,
                  attempts,
                  jobs[job_index]);
              }
            }));
      }
      for (auto & future : futures) {
        future.get();
      }
    } else {
      for (std::size_t job_index = 0; job_index < jobs.size(); ++job_index) {
        results[job_index] = evaluate_internal_candidate_with_group(
          *arm_group_,
          current_start_state,
          current_pose,
          target_point,
          stage_variants,
          attempts,
          jobs[job_index]);
      }
    }

    for (std::size_t job_index = 0; job_index < jobs.size(); ++job_index) {
      const auto & result = results[job_index];
      evaluation.tested_options += result.tested_options;
      if (result.best_option.has_value()) {
        ++evaluation.feasible_count;
        evaluation.feasible_options.push_back(result.best_option.value());
      }
      const std::string internal_status =
        result.best_option.has_value() ? "feasible" :
        (result.quick_ik_prefilter_rejected ?
        ("ik prefilter fail " + std::to_string(result.quick_ik_hits) + "/" +
        std::to_string(result.quick_ik_samples)) :
        "rejected");
      log_planning_progress(
        "planning/internal",
        job_index + 1,
        jobs.size(),
        evaluation.feasible_count,
        "candidate " + std::to_string(jobs[job_index].candidate_index),
        internal_status);
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
    if (require_external_grasp_candidates_ && !allow_internal_fallback_for_current_pick_) {
      RCLCPP_WARN(
        this->get_logger(),
        "external grasp candidates are required; aborting without internal fallback");
      return std::nullopt;
    }
    if (allow_internal_fallback_for_current_pick_) {
      RCLCPP_WARN(
        this->get_logger(),
        "external grasp candidates unavailable; using internal fallback planner");
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
  std::string target_candidate_visible_topic_;
  std::string refresh_grasp_candidates_topic_;
  std::string pick_active_topic_;
  std::string pick_status_topic_;
  std::string execute_pick_service_;
  std::string reset_pick_session_service_;
  std::string return_home_service_;
  std::string return_named_target_{"home"};
  std::string home_gripper_named_target_{"open"};
  bool apply_home_gripper_named_target_on_startup_{true};
  bool return_home_allow_named_target_fallback_{false};
  std::vector<int64_t> return_home_joint_ids_;
  std::vector<double> return_home_joint_angles_deg_;
  int return_home_duration_ms_{3500};
  bool return_home_wait_for_completion_{false};
  int return_home_request_timeout_ms_{12000};
  std::string control_arm_move_joints_service_{"/control/arm/move_joints"};
  int control_gripper_joint_id_{6};
  int control_gripper_duration_ms_{1500};
  bool control_gripper_wait_{true};
  std::string selected_pregrasp_pose_topic_;
  std::string selected_grasp_pose_topic_;
  std::string external_pregrasp_pose_array_topic_;
  std::string external_grasp_pose_array_topic_;
  std::string external_grasp_proposal_array_topic_;
  std::string execution_debug_markers_topic_;
  std::string joint_states_topic_;
  std::string base_frame_;
  std::string ee_link_;
  std::string arm_group_name_;
  std::string gripper_group_name_;
  double planning_time_sec_{5.0};
  int planning_attempts_{10};
  double candidate_screening_planning_time_sec_{0.3};
  int candidate_screening_planning_attempts_{1};
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
  bool external_orientation_sampling_enabled_{true};
  int external_orientation_sample_count_{11};
  double external_orientation_sample_max_deg_{30.0};
  bool external_reduced_orientation_constraint_enabled_{true};
  bool external_stage_enabled_{true};
  double external_reduced_free_axis_tolerance_deg_{180.0};
  double external_reduced_constrained_axis_tolerance_deg_{20.0};
  int external_screening_candidate_limit_{6};
  bool external_first_round_rerank_enabled_{true};
  double external_first_round_network_weight_{0.20};
  double external_first_round_table_weight_{0.20};
  double external_first_round_contact_weight_{0.20};
  double external_first_round_com_weight_{0.20};
  double external_first_round_width_weight_{0.10};
  double external_first_round_surface_weight_{0.10};
  bool external_second_round_rerank_enabled_{true};
  int external_second_round_candidate_limit_{3};
  bool external_second_round_keep_geometry_fallback_{true};
  int external_second_round_geometry_fallback_count_{1};
  double external_second_round_standoff_delta_m_{0.020};
  double external_second_round_lateral_delta_m_{0.018};
  double external_second_round_vertical_delta_m_{0.010};
  double external_second_round_roll_slack_deg_{10.0};
  double external_second_round_pitch_slack_deg_{8.0};
  bool external_second_round_refine_enabled_{true};
  int external_second_round_refine_top_seed_count_{1};
  double external_second_round_refine_scale_{0.5};
  int external_second_round_planning_variant_limit_{4};
  double external_second_round_region_weight_{0.45};
  double external_second_round_deviation_weight_{0.25};
  double external_second_round_joint_weight_{0.15};
  double external_second_round_pregrasp_weight_{0.15};
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
  bool allow_internal_fallback_after_external_timeout_{false};
  double internal_fallback_after_external_timeout_sec_{3.5};
  double refresh_grasp_candidates_retry_sec_{1.5};
  double external_semantic_score_weight_{0.8};
  double external_confidence_score_weight_{0.6};
  bool include_table_collision_object_{true};
  bool include_target_collision_object_{false};
  bool require_user_confirmation_{false};
  bool allow_soft_lock_pregrasp_prep_{true};
  std::vector<double> table_size_m_;
  std::vector<double> table_pose_xyz_;
  bool parallel_candidate_evaluation_enabled_{true};
  int parallel_candidate_worker_count_{3};
  int parallel_candidate_min_jobs_{2};
  bool quick_ik_prefilter_enabled_{true};
  bool quick_ik_prefilter_parallel_enabled_{true};
  double quick_ik_prefilter_timeout_sec_{0.015};
  int quick_ik_prefilter_required_hits_{1};

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  std::vector<PlanningWorker> planning_workers_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_locked_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_candidate_visible_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr selected_pregrasp_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr selected_grasp_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
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
  rclcpp::CallbackGroup::SharedPtr service_client_callback_group_;
  rclcpp::Client<tactile_interfaces::srv::MoveArmJoints>::SharedPtr control_arm_move_joints_client_;
  rclcpp::TimerBase::SharedPtr check_timer_;

  std::mutex target_mutex_;
  std::mutex grasp_candidate_mutex_;
  std::mutex joint_state_mutex_;
  geometry_msgs::msg::PoseStamped latest_target_pose_;
  geometry_msgs::msg::PoseStamped latched_target_pose_;
  geometry_msgs::msg::PoseStamped latest_selected_pregrasp_pose_;
  geometry_msgs::msg::PoseStamped latest_selected_grasp_pose_;
  geometry_msgs::msg::PoseArray latest_external_pregrasp_pose_array_;
  geometry_msgs::msg::PoseArray latest_external_grasp_pose_array_;
  tactile_interfaces::msg::GraspProposalArray latest_external_grasp_proposal_array_;
  sensor_msgs::msg::JointState latest_joint_state_;
  bool has_target_pose_{false};
  bool has_latched_target_pose_{false};
  bool has_selected_pregrasp_pose_{false};
  bool has_selected_grasp_pose_{false};
  bool has_external_pregrasp_pose_array_{false};
  bool has_external_grasp_pose_array_{false};
  bool has_external_grasp_proposal_array_{false};
  bool has_latest_joint_state_{false};
  bool target_locked_{false};
  bool target_candidate_visible_{false};
  bool moveit_ready_{false};
  bool locked_joint_reference_valid_{false};
  double locked_joint_reference_rad_{0.0};
  bool locked_wrist_joint_reference_valid_{false};
  double locked_wrist_joint_reference_rad_{0.0};
  std::atomic_bool pick_armed_{false};
  std::atomic_bool executing_{false};
  std::atomic_bool completed_{false};
  bool waiting_for_final_lock_{false};
  double waiting_for_final_lock_started_sec_{0.0};
  double pick_request_started_sec_{0.0};
  bool allow_internal_fallback_for_current_pick_{false};
  bool allow_stale_external_candidates_for_current_pick_{false};
  bool suppress_target_relock_{false};
  double last_return_home_request_sec_{0.0};
  double manual_home_hold_sec_{30.0};
  double manual_home_hold_until_sec_{0.0};
  double failure_rearm_hold_sec_{4.0};
  double failure_rearm_hold_until_sec_{0.0};
  double latest_external_grasp_update_sec_{0.0};
  double latest_target_pose_update_sec_{0.0};
  double last_grasp_refresh_request_sec_{0.0};
  double soft_lock_final_lock_timeout_sec_{1.0};
  double soft_prep_target_pose_timeout_sec_{1.5};
  bool planning_seed_bias_enabled_{true};
  std::string planning_seed_joint3_name_{"arm_joint3"};
  std::string planning_seed_joint4_name_{"arm_joint4"};
  double planning_seed_joint3_bias_rad_{-0.15};
  double planning_seed_joint4_bias_rad_{0.0};
  std::atomic<std::uint64_t> sequence_token_{1};
  std::atomic<std::uint64_t> armed_sequence_token_{0};
  std::mutex sequence_mutex_;
  std::string interrupted_sequence_reason_;
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
