import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _normalized_secret(value: str) -> str:
    secret = str(value or "").strip()
    if not secret or secret.upper() in {"EMPTY", "NONE", "NULL"}:
        return ""
    return secret


def _normalize_openai_compatible_endpoint(value: str) -> str:
    endpoint = str(value or "").strip()
    if not endpoint:
        return ""
    if endpoint.endswith("/v1"):
        return endpoint + "/chat/completions"
    if endpoint.endswith("/chat/completions"):
        return endpoint
    if endpoint.endswith("/"):
        endpoint = endpoint[:-1]
    if endpoint.endswith("/compatible-mode"):
        return endpoint + "/v1/chat/completions"
    return endpoint


def _load_remote_vlm_file() -> dict[str, str]:
    env_file = os.path.expanduser("~/.config/programme/remote_vlm.env")
    values: dict[str, str] = {}
    try:
        with open(env_file, "r", encoding="utf-8") as handle:
            for raw_line in handle:
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                if line.startswith("export "):
                    line = line[len("export ") :].strip()
                if "=" not in line:
                    continue
                key, value = line.split("=", 1)
                key = key.strip()
                value = value.strip()
                if len(value) >= 2 and value[0] == value[-1] and value[0] in {'"', "'"}:
                    value = value[1:-1]
                if key and value:
                    values[key] = value
    except OSError:
        return {}
    return values


def _remote_vlm_params() -> tuple[dict[str, str], dict[str, str]]:
    file_values = _load_remote_vlm_file()
    api_key = _normalized_secret(
        os.getenv("PROGRAMME_DIALOG_API_KEY")
        or file_values.get("PROGRAMME_DIALOG_API_KEY")
        or os.getenv("PROGRAMME_REMOTE_VLM_API_KEY")
        or file_values.get("PROGRAMME_REMOTE_VLM_API_KEY")
        or os.getenv("DASHSCOPE_API_KEY")
        or file_values.get("DASHSCOPE_API_KEY")
        or os.getenv("OPENAI_API_KEY")
        or file_values.get("OPENAI_API_KEY")
    )
    endpoint = _normalize_openai_compatible_endpoint(
        os.getenv("PROGRAMME_DIALOG_MODEL_ENDPOINT")
        or file_values.get("PROGRAMME_DIALOG_MODEL_ENDPOINT")
        or os.getenv("PROGRAMME_REMOTE_VLM_ENDPOINT")
        or file_values.get("PROGRAMME_REMOTE_VLM_ENDPOINT")
        or os.getenv("DASHSCOPE_BASE_URL")
        or file_values.get("DASHSCOPE_BASE_URL")
        or os.getenv("OPENAI_BASE_URL")
        or file_values.get("OPENAI_BASE_URL")
    )
    model_name = str(
        os.getenv("PROGRAMME_DIALOG_MODEL_NAME")
        or file_values.get("PROGRAMME_DIALOG_MODEL_NAME")
        or os.getenv("PROGRAMME_REMOTE_VLM_MODEL")
        or file_values.get("PROGRAMME_REMOTE_VLM_MODEL")
        or os.getenv("DASHSCOPE_MODEL")
        or file_values.get("DASHSCOPE_MODEL")
        or os.getenv("OPENAI_MODEL")
        or file_values.get("OPENAI_MODEL")
        or ""
    ).strip()

    if api_key and not endpoint:
        endpoint = "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions"
    if api_key and not model_name:
        model_name = "qwen-vl-max-latest"

    semantic_params: dict[str, str] = {}
    web_params: dict[str, str] = {}
    if endpoint:
        semantic_params["model_endpoint"] = endpoint
        web_params["dialog_model_endpoint"] = endpoint
    if model_name:
        semantic_params["model_name"] = model_name
        web_params["dialog_model_name"] = model_name
    if api_key:
        semantic_params["api_key"] = api_key
        web_params["dialog_api_key"] = api_key
    return semantic_params, web_params


def generate_launch_description() -> LaunchDescription:
    core_info_args = ["--ros-args", "--log-level", "info"]
    pick_info_args = [
        "--ros-args",
        "--log-level",
        "info",
        "--log-level",
        "sim_pick_task_node.moveit:=error",
        "--log-level",
        "rcl.logging_rosout:=error",
    ]
    respawn_kwargs = {"respawn": True, "respawn_delay": 2.0}

    default_param_file = PathJoinSubstitution(
        [FindPackageShare("tactile_bringup"), "config", "programme_grasp_stack.yaml"]
    )
    moveit_demo_launch = PathJoinSubstitution(
        [FindPackageShare("tactile_moveit_config"), "launch", "moveit_gazebo_demo.launch.py"]
    )

    stack_param_file_arg = DeclareLaunchArgument(
        "stack_param_file",
        default_value=default_param_file,
        description="Path to the Programme grasp stack parameter YAML",
    )
    search_start_delay_sec_arg = DeclareLaunchArgument(
        "search_start_delay_sec",
        default_value="12.0",
        description="Delay the search sweep until simulation topics and controllers are stable",
    )
    pick_start_delay_sec_arg = DeclareLaunchArgument(
        "pick_start_delay_sec",
        default_value="16.0",
        description="Delay the pick task until modular perception is stable",
    )
    shadow_only_mode_arg = DeclareLaunchArgument(
        "shadow_only_mode",
        default_value="false",
        description="Run only the GG-CNN shadow chain and visualization, without formal grasp backend or pick execution",
    )
    start_ggcnn_shadow_node_arg = DeclareLaunchArgument(
        "start_ggcnn_shadow_node",
        default_value="true",
        description="Start a shadow GG-CNN grasp backend for visualization-only proposals",
    )
    start_ggcnn_shadow_image_view_arg = DeclareLaunchArgument(
        "start_ggcnn_shadow_image_view",
        default_value="false",
        description="Open rqt_image_view on the GG-CNN shadow grasp overlay topic",
    )
    start_graspgen_topk_bridge_arg = DeclareLaunchArgument(
        "start_graspgen_topk_bridge",
        default_value="false",
        description="Start the GraspGen shadow bridge that publishes a top-k point cloud for RViz",
    )
    start_graspgen_shadow_node_arg = DeclareLaunchArgument(
        "start_graspgen_shadow_node",
        default_value="false",
        description="Start a shadow GraspGen backend for proposal and marker visualization",
    )
    start_web_gateway_arg = DeclareLaunchArgument(
        "start_web_gateway",
        default_value="false",
        description="Start the FastAPI/WebSocket gateway for the Programme Web UI",
    )
    detector_ultralytics_device_arg = DeclareLaunchArgument(
        "detector_ultralytics_device",
        default_value="",
        description="Override detector_seg_node ultralytics device, for example cpu or cuda:0",
    )
    detector_max_inference_rate_hz_arg = DeclareLaunchArgument(
        "detector_max_inference_rate_hz",
        default_value="2.0",
        description="Override detector_seg_node max inference rate in Hz",
    )
    detector_publish_debug_overlay_arg = DeclareLaunchArgument(
        "detector_publish_debug_overlay",
        default_value="true",
        description="Publish detector debug overlay images",
    )
    ggcnn_device_arg = DeclareLaunchArgument(
        "ggcnn_device",
        default_value="",
        description="Override local GG-CNN device, for example cpu or cuda:0",
    )
    auto_infer_on_start_arg = DeclareLaunchArgument(
        "auto_infer_on_start",
        default_value="true",
        description="Allow qwen_semantic_node to publish a default semantic task at startup",
    )
    auto_start_search_sweep_arg = DeclareLaunchArgument(
        "auto_start_search_sweep",
        default_value="true",
        description="Allow sim_search_sweep_node to begin scanning automatically at startup",
    )
    start_gazebo_gui_arg = DeclareLaunchArgument(
        "start_gazebo_gui",
        default_value="true",
        description="Launch Gazebo GUI for the MoveIt demo stack",
    )
    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="true",
        description="Launch RViz for the MoveIt demo stack",
    )
    use_gpu_accel_arg = DeclareLaunchArgument(
        "use_gpu_accel",
        default_value="true",
        description="Use WSLg GPU acceleration for Gazebo GUI rendering",
    )
    server_use_gpu_accel_arg = DeclareLaunchArgument(
        "server_use_gpu_accel",
        default_value="false",
        description="Use GPU rendering on the Gazebo server",
    )
    gpu_adapter_arg = DeclareLaunchArgument(
        "gpu_adapter",
        default_value="NVIDIA",
        description="Preferred GPU adapter name for WSLg D3D12 rendering",
    )
    require_user_confirmation_arg = DeclareLaunchArgument(
        "require_user_confirmation",
        default_value="false",
        description="Require an explicit execute command before the pick task starts",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock across the modular grasp stack",
    )

    stack_param_file = LaunchConfiguration("stack_param_file")
    search_start_delay_sec = LaunchConfiguration("search_start_delay_sec")
    pick_start_delay_sec = LaunchConfiguration("pick_start_delay_sec")
    shadow_only_mode = LaunchConfiguration("shadow_only_mode")
    start_ggcnn_shadow_node = LaunchConfiguration("start_ggcnn_shadow_node")
    start_ggcnn_shadow_image_view = LaunchConfiguration("start_ggcnn_shadow_image_view")
    start_graspgen_topk_bridge = LaunchConfiguration("start_graspgen_topk_bridge")
    start_graspgen_shadow_node = LaunchConfiguration("start_graspgen_shadow_node")
    start_web_gateway = LaunchConfiguration("start_web_gateway")
    detector_ultralytics_device = LaunchConfiguration("detector_ultralytics_device")
    detector_max_inference_rate_hz = LaunchConfiguration("detector_max_inference_rate_hz")
    detector_publish_debug_overlay = LaunchConfiguration("detector_publish_debug_overlay")
    ggcnn_device = LaunchConfiguration("ggcnn_device")
    auto_infer_on_start = LaunchConfiguration("auto_infer_on_start")
    auto_start_search_sweep = LaunchConfiguration("auto_start_search_sweep")
    start_gazebo_gui = LaunchConfiguration("start_gazebo_gui")
    start_rviz = LaunchConfiguration("start_rviz")
    use_gpu_accel = LaunchConfiguration("use_gpu_accel")
    server_use_gpu_accel = LaunchConfiguration("server_use_gpu_accel")
    gpu_adapter = LaunchConfiguration("gpu_adapter")
    require_user_confirmation = LaunchConfiguration("require_user_confirmation")
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_time_params = {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
    qwen_runtime_params, dialog_runtime_params = _remote_vlm_params()
    sim_pick_moveit_config = (
        MoveItConfigsBuilder("dofbot", package_name="tactile_moveit_config")
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics()
        .to_moveit_configs()
    )

    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_demo_launch),
        launch_arguments={
            "start_search_demo": "false",
            "start_pick_demo": "false",
            "use_sim_time": use_sim_time,
            "start_gazebo_gui": start_gazebo_gui,
            "start_rviz": start_rviz,
            "use_gpu_accel": use_gpu_accel,
            "server_use_gpu_accel": server_use_gpu_accel,
            "gpu_adapter": gpu_adapter,
        }.items(),
    )

    qwen_semantic_node = Node(
        package="tactile_vision",
        executable="qwen_semantic_node",
        name="qwen_semantic_node",
        output="screen",
        arguments=core_info_args,
        parameters=[
            stack_param_file,
            sim_time_params,
            {"auto_infer_on_start": ParameterValue(auto_infer_on_start, value_type=bool)},
            qwen_runtime_params,
        ],
        **respawn_kwargs,
    )

    detector_seg_node = Node(
        package="tactile_vision",
        executable="detector_seg_node",
        name="detector_seg_node",
        output="screen",
        arguments=core_info_args,
        parameters=[
            stack_param_file,
            sim_time_params,
            {
                "ultralytics_device": detector_ultralytics_device,
                "max_inference_rate_hz": detector_max_inference_rate_hz,
                "publish_debug_overlay": ParameterValue(
                    detector_publish_debug_overlay, value_type=bool
                ),
            },
        ],
        **respawn_kwargs,
    )

    cloud_filter_node = Node(
        package="tactile_vision",
        executable="cloud_filter_node",
        name="cloud_filter_node",
        output="screen",
        arguments=core_info_args,
        parameters=[stack_param_file, sim_time_params],
        **respawn_kwargs,
    )

    primitive_fit_node = Node(
        package="tactile_vision",
        executable="primitive_fit_node",
        name="primitive_fit_node",
        output="screen",
        arguments=core_info_args,
        parameters=[stack_param_file, sim_time_params],
        **respawn_kwargs,
    )

    grasp_input_cloud_node = Node(
        package="tactile_vision",
        executable="grasp_input_cloud_node",
        name="grasp_input_cloud_node",
        output="screen",
        arguments=core_info_args,
        parameters=[stack_param_file, sim_time_params],
        **respawn_kwargs,
    )

    stm32_bridge_node = Node(
        package="tactile_hardware",
        executable="stm32_bridge_node",
        name="stm32_bridge_node",
        output="screen",
        arguments=core_info_args,
        parameters=[stack_param_file],
        **respawn_kwargs,
    )

    grasp_profile_node = Node(
        package="tactile_control",
        executable="grasp_profile_node",
        name="grasp_profile_node",
        output="screen",
        arguments=core_info_args,
        parameters=[stack_param_file],
        **respawn_kwargs,
    )

    web_gateway_node = Node(
        package="tactile_web_bridge",
        executable="tactile_web_gateway",
        name="tactile_web_gateway",
        output="screen",
        arguments=core_info_args,
        condition=IfCondition(start_web_gateway),
        parameters=[stack_param_file, sim_time_params, dialog_runtime_params],
        **respawn_kwargs,
    )

    grasp_backend_node = Node(
        package="tactile_vision",
        executable="grasp_backend_node",
        name="grasp_backend_node",
        output="screen",
        arguments=core_info_args,
        condition=UnlessCondition(shadow_only_mode),
        parameters=[
            stack_param_file,
            sim_time_params,
            {
                "ggcnn_device": ggcnn_device,
            },
        ],
        **respawn_kwargs,
    )

    ggcnn_shadow_node = Node(
        package="tactile_vision",
        executable="grasp_backend_node",
        name="ggcnn_shadow_node",
        output="screen",
        arguments=core_info_args,
        condition=IfCondition(start_ggcnn_shadow_node),
        parameters=[
            stack_param_file,
            sim_time_params,
            {
                "ggcnn_device": ggcnn_device,
                "backend": "ggcnn_local",
                "candidate_grasp_proposal_array_topic": "/grasp/ggcnn_shadow/candidate_grasp_proposals",
                "candidate_markers_topic": "/grasp/ggcnn_shadow/candidate_markers",
                "selected_contact_point_1_topic": "/grasp/ggcnn_shadow/selected_contact_point_1_cloud",
                "selected_contact_point_2_topic": "/grasp/ggcnn_shadow/selected_contact_point_2_cloud",
                "selected_grasp_center_topic": "/grasp/ggcnn_shadow/selected_grasp_center_cloud",
                "ggcnn_depth_roi_topic": "/grasp/ggcnn_shadow/depth_roi",
                "ggcnn_q_heatmap_topic": "/grasp/ggcnn_shadow/q_heatmap",
                "ggcnn_angle_map_topic": "/grasp/ggcnn_shadow/angle_map",
                "ggcnn_width_map_topic": "/grasp/ggcnn_shadow/width_map",
                "ggcnn_grasp_overlay_topic": "/grasp/ggcnn_shadow/grasp_overlay",
                "sensor_stale_sec": 0.0,
                "publish_empty_on_failure": False,
            },
        ],
        **respawn_kwargs,
    )

    ggcnn_shadow_image_view = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="ggcnn_shadow_image_view",
        output="screen",
        condition=IfCondition(start_ggcnn_shadow_image_view),
        arguments=["/grasp/ggcnn_shadow/grasp_overlay"],
        parameters=[sim_time_params],
        **respawn_kwargs,
    )

    graspgen_topk_bridge_node = Node(
        package="tactile_vision",
        executable="graspgen_topk_bridge_node",
        name="graspgen_topk_bridge_node",
        output="screen",
        arguments=core_info_args,
        condition=IfCondition(start_graspgen_topk_bridge),
        parameters=[stack_param_file, sim_time_params],
        **respawn_kwargs,
    )

    graspgen_shadow_node = Node(
        package="tactile_vision",
        executable="grasp_backend_node",
        name="graspgen_shadow_node",
        output="screen",
        arguments=core_info_args,
        condition=IfCondition(start_graspgen_shadow_node),
        parameters=[
            stack_param_file,
            sim_time_params,
            {
                "backend": "graspgen_zmq",
                "candidate_grasp_proposal_array_topic": "/grasp/graspgen_shadow/candidate_grasp_proposals",
                "candidate_markers_topic": "/grasp/graspgen_shadow/candidate_markers",
                "selected_contact_point_1_topic": "/grasp/graspgen_shadow/selected_contact_point_1_cloud",
                "selected_contact_point_2_topic": "/grasp/graspgen_shadow/selected_contact_point_2_cloud",
                "selected_grasp_center_topic": "/grasp/graspgen_shadow/selected_grasp_center_cloud",
                "publish_empty_on_failure": False,
            },
        ],
        **respawn_kwargs,
    )

    search_sweep_node = Node(
        package="tactile_sim",
        executable="sim_search_sweep_node",
        name="sim_search_sweep_node",
        output="screen",
        arguments=core_info_args,
        parameters=[
            stack_param_file,
            sim_time_params,
            {"auto_start_enabled": ParameterValue(auto_start_search_sweep, value_type=bool)},
        ],
        **respawn_kwargs,
    )

    delayed_search_sweep = TimerAction(
        period=search_start_delay_sec,
        actions=[search_sweep_node],
    )

    task_executive_node = Node(
        package="tactile_task",
        executable="task_executive_node",
        name="task_executive_node",
        output="screen",
        arguments=core_info_args,
        condition=UnlessCondition(shadow_only_mode),
        parameters=[stack_param_file, sim_time_params],
        **respawn_kwargs,
    )

    search_target_skill_node = Node(
        package="tactile_task",
        executable="search_target_skill_node",
        name="search_target_skill_node",
        output="screen",
        arguments=core_info_args,
        condition=UnlessCondition(shadow_only_mode),
        parameters=[stack_param_file, sim_time_params],
        **respawn_kwargs,
    )

    pick_task_node = Node(
        package="tactile_task_cpp",
        executable="sim_pick_task_node",
        name="sim_pick_task_node",
        output="screen",
        arguments=pick_info_args,
        parameters=[
            stack_param_file,
            sim_pick_moveit_config.to_dict(),
            sim_time_params,
            {
                "require_user_confirmation": ParameterValue(
                    require_user_confirmation, value_type=bool
                )
            },
        ],
        **respawn_kwargs,
    )

    delayed_pick_task = TimerAction(
        period=pick_start_delay_sec,
        condition=UnlessCondition(shadow_only_mode),
        actions=[pick_task_node],
    )

    return LaunchDescription(
        [
            stack_param_file_arg,
            search_start_delay_sec_arg,
            pick_start_delay_sec_arg,
            shadow_only_mode_arg,
            start_ggcnn_shadow_node_arg,
            start_ggcnn_shadow_image_view_arg,
            start_graspgen_topk_bridge_arg,
            start_graspgen_shadow_node_arg,
            start_web_gateway_arg,
            detector_ultralytics_device_arg,
            detector_max_inference_rate_hz_arg,
            detector_publish_debug_overlay_arg,
            ggcnn_device_arg,
            auto_infer_on_start_arg,
            auto_start_search_sweep_arg,
            start_gazebo_gui_arg,
            start_rviz_arg,
            use_gpu_accel_arg,
            server_use_gpu_accel_arg,
            gpu_adapter_arg,
            require_user_confirmation_arg,
            use_sim_time_arg,
            moveit_demo,
            qwen_semantic_node,
            detector_seg_node,
            cloud_filter_node,
            primitive_fit_node,
            grasp_input_cloud_node,
            stm32_bridge_node,
            grasp_profile_node,
            web_gateway_node,
            grasp_backend_node,
            ggcnn_shadow_node,
            ggcnn_shadow_image_view,
            graspgen_topk_bridge_node,
            graspgen_shadow_node,
            delayed_search_sweep,
            search_target_skill_node,
            task_executive_node,
            delayed_pick_task,
        ]
    )
