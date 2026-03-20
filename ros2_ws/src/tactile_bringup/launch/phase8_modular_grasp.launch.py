from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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
        [FindPackageShare("tactile_bringup"), "config", "phase8_modular_grasp.yaml"]
    )
    moveit_demo_launch = PathJoinSubstitution(
        [FindPackageShare("tactile_moveit_config"), "launch", "moveit_gazebo_demo.launch.py"]
    )

    phase8_param_file_arg = DeclareLaunchArgument(
        "phase8_param_file",
        default_value=default_param_file,
        description="Path to modular grasp parameter YAML",
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

    phase8_param_file = LaunchConfiguration("phase8_param_file")
    search_start_delay_sec = LaunchConfiguration("search_start_delay_sec")
    pick_start_delay_sec = LaunchConfiguration("pick_start_delay_sec")
    shadow_only_mode = LaunchConfiguration("shadow_only_mode")
    start_ggcnn_shadow_node = LaunchConfiguration("start_ggcnn_shadow_node")
    start_ggcnn_shadow_image_view = LaunchConfiguration("start_ggcnn_shadow_image_view")
    start_web_gateway = LaunchConfiguration("start_web_gateway")
    detector_ultralytics_device = LaunchConfiguration("detector_ultralytics_device")
    detector_max_inference_rate_hz = LaunchConfiguration("detector_max_inference_rate_hz")
    detector_publish_debug_overlay = LaunchConfiguration("detector_publish_debug_overlay")
    ggcnn_device = LaunchConfiguration("ggcnn_device")
    auto_infer_on_start = LaunchConfiguration("auto_infer_on_start")
    auto_start_search_sweep = LaunchConfiguration("auto_start_search_sweep")
    start_gazebo_gui = LaunchConfiguration("start_gazebo_gui")
    start_rviz = LaunchConfiguration("start_rviz")
    require_user_confirmation = LaunchConfiguration("require_user_confirmation")
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_time_params = {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}

    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_demo_launch),
        launch_arguments={
            "start_search_demo": "false",
            "start_pick_demo": "false",
            "use_sim_time": use_sim_time,
            "start_gazebo_gui": start_gazebo_gui,
            "start_rviz": start_rviz,
        }.items(),
    )

    qwen_semantic_node = Node(
        package="tactile_vision",
        executable="qwen_semantic_node",
        name="qwen_semantic_node",
        output="screen",
        arguments=core_info_args,
        parameters=[
            phase8_param_file,
            sim_time_params,
            {"auto_infer_on_start": ParameterValue(auto_infer_on_start, value_type=bool)},
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
            phase8_param_file,
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
        parameters=[phase8_param_file, sim_time_params],
        **respawn_kwargs,
    )

    web_gateway_node = Node(
        package="tactile_web_bridge",
        executable="tactile_web_gateway",
        name="tactile_web_gateway",
        output="screen",
        arguments=core_info_args,
        condition=IfCondition(start_web_gateway),
        parameters=[phase8_param_file, sim_time_params],
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
            phase8_param_file,
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
            phase8_param_file,
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

    search_sweep_node = Node(
        package="tactile_sim",
        executable="sim_search_sweep_node",
        name="sim_search_sweep_node",
        output="screen",
        arguments=core_info_args,
        parameters=[
            phase8_param_file,
            sim_time_params,
            {"auto_start_enabled": ParameterValue(auto_start_search_sweep, value_type=bool)},
        ],
        **respawn_kwargs,
    )

    delayed_search_sweep = TimerAction(
        period=search_start_delay_sec,
        actions=[search_sweep_node],
    )

    pick_task_node = Node(
        package="tactile_task_cpp",
        executable="sim_pick_task_node",
        name="sim_pick_task_node",
        output="screen",
        arguments=pick_info_args,
        parameters=[
            phase8_param_file,
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
            phase8_param_file_arg,
            search_start_delay_sec_arg,
            pick_start_delay_sec_arg,
            shadow_only_mode_arg,
            start_ggcnn_shadow_node_arg,
            start_ggcnn_shadow_image_view_arg,
            start_web_gateway_arg,
            detector_ultralytics_device_arg,
            detector_max_inference_rate_hz_arg,
            detector_publish_debug_overlay_arg,
            ggcnn_device_arg,
            auto_infer_on_start_arg,
            auto_start_search_sweep_arg,
            start_gazebo_gui_arg,
            start_rviz_arg,
            require_user_confirmation_arg,
            use_sim_time_arg,
            moveit_demo,
            qwen_semantic_node,
            detector_seg_node,
            cloud_filter_node,
            web_gateway_node,
            grasp_backend_node,
            ggcnn_shadow_node,
            ggcnn_shadow_image_view,
            delayed_search_sweep,
            delayed_pick_task,
        ]
    )
