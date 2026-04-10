from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    moveit_demo_launch = PathJoinSubstitution(
        [FindPackageShare("tactile_moveit_config"), "launch", "moveit_gazebo_demo.launch.py"]
    )
    stack_param_file_arg = DeclareLaunchArgument(
        "stack_param_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("tactile_bringup"), "config", "programme_grasp_stack.yaml"]
        ),
        description="Path to the Programme grasp stack parameter YAML",
    )
    detector_ultralytics_device_arg = DeclareLaunchArgument(
        "detector_ultralytics_device",
        default_value="cuda:0",
        description="Detector device, for example cpu or cuda:0",
    )
    detector_max_inference_rate_hz_arg = DeclareLaunchArgument(
        "detector_max_inference_rate_hz",
        default_value="4.0",
        description="Detector maximum inference rate in Hz",
    )
    detector_publish_debug_overlay_arg = DeclareLaunchArgument(
        "detector_publish_debug_overlay",
        default_value="true",
        description="Publish detector debug overlay images",
    )
    ggcnn_device_arg = DeclareLaunchArgument(
        "ggcnn_device",
        default_value="cpu",
        description="Left for compatibility; ignored by the GraspGen main path",
    )
    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="true",
        description="Launch RViz for the MoveIt demo stack",
    )
    start_gazebo_gui_arg = DeclareLaunchArgument(
        "start_gazebo_gui",
        default_value="false",
        description="Launch Gazebo GUI for the MoveIt demo stack",
    )
    use_gpu_accel_arg = DeclareLaunchArgument(
        "use_gpu_accel",
        default_value="true",
        description="Use WSLg GPU acceleration for Gazebo GUI rendering",
    )
    server_use_gpu_accel_arg = DeclareLaunchArgument(
        "server_use_gpu_accel",
        default_value="true",
        description="Use GPU rendering on the Gazebo server",
    )
    gpu_adapter_arg = DeclareLaunchArgument(
        "gpu_adapter",
        default_value="NVIDIA",
        description="Preferred GPU adapter name for WSLg D3D12 rendering",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock across the GraspGen visualization stack",
    )

    stack_param_file = LaunchConfiguration("stack_param_file")
    detector_ultralytics_device = LaunchConfiguration("detector_ultralytics_device")
    detector_max_inference_rate_hz = LaunchConfiguration("detector_max_inference_rate_hz")
    detector_publish_debug_overlay = LaunchConfiguration("detector_publish_debug_overlay")
    ggcnn_device = LaunchConfiguration("ggcnn_device")
    start_rviz = LaunchConfiguration("start_rviz")
    start_gazebo_gui = LaunchConfiguration("start_gazebo_gui")
    use_gpu_accel = LaunchConfiguration("use_gpu_accel")
    server_use_gpu_accel = LaunchConfiguration("server_use_gpu_accel")
    gpu_adapter = LaunchConfiguration("gpu_adapter")
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
            "use_gpu_accel": use_gpu_accel,
            "server_use_gpu_accel": server_use_gpu_accel,
            "gpu_adapter": gpu_adapter,
            "sim_start_demo_task_node": "false",
            "sim_start_ui_subscriber": "false",
        }.items(),
    )

    detector_seg_node = Node(
        package="tactile_vision",
        executable="detector_seg_node",
        name="detector_seg_node",
        output="screen",
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
    )

    cloud_filter_node = Node(
        package="tactile_vision",
        executable="cloud_filter_node",
        name="cloud_filter_node",
        output="screen",
        parameters=[stack_param_file, sim_time_params],
    )

    primitive_fit_node = Node(
        package="tactile_vision",
        executable="primitive_fit_node",
        name="primitive_fit_node",
        output="screen",
        parameters=[stack_param_file, sim_time_params],
    )

    grasp_input_cloud_node = Node(
        package="tactile_vision",
        executable="grasp_input_cloud_node",
        name="grasp_input_cloud_node",
        output="screen",
        parameters=[stack_param_file, sim_time_params],
    )

    grasp_backend_node = Node(
        package="tactile_vision",
        executable="grasp_backend_node",
        name="grasp_backend_node",
        output="screen",
        parameters=[
            stack_param_file,
            sim_time_params,
            {
                "ggcnn_device": ggcnn_device,
            },
        ],
    )

    return LaunchDescription(
        [
            stack_param_file_arg,
            detector_ultralytics_device_arg,
            detector_max_inference_rate_hz_arg,
            detector_publish_debug_overlay_arg,
            ggcnn_device_arg,
            start_rviz_arg,
            start_gazebo_gui_arg,
            use_gpu_accel_arg,
            server_use_gpu_accel_arg,
            gpu_adapter_arg,
            use_sim_time_arg,
            moveit_demo,
            detector_seg_node,
            cloud_filter_node,
            primitive_fit_node,
            grasp_input_cloud_node,
            grasp_backend_node,
        ]
    )
