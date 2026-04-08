from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    stack_launch = PathJoinSubstitution(
        [FindPackageShare("tactile_bringup"), "launch", "phase8_modular_grasp.launch.py"]
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
    use_gpu_accel = LaunchConfiguration("use_gpu_accel")
    server_use_gpu_accel = LaunchConfiguration("server_use_gpu_accel")
    gpu_adapter = LaunchConfiguration("gpu_adapter")

    return LaunchDescription(
        [
            use_gpu_accel_arg,
            server_use_gpu_accel_arg,
            gpu_adapter_arg,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(stack_launch),
                launch_arguments={
                    "start_web_gateway": "true",
                    "require_user_confirmation": "false",
                    "auto_infer_on_start": "false",
                    "auto_start_search_sweep": "false",
                    "detector_ultralytics_device": "cuda:0",
                    "detector_max_inference_rate_hz": "4.0",
                    "detector_publish_debug_overlay": "true",
                    "ggcnn_device": "cpu",
                    "start_ggcnn_shadow_node": "false",
                    "start_gazebo_gui": "false",
                    "start_rviz": "false",
                    "use_gpu_accel": use_gpu_accel,
                    "server_use_gpu_accel": server_use_gpu_accel,
                    "gpu_adapter": gpu_adapter,
                }.items(),
            )
        ]
    )
