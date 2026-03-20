from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    phase8_launch = PathJoinSubstitution(
        [FindPackageShare("tactile_bringup"), "launch", "phase8_modular_grasp.launch.py"]
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(phase8_launch),
                launch_arguments={
                    "start_web_gateway": "true",
                    "require_user_confirmation": "true",
                    "auto_infer_on_start": "false",
                    "auto_start_search_sweep": "false",
                    "detector_ultralytics_device": "cpu",
                    "detector_max_inference_rate_hz": "1.0",
                    "detector_publish_debug_overlay": "false",
                    "ggcnn_device": "cpu",
                    "start_ggcnn_shadow_node": "false",
                    "start_gazebo_gui": "false",
                    "start_rviz": "false",
                }.items(),
            )
        ]
    )
