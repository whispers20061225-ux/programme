from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_param_file = PathJoinSubstitution(
        [FindPackageShare("tactile_bringup"), "config", "phase6_vision.yaml"]
    )

    param_file_arg = DeclareLaunchArgument(
        "param_file",
        default_value=default_param_file,
        description="Path to phase6 vision parameter YAML",
    )
    start_realsense_arg = DeclareLaunchArgument(
        "start_realsense",
        default_value="true",
        description="Whether to launch realsense2_camera_node together",
    )
    realsense_serial_arg = DeclareLaunchArgument(
        "realsense_serial_no",
        default_value="",
        description="Optional RealSense serial number filter",
    )

    param_file = LaunchConfiguration("param_file")
    start_realsense = LaunchConfiguration("start_realsense")
    realsense_serial_no = LaunchConfiguration("realsense_serial_no")

    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="realsense2_camera",
        output="screen",
        parameters=[
            {
                "enable_color": True,
                "enable_depth": True,
                "align_depth.enable": True,
                "enable_infra1": False,
                "enable_infra2": False,
                "pointcloud.enable": False,
                "rgb_camera.profile": "640x480x30",
                "depth_module.profile": "640x480x30",
                "serial_no": realsense_serial_no,
            }
        ],
        condition=IfCondition(start_realsense),
    )

    realsense_monitor_node = Node(
        package="tactile_vision",
        executable="realsense_monitor_node",
        name="realsense_monitor_node",
        output="screen",
        parameters=[param_file],
    )

    return LaunchDescription(
        [
            param_file_arg,
            start_realsense_arg,
            realsense_serial_arg,
            realsense_node,
            realsense_monitor_node,
        ]
    )

