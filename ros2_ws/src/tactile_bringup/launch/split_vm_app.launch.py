from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_param_file = PathJoinSubstitution(
        [FindPackageShare("tactile_bringup"), "config", "split_vm_app.yaml"]
    )

    param_file_arg = DeclareLaunchArgument(
        "param_file",
        default_value=default_param_file,
        description="Path to split VM app parameter YAML",
    )
    start_tactile_sensor_arg = DeclareLaunchArgument(
        "start_tactile_sensor",
        default_value="true",
        description="Whether to launch tactile_sensor_node on VM side",
    )
    start_latest_frame_relay_arg = DeclareLaunchArgument(
        "start_latest_frame_relay",
        default_value="true",
        description="Whether to launch tactile_vision_cpp latest_frame_relay_node",
    )

    param_file = LaunchConfiguration("param_file")
    start_tactile_sensor = LaunchConfiguration("start_tactile_sensor")
    start_latest_frame_relay = LaunchConfiguration("start_latest_frame_relay")

    tactile_sensor_node = Node(
        package="tactile_hardware",
        executable="tactile_sensor_node",
        name="tactile_sensor_node",
        output="screen",
        parameters=[param_file],
        condition=IfCondition(start_tactile_sensor),
    )

    arm_control_node = Node(
        package="tactile_control",
        executable="arm_control_node",
        name="arm_control_node",
        output="screen",
        parameters=[param_file],
    )

    demo_task_node = Node(
        package="tactile_task",
        executable="demo_task_node",
        name="demo_task_node",
        output="screen",
        parameters=[param_file],
    )

    tactile_ui_subscriber = Node(
        package="tactile_ui_bridge",
        executable="tactile_ui_subscriber",
        name="tactile_ui_subscriber",
        output="screen",
        parameters=[param_file],
    )

    latest_frame_relay_node = Node(
        package="tactile_vision_cpp",
        executable="latest_frame_relay_node",
        name="latest_frame_relay_node",
        output="screen",
        parameters=[param_file],
        condition=IfCondition(start_latest_frame_relay),
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
            start_tactile_sensor_arg,
            start_latest_frame_relay_arg,
            tactile_sensor_node,
            arm_control_node,
            demo_task_node,
            tactile_ui_subscriber,
            latest_frame_relay_node,
            realsense_monitor_node,
        ]
    )
