from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_param_file = PathJoinSubstitution(
        [FindPackageShare("tactile_bringup"), "config", "phase5_task_hardware.yaml"]
    )
    param_file_arg = DeclareLaunchArgument(
        "param_file",
        default_value=default_param_file,
        description="Path to phase5 ROS2 parameter YAML",
    )
    param_file = LaunchConfiguration("param_file")

    tactile_sensor_node = Node(
        package="tactile_hardware",
        executable="tactile_sensor_node",
        name="tactile_sensor_node",
        output="screen",
        parameters=[param_file],
    )

    arm_driver_node = Node(
        package="tactile_hardware",
        executable="arm_driver_node",
        name="arm_driver_node",
        output="screen",
        parameters=[param_file],
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

    return LaunchDescription(
        [
            param_file_arg,
            tactile_sensor_node,
            arm_driver_node,
            arm_control_node,
            demo_task_node,
            tactile_ui_subscriber,
        ]
    )
