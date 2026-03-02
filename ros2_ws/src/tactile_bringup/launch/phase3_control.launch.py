from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    param_file = PathJoinSubstitution(
        [FindPackageShare("tactile_bringup"), "config", "phase3_control.yaml"]
    )

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

    tactile_ui_subscriber = Node(
        package="tactile_ui_bridge",
        executable="tactile_ui_subscriber",
        name="tactile_ui_subscriber",
        output="screen",
        parameters=[param_file],
    )

    return LaunchDescription(
        [
            tactile_sensor_node,
            arm_driver_node,
            arm_control_node,
            tactile_ui_subscriber,
        ]
    )
