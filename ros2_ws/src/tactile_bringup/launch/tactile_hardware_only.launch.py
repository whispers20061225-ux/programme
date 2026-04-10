from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_file = LaunchConfiguration("config_file")
    start_web_gateway = LaunchConfiguration("start_web_gateway")
    log_level = LaunchConfiguration("log_level")

    default_config = PathJoinSubstitution(
        [FindPackageShare("tactile_bringup"), "config", "tactile_hardware_only.yaml"]
    )

    common_args = ["--ros-args", "--log-level", log_level]

    stm32_bridge_node = Node(
        package="tactile_hardware",
        executable="stm32_bridge_node",
        name="stm32_bridge_node",
        output="screen",
        arguments=common_args,
        parameters=[config_file],
    )

    grasp_profile_node = Node(
        package="tactile_control",
        executable="grasp_profile_node",
        name="grasp_profile_node",
        output="screen",
        arguments=common_args,
        parameters=[config_file],
    )

    tactile_web_gateway = Node(
        package="tactile_web_bridge",
        executable="tactile_web_gateway",
        name="tactile_web_gateway",
        output="screen",
        arguments=common_args,
        parameters=[config_file],
        condition=IfCondition(start_web_gateway),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("start_web_gateway", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            stm32_bridge_node,
            grasp_profile_node,
            tactile_web_gateway,
        ]
    )
