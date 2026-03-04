from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_param_file = PathJoinSubstitution(
        [FindPackageShare("tactile_bringup"), "config", "phase6_sim_gazebo.yaml"]
    )

    param_file_arg = DeclareLaunchArgument(
        "param_file",
        default_value=default_param_file,
        description="Path to phase6 Gazebo simulation YAML",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )

    param_file = LaunchConfiguration("param_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    gazebo_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("tactile_sim"), "launch", "gazebo_arm.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    tactile_sim_node = Node(
        package="tactile_sim",
        executable="tactile_sim_node",
        name="tactile_sim_node",
        output="screen",
        parameters=[param_file],
    )

    arm_sim_driver_node = Node(
        package="tactile_sim",
        executable="arm_sim_driver_node",
        name="arm_sim_driver_node",
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
            use_sim_time_arg,
            gazebo_arm_launch,
            tactile_sim_node,
            arm_sim_driver_node,
            arm_control_node,
            demo_task_node,
            tactile_ui_subscriber,
        ]
    )
