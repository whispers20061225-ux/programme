from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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

    phase8_param_file = LaunchConfiguration("phase8_param_file")
    search_start_delay_sec = LaunchConfiguration("search_start_delay_sec")
    pick_start_delay_sec = LaunchConfiguration("pick_start_delay_sec")

    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_demo_launch),
        launch_arguments={
            "start_search_demo": "false",
            "start_pick_demo": "false",
        }.items(),
    )

    qwen_semantic_node = Node(
        package="tactile_vision",
        executable="qwen_semantic_node",
        name="qwen_semantic_node",
        output="screen",
        arguments=core_info_args,
        parameters=[phase8_param_file],
    )

    detector_seg_node = Node(
        package="tactile_vision",
        executable="detector_seg_node",
        name="detector_seg_node",
        output="screen",
        arguments=core_info_args,
        parameters=[phase8_param_file],
    )

    cloud_filter_node = Node(
        package="tactile_vision",
        executable="cloud_filter_node",
        name="cloud_filter_node",
        output="screen",
        arguments=core_info_args,
        parameters=[phase8_param_file],
    )

    grasp_backend_node = Node(
        package="tactile_vision",
        executable="grasp_backend_node",
        name="grasp_backend_node",
        output="screen",
        arguments=core_info_args,
        parameters=[phase8_param_file],
    )

    search_sweep_node = Node(
        package="tactile_sim",
        executable="sim_search_sweep_node",
        name="sim_search_sweep_node",
        output="screen",
        arguments=core_info_args,
        parameters=[phase8_param_file],
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
            {"use_sim_time": True},
        ],
    )

    delayed_pick_task = TimerAction(
        period=pick_start_delay_sec,
        actions=[pick_task_node],
    )

    return LaunchDescription(
        [
            phase8_param_file_arg,
            search_start_delay_sec_arg,
            pick_start_delay_sec_arg,
            moveit_demo,
            qwen_semantic_node,
            detector_seg_node,
            cloud_filter_node,
            grasp_backend_node,
            delayed_search_sweep,
            delayed_pick_task,
        ]
    )
