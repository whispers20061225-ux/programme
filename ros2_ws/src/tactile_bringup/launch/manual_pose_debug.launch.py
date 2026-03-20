from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    quiet_pick_args = [
        "--ros-args",
        "--log-level",
        "info",
        "--log-level",
        "move_group:=error",
        "--log-level",
        "rcl.logging_rosout:=error",
    ]

    default_param_file = PathJoinSubstitution(
        [FindPackageShare("tactile_bringup"), "config", "manual_pose_debug.yaml"]
    )
    moveit_demo_launch = PathJoinSubstitution(
        [FindPackageShare("tactile_moveit_config"), "launch", "moveit_gazebo_demo.launch.py"]
    )
    moveit_config = (
        MoveItConfigsBuilder("dofbot", package_name="tactile_moveit_config")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .trajectory_execution(moveit_manage_controllers=False)
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    param_file_arg = DeclareLaunchArgument(
        "debug_param_file",
        default_value=default_param_file,
        description="Path to manual pose debug YAML",
    )
    start_gazebo_gui_arg = DeclareLaunchArgument(
        "start_gazebo_gui",
        default_value="true",
        description="Launch Gazebo GUI",
    )
    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="true",
        description="Launch RViz MotionPlanning",
    )
    use_gpu_accel_arg = DeclareLaunchArgument(
        "use_gpu_accel",
        default_value="true",
        description="Use WSLg GPU acceleration for Gazebo GUI rendering",
    )
    server_use_gpu_accel_arg = DeclareLaunchArgument(
        "server_use_gpu_accel",
        default_value="false",
        description="Use GPU rendering on Gazebo server",
    )
    gpu_adapter_arg = DeclareLaunchArgument(
        "gpu_adapter",
        default_value="NVIDIA",
        description="Preferred GPU adapter name for WSLg D3D12 rendering",
    )
    debug_start_delay_sec_arg = DeclareLaunchArgument(
        "debug_start_delay_sec",
        default_value="16.0",
        description="Delay manual pose debug node until MoveIt and Gazebo are stable",
    )

    debug_param_file = LaunchConfiguration("debug_param_file")
    start_gazebo_gui = LaunchConfiguration("start_gazebo_gui")
    start_rviz = LaunchConfiguration("start_rviz")
    use_gpu_accel = LaunchConfiguration("use_gpu_accel")
    server_use_gpu_accel = LaunchConfiguration("server_use_gpu_accel")
    gpu_adapter = LaunchConfiguration("gpu_adapter")
    debug_start_delay_sec = LaunchConfiguration("debug_start_delay_sec")

    moveit_demo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(moveit_demo_launch),
      launch_arguments={
          "start_search_demo": "false",
          "start_pick_demo": "false",
          "start_gazebo_gui": start_gazebo_gui,
          "start_rviz": start_rviz,
          "use_gpu_accel": use_gpu_accel,
          "server_use_gpu_accel": server_use_gpu_accel,
          "gpu_adapter": gpu_adapter,
      }.items(),
    )

    manual_pose_debug_node = Node(
        package="tactile_task_cpp",
        executable="manual_pose_debug_node",
        name="manual_pose_debug_node",
        output="screen",
        arguments=quiet_pick_args,
        parameters=[
            moveit_config.to_dict(),
            debug_param_file,
            {
                "use_sim_time": True,
            },
        ],
    )

    delayed_manual_pose_debug = TimerAction(
        period=debug_start_delay_sec,
        actions=[manual_pose_debug_node],
    )

    return LaunchDescription(
        [
            param_file_arg,
            start_gazebo_gui_arg,
            start_rviz_arg,
            use_gpu_accel_arg,
            server_use_gpu_accel_arg,
            gpu_adapter_arg,
            debug_start_delay_sec_arg,
            moveit_demo,
            delayed_manual_pose_debug,
        ]
    )
