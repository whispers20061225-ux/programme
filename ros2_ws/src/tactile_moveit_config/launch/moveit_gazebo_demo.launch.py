import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    default_world = PathJoinSubstitution(
        [FindPackageShare("tactile_sim"), "worlds", "phase6_tabletop_camera_gui.world"]
    )

    start_sim_arg = DeclareLaunchArgument(
        "start_sim",
        default_value="true",
        description="Launch Gazebo simulation chain together with MoveIt",
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
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock in MoveIt and RViz",
    )
    use_gpu_accel_arg = DeclareLaunchArgument(
        "use_gpu_accel",
        default_value="true",
        description="Use WSLg D3D12 GPU acceleration for Gazebo rendering",
    )
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo world file used by the MoveIt + Gazebo demo",
    )
    gpu_adapter_arg = DeclareLaunchArgument(
        "gpu_adapter",
        default_value="NVIDIA",
        description="Preferred GPU adapter name for WSLg D3D12 rendering",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("tactile_moveit_config"), "config", "moveit.rviz"]
        ),
        description="Path to the RViz configuration",
    )
    moveit_start_delay_sec_arg = DeclareLaunchArgument(
        "moveit_start_delay_sec",
        default_value="8.0",
        description="Delay RViz and move_group until Gazebo clock and robot state are stable",
    )

    start_sim = LaunchConfiguration("start_sim")
    start_gazebo_gui = LaunchConfiguration("start_gazebo_gui")
    start_rviz = LaunchConfiguration("start_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gpu_accel = LaunchConfiguration("use_gpu_accel")
    world = LaunchConfiguration("world")
    gpu_adapter = LaunchConfiguration("gpu_adapter")
    rviz_config = LaunchConfiguration("rviz_config")
    moveit_start_delay_sec = LaunchConfiguration("moveit_start_delay_sec")

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

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("tactile_bringup"), "launch", "phase6_sim_gazebo.launch.py"]
            )
        ),
        launch_arguments={
            "world": world,
            "start_gui": start_gazebo_gui,
            "use_sim_time": use_sim_time,
            "use_gpu_accel": use_gpu_accel,
            "gpu_adapter": gpu_adapter,
        }.items(),
        condition=IfCondition(start_sim),
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
                "allow_trajectory_execution": True,
                "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
                "trajectory_execution.allowed_execution_duration_scaling": 1.2,
                "trajectory_execution.allowed_goal_duration_margin": 0.5,
                "trajectory_execution.allowed_start_tolerance": 0.02,
                "publish_robot_description_semantic": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
            },
        ],
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="moveit_rviz",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
            },
        ],
        condition=IfCondition(start_rviz),
    )

    delayed_moveit_stack = TimerAction(
        period=moveit_start_delay_sec,
        actions=[
            move_group_node,
            rviz_node,
        ],
    )

    return LaunchDescription(
        [
            start_sim_arg,
            start_gazebo_gui_arg,
            start_rviz_arg,
            use_sim_time_arg,
            use_gpu_accel_arg,
            world_arg,
            gpu_adapter_arg,
            rviz_config_arg,
            moveit_start_delay_sec_arg,
            sim_launch,
            delayed_moveit_stack,
        ]
    )
