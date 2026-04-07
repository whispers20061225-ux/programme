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
    quiet_node_args = ["--ros-args", "--log-level", "error"]
    respawn_kwargs = {"respawn": True, "respawn_delay": 2.0}

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
        description="Use WSLg D3D12 GPU acceleration for the Gazebo GUI client.",
    )
    server_use_gpu_accel_arg = DeclareLaunchArgument(
        "server_use_gpu_accel",
        default_value="false",
        description="Use GPU rendering on the Gazebo server. Keep false under WSLg when you need valid simulated depth images.",
    )
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo world file used by the MoveIt + Gazebo demo",
    )
    spawn_x_arg = DeclareLaunchArgument(
        "spawn_x",
        default_value="0.24",
        description="Robot spawn x position propagated to Gazebo and MoveIt robot_description",
    )
    spawn_y_arg = DeclareLaunchArgument(
        "spawn_y",
        default_value="0.0",
        description="Robot spawn y position propagated to Gazebo and MoveIt robot_description",
    )
    spawn_z_arg = DeclareLaunchArgument(
        "spawn_z",
        default_value="0.405",
        description="Robot spawn z position propagated to Gazebo and MoveIt robot_description",
    )
    gpu_adapter_arg = DeclareLaunchArgument(
        "gpu_adapter",
        default_value="NVIDIA",
        description="Preferred GPU adapter name for WSLg D3D12 rendering",
    )
    sim_start_demo_task_node_arg = DeclareLaunchArgument(
        "sim_start_demo_task_node",
        default_value="true",
        description="Start the legacy phase6 demo_task_node inside the Gazebo sim chain",
    )
    sim_start_ui_subscriber_arg = DeclareLaunchArgument(
        "sim_start_ui_subscriber",
        default_value="true",
        description="Start the legacy phase6 tactile_ui_subscriber inside the Gazebo sim chain",
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
    start_search_demo_arg = DeclareLaunchArgument(
        "start_search_demo",
        default_value="false",
        description="Start the simulated perception + joint1 search sweep demo",
    )
    search_start_delay_sec_arg = DeclareLaunchArgument(
        "search_start_delay_sec",
        default_value="12.0",
        description="Delay perception/search nodes until Gazebo, TF and controllers are stable",
    )
    start_pick_demo_arg = DeclareLaunchArgument(
        "start_pick_demo",
        default_value="false",
        description="Start the MoveIt-based simulated pick task demo",
    )
    pick_start_delay_sec_arg = DeclareLaunchArgument(
        "pick_start_delay_sec",
        default_value="16.0",
        description="Delay the pick task until perception/search are publishing stable target data",
    )

    start_sim = LaunchConfiguration("start_sim")
    start_gazebo_gui = LaunchConfiguration("start_gazebo_gui")
    start_rviz = LaunchConfiguration("start_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gpu_accel = LaunchConfiguration("use_gpu_accel")
    server_use_gpu_accel = LaunchConfiguration("server_use_gpu_accel")
    world = LaunchConfiguration("world")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    gpu_adapter = LaunchConfiguration("gpu_adapter")
    sim_start_demo_task_node = LaunchConfiguration("sim_start_demo_task_node")
    sim_start_ui_subscriber = LaunchConfiguration("sim_start_ui_subscriber")
    rviz_config = LaunchConfiguration("rviz_config")
    moveit_start_delay_sec = LaunchConfiguration("moveit_start_delay_sec")
    start_search_demo = LaunchConfiguration("start_search_demo")
    search_start_delay_sec = LaunchConfiguration("search_start_delay_sec")
    start_pick_demo = LaunchConfiguration("start_pick_demo")
    pick_start_delay_sec = LaunchConfiguration("pick_start_delay_sec")

    moveit_config = (
        MoveItConfigsBuilder("dofbot", package_name="tactile_moveit_config")
        .robot_description(
            mappings={
                "base_world_x": spawn_x,
                "base_world_y": spawn_y,
                "base_world_z": spawn_z,
            }
        )
        .robot_description_kinematics()
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
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
            "start_gui": start_gazebo_gui,
            "use_sim_time": use_sim_time,
            "use_gpu_accel": use_gpu_accel,
            "server_use_gpu_accel": server_use_gpu_accel,
            "gpu_adapter": gpu_adapter,
            "start_demo_task_node": sim_start_demo_task_node,
            "start_ui_subscriber": sim_start_ui_subscriber,
        }.items(),
        condition=IfCondition(start_sim),
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output={"stdout": "log", "stderr": "log"},
        arguments=quiet_node_args,
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
                "allow_trajectory_execution": True,
                "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
                "trajectory_execution.allowed_execution_duration_scaling": 1.2,
                "trajectory_execution.allowed_goal_duration_margin": 0.5,
                "trajectory_execution.allowed_start_tolerance": 0.05,
                "publish_robot_description_semantic": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
            },
        ],
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
        **respawn_kwargs,
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
        **respawn_kwargs,
    )

    delayed_moveit_stack = TimerAction(
        period=moveit_start_delay_sec,
        actions=[
            move_group_node,
            rviz_node,
        ],
    )

    target_pose_node = Node(
        package="tactile_sim",
        executable="sim_target_pose_node",
        output="screen",
        parameters=[
            {
                "target_frame": "world",
            }
        ],
        condition=IfCondition(start_search_demo),
        **respawn_kwargs,
    )

    search_sweep_node = Node(
        package="tactile_sim",
        executable="sim_search_sweep_node",
        output="screen",
        parameters=[
            {
                "search_pose_deg": [-87.0, -72.0, 89.0, 89.0, -90.0],
                "scan_joint1_angles_deg": [-90.0, -84.0, -78.0, -72.0],
            }
        ],
        condition=IfCondition(start_search_demo),
        **respawn_kwargs,
    )

    delayed_search_demo = TimerAction(
        period=search_start_delay_sec,
        actions=[
            target_pose_node,
            search_sweep_node,
        ],
        condition=IfCondition(start_search_demo),
    )

    pick_task_node = Node(
        package="tactile_task_cpp",
        executable="sim_pick_task_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
        condition=IfCondition(start_pick_demo),
        **respawn_kwargs,
    )

    delayed_pick_demo = TimerAction(
        period=pick_start_delay_sec,
        actions=[pick_task_node],
        condition=IfCondition(start_pick_demo),
    )

    return LaunchDescription(
        [
            start_sim_arg,
            start_gazebo_gui_arg,
            start_rviz_arg,
            use_sim_time_arg,
            use_gpu_accel_arg,
            server_use_gpu_accel_arg,
            world_arg,
            spawn_x_arg,
            spawn_y_arg,
            spawn_z_arg,
            gpu_adapter_arg,
            sim_start_demo_task_node_arg,
            sim_start_ui_subscriber_arg,
            rviz_config_arg,
            moveit_start_delay_sec_arg,
            start_search_demo_arg,
            search_start_delay_sec_arg,
            start_pick_demo_arg,
            pick_start_delay_sec_arg,
            sim_launch,
            delayed_moveit_stack,
            delayed_search_demo,
            delayed_pick_demo,
        ]
    )
