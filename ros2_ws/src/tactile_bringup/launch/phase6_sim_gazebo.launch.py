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
    default_world = PathJoinSubstitution(
        [FindPackageShare("tactile_sim"), "worlds", "phase6_tabletop.world"]
    )

    param_file_arg = DeclareLaunchArgument(
        "param_file",
        default_value=default_param_file,
        description="Path to phase6 Gazebo simulation YAML",
    )
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo world file passed through to tactile_sim/gazebo_arm.launch.py",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )
    start_gui_arg = DeclareLaunchArgument(
        "start_gui",
        default_value="false",
        description="Set true to launch Gazebo Sim GUI",
    )
    spawn_x_arg = DeclareLaunchArgument(
        "spawn_x",
        default_value="0.24",
        description="Spawn x position for the Gazebo robot entity",
    )
    spawn_y_arg = DeclareLaunchArgument(
        "spawn_y",
        default_value="0.0",
        description="Spawn y position for the Gazebo robot entity",
    )
    spawn_z_arg = DeclareLaunchArgument(
        "spawn_z",
        default_value="0.405",
        description="Spawn z position for the Gazebo robot entity",
    )
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="phase6_tabletop_world",
        description="Gazebo world name used for /clock bridge",
    )
    bridge_clock_arg = DeclareLaunchArgument(
        "bridge_clock",
        default_value="true",
        description="Bridge Gazebo clock to ROS /clock",
    )
    use_gpu_accel_arg = DeclareLaunchArgument(
        "use_gpu_accel",
        default_value="true",
        description="Use WSLg D3D12 GPU acceleration for Gazebo rendering",
    )
    gpu_adapter_arg = DeclareLaunchArgument(
        "gpu_adapter",
        default_value="NVIDIA",
        description="Preferred GPU adapter name for WSLg D3D12 rendering",
    )

    param_file = LaunchConfiguration("param_file")
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_gui = LaunchConfiguration("start_gui")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    world_name = LaunchConfiguration("world_name")
    bridge_clock = LaunchConfiguration("bridge_clock")
    use_gpu_accel = LaunchConfiguration("use_gpu_accel")
    gpu_adapter = LaunchConfiguration("gpu_adapter")

    gazebo_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("tactile_sim"), "launch", "gazebo_arm.launch.py"]
            )
        ),
        launch_arguments={
            "world": world,
            "use_sim_time": use_sim_time,
            "start_gui": start_gui,
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
            "world_name": world_name,
            "bridge_clock": bridge_clock,
            "use_gpu_accel": use_gpu_accel,
            "gpu_adapter": gpu_adapter,
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
            world_arg,
            use_sim_time_arg,
            start_gui_arg,
            spawn_x_arg,
            spawn_y_arg,
            spawn_z_arg,
            world_name_arg,
            bridge_clock_arg,
            use_gpu_accel_arg,
            gpu_adapter_arg,
            gazebo_arm_launch,
            tactile_sim_node,
            arm_sim_driver_node,
            arm_control_node,
            demo_task_node,
            tactile_ui_subscriber,
        ]
    )
