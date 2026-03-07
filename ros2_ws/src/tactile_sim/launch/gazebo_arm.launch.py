import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_world = PathJoinSubstitution(
        [FindPackageShare("tactile_sim"), "worlds", "phase6_tabletop.world"]
    )
    default_xacro = PathJoinSubstitution(
        [FindPackageShare("tactile_sim"), "urdf", "dofbot_gazebo.urdf.xacro"]
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo Sim world file",
    )
    xacro_file_arg = DeclareLaunchArgument(
        "xacro_file",
        default_value=default_xacro,
        description="Robot xacro file",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )
    entity_name_arg = DeclareLaunchArgument(
        "entity_name",
        default_value="dofbot",
        description="Spawned entity name",
    )
    spawn_x_arg = DeclareLaunchArgument(
        "spawn_x",
        default_value="0.24",
        description="Spawn x position in the Gazebo world",
    )
    spawn_y_arg = DeclareLaunchArgument(
        "spawn_y",
        default_value="0.0",
        description="Spawn y position in the Gazebo world",
    )
    spawn_z_arg = DeclareLaunchArgument(
        "spawn_z",
        default_value="0.405",
        description="Spawn z position in the Gazebo world",
    )
    start_gui_arg = DeclareLaunchArgument(
        "start_gui",
        default_value="false",
        description="Set true to launch Gazebo Sim GUI",
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

    world = LaunchConfiguration("world")
    xacro_file = LaunchConfiguration("xacro_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    entity_name = LaunchConfiguration("entity_name")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    start_gui = LaunchConfiguration("start_gui")
    world_name = LaunchConfiguration("world_name")
    bridge_clock = LaunchConfiguration("bridge_clock")
    tactile_sim_share = FindPackageShare("tactile_sim")
    tactile_bringup_share = FindPackageShare("tactile_bringup")

    current_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    cleaned_ld_library_path = ":".join(
        p for p in current_ld_library_path.split(":") if p and "/snap/" not in p
    )
    sanitize_ld_library_path = SetEnvironmentVariable(
        name="LD_LIBRARY_PATH",
        value=cleaned_ld_library_path,
    )
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            tactile_sim_share,
            os.pathsep,
            tactile_bringup_share,
            os.pathsep,
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
        ],
    )
    set_ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            tactile_sim_share,
            os.pathsep,
            tactile_bringup_share,
            os.pathsep,
            EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
        ],
    )

    gazebo_headless_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": ["-r -s ", world]}.items(),
        condition=UnlessCondition(start_gui),
    )

    gazebo_gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": ["-r ", world]}.items(),
        condition=IfCondition(start_gui),
    )

    robot_description = Command([FindExecutable(name="xacro"), " ", xacro_file])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_dofbot",
        output="screen",
        arguments=[
            "-name",
            entity_name,
            "-topic",
            "robot_description",
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
        ],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=[
            ["/world/", world_name, "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            "--ros-args",
            "-r",
            ["/world/", world_name, "/clock:=/clock"],
        ],
        condition=IfCondition(bridge_clock),
    )

    spawner_joint_state = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    spawner_joint_trajectory = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_trajectory_controller",
        output="screen",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    delay_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[spawner_joint_state],
        )
    )

    delay_joint_trajectory = RegisterEventHandler(
        OnProcessExit(
            target_action=spawner_joint_state,
            on_exit=[spawner_joint_trajectory],
        )
    )

    return LaunchDescription(
        [
            world_arg,
            xacro_file_arg,
            use_sim_time_arg,
            entity_name_arg,
            spawn_x_arg,
            spawn_y_arg,
            spawn_z_arg,
            start_gui_arg,
            world_name_arg,
            bridge_clock_arg,
            sanitize_ld_library_path,
            set_gz_resource_path,
            set_ign_resource_path,
            gazebo_headless_launch,
            gazebo_gui_launch,
            robot_state_publisher,
            clock_bridge,
            spawn_entity,
            delay_joint_state,
            delay_joint_trajectory,
        ]
    )
