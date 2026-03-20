import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    quiet_node_args = ["--ros-args", "--log-level", "warn"]
    node_respawn_kwargs = {"respawn": True, "respawn_delay": 2.0}

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
    startup_delay_sec_arg = DeclareLaunchArgument(
        "startup_delay_sec",
        default_value="2.0",
        description="Delay ROS nodes that consume sim time until /clock is stable",
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
    bridge_camera_arg = DeclareLaunchArgument(
        "bridge_camera",
        default_value="true",
        description="Bridge Gazebo wrist RGB camera topics into ROS",
    )
    bridge_depth_arg = DeclareLaunchArgument(
        "bridge_depth",
        default_value="true",
        description="Bridge Gazebo wrist depth camera topic into ROS",
    )
    bridge_imu_arg = DeclareLaunchArgument(
        "bridge_imu",
        default_value="true",
        description="Bridge Gazebo wrist IMU topic into ROS",
    )
    start_realsense_adapter_arg = DeclareLaunchArgument(
        "start_realsense_adapter",
        default_value="true",
        description="Expose simulated depth and IMU as RealSense-style ROS topics",
    )
    ros_camera_topic_arg = DeclareLaunchArgument(
        "ros_camera_topic",
        default_value="/camera/camera/color/image_raw",
        description="ROS image topic exposed for the simulated wrist RGB camera",
    )
    ros_camera_info_topic_arg = DeclareLaunchArgument(
        "ros_camera_info_topic",
        default_value="/camera/camera/color/camera_info",
        description="ROS camera_info topic exposed for the simulated wrist RGB camera",
    )
    ros_depth_topic_arg = DeclareLaunchArgument(
        "ros_depth_topic",
        default_value="/camera/camera/depth/image_rect_raw",
        description="ROS depth image topic exposed for the simulated wrist camera",
    )
    ros_aligned_depth_topic_arg = DeclareLaunchArgument(
        "ros_aligned_depth_topic",
        default_value="/camera/camera/aligned_depth_to_color/image_raw",
        description="ROS aligned depth image topic exposed for the simulated wrist camera",
    )
    ros_depth_camera_info_topic_arg = DeclareLaunchArgument(
        "ros_depth_camera_info_topic",
        default_value="/camera/camera/depth/camera_info",
        description="ROS depth camera_info topic exposed for the simulated wrist camera",
    )
    ros_aligned_depth_camera_info_topic_arg = DeclareLaunchArgument(
        "ros_aligned_depth_camera_info_topic",
        default_value="/camera/camera/aligned_depth_to_color/camera_info",
        description="ROS aligned depth camera_info topic exposed for the simulated wrist camera",
    )
    ros_imu_topic_arg = DeclareLaunchArgument(
        "ros_imu_topic",
        default_value="/camera/camera/imu",
        description="ROS combined IMU topic exposed for the simulated wrist camera",
    )
    ros_gyro_topic_arg = DeclareLaunchArgument(
        "ros_gyro_topic",
        default_value="/camera/camera/gyro/sample",
        description="ROS gyro topic exposed for the simulated wrist camera",
    )
    ros_accel_topic_arg = DeclareLaunchArgument(
        "ros_accel_topic",
        default_value="/camera/camera/accel/sample",
        description="ROS accel topic exposed for the simulated wrist camera",
    )
    use_gpu_accel_arg = DeclareLaunchArgument(
        "use_gpu_accel",
        default_value="true",
        description="Use WSLg D3D12 OpenGL acceleration for the Gazebo GUI client",
    )
    server_use_gpu_accel_arg = DeclareLaunchArgument(
        "server_use_gpu_accel",
        default_value="false",
        description="Use GPU rendering on the Gazebo server. Keep false under WSLg when you need valid depth camera images.",
    )
    gpu_adapter_arg = DeclareLaunchArgument(
        "gpu_adapter",
        default_value="NVIDIA",
        description="Preferred WSLg adapter name passed to Mesa D3D12",
    )

    world = LaunchConfiguration("world")
    xacro_file = LaunchConfiguration("xacro_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    entity_name = LaunchConfiguration("entity_name")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    start_gui = LaunchConfiguration("start_gui")
    startup_delay_sec = LaunchConfiguration("startup_delay_sec")
    world_name = LaunchConfiguration("world_name")
    bridge_clock = LaunchConfiguration("bridge_clock")
    bridge_camera = LaunchConfiguration("bridge_camera")
    bridge_depth = LaunchConfiguration("bridge_depth")
    bridge_imu = LaunchConfiguration("bridge_imu")
    start_realsense_adapter = LaunchConfiguration("start_realsense_adapter")
    ros_camera_topic = LaunchConfiguration("ros_camera_topic")
    ros_camera_info_topic = LaunchConfiguration("ros_camera_info_topic")
    ros_depth_topic = LaunchConfiguration("ros_depth_topic")
    ros_aligned_depth_topic = LaunchConfiguration("ros_aligned_depth_topic")
    ros_depth_camera_info_topic = LaunchConfiguration("ros_depth_camera_info_topic")
    ros_aligned_depth_camera_info_topic = LaunchConfiguration(
        "ros_aligned_depth_camera_info_topic"
    )
    ros_imu_topic = LaunchConfiguration("ros_imu_topic")
    ros_gyro_topic = LaunchConfiguration("ros_gyro_topic")
    ros_accel_topic = LaunchConfiguration("ros_accel_topic")
    use_gpu_accel = LaunchConfiguration("use_gpu_accel")
    server_use_gpu_accel = LaunchConfiguration("server_use_gpu_accel")
    gpu_adapter = LaunchConfiguration("gpu_adapter")
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
    set_gz_system_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=[
            EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
            os.pathsep,
            EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
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

    gazebo_server_gpu = ExecuteProcess(
        cmd=[
            FindExecutable(name="gz"),
            "sim",
            "-r",
            "-s",
            world,
            "--force-version",
            "8",
        ],
        name="gazebo",
        output={"stdout": "log", "stderr": "log"},
        additional_env={
            "LIBGL_ALWAYS_SOFTWARE": "0",
            "GALLIUM_DRIVER": "d3d12",
            "MESA_D3D12_DEFAULT_ADAPTER_NAME": gpu_adapter,
        },
        condition=IfCondition(server_use_gpu_accel),
    )

    gazebo_server_cpu = ExecuteProcess(
        cmd=[
            FindExecutable(name="gz"),
            "sim",
            "-r",
            "-s",
            world,
            "--force-version",
            "8",
        ],
        name="gazebo",
        output={"stdout": "log", "stderr": "log"},
        additional_env={
            "LIBGL_ALWAYS_SOFTWARE": "1",
            "GALLIUM_DRIVER": "llvmpipe",
            "MESA_D3D12_DEFAULT_ADAPTER_NAME": "",
        },
        condition=UnlessCondition(server_use_gpu_accel),
    )

    restart_on_gazebo_server_gpu_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=gazebo_server_gpu,
            on_exit=[
                LogInfo(msg="gazebo server exited; shutting down launch so the supervisor can restart the full stack"),
                EmitEvent(event=Shutdown(reason="gazebo server exited")),
            ],
        )
    )

    restart_on_gazebo_server_cpu_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=gazebo_server_cpu,
            on_exit=[
                LogInfo(msg="gazebo server exited; shutting down launch so the supervisor can restart the full stack"),
                EmitEvent(event=Shutdown(reason="gazebo server exited")),
            ],
        )
    )

    gazebo_gui_gpu = ExecuteProcess(
        cmd=[
            FindExecutable(name="gz"),
            "sim",
            "-g",
            "--force-version",
            "8",
        ],
        name="gazebo_gui",
        output={"stdout": "log", "stderr": "log"},
        additional_env={
            "LIBGL_ALWAYS_SOFTWARE": "0",
            "GALLIUM_DRIVER": "d3d12",
            "MESA_D3D12_DEFAULT_ADAPTER_NAME": gpu_adapter,
        },
        condition=IfCondition(
            PythonExpression(
                ["'", start_gui, "' == 'true' and '", use_gpu_accel, "' == 'true'"]
            )
        ),
    )

    gazebo_gui_cpu = ExecuteProcess(
        cmd=[
            FindExecutable(name="gz"),
            "sim",
            "-g",
            "--force-version",
            "8",
        ],
        name="gazebo_gui",
        output={"stdout": "log", "stderr": "log"},
        additional_env={
            "LIBGL_ALWAYS_SOFTWARE": "1",
            "GALLIUM_DRIVER": "llvmpipe",
            "MESA_D3D12_DEFAULT_ADAPTER_NAME": "",
        },
        condition=IfCondition(
            PythonExpression(
                ["'", start_gui, "' == 'true' and '", use_gpu_accel, "' != 'true'"]
            )
        ),
    )

    delayed_gazebo_gui = TimerAction(
        period=1.0,
        actions=[
            gazebo_gui_gpu,
            gazebo_gui_cpu,
        ],
        condition=IfCondition(start_gui),
    )

    robot_description = Command([FindExecutable(name="xacro"), " ", xacro_file])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output={"stdout": "log", "stderr": "log"},
        arguments=quiet_node_args,
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
        **node_respawn_kwargs,
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_dofbot",
        output={"stdout": "log", "stderr": "log"},
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
        ] + quiet_node_args,
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output={"stdout": "log", "stderr": "log"},
        arguments=[
            ["/world/", world_name, "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            "--ros-args",
            "--log-level",
            "warn",
            "-r",
            ["/world/", world_name, "/clock:=/clock"],
        ],
        condition=IfCondition(bridge_clock),
        **node_respawn_kwargs,
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="wrist_rgb_camera_bridge",
        output={"stdout": "log", "stderr": "log"},
        arguments=[
            "/camera@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        ] + quiet_node_args,
        remappings=[
            ("/camera", ros_camera_topic),
            ("/camera_info", ros_camera_info_topic),
        ],
        condition=IfCondition(bridge_camera),
        **node_respawn_kwargs,
    )

    depth_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="wrist_depth_camera_bridge",
        output={"stdout": "log", "stderr": "log"},
        arguments=[
            "/camera_depth@sensor_msgs/msg/Image@gz.msgs.Image",
        ] + quiet_node_args,
        remappings=[
            ("/camera_depth", "/sim/camera/depth/image_raw"),
        ],
        condition=IfCondition(bridge_depth),
        **node_respawn_kwargs,
    )

    imu_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="wrist_imu_bridge",
        output={"stdout": "log", "stderr": "log"},
        arguments=[
            "/camera_imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
        ] + quiet_node_args,
        remappings=[
            ("/camera_imu", "/sim/camera/imu"),
        ],
        condition=IfCondition(bridge_imu),
        **node_respawn_kwargs,
    )

    sim_realsense_adapter = Node(
        package="tactile_sim",
        executable="sim_realsense_adapter_node",
        name="sim_realsense_adapter_node",
        output={"stdout": "log", "stderr": "log"},
        arguments=quiet_node_args,
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_depth_topic": "/sim/camera/depth/image_raw",
                "input_imu_topic": "/sim/camera/imu",
                "input_color_camera_info_topic": ros_camera_info_topic,
                "output_depth_topic": ros_depth_topic,
                "output_aligned_depth_topic": ros_aligned_depth_topic,
                "output_depth_camera_info_topic": ros_depth_camera_info_topic,
                "output_aligned_depth_camera_info_topic": ros_aligned_depth_camera_info_topic,
                "output_imu_topic": ros_imu_topic,
                "output_gyro_topic": ros_gyro_topic,
                "output_accel_topic": ros_accel_topic,
                "depth_frame_id": "camera_depth_optical_frame",
                "aligned_depth_frame_id": "camera_color_optical_frame",
                "imu_frame_id": "camera_imu_optical_frame",
                "gyro_frame_id": "camera_gyro_optical_frame",
                "accel_frame_id": "camera_accel_optical_frame",
            }
        ],
        condition=IfCondition(start_realsense_adapter),
        **node_respawn_kwargs,
    )

    spawner_joint_state = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        output={"stdout": "log", "stderr": "log"},
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ] + quiet_node_args,
    )

    spawner_joint_trajectory = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_trajectory_controller",
        output="log",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ] + quiet_node_args,
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

    delayed_robot_pipeline = TimerAction(
        period=startup_delay_sec,
        actions=[
            robot_state_publisher,
            camera_bridge,
            depth_bridge,
            imu_bridge,
            sim_realsense_adapter,
            spawn_entity,
        ],
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
            startup_delay_sec_arg,
            world_name_arg,
            bridge_clock_arg,
            bridge_camera_arg,
            bridge_depth_arg,
            bridge_imu_arg,
            start_realsense_adapter_arg,
            ros_camera_topic_arg,
            ros_camera_info_topic_arg,
            ros_depth_topic_arg,
            ros_aligned_depth_topic_arg,
            ros_depth_camera_info_topic_arg,
            ros_aligned_depth_camera_info_topic_arg,
            ros_imu_topic_arg,
            ros_gyro_topic_arg,
            ros_accel_topic_arg,
            use_gpu_accel_arg,
            server_use_gpu_accel_arg,
            gpu_adapter_arg,
            sanitize_ld_library_path,
            set_gz_resource_path,
            set_gz_system_plugin_path,
            set_ign_resource_path,
            gazebo_server_gpu,
            gazebo_server_cpu,
            restart_on_gazebo_server_gpu_exit,
            restart_on_gazebo_server_cpu_exit,
            delayed_gazebo_gui,
            clock_bridge,
            delayed_robot_pipeline,
            delay_joint_state,
            delay_joint_trajectory,
        ]
    )
