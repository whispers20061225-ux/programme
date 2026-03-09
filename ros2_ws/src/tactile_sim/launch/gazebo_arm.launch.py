import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
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
        description="Use WSLg D3D12 OpenGL acceleration for Gazebo rendering",
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
    set_libgl_software = SetEnvironmentVariable(
        name="LIBGL_ALWAYS_SOFTWARE",
        value=["0"],
        condition=IfCondition(use_gpu_accel),
    )
    set_gallium_driver = SetEnvironmentVariable(
        name="GALLIUM_DRIVER",
        value=["d3d12"],
        condition=IfCondition(use_gpu_accel),
    )
    set_mesa_adapter = SetEnvironmentVariable(
        name="MESA_D3D12_DEFAULT_ADAPTER_NAME",
        value=[gpu_adapter],
        condition=IfCondition(use_gpu_accel),
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

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="wrist_rgb_camera_bridge",
        output="screen",
        arguments=[
            "/camera@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        ],
        remappings=[
            ("/camera", ros_camera_topic),
            ("/camera_info", ros_camera_info_topic),
        ],
        condition=IfCondition(bridge_camera),
    )

    depth_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="wrist_depth_camera_bridge",
        output="screen",
        arguments=[
            "/camera_depth@sensor_msgs/msg/Image@gz.msgs.Image",
        ],
        remappings=[
            ("/camera_depth", "/sim/camera/depth/image_raw"),
        ],
        condition=IfCondition(bridge_depth),
    )

    imu_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="wrist_imu_bridge",
        output="screen",
        arguments=[
            "/camera_imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
        ],
        remappings=[
            ("/camera_imu", "/sim/camera/imu"),
        ],
        condition=IfCondition(bridge_imu),
    )

    sim_realsense_adapter = Node(
        package="tactile_sim",
        executable="sim_realsense_adapter_node",
        name="sim_realsense_adapter_node",
        output="screen",
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
            gpu_adapter_arg,
            sanitize_ld_library_path,
            set_gz_resource_path,
            set_ign_resource_path,
            set_libgl_software,
            set_gallium_driver,
            set_mesa_adapter,
            gazebo_headless_launch,
            gazebo_gui_launch,
            clock_bridge,
            delayed_robot_pipeline,
            delay_joint_state,
            delay_joint_trajectory,
        ]
    )
