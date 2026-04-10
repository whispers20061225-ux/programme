from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_stack_param_file = PathJoinSubstitution(
        [FindPackageShare("tactile_bringup"), "config", "programme_grasp_stack.yaml"]
    )
    default_vision_param_file = PathJoinSubstitution(
        [FindPackageShare("tactile_bringup"), "config", "vision_pipeline.yaml"]
    )
    grasp_stack_launch = PathJoinSubstitution(
        [FindPackageShare("tactile_bringup"), "launch", "programme_grasp_stack.launch.py"]
    )
    vision_pipeline_launch = PathJoinSubstitution(
        [FindPackageShare("tactile_bringup"), "launch", "vision_pipeline.launch.py"]
    )

    stack_param_file_arg = DeclareLaunchArgument(
        "stack_param_file",
        default_value=default_stack_param_file,
        description="Path to the Programme grasp stack parameter YAML",
    )
    vision_param_file_arg = DeclareLaunchArgument(
        "vision_param_file",
        default_value=default_vision_param_file,
        description="Path to the vision/realsense parameter YAML",
    )
    start_realsense_arg = DeclareLaunchArgument(
        "start_realsense",
        default_value="true",
        description="Start the RealSense camera driver and monitoring chain",
    )
    realsense_serial_no_arg = DeclareLaunchArgument(
        "realsense_serial_no",
        default_value="333422301846",
        description="Optional RealSense serial number filter",
    )
    start_web_gateway_arg = DeclareLaunchArgument(
        "start_web_gateway",
        default_value="true",
        description="Start the Programme web gateway together with the main stack",
    )
    start_gazebo_gui_arg = DeclareLaunchArgument(
        "start_gazebo_gui",
        default_value="true",
        description="Launch Gazebo GUI for the MoveIt demo stack",
    )
    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="true",
        description="Launch RViz for the MoveIt demo stack",
    )
    use_gpu_accel_arg = DeclareLaunchArgument(
        "use_gpu_accel",
        default_value="true",
        description="Use WSLg GPU acceleration for Gazebo GUI rendering",
    )
    server_use_gpu_accel_arg = DeclareLaunchArgument(
        "server_use_gpu_accel",
        default_value="false",
        description="Use GPU rendering on the Gazebo server",
    )
    gpu_adapter_arg = DeclareLaunchArgument(
        "gpu_adapter",
        default_value="NVIDIA",
        description="Preferred GPU adapter name for WSLg D3D12 rendering",
    )
    shadow_only_mode_arg = DeclareLaunchArgument(
        "shadow_only_mode",
        default_value="false",
        description="Run only perception/shadow visualization, without the formal task backend",
    )
    require_user_confirmation_arg = DeclareLaunchArgument(
        "require_user_confirmation",
        default_value="false",
        description="Require an explicit execute command before the pick task starts",
    )

    stack_param_file = LaunchConfiguration("stack_param_file")
    vision_param_file = LaunchConfiguration("vision_param_file")
    start_realsense = LaunchConfiguration("start_realsense")
    realsense_serial_no = LaunchConfiguration("realsense_serial_no")
    start_web_gateway = LaunchConfiguration("start_web_gateway")
    start_gazebo_gui = LaunchConfiguration("start_gazebo_gui")
    start_rviz = LaunchConfiguration("start_rviz")
    use_gpu_accel = LaunchConfiguration("use_gpu_accel")
    server_use_gpu_accel = LaunchConfiguration("server_use_gpu_accel")
    gpu_adapter = LaunchConfiguration("gpu_adapter")
    shadow_only_mode = LaunchConfiguration("shadow_only_mode")
    require_user_confirmation = LaunchConfiguration("require_user_confirmation")

    vision_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vision_pipeline_launch),
        launch_arguments={
            "param_file": vision_param_file,
            "start_realsense": start_realsense,
            "realsense_serial_no": realsense_serial_no,
        }.items(),
    )

    grasp_stack_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(grasp_stack_launch),
        launch_arguments={
            "stack_param_file": stack_param_file,
            "start_web_gateway": start_web_gateway,
            "start_gazebo_gui": start_gazebo_gui,
            "start_rviz": start_rviz,
            "use_gpu_accel": use_gpu_accel,
            "server_use_gpu_accel": server_use_gpu_accel,
            "gpu_adapter": gpu_adapter,
            "shadow_only_mode": shadow_only_mode,
            "require_user_confirmation": require_user_confirmation,
        }.items(),
    )

    return LaunchDescription(
        [
            stack_param_file_arg,
            vision_param_file_arg,
            start_realsense_arg,
            realsense_serial_no_arg,
            start_web_gateway_arg,
            start_gazebo_gui_arg,
            start_rviz_arg,
            use_gpu_accel_arg,
            server_use_gpu_accel_arg,
            gpu_adapter_arg,
            shadow_only_mode_arg,
            require_user_confirmation_arg,
            vision_bringup,
            grasp_stack_bringup,
        ]
    )
