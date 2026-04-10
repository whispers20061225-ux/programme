import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _normalized_secret(value: str) -> str:
    secret = str(value or "").strip()
    if not secret or secret.upper() in {"EMPTY", "NONE", "NULL"}:
        return ""
    return secret


def _normalize_openai_compatible_endpoint(value: str) -> str:
    endpoint = str(value or "").strip()
    if not endpoint:
        return ""
    if endpoint.endswith("/v1"):
        return endpoint + "/chat/completions"
    if endpoint.endswith("/chat/completions"):
        return endpoint
    if endpoint.endswith("/"):
        endpoint = endpoint[:-1]
    if endpoint.endswith("/compatible-mode"):
        return endpoint + "/v1/chat/completions"
    return endpoint


def _load_remote_vlm_file() -> dict[str, str]:
    env_file = os.path.expanduser("~/.config/programme/remote_vlm.env")
    values: dict[str, str] = {}
    try:
        with open(env_file, "r", encoding="utf-8") as handle:
            for raw_line in handle:
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                if line.startswith("export "):
                    line = line[len("export ") :].strip()
                if "=" not in line:
                    continue
                key, value = line.split("=", 1)
                key = key.strip()
                value = value.strip()
                if len(value) >= 2 and value[0] == value[-1] and value[0] in {'"', "'"}:
                    value = value[1:-1]
                if key and value:
                    values[key] = value
    except OSError:
        return {}
    return values


def _remote_dialog_params() -> dict[str, str]:
    file_values = _load_remote_vlm_file()
    api_key = _normalized_secret(
        os.getenv("PROGRAMME_DIALOG_API_KEY")
        or file_values.get("PROGRAMME_DIALOG_API_KEY")
        or os.getenv("PROGRAMME_REMOTE_VLM_API_KEY")
        or file_values.get("PROGRAMME_REMOTE_VLM_API_KEY")
        or os.getenv("DASHSCOPE_API_KEY")
        or file_values.get("DASHSCOPE_API_KEY")
        or os.getenv("OPENAI_API_KEY")
        or file_values.get("OPENAI_API_KEY")
    )
    endpoint = _normalize_openai_compatible_endpoint(
        os.getenv("PROGRAMME_DIALOG_MODEL_ENDPOINT")
        or file_values.get("PROGRAMME_DIALOG_MODEL_ENDPOINT")
        or os.getenv("PROGRAMME_REMOTE_VLM_ENDPOINT")
        or file_values.get("PROGRAMME_REMOTE_VLM_ENDPOINT")
        or os.getenv("DASHSCOPE_BASE_URL")
        or file_values.get("DASHSCOPE_BASE_URL")
        or os.getenv("OPENAI_BASE_URL")
        or file_values.get("OPENAI_BASE_URL")
    )
    model_name = str(
        os.getenv("PROGRAMME_DIALOG_MODEL_NAME")
        or file_values.get("PROGRAMME_DIALOG_MODEL_NAME")
        or os.getenv("PROGRAMME_REMOTE_VLM_MODEL")
        or file_values.get("PROGRAMME_REMOTE_VLM_MODEL")
        or os.getenv("DASHSCOPE_MODEL")
        or file_values.get("DASHSCOPE_MODEL")
        or os.getenv("OPENAI_MODEL")
        or file_values.get("OPENAI_MODEL")
        or ""
    ).strip()

    if api_key and not endpoint:
        endpoint = "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions"
    if api_key and not model_name:
        model_name = "qwen-vl-max-latest"

    params: dict[str, str] = {}
    if endpoint:
        params["dialog_model_endpoint"] = endpoint
    if model_name:
        params["dialog_model_name"] = model_name
    if api_key:
        params["dialog_api_key"] = api_key
    return params


def generate_launch_description() -> LaunchDescription:
    stack_param_file = PathJoinSubstitution(
        [FindPackageShare("tactile_bringup"), "config", "programme_grasp_stack.yaml"]
    )
    gazebo_arm_launch = PathJoinSubstitution(
        [FindPackageShare("tactile_sim"), "launch", "gazebo_arm.launch.py"]
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
    use_gpu_accel = LaunchConfiguration("use_gpu_accel")
    server_use_gpu_accel = LaunchConfiguration("server_use_gpu_accel")
    gpu_adapter = LaunchConfiguration("gpu_adapter")
    sim_time_params = {"use_sim_time": ParameterValue(True, value_type=bool)}
    respawn_kwargs = {"respawn": True, "respawn_delay": 2.0}
    core_info_args = ["--ros-args", "--log-level", "info"]
    dialog_runtime_params = _remote_dialog_params()

    gazebo_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_arm_launch),
        launch_arguments={
            "use_sim_time": "true",
            "start_gui": "false",
            "startup_delay_sec": "2.0",
            "bridge_clock": "true",
            "bridge_camera": "true",
            "bridge_depth": "true",
            "bridge_imu": "true",
            "bridge_world_pose_info": "true",
            "bridge_world_control": "true",
            "bridge_world_set_pose": "true",
            "start_realsense_adapter": "true",
            "use_gpu_accel": use_gpu_accel,
            "server_use_gpu_accel": server_use_gpu_accel,
            "gpu_adapter": gpu_adapter,
        }.items(),
    )

    detector_seg_node = Node(
        package="tactile_vision",
        executable="detector_seg_node",
        name="detector_seg_node",
        output="screen",
        arguments=core_info_args,
        parameters=[
            stack_param_file,
            sim_time_params,
            {
                "backend": "yoloe_local",
                "open_vocab_model_path": "/home/whispers/programme/ros2_ws/yoloe-11s-seg.pt",
                "ultralytics_device": "cuda:0",
                "max_inference_rate_hz": 2.0,
                "publish_debug_overlay": True,
                "vlm_relabel_enabled": False,
            },
        ],
        **respawn_kwargs,
    )

    cloud_filter_node = Node(
        package="tactile_vision",
        executable="cloud_filter_node",
        name="cloud_filter_node",
        output="screen",
        arguments=core_info_args,
        parameters=[stack_param_file, sim_time_params],
        **respawn_kwargs,
    )

    web_gateway_node = Node(
        package="tactile_web_bridge",
        executable="tactile_web_gateway",
        name="tactile_web_gateway",
        output="screen",
        arguments=core_info_args,
        parameters=[stack_param_file, sim_time_params, dialog_runtime_params],
        **respawn_kwargs,
    )

    return LaunchDescription(
        [
            use_gpu_accel_arg,
            server_use_gpu_accel_arg,
            gpu_adapter_arg,
            gazebo_arm,
            detector_seg_node,
            cloud_filter_node,
            web_gateway_node,
        ]
    )
