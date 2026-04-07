from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    phase8_param_file_arg = DeclareLaunchArgument(
        "phase8_param_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("tactile_bringup"), "config", "phase8_modular_grasp.yaml"]
        ),
        description="Path to modular grasp parameter YAML",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock across the GraspGen shadow chain",
    )
    start_topk_bridge_arg = DeclareLaunchArgument(
        "start_topk_bridge",
        default_value="true",
        description="Publish the GraspGen top-k point cloud for RViz",
    )

    phase8_param_file = LaunchConfiguration("phase8_param_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_topk_bridge = LaunchConfiguration("start_topk_bridge")
    sim_time_params = {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}

    graspgen_shadow_node = Node(
        package="tactile_vision",
        executable="grasp_backend_node",
        name="graspgen_shadow_node",
        output="screen",
        parameters=[
            phase8_param_file,
            sim_time_params,
            {
                "backend": "graspgen_zmq",
                "candidate_grasp_proposal_array_topic": "/grasp/graspgen_shadow/candidate_grasp_proposals",
                "candidate_markers_topic": "/grasp/graspgen_shadow/candidate_markers",
                "selected_contact_point_1_topic": "/grasp/graspgen_shadow/selected_contact_point_1_cloud",
                "selected_contact_point_2_topic": "/grasp/graspgen_shadow/selected_contact_point_2_cloud",
                "selected_grasp_center_topic": "/grasp/graspgen_shadow/selected_grasp_center_cloud",
                "publish_empty_on_failure": False,
            },
        ],
    )

    graspgen_topk_bridge_node = Node(
        package="tactile_vision",
        executable="graspgen_topk_bridge_node",
        name="graspgen_topk_bridge_node",
        output="screen",
        condition=IfCondition(start_topk_bridge),
        parameters=[phase8_param_file, sim_time_params],
    )

    return LaunchDescription(
        [
            phase8_param_file_arg,
            use_sim_time_arg,
            start_topk_bridge_arg,
            graspgen_shadow_node,
            graspgen_topk_bridge_node,
        ]
    )
