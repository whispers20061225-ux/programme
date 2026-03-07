from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]


class GazeboDofbotConfigTests(unittest.TestCase):
    def test_gazebo_launch_defaults_to_dofbot_model(self):
        launch_text = (
            REPO_ROOT
            / "ros2_ws"
            / "src"
            / "tactile_sim"
            / "launch"
            / "gazebo_arm.launch.py"
        ).read_text(encoding="utf-8")

        self.assertIn("dofbot_gazebo.urdf.xacro", launch_text)
        self.assertIn('default_value="dofbot"', launch_text)
        self.assertIn('default_value="0.24"', launch_text)

    def test_controller_config_uses_dofbot_joint_names(self):
        controller_text = (
            REPO_ROOT
            / "ros2_ws"
            / "src"
            / "tactile_sim"
            / "config"
            / "ros2_controllers.yaml"
        ).read_text(encoding="utf-8")

        for joint_name in (
            "arm_joint1",
            "arm_joint2",
            "arm_joint3",
            "arm_joint4",
            "arm_joint5",
            "grip_joint",
        ):
            self.assertIn(joint_name, controller_text)

        self.assertNotIn("\n      - joint1\n", controller_text)

    def test_phase6_gazebo_profile_matches_dofbot_semantics(self):
        profile_text = (
            REPO_ROOT
            / "ros2_ws"
            / "src"
            / "tactile_bringup"
            / "config"
            / "phase6_sim_gazebo.yaml"
        ).read_text(encoding="utf-8")

        self.assertIn('joint_names: ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5", "grip_joint"]', profile_text)
        self.assertIn("joint_zero_offsets_deg: [-90.0, 0.0, 0.0, 0.0, -90.0, 0.0]", profile_text)
        self.assertIn("default_open_angle_deg: 0.0", profile_text)
        self.assertIn("default_close_angle_deg: -80.0", profile_text)

    def test_dofbot_xacro_uses_packaged_meshes_and_ros2_control(self):
        xacro_text = (
            REPO_ROOT
            / "ros2_ws"
            / "src"
            / "tactile_sim"
            / "urdf"
            / "dofbot_gazebo.urdf.xacro"
        ).read_text(encoding="utf-8")

        self.assertIn("file://$(find tactile_sim)/meshes", xacro_text)
        self.assertIn('joint name="arm_joint1"', xacro_text)
        self.assertIn('joint name="grip_joint"', xacro_text)
        self.assertIn("gz_ros2_control/GazeboSimSystem", xacro_text)

    def test_gazebo_runtime_exports_resource_paths(self):
        vm_env_text = (
            REPO_ROOT
            / "deploy"
            / "vm"
            / "env_ros2_vm.sh"
        ).read_text(encoding="utf-8")
        launch_text = (
            REPO_ROOT
            / "ros2_ws"
            / "src"
            / "tactile_sim"
            / "launch"
            / "gazebo_arm.launch.py"
        ).read_text(encoding="utf-8")

        self.assertIn("GZ_SIM_RESOURCE_PATH", vm_env_text)
        self.assertIn("IGN_GAZEBO_RESOURCE_PATH", vm_env_text)
        self.assertIn("GZ_SIM_RESOURCE_PATH", launch_text)


if __name__ == "__main__":
    unittest.main()
