import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO_ROOT / "src" / "core" / "ros2_runtime_stubs.py"
MODULE_ERROR = None
Ros2ControlThread = None

try:
    spec = importlib.util.spec_from_file_location("ros2_runtime_stubs_module", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    Ros2ControlThread = module.Ros2ControlThread
except Exception as exc:  # pragma: no cover - environment dependent
    MODULE_ERROR = exc


@unittest.skipIf(MODULE_ERROR is not None, f"ros2_runtime_stubs import unavailable: {MODULE_ERROR}")
class Ros2ControlThreadSimulationTests(unittest.TestCase):
    def test_on_health_accepts_simulated_nodes(self):
        thread = Ros2ControlThread()

        thread.on_health(
            SimpleNamespace(
                node_name="tactile_sim_node",
                healthy=True,
                level=0,
                message="simulation active",
            )
        )
        self.assertTrue(thread._tactile_connected)
        self.assertTrue(thread._sensor_simulation)

        thread._arm_state_received_at = 0.0
        thread._arm_snapshot["connected"] = False
        thread.on_health(
            SimpleNamespace(
                node_name="arm_sim_driver_node",
                healthy=True,
                level=0,
                message="sim arm active",
            )
        )
        self.assertTrue(thread._stm32_connected)
        self.assertTrue(thread._servo_simulation)

    def test_on_arm_state_marks_simulation_connection_type(self):
        thread = Ros2ControlThread()
        payloads = []
        thread.status_updated.connect(lambda status, info: payloads.append((status, info)))

        thread.on_arm_state(
            SimpleNamespace(
                header=SimpleNamespace(frame_id="sim_arm_ros2_control"),
                connected=True,
                joint_angles=[0.0] * 6,
                joint_positions=[0.0] * 6,
                battery_voltage=12.0,
                error=False,
                error_message="",
                moving=False,
            )
        )

        self.assertTrue(thread._servo_simulation)
        self.assertTrue(payloads)
        status, snapshot = payloads[-1]
        self.assertEqual(status, "arm_state")
        self.assertEqual(snapshot["connection_type"], "simulation")


if __name__ == "__main__":
    unittest.main()
