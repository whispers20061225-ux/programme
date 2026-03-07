import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO_ROOT / "main_ros2.py"
MODULE_ERROR = None
Ros2PhaseApp = None

try:
    spec = importlib.util.spec_from_file_location("main_ros2_module", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    Ros2PhaseApp = module.Ros2PhaseApp
except Exception as exc:  # pragma: no cover - environment dependent
    MODULE_ERROR = exc


@unittest.skipIf(MODULE_ERROR is not None, f"main_ros2 import unavailable: {MODULE_ERROR}")
class Ros2PhaseAppUiOverrideTests(unittest.TestCase):
    def test_runtime_disabled_hides_vision_ui_by_default(self):
        app = Ros2PhaseApp.__new__(Ros2PhaseApp)
        app.vision_enabled = False
        app.config = SimpleNamespace(
            ui=SimpleNamespace(
                vision_ui={"show_camera_view": True},
                simulation_ui={"show_simulation_view": True},
            )
        )

        Ros2PhaseApp._apply_ui_overrides(
            app,
            show_vision_ui=None,
            show_simulation_ui=False,
        )

        self.assertFalse(app.config.ui.vision_ui["show_camera_view"])
        self.assertFalse(app.config.ui.simulation_ui["show_simulation_view"])

    def test_explicit_vision_ui_override_is_respected(self):
        app = Ros2PhaseApp.__new__(Ros2PhaseApp)
        app.vision_enabled = False
        app.config = SimpleNamespace(
            ui=SimpleNamespace(
                vision_ui={"show_camera_view": False},
                simulation_ui={"show_simulation_view": False},
            )
        )

        Ros2PhaseApp._apply_ui_overrides(
            app,
            show_vision_ui=True,
            show_simulation_ui=None,
        )

        self.assertTrue(app.config.ui.vision_ui["show_camera_view"])


if __name__ == "__main__":
    unittest.main()
