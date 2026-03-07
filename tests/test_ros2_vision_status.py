import importlib.util
from pathlib import Path
import sys
import unittest

REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO_ROOT / "src" / "core" / "ros2_data_acquisition.py"
MODULE_ERROR = None
Ros2CameraFrame = None
Ros2DataAcquisitionThread = None

try:
    spec = importlib.util.spec_from_file_location("ros2_data_acquisition_module", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    Ros2CameraFrame = module.Ros2CameraFrame
    Ros2DataAcquisitionThread = module.Ros2DataAcquisitionThread
except Exception as exc:  # pragma: no cover - environment dependent
    MODULE_ERROR = exc


@unittest.skipIf(MODULE_ERROR is not None, f"ros2_data_acquisition import unavailable: {MODULE_ERROR}")
class Ros2VisionStatusTests(unittest.TestCase):
    def test_emit_vision_status_exposes_extended_metrics(self):
        thread = Ros2DataAcquisitionThread(config=None, vision_enabled=True)
        payloads = []
        thread.vision_status.connect(payloads.append)

        with thread._vision_lock:
            thread._vision_stream_requested = True
            thread._vision_connected = True
            thread._vision_last_color_ts = 10.0
            thread._vision_last_depth_ts = 10.0
            thread._vision_status_emit_ts = 0.0
            thread._vision_prev_status_ts = 9.0
            thread._vision_prev_color_count = 10
            thread._vision_color_count = 25
            thread._vision_resolution = "640x480"
            thread._vision_dropped_frames = 7
            thread._vision_queue_overwrite_count = 2
            thread._vision_latest_frame = Ros2CameraFrame(10.0, None, None, {})
            thread._vision_latest_frame_seq = 3
            thread._vision_consumed_frame_seq = 1

        thread._emit_vision_status(now=11.0, force=True, message="ok")
        self.assertTrue(payloads)
        status = payloads[-1]
        self.assertIn("fps", status)
        self.assertIn("rx_fps", status)
        self.assertIn("render_fps", status)
        self.assertIn("dropped_frames", status)
        self.assertIn("queue_overwrite_count", status)
        self.assertIn("last_frame_age_ms", status)
        self.assertIn("stall_count", status)
        self.assertEqual(status["rx_fps"], status["fps"])
        self.assertEqual(status["dropped_frames"], 7)
        self.assertEqual(status["queue_overwrite_count"], 2)
        self.assertGreaterEqual(status["last_frame_age_ms"], 0.0)

    def test_request_vision_connect_is_noop_when_sidecar_stream_already_requested(self):
        thread = Ros2DataAcquisitionThread(config=None, vision_enabled=True)
        thread._vision_sidecar_enabled = True
        thread._vision_stream_requested = True
        commands = []
        payloads = []
        thread._send_vision_sidecar_command = lambda action, **payload: commands.append((action, payload)) or True
        thread.vision_status.connect(payloads.append)

        thread.request_vision_connect()

        self.assertEqual(commands, [])
        self.assertEqual(payloads, [])

    def test_request_vision_connect_force_restart_reissues_sidecar_connect(self):
        thread = Ros2DataAcquisitionThread(config=None, vision_enabled=True)
        thread._vision_sidecar_enabled = True
        thread._vision_stream_requested = True
        thread._vision_connected = True
        thread._vision_last_color_ts = 10.0
        commands = []
        payloads = []
        thread._send_vision_sidecar_command = lambda action, **payload: commands.append((action, payload)) or True
        thread.vision_status.connect(payloads.append)

        thread.request_vision_connect(force_restart=True)

        self.assertEqual(commands, [("connect", {})])
        self.assertTrue(payloads)
        self.assertFalse(payloads[-1]["connected"])
        self.assertEqual(payloads[-1]["message"], "Waiting for ROS2 camera frames...")
        self.assertEqual(thread._vision_last_color_ts, 0.0)


if __name__ == "__main__":
    unittest.main()
