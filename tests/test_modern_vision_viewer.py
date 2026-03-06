import importlib.util
import os
from pathlib import Path
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO_ROOT / "src" / "gui" / "modern_vision_viewer.py"
MODULE_ERROR = None
module = None
np = None
QImage = None

try:
    spec = importlib.util.spec_from_file_location("modern_vision_viewer_module", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    import numpy as np
    from PyQt5.QtGui import QImage
except Exception as exc:  # pragma: no cover - environment dependent
    MODULE_ERROR = exc
    module = None

if module is not None:
    depth_array_to_qimage = module.depth_array_to_qimage
    rgb_array_to_qimage = module.rgb_array_to_qimage
else:
    depth_array_to_qimage = None
    rgb_array_to_qimage = None


@unittest.skipIf(MODULE_ERROR is not None, f"modern_vision_viewer import unavailable: {MODULE_ERROR}")
class ModernVisionViewerTests(unittest.TestCase):
    def test_uint16_depth_uses_grayscale16(self):
        depth = (np.arange(16, dtype=np.uint16).reshape(4, 4) * 100)
        image = depth_array_to_qimage(depth)
        self.assertIsInstance(image, QImage)
        self.assertFalse(image.isNull())
        self.assertEqual(image.format(), QImage.Format_Grayscale16)

    def test_float_depth_uses_fixed_grayscale_fallback(self):
        depth = np.linspace(0.0, 5.0, 16, dtype=np.float32).reshape(4, 4)
        image = depth_array_to_qimage(depth)
        self.assertIsInstance(image, QImage)
        self.assertFalse(image.isNull())
        self.assertEqual(image.format(), QImage.Format_Grayscale16)

    def test_rgb_array_to_qimage_preserves_rgb888(self):
        rgb = np.zeros((4, 4, 3), dtype=np.uint8)
        rgb[:, :, 0] = 255
        image = rgb_array_to_qimage(rgb)
        self.assertIsInstance(image, QImage)
        self.assertFalse(image.isNull())
        self.assertEqual(image.format(), QImage.Format_RGB888)

    def test_viewer_init_defers_pointcloud_tab_signal(self):
        from PyQt5.QtWidgets import QApplication

        app = QApplication.instance() or QApplication([])

        class DummyConfig:
            detection_model = "YOLOv5"
            confidence_threshold = 0.5
            nms_threshold = 0.45

        viewer = module.VisionViewer(DummyConfig())
        try:
            self.assertTrue(hasattr(viewer, "pointcloud_tab"))
            self.assertIsNotNone(viewer.pointcloud_tab)
            self.assertGreaterEqual(viewer.image_tabs.count(), 5)
        finally:
            viewer.close()


if __name__ == "__main__":
    unittest.main()
