from pathlib import Path
import unittest


class CameraPreviewProfileTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = Path("quest3_camera_stream.py").read_text(encoding="utf-8")

    def test_registered_profile_remains_default_for_capture_fidelity(self):
        self.assertIn('default="registered"', self.source)
        self.assertIn('input_width = int(config["width"])', self.source)
        self.assertIn('input_height = int(config["height"])', self.source)
        self.assertIn('input_fps = int(config["fps"])', self.source)

    def test_low_profile_changes_only_camera_stream_input(self):
        self.assertIn('choices=("registered", "preview-low")', self.source)
        self.assertIn("WIDTH = 640", self.source)
        self.assertIn("HEIGHT = 360", self.source)


if __name__ == "__main__":
    unittest.main()
