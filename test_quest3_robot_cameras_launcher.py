from pathlib import Path
import unittest


class QuestRobotCamerasLauncherTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.launcher = Path("tools/run_quest3_robot_cameras_test.sh").read_text(
            encoding="utf-8"
        )
        cls.stream = Path("quest3_camera_stream.py").read_text(encoding="utf-8")

    def test_recording_is_not_started(self):
        self.assertNotIn("RobotRecorder", self.launcher)
        self.assertNotIn("capture_demo.py --", self.launcher)

    def test_robot_is_ready_before_cameras_and_quest(self):
        ready = self.launcher.index('[[ -f "$READY_FILE" ]]')
        cameras = self.launcher.index("quest3_camera_stream.py", ready)
        quest = self.launcher.index("shell monkey", cameras)
        self.assertLess(ready, cameras)
        self.assertLess(cameras, quest)

    def test_cleanup_prioritizes_robot_before_cameras(self):
        cleanup = self.launcher.split("cleanup()", 1)[1].split("trap cleanup", 1)[0]
        self.assertLess(
            cleanup.index("request_robot_shutdown"),
            cleanup.index("stop_camera_publisher"),
        )

    def test_robot_process_keeps_operator_terminal_for_power_off_confirmation(self):
        self.assertIn("< /dev/tty", self.launcher)

    def test_camera_sigterm_runs_cleanup(self):
        self.assertIn("signal.SIGTERM", self.stream)
        self.assertIn("raise KeyboardInterrupt", self.stream)
        self.assertIn("stop_cameras(cameras.values())", self.stream)

    def test_launcher_collects_per_side_can_traces(self):
        self.assertIn("candump -t a -e -x can0", self.launcher)
        self.assertIn("candump -t a -e -x can1", self.launcher)

    def test_joint_preview_uses_low_bandwidth_camera_inputs(self):
        self.assertIn("--input-profile preview-low", self.launcher)
        self.assertIn('input_profile == "preview-low"', self.stream)
        self.assertIn("input_width, input_height, input_fps = 640, 480, 30", self.stream)


if __name__ == "__main__":
    unittest.main()
