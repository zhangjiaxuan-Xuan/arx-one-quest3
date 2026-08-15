from pathlib import Path
import unittest


class QuestRobotOnlyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = Path("validate_quest3_robot_only.py").read_text(encoding="utf-8")
        cls.launcher = Path("tools/run_quest3_robot_only_test.sh").read_text(
            encoding="utf-8"
        )

    def test_cameras_and_recording_are_absent(self):
        self.assertNotIn("quest3_camera_stream", self.source)
        self.assertNotIn("RobotRecorder", self.source)
        self.assertNotIn("quest3_camera_stream", self.launcher)

    def test_uses_production_session_and_safe_return(self):
        self.assertIn("PersistentArms(expected_shutdown_pose=shutdown)", self.source)
        self.assertIn("harness.safe_return_to_pose", self.source)
        self.assertIn("controller.close()", self.source)

    def test_signal_revokes_quest_without_direct_process_exit(self):
        handler = self.source.split("def request_stop", 1)[1].split("def load_pose", 1)[0]
        self.assertIn("stop_requested = True", handler)
        self.assertNotIn("raise", handler)

    def test_sdk_release_requires_shutdown_or_power_off(self):
        self.assertIn("if at_shutdown:", self.source)
        self.assertIn("elif powered_off:", self.source)
        self.assertIn("拒绝释放SDK", self.source)

    def test_launcher_collects_passive_per_side_can_traces(self):
        self.assertIn("candump -t a -e -x can0", self.launcher)
        self.assertIn("candump -t a -e -x can1", self.launcher)
        cleanup = self.launcher.split("cleanup()", 1)[1].split("trap cleanup", 1)[0]
        self.assertLess(cleanup.index("request_shutdown"), cleanup.index("stop_can_traces"))


if __name__ == "__main__":
    unittest.main()
