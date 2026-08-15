from pathlib import Path
import unittest


class PersistentMotionDiagnosticsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = Path("validate_persistent_session_motion.py").read_text(
            encoding="utf-8"
        )

    def test_no_peripherals_are_imported(self):
        self.assertNotIn("Quest3Receiver", self.source)
        self.assertNotIn("quest3_camera_stream", self.source)
        self.assertNotIn("RobotRecorder", self.source)

    def test_uses_production_session_and_safe_return(self):
        self.assertIn("PersistentArms(expected_shutdown_pose=shutdown)", self.source)
        self.assertIn("harness.safe_return_to_pose", self.source)
        self.assertIn("mode=single_owner_position_hold", self.source)
        self.assertIn("当前为双臂位姿PID闭环固定", self.source)

    def test_active_hold_has_one_explicit_command_owner(self):
        self.assertIn('parser.add_argument("--refresh-hz"', self.source)
        self.assertIn("arms.set_bimanual_commands(left_hold_cmd, right_hold_cmd)", self.source)
        self.assertNotIn("threading.Thread", self.source)

    def test_signal_only_requests_orderly_shutdown(self):
        handler = self.source.split("def request_stop", 1)[1].split("def load_pose", 1)[0]
        self.assertIn("stop_requested = True", handler)
        self.assertNotIn("raise", handler)

    def test_release_requires_shutdown_or_explicit_power_off(self):
        self.assertIn("if at_shutdown:", self.source)
        self.assertIn("elif powered_off:", self.source)
        self.assertIn('== "off"', self.source)
        self.assertIn('input("断电确认 > ")', self.source)
        self.assertIn("except EOFError", self.source)
        self.assertIn("不会因EOF退出", self.source)
        self.assertIn("拒绝释放SDK", self.source)

    def test_leaving_shutdown_revokes_disconnect_eligibility_first(self):
        revoke = self.source.index("at_shutdown = False")
        leave = self.source.index(
            'harness.safe_return_to_pose(\n            initial, "进入统一采集初始位"'
        )
        self.assertLess(revoke, leave)


if __name__ == "__main__":
    unittest.main()
