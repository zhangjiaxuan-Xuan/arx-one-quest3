from pathlib import Path
import unittest


class CleanGravityKernelFeedbackTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = Path("validate_clean_gravity_compensation.py").read_text(
            encoding="utf-8"
        )
        cls.launcher = Path("tools/run_clean_gravity_compensation_test.sh").read_text(
            encoding="utf-8"
        )

    def test_baseline_uses_kernel_rx_not_vendor_timestamp(self):
        self.assertIn("CanRxWatchdog", self.source)
        self.assertIn("watchdog.fault(0.20)", self.source)
        self.assertNotIn("left_state.timestamp", self.source)
        self.assertNotIn("right_state.timestamp", self.source)

    def test_no_position_or_gain_commands(self):
        self.assertNotIn("set_joint_cmd", self.source)
        self.assertNotIn("set_gain", self.source)

    def test_startup_does_not_repeat_constructor_damping_transition(self):
        startup = self.source.split("watchdog.reset()", 1)[0]
        self.assertNotIn("left.set_to_damping()", startup)
        self.assertNotIn("right.set_to_damping()", startup)
        self.assertIn("startup_mode_write=off", self.source)

    def test_feedback_loss_requires_power_off_token(self):
        self.assertIn("confirm_power_off()", self.source)
        self.assertIn('== "off"', self.source)
        self.assertIn("except EOFError", self.source)

    def test_feedback_loss_is_observed_without_motion_writes(self):
        self.assertIn("--observe-after-loss", self.source)
        self.assertIn("OFFICIAL_BASELINE_RX_GAP_DETECTED", self.source)
        self.assertIn("OFFICIAL_BASELINE_LOSS_OBSERVATION_COMPLETE", self.source)
        detected = self.source.index("OFFICIAL_BASELINE_RX_GAP_DETECTED")
        final_raise = self.source.index("raise KernelFeedbackLost", detected)
        self.assertLess(detected, final_raise)

    def test_launcher_keeps_tty_and_can_traces(self):
        self.assertIn("< /dev/tty", self.launcher)
        self.assertIn("candump -t a -e -x can0", self.launcher)
        self.assertIn("candump -t a -e -x can1", self.launcher)


if __name__ == "__main__":
    unittest.main()
