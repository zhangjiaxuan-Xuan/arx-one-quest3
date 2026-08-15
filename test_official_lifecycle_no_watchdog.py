from pathlib import Path
import unittest


class OfficialLifecycleNoWatchdogTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = Path("validate_official_lifecycle_no_watchdog.py").read_text(
            encoding="utf-8"
        )

    def test_has_no_custom_feedback_watchdog(self):
        self.assertNotIn("CanRxWatchdog", self.source)
        self.assertNotIn("can_rx_watchdog", self.source)
        self.assertNotIn("rx_packets", self.source)
        self.assertNotIn("wait_for_feedback_recovery", self.source)

    def test_stage_order_is_explicit(self):
        initial = self.source.index('"collection_initial"')
        initial_gravity = self.source.index('gravity_stage(left, right, args.gravity_seconds, "initial")')
        shutdown = self.source.index('"shutdown"', initial_gravity)
        shutdown_gravity = self.source.index('gravity_stage(left, right, args.gravity_seconds, "shutdown")')
        self.assertLess(initial, initial_gravity)
        self.assertLess(initial_gravity, shutdown)
        self.assertLess(shutdown, shutdown_gravity)

    def test_signal_does_not_destroy_controller(self):
        handler = self.source.split("def request_stop", 1)[1].split("def state_pair", 1)[0]
        self.assertIn("stop_requested = True", handler)
        self.assertNotIn("del left", handler)
        self.assertNotIn("del right", handler)


if __name__ == "__main__":
    unittest.main()
