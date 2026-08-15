from pathlib import Path
import unittest

import numpy as np

from validate_quest3_output_stability import (
    CommandEvent,
    InstrumentedCommandGate,
    VirtualArms,
    analyze,
)


class NoArmOutputStabilityTest(unittest.TestCase):
    def test_module_cannot_construct_hardware(self):
        source = Path("validate_quest3_output_stability.py").read_text(encoding="utf-8")
        self.assertNotIn("make_arm", source)
        self.assertNotIn("Arx5JointController", source)
        self.assertNotIn("PersistentArms", source)

    def test_one_key_collection_runs_no_arm_gate_before_can_and_sdk(self):
        source = Path("tools/start_all_quest3_collection.sh").read_text(
            encoding="utf-8"
        )
        gate = source.index("run_quest3_output_stability_test.sh")
        can_recovery = source.index("recover_arx_can.sh", gate)
        workflow = source.index("start_quest3_collection_test.sh", can_recovery)
        self.assertLess(gate, can_recovery)
        self.assertLess(can_recovery, workflow)
        self.assertIn("拒绝连接机械臂", source)

    def test_analysis_rejects_joint_spike(self):
        arms = VirtualArms(np.zeros(14), lambda side: (1, "smooth"))
        arms.events = [
            CommandEvent(1.0, "left", 1, "smooth", 10, (0, 0, 0, 0, 0, 0), 0.0),
            CommandEvent(1.02, "left", 2, "smooth", 10, (0.2, 0, 0, 0, 0, 0), 0.0),
            CommandEvent(1.0, "right", 1, "smooth", 10, (0, 0, 0, 0, 0, 0), 0.0),
            CommandEvent(1.02, "right", 2, "smooth", 10, (0, 0, 0, 0, 0, 0), 0.0),
        ] * 5
        report = analyze(arms, mode="synthetic", duration=16.0, controller_error=None)
        self.assertFalse(report["passed"])
        self.assertTrue(any("command velocity" in item for item in report["failures"]))

    def test_analysis_rejects_writes_during_stale_block(self):
        arms = VirtualArms(np.zeros(14), lambda side: (1, "smooth"))
        arms.events = [
            CommandEvent(float(i), side, i, "stale_blocked", 10, (0, 0, 0, 0, 0, 0), 0.0)
            for i in range(10)
            for side in ("left", "right")
        ]
        report = analyze(arms, mode="synthetic", duration=16.0, controller_error=None)
        self.assertFalse(report["passed"])
        self.assertGreater(report["forbidden_phase_writes"], 0)

    def test_analysis_rejects_legal_steps_that_reverse_at_full_speed(self):
        arms = VirtualArms(np.zeros(14), lambda side: (1, "smooth"))
        events = []
        for index in range(12):
            value = 0.015 if index % 2 else 0.0
            for side in ("left", "right"):
                events.append(CommandEvent(
                    1.0 + index * 0.02,
                    side,
                    index,
                    "smooth",
                    10,
                    (value, 0, 0, 0, 0, 0),
                    0.0,
                ))
        arms.events = events
        report = analyze(arms, mode="synthetic", duration=16.0, controller_error=None)
        self.assertFalse(report["passed"])
        self.assertTrue(any("full-speed reversals" in item for item in report["failures"]))


if __name__ == "__main__":
    unittest.main()
