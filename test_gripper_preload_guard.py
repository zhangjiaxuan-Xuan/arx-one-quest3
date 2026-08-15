import unittest

from arx_common import GRIPPER_WIDTH
from remote_delta_roundtrip import (
    GRIPPER_CONTACT_PRELOAD_M,
    GRIPPER_HARD_TORQUE,
    GRIPPER_SLIP_FOLLOW_M,
    GripperGuard,
)


class GripperPreloadGuardTest(unittest.TestCase):
    def test_contact_keeps_a_closing_preload(self):
        guard = GripperGuard()
        target = guard.target(1, measured_width=0.040, measured_torque=0.35)
        self.assertAlmostEqual(target, 0.040 - GRIPPER_CONTACT_PRELOAD_M)

    def test_slip_continues_closing_instead_of_holding_old_width(self):
        guard = GripperGuard()
        first = guard.target(1, measured_width=0.040, measured_torque=0.35)
        slipped = guard.target(1, measured_width=0.034, measured_torque=0.10)
        self.assertLess(slipped, first)
        self.assertAlmostEqual(slipped, 0.034 - GRIPPER_SLIP_FOLLOW_M)

    def test_hard_torque_stops_adding_compression_but_can_resume(self):
        guard = GripperGuard()
        protected = guard.target(
            1, measured_width=0.030, measured_torque=GRIPPER_HARD_TORQUE
        )
        self.assertAlmostEqual(protected, 0.030)
        resumed = guard.target(1, measured_width=0.029, measured_torque=0.10)
        self.assertLess(resumed, protected)

    def test_open_clears_preload_state(self):
        guard = GripperGuard()
        guard.target(1, measured_width=0.040, measured_torque=0.35)
        self.assertEqual(guard.target(0, 0.040, 0.35), GRIPPER_WIDTH)
        self.assertIsNone(guard.preload_target)


if __name__ == "__main__":
    unittest.main()
