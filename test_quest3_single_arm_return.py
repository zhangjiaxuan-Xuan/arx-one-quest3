import unittest
from types import SimpleNamespace

import numpy as np

from quest3_teleop import (
    QuestTeleopController,
    SINGLE_ARM_RETURN_COMPLETE_RAD,
    SINGLE_ARM_RETURN_STEP_RAD,
    advance_single_arm_return,
    trigger_gripper_command,
)


class SingleArmReturnTest(unittest.TestCase):
    def test_grip_engagement_never_infers_close_from_measured_width(self):
        self.assertEqual(trigger_gripper_command(0.0), 0)
        self.assertEqual(trigger_gripper_command(0.40), 0)
        self.assertEqual(trigger_gripper_command(0.65), 1)
        self.assertEqual(trigger_gripper_command(1.0), 1)

    def test_slew_limited_and_not_complete_early(self):
        next_command, complete = advance_single_arm_return(
            np.zeros(6), np.full(6, 0.2), np.zeros(6)
        )
        np.testing.assert_allclose(next_command, SINGLE_ARM_RETURN_STEP_RAD)
        self.assertFalse(complete)

    def test_uses_measured_completion_tolerance(self):
        target = np.linspace(-0.3, 0.3, 6)
        measured = target.copy()
        measured[2] += SINGLE_ARM_RETURN_COMPLETE_RAD
        _, complete = advance_single_arm_return(target, target, measured)
        self.assertTrue(complete)

    def test_rejects_invalid_joint_feedback(self):
        for bad in (np.zeros(5), np.full(6, np.nan)):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                advance_single_arm_return(np.zeros(6), np.zeros(6), bad)

    @staticmethod
    def snapshot(*, left_x=False, right_a=False, right_b=False):
        return SimpleNamespace(
            left=SimpleNamespace(primary=left_x, secondary=False),
            right=SimpleNamespace(primary=right_a, secondary=right_b),
        )

    def test_x_and_a_are_independent_rising_edges(self):
        controller = QuestTeleopController.__new__(QuestTeleopController)
        controller._return_button_down = {"left": False, "right": False}

        self.assertEqual(
            controller._single_arm_return_requests(self.snapshot(left_x=True)),
            {"left": True, "right": False},
        )
        self.assertEqual(
            controller._single_arm_return_requests(self.snapshot(left_x=True)),
            {"left": False, "right": False},
        )
        controller._single_arm_return_requests(self.snapshot())
        self.assertEqual(
            controller._single_arm_return_requests(self.snapshot(right_a=True)),
            {"left": False, "right": True},
        )

    def test_shutdown_chord_suppresses_left_return(self):
        controller = QuestTeleopController.__new__(QuestTeleopController)
        controller._return_button_down = {"left": False, "right": False}
        self.assertEqual(
            controller._single_arm_return_requests(
                self.snapshot(left_x=True, right_b=True)
            ),
            {"left": False, "right": False},
        )


if __name__ == "__main__":
    unittest.main()
