import unittest

import numpy as np

from quest3_teleop import (
    IK_TARGET_FILTER_TIME_CONSTANT_SECONDS,
    MAX_JOINT_ACCELERATION_RAD_S2,
    MAX_JOINT_VELOCITY_RAD_S,
    acceleration_limited_joint_step,
    low_pass_ik_target,
)


class QuestCommandSmoothingTest(unittest.TestCase):
    def test_full_speed_reversal_decelerates_before_reversing(self):
        previous_velocity = np.full(6, MAX_JOINT_VELOCITY_RAD_S)
        step, velocity = acceleration_limited_joint_step(
            np.full(6, -1.0), previous_velocity, 0.02
        )
        self.assertTrue((velocity > 0.0).all())
        self.assertTrue((step > 0.0).all())
        np.testing.assert_allclose(
            previous_velocity - velocity,
            MAX_JOINT_ACCELERATION_RAD_S2 * 0.02,
        )

    def test_velocity_change_never_exceeds_acceleration_limit(self):
        rng = np.random.default_rng(7)
        for _ in range(100):
            previous = rng.uniform(
                -MAX_JOINT_VELOCITY_RAD_S,
                MAX_JOINT_VELOCITY_RAD_S,
                6,
            )
            _, velocity = acceleration_limited_joint_step(
                rng.uniform(-1.0, 1.0, 6), previous, 0.02
            )
            self.assertLessEqual(
                float(np.max(np.abs(velocity - previous))),
                MAX_JOINT_ACCELERATION_RAD_S2 * 0.02 + 1e-12,
            )

    def test_ik_filter_reduces_noise_without_biasing_constant_target(self):
        previous = np.zeros(6)
        target = np.ones(6)
        first = low_pass_ik_target(previous, target, 0.02)
        expected_alpha = 1.0 - np.exp(-0.02 / IK_TARGET_FILTER_TIME_CONSTANT_SECONDS)
        np.testing.assert_allclose(first, expected_alpha)
        value = first
        for _ in range(200):
            value = low_pass_ik_target(value, target, 0.02)
        np.testing.assert_allclose(value, target, atol=1e-12)


if __name__ == "__main__":
    unittest.main()
