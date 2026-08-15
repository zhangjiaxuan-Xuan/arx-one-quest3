import unittest

import numpy as np

from quest3_teleop import IK_BACKOFF_SCALES, solve_ik_with_backoff


class FakeKinematics:
    def __init__(self, first_success_scale=None):
        self.first_success_scale = first_success_scale
        self.translation_attempts = []

    def multi_trial_ik(self, pose, joints, dimensions):
        scale = float(pose[0])
        self.translation_attempts.append(scale)
        if self.first_success_scale is not None and scale <= self.first_success_scale:
            return 0, np.asarray(joints) + scale
        return 4, np.asarray(joints)


class QuestIkBackoffTest(unittest.TestCase):
    def test_retries_full_6d_ik_with_smaller_steps(self):
        kinematics = FakeKinematics(first_success_scale=0.25)
        correction = np.asarray([1.0, 0, 0, 0, 0, 0], dtype=float)
        status, joints, scale = solve_ik_with_backoff(
            kinematics, np.zeros(6), np.zeros(6), correction
        )
        self.assertEqual(status, 0)
        self.assertEqual(scale, 0.25)
        self.assertEqual(kinematics.translation_attempts, [1.0, 0.5, 0.25])
        np.testing.assert_allclose(joints, np.full(6, 0.25))

    def test_reports_failure_only_after_all_backoff_levels(self):
        kinematics = FakeKinematics()
        status, joints, scale = solve_ik_with_backoff(
            kinematics,
            np.zeros(6),
            np.zeros(6),
            np.asarray([1.0, 0, 0, 0, 0, 0]),
        )
        self.assertNotEqual(status, 0)
        self.assertIsNone(joints)
        self.assertIsNone(scale)
        self.assertEqual(tuple(kinematics.translation_attempts), IK_BACKOFF_SCALES)


if __name__ == "__main__":
    unittest.main()
