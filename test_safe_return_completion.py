import unittest

import numpy as np

from collect_workflow import (
    GRIPPER_WIDTH,
    SAFE_RETURN_COMPLETION_RAD,
    collection_initial_complete,
    safe_return_complete,
    safe_return_joint_errors,
)


class SafeReturnCompletionTest(unittest.TestCase):
    def test_both_arms_under_50_milliradians_complete(self):
        target = np.zeros(14, dtype=np.float32)
        measured = target.copy()
        measured[2] = 0.049
        measured[10] = -0.0499
        self.assertEqual(SAFE_RETURN_COMPLETION_RAD, 0.050)
        self.assertTrue(safe_return_complete(target, measured))

    def test_either_arm_at_or_above_limit_blocks_completion(self):
        target = np.zeros(14, dtype=np.float32)
        measured = target.copy()
        measured[8] = 0.050
        self.assertFalse(safe_return_complete(target, measured))

    def test_gripper_error_does_not_delay_safe_joint_return(self):
        target = np.zeros(14, dtype=np.float32)
        measured = target.copy()
        measured[[6, 13]] = 1.0
        self.assertTrue(safe_return_complete(target, measured))
        self.assertEqual(safe_return_joint_errors(target, measured), {"right": 0.0, "left": 0.0})

    def test_collection_initial_requires_both_grippers_open(self):
        target = np.zeros(14, dtype=np.float32)
        target[[6, 13]] = GRIPPER_WIDTH
        measured = target.copy()
        self.assertTrue(collection_initial_complete(target, measured))
        measured[13] = GRIPPER_WIDTH - 0.003
        self.assertFalse(collection_initial_complete(target, measured))


if __name__ == "__main__":
    unittest.main()
