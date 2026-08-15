import unittest

import numpy as np

from quest3_teleop import (
    PID_CORRECTION_LIMIT,
    PID_INTEGRAL_LEAK,
    safe_pid_correction,
)


class SafeIntegralTest(unittest.TestCase):
    def update(self, error, integral=None, kp=None, ki=None):
        error = np.asarray(error, dtype=float)
        return safe_pid_correction(
            error,
            np.zeros(6),
            np.zeros(6) if integral is None else np.asarray(integral, dtype=float),
            0.02,
            np.zeros(6) if kp is None else np.asarray(kp, dtype=float),
            np.ones(6) if ki is None else np.asarray(ki, dtype=float),
            np.zeros(6),
        )

    def test_small_steady_error_accumulates(self):
        _, integral = self.update([0.01, 0, 0, 0, 0, 0])
        self.assertGreater(integral[0], 0.0)

    def test_large_error_does_not_integrate(self):
        _, integral = self.update([0.10, 0, 0, 0, 0, 0])
        self.assertEqual(integral[0], 0.0)

    def test_saturated_output_blocks_same_direction_windup(self):
        old = np.asarray([0.01, 0, 0, 0, 0, 0], dtype=float)
        correction, integral = self.update(
            [0.03, 0, 0, 0, 0, 0], integral=old, kp=[2, 0, 0, 0, 0, 0]
        )
        self.assertEqual(correction[0], PID_CORRECTION_LIMIT[0])
        self.assertAlmostEqual(integral[0], old[0] * PID_INTEGRAL_LEAK)

    def test_unused_integral_leaks_toward_zero(self):
        old = np.asarray([0.01, 0, 0, 0, 0, 0], dtype=float)
        _, integral = self.update([0, 0, 0, 0, 0, 0], integral=old)
        self.assertLess(integral[0], old[0])


if __name__ == "__main__":
    unittest.main()
