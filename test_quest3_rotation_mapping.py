import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from quest3_input import ControllerState
from quest3_teleop import SideConfig, forward_motion_calibration_matrix, target_pose


class QuestRotationMappingTest(unittest.TestCase):
    def setUp(self):
        self.config = SideConfig(
            enabled=True,
            gripper_enabled=True,
            translation_scale=1.25,
            rotation_scale=1.25,
            max_translation_m=0.30,
            quest_to_robot=np.asarray([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float),
            rotation_quest_to_eef=np.asarray(
                [[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float
            ),
            pid_kp=np.zeros(6),
            pid_ki=np.zeros(6),
            pid_kd=np.zeros(6),
        )
        self.controller_anchor = Rotation.from_euler("xyz", [0.3, -0.4, 0.7])
        self.robot_anchor = Rotation.from_euler("xyz", [-0.2, 0.6, -0.5])

    def mapped_delta(self, quest_axis):
        angle = 0.2
        current = self.controller_anchor * Rotation.from_rotvec(
            np.asarray(quest_axis, dtype=float) * angle
        )
        pose = target_pose(
            np.r_[np.zeros(3), self.robot_anchor.as_rotvec()],
            np.zeros(3),
            self.controller_anchor,
            ControllerState(orientation_xyzw=tuple(current.as_quat())),
            self.config,
        )
        target = Rotation.from_rotvec(pose[3:6])
        return (self.robot_anchor.inv() * target).as_rotvec()

    def test_pitch_maps_to_local_wrist_y(self):
        np.testing.assert_allclose(self.mapped_delta([1, 0, 0]), [0, -0.25, 0], atol=1e-7)

    def test_yaw_maps_to_local_wrist_z(self):
        np.testing.assert_allclose(self.mapped_delta([0, 1, 0]), [0, 0, 0.25], atol=1e-7)

    def test_roll_maps_to_local_wrist_x(self):
        np.testing.assert_allclose(self.mapped_delta([0, 0, 1]), [-0.25, 0, 0], atol=1e-7)

    def test_forward_stroke_on_canonical_x_needs_no_correction(self):
        np.testing.assert_allclose(
            forward_motion_calibration_matrix(np.asarray([0.15, 0.02, 0.0])),
            np.eye(3),
            atol=1e-7,
        )

    def test_reverse_facing_stroke_corrects_front_back_and_left_right(self):
        correction = forward_motion_calibration_matrix(
            np.asarray([-0.15, 0.0, 0.0])
        )
        np.testing.assert_allclose(
            correction @ np.asarray([-1.0, 0.0, 0.0]),
            np.asarray([1.0, 0.0, 0.0]),
            atol=1e-7,
        )
        np.testing.assert_allclose(
            correction @ np.asarray([0.0, 0.0, -1.0]),
            np.asarray([0.0, 0.0, 1.0]),
            atol=1e-7,
        )

    def test_forward_stroke_yaw_correction_preserves_vertical(self):
        correction = forward_motion_calibration_matrix(
            np.asarray([0.08, 0.20, 0.08])
        )
        np.testing.assert_allclose(
            correction @ np.asarray([0.0, 1.0, 0.0]),
            np.asarray([0.0, 1.0, 0.0]),
            atol=1e-7,
        )

if __name__ == "__main__":
    unittest.main()
