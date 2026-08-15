import unittest

import numpy as np

from pi05_arx_adapter import (
    ACTION_DIM,
    ACTION_HORIZON,
    MODEL_ACTION_DIM,
    PI05_PROTOCOL,
    WIRE_SCHEMA,
    build_policy_observation,
    pad_actions_for_model,
    parse_policy_response,
    unpad_model_actions,
)


class Pi05ArxAdapterTest(unittest.TestCase):
    def valid_actions(self):
        actions = np.zeros((ACTION_HORIZON, ACTION_DIM), dtype=np.float32)
        actions[:, [6, 13]] = 1.0
        return actions

    def test_model_padding_roundtrip(self):
        actions = self.valid_actions()
        model = pad_actions_for_model(actions)
        self.assertEqual(model.shape, (ACTION_HORIZON, MODEL_ACTION_DIM))
        np.testing.assert_array_equal(unpad_model_actions(model), actions)

    def test_fractional_gripper_rejected(self):
        actions = self.valid_actions()
        actions[0, 6] = 0.5
        with self.assertRaisesRegex(ValueError, "0/1"):
            pad_actions_for_model(actions)

    def test_vector_norm_bound_rejected(self):
        actions = self.valid_actions()
        actions[0, :3] = 0.02
        with self.assertRaisesRegex(ValueError, "translation"):
            pad_actions_for_model(actions)

    def test_protocol_and_normalization_are_mandatory(self):
        response = {
            "protocol": PI05_PROTOCOL,
            "action_schema": WIRE_SCHEMA,
            "normalization_id": "arx-v1",
            "fps": 50,
            "action_horizon": 50,
            "actions": self.valid_actions(),
        }
        parsed = parse_policy_response(response, expected_normalization_id="arx-v1")
        self.assertEqual(parsed.shape, (50, 14))
        response["normalization_id"] = "wrong"
        with self.assertRaisesRegex(ValueError, "normalization"):
            parse_policy_response(response, expected_normalization_id="arx-v1")

    def test_full_resolution_images_are_not_resized(self):
        base = np.zeros((1536, 2048, 3), dtype=np.uint8)
        wrist = np.zeros((720, 1280, 3), dtype=np.uint8)
        obs = build_policy_observation(
            state=np.zeros(14),
            base_rgb=base,
            left_wrist_rgb=wrist,
            right_wrist_rgb=wrist,
            prompt="put fruit",
            normalization_id="arx-v1",
            timestamp_unix=1.0,
        )
        self.assertEqual(obs["images"]["base_0_rgb"].shape, base.shape)
        self.assertEqual(obs["images"]["left_wrist_0_rgb"].shape, wrist.shape)


if __name__ == "__main__":
    unittest.main()
