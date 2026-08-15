"""Hardware-free π0.5 protocol boundary for the ARX AC One.

The policy server owns image transforms, normalization, model padding and
denormalization.  The robot process only accepts the explicit 14-D ARX wire
schema, so a raw normalized 32-D model tensor can never reach the SDK.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np

from arx_common import ACTION_DIM, ACTION_HORIZON, FPS, MODEL_ACTION_DIM
from remote_delta_roundtrip import (
    MAX_ROTATION_DELTA_RAD,
    MAX_TRANSLATION_DELTA_M,
    WIRE_SCHEMA,
)

PI05_PROTOCOL = "arx_ac_one.pi05_policy.v1"
IMAGE_KEYS = ("base_0_rgb", "left_wrist_0_rgb", "right_wrist_0_rgb")


def _finite_array(value: Any, shape: tuple[int, ...], name: str) -> np.ndarray:
    array = np.asarray(value)
    if array.shape != shape:
        raise ValueError(f"{name} shape must be {shape}, got {array.shape}")
    if not np.isfinite(array).all():
        raise ValueError(f"{name} contains non-finite values")
    return array


def pad_state_for_model(state: np.ndarray) -> np.ndarray:
    state = _finite_array(state, (ACTION_DIM,), "state").astype(np.float32, copy=False)
    padded = np.zeros(MODEL_ACTION_DIM, dtype=np.float32)
    padded[:ACTION_DIM] = state
    return padded


def pad_actions_for_model(actions: np.ndarray) -> np.ndarray:
    actions = validate_robot_action_chunk(actions)
    padded = np.zeros((ACTION_HORIZON, MODEL_ACTION_DIM), dtype=np.float32)
    padded[:, :ACTION_DIM] = actions
    return padded


def unpad_model_actions(actions: np.ndarray) -> np.ndarray:
    """Server-side output transform from raw π0.5 [50,32] to ARX [50,14]."""
    actions = _finite_array(
        actions, (ACTION_HORIZON, MODEL_ACTION_DIM), "raw π0.5 action chunk"
    ).astype(np.float32, copy=False)
    return validate_robot_action_chunk(actions[:, :ACTION_DIM])


def validate_robot_action_chunk(actions: np.ndarray) -> np.ndarray:
    actions = _finite_array(
        actions, (ACTION_HORIZON, ACTION_DIM), "ARX action chunk"
    ).astype(np.float32, copy=False)
    gripper = actions[:, [6, 13]]
    if not np.logical_or(np.isclose(gripper, 0.0), np.isclose(gripper, 1.0)).all():
        raise ValueError("ARX gripper action must be exactly 0/1")
    translation = max(
        np.linalg.norm(actions[:, :3], axis=1).max(),
        np.linalg.norm(actions[:, 7:10], axis=1).max(),
    )
    rotation = max(
        np.linalg.norm(actions[:, 3:6], axis=1).max(),
        np.linalg.norm(actions[:, 10:13], axis=1).max(),
    )
    if translation > MAX_TRANSLATION_DELTA_M:
        raise ValueError("ARX action chunk exceeds translation safety bound")
    if rotation > MAX_ROTATION_DELTA_RAD:
        raise ValueError("ARX action chunk exceeds rotation safety bound")
    return actions


def build_policy_observation(
    *,
    state: np.ndarray,
    base_rgb: np.ndarray,
    left_wrist_rgb: np.ndarray,
    right_wrist_rgb: np.ndarray,
    prompt: str,
    normalization_id: str,
    timestamp_unix: float,
) -> dict[str, Any]:
    """Build a full-fidelity request; resize/pad remains a server transform."""
    if not prompt.strip():
        raise ValueError("prompt must not be empty")
    if not normalization_id.strip():
        raise ValueError("normalization_id must not be empty")
    images = {
        "base_0_rgb": np.asarray(base_rgb),
        "left_wrist_0_rgb": np.asarray(left_wrist_rgb),
        "right_wrist_0_rgb": np.asarray(right_wrist_rgb),
    }
    for key, image in images.items():
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(f"{key} must be full-resolution uint8 RGB HWC")
        if image.size == 0:
            raise ValueError(f"{key} is empty")
    if not np.isfinite(timestamp_unix):
        raise ValueError("timestamp_unix must be finite")
    return {
        "protocol": PI05_PROTOCOL,
        "action_schema": WIRE_SCHEMA,
        "normalization_id": normalization_id,
        "fps": FPS,
        "action_horizon": ACTION_HORIZON,
        "images": images,
        "state": _finite_array(state, (ACTION_DIM,), "state").astype(np.float32),
        "prompt": prompt.strip(),
        "timestamp_unix": float(timestamp_unix),
    }


def parse_policy_response(
    response: Mapping[str, Any], *, expected_normalization_id: str
) -> np.ndarray:
    if response.get("protocol") != PI05_PROTOCOL:
        raise ValueError("π0.5 response protocol mismatch")
    if response.get("action_schema") != WIRE_SCHEMA:
        raise ValueError("π0.5 response action schema mismatch")
    if response.get("normalization_id") != expected_normalization_id:
        raise ValueError("π0.5 response normalization metadata mismatch")
    if int(response.get("fps", -1)) != FPS:
        raise ValueError("π0.5 response fps mismatch")
    if int(response.get("action_horizon", -1)) != ACTION_HORIZON:
        raise ValueError("π0.5 response action horizon mismatch")
    return validate_robot_action_chunk(np.asarray(response.get("actions")))
