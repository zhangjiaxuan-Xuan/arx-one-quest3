from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil

# Keep Hugging Face's transient Arrow cache inside this writable project.
os.environ.setdefault("HF_HOME", str(Path(__file__).parent / ".cache" / "huggingface"))

import numpy as np
import pandas as pd
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset

from arx_common import ACTION_DIM, ACTION_HORIZON, MODEL_ACTION_DIM, NAMES
from remote_delta_roundtrip import WIRE_SCHEMA, delta_eef_actions


ACTION_SCHEMA = WIRE_SCHEMA
ACTION_NAMES = [
    "right_dx_m", "right_dy_m", "right_dz_m",
    "right_drx_rad", "right_dry_rad", "right_drz_rad", "right_gripper_01",
    "left_dx_m", "left_dy_m", "left_dz_m",
    "left_drx_rad", "left_dry_rad", "left_drz_rad", "left_gripper_01",
]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    args = parser.parse_args()
    run_dir = args.input.parent
    raw = np.load(args.input, allow_pickle=False)
    state = raw["observation_state"].astype(np.float32)
    velocity = raw["observation_velocity"].astype(np.float32)
    effort = raw["observation_effort"].astype(np.float32)
    recorded_action = raw["action"].astype(np.float32)
    timestamp = raw["timestamp"].astype(np.float64)
    fps = int(raw["fps"])
    task = str(raw["task"])

    if state.shape != recorded_action.shape or state.shape[1] != ACTION_DIM:
        raise ValueError(f"Unexpected state/action shapes: {state.shape}, {recorded_action.shape}")
    if not all(np.isfinite(x).all() for x in (state, velocity, effort, recorded_action, timestamp)):
        raise ValueError("Dataset contains non-finite values")
    action = delta_eef_actions(state).astype(np.float32)

    # Exact OpenPI model-output shape: [num_chunks, action_horizon, model_action_dim].
    padded_count = int(np.ceil(len(action) / ACTION_HORIZON) * ACTION_HORIZON)
    padded = np.repeat(action[-1:], padded_count, axis=0)
    padded[: len(action)] = action
    model_actions = np.zeros((padded_count, MODEL_ACTION_DIM), dtype=np.float32)
    model_actions[:, :ACTION_DIM] = padded
    chunks = model_actions.reshape(-1, ACTION_HORIZON, MODEL_ACTION_DIM)
    np.save(run_dir / "pi05_action_chunks.npy", chunks)

    frame = pd.DataFrame(
        {
            "timestamp": timestamp,
            "frame_index": np.arange(len(state), dtype=np.int64),
            "episode_index": np.zeros(len(state), dtype=np.int64),
            "task": [task] * len(state),
        }
    )
    for i, name in enumerate(NAMES):
        frame[f"observation.state.{name}"] = state[:, i]
        frame[f"observation.velocity.{name}"] = velocity[:, i]
        frame[f"observation.effort.{name}"] = effort[:, i]
    for i, name in enumerate(ACTION_NAMES):
        frame[f"action.{name}"] = action[:, i]
    frame.to_parquet(run_dir / "episode_000000.parquet", index=False)

    # Materialize a real LeRobot v3 dataset directory with metadata and stats.
    lerobot_root = run_dir / "lerobot_dataset"
    if lerobot_root.exists():
        shutil.rmtree(lerobot_root)
    state_feature = {
        "dtype": "float32",
        "shape": (ACTION_DIM,),
        "names": [NAMES],
    }
    action_feature = {
        "dtype": "float32",
        "shape": (ACTION_DIM,),
        "names": [ACTION_NAMES],
    }
    dataset = LeRobotDataset.create(
        repo_id="local/arx_ac_one_pi05_demo",
        root=lerobot_root,
        robot_type="arx_ac_one_x5_bimanual",
        fps=fps,
        features={
            "observation.state": state_feature,
            "observation.velocity": state_feature,
            "observation.effort": state_feature,
            "action": action_feature,
        },
        use_videos=False,
    )
    for i in range(len(state)):
        dataset.add_frame(
            {
                "observation.state": torch.from_numpy(state[i]),
                "observation.velocity": torch.from_numpy(velocity[i]),
                "observation.effort": torch.from_numpy(effort[i]),
                "action": torch.from_numpy(action[i]),
                "task": task,
            }
        )
    dataset.save_episode()

    q01 = np.quantile(action, 0.01, axis=0)
    q99 = np.quantile(action, 0.99, axis=0)
    metadata = {
        "format": "OpenPI pi0.5 / LeRobot-compatible proprioception episode",
        "robot_type": "arx_ac_one_x5_bimanual",
        "fps": fps,
        "frames": len(state),
        "task": task,
        "state_dim": ACTION_DIM,
        "action_dim": ACTION_DIM,
        "model_action_dim": MODEL_ACTION_DIM,
        "action_horizon": ACTION_HORIZON,
        "action_schema": ACTION_SCHEMA,
        "state_ordering": NAMES,
        "action_ordering": ACTION_NAMES,
        "has_camera_observations": False,
        "training_ready": False,
        "q01": q01.tolist(),
        "q99": q99.tolist(),
        "mean": action.mean(axis=0).tolist(),
        "std": action.std(axis=0).tolist(),
    }
    (run_dir / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(
        f"CONVERT_COMPLETE chunks={chunks.shape} parquet={run_dir / 'episode_000000.parquet'} "
        f"lerobot={lerobot_root}"
    )
    print(
        "NOT_TRAINING_READY: cameras are not included; action schema is now "
        f"deployment-compatible {ACTION_SCHEMA}"
    )
    print(
        f"ACTION_RANGE max_translation_component={np.max(np.abs(action[:, [0,1,2,7,8,9]])):.6f} "
        f"max_rotation_component={np.max(np.abs(action[:, [3,4,5,10,11,12]])):.6f}"
    )


if __name__ == "__main__":
    main()
