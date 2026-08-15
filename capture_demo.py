from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import time

import numpy as np

from arx_common import FPS, NAMES, copy_state, make_arm, pack_bimanual
from legacy_hardware_guard import reject_legacy_direct_hardware


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--fps", type=int, default=FPS)
    parser.add_argument("--output-root", type=Path, default=Path(__file__).parent / "runs")
    parser.add_argument("--task", default="bimanual demonstration")
    args = parser.parse_args()
    reject_legacy_direct_hardware("capture_demo.py")

    run_dir = args.output_root / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=False)
    period = 1.0 / args.fps
    target_frames = int(round(args.duration * args.fps))

    left = right = None
    timestamps, states, velocities, efforts = [], [], [], []
    try:
        left, _, _ = make_arm("can0", gravity_compensation=True)
        right, _, _ = make_arm("can1", gravity_compensation=True)
        start = time.monotonic()
        next_tick = start
        print(f"CAPTURE_ACTIVE duration={args.duration:.1f}s fps={args.fps} output={run_dir}", flush=True)
        for frame_index in range(target_frames):
            now = time.monotonic()
            if now < next_tick:
                time.sleep(next_tick - now)
            sample_time = time.monotonic()
            right_state = copy_state(right)
            left_state = copy_state(left)
            state, velocity, effort = pack_bimanual(right_state, left_state)
            timestamps.append(sample_time - start)
            states.append(state)
            velocities.append(velocity)
            efforts.append(effort)
            next_tick += period
            if (frame_index + 1) % (args.fps * 10) == 0:
                print(f"captured={frame_index + 1}/{target_frames}", flush=True)
    finally:
        if right is not None:
            del right
        if left is not None:
            del left
        print("CONTROLLERS_DISCONNECTED", flush=True)

    timestamp = np.asarray(timestamps, dtype=np.float64)
    observation_state = np.asarray(states, dtype=np.float32)
    observation_velocity = np.asarray(velocities, dtype=np.float32)
    observation_effort = np.asarray(efforts, dtype=np.float32)
    if len(observation_state) < 2:
        raise RuntimeError("Not enough frames captured")
    action = np.concatenate([observation_state[1:], observation_state[-1:]], axis=0)
    np.savez_compressed(
        run_dir / "raw_demo.npz",
        timestamp=timestamp,
        observation_state=observation_state,
        observation_velocity=observation_velocity,
        observation_effort=observation_effort,
        action=action,
        fps=np.int32(args.fps),
        task=np.asarray(args.task),
        names=np.asarray(NAMES),
    )
    actual_fps = (len(timestamp) - 1) / (timestamp[-1] - timestamp[0])
    print(f"CAPTURE_COMPLETE frames={len(timestamp)} actual_fps={actual_fps:.4f} file={run_dir / 'raw_demo.npz'}")


if __name__ == "__main__":
    main()
