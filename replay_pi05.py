from __future__ import annotations

import argparse
from pathlib import Path
import time

import numpy as np
import arx5_interface as arx5

from arx_common import ACTION_DIM, FPS, GRIPPER_WIDTH, copy_state, make_arm, pack_bimanual, unpack_action
from legacy_hardware_guard import reject_legacy_direct_hardware


def set_reduced_gain(arm, controller, dof: int) -> None:
    gain = arx5.Gain(dof)
    gain.kp()[:] = np.asarray(controller.default_kp) * 0.25
    gain.kd()[:] = np.asarray(controller.default_kd) * 0.5
    gain.gripper_kp = float(controller.default_gripper_kp) * 0.25
    gain.gripper_kd = float(controller.default_gripper_kd) * 0.5
    arm.set_gain(gain)


def command_pair(left, right, action, left_cmd, right_cmd, max_joint_step, max_gripper_step):
    right_target, right_gripper, left_target, left_gripper = unpack_action(action)
    right_cmd.pos()[:] += np.clip(right_target - right_cmd.pos().copy(), -max_joint_step, max_joint_step)
    left_cmd.pos()[:] += np.clip(left_target - left_cmd.pos().copy(), -max_joint_step, max_joint_step)
    right_cmd.gripper_pos += float(np.clip(right_gripper - right_cmd.gripper_pos, -max_gripper_step, max_gripper_step))
    left_cmd.gripper_pos += float(np.clip(left_gripper - left_cmd.gripper_pos, -max_gripper_step, max_gripper_step))
    right.set_joint_cmd(right_cmd)
    left.set_joint_cmd(left_cmd)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--fps", type=int, default=FPS)
    parser.add_argument("--alignment-seconds", type=float, default=8.0)
    args = parser.parse_args()
    reject_legacy_direct_hardware("replay_pi05.py")
    chunks = np.load(args.input).astype(np.float32)
    if chunks.ndim != 3 or chunks.shape[1:] != (50, 32):
        raise ValueError(f"Expected model action chunks [N,50,32], got {chunks.shape}")
    actions = chunks.reshape(-1, 32)[:, :ACTION_DIM]
    if not np.isfinite(actions).all():
        raise ValueError("Actions contain non-finite values")
    actions[:, [6, 13]] = np.clip(actions[:, [6, 13]], 0.0, GRIPPER_WIDTH)

    left = right = None
    errors = []
    try:
        left, left_robot, left_ctrl = make_arm("can0", gravity_compensation=True)
        right, right_robot, right_ctrl = make_arm("can1", gravity_compensation=True)
        ls = copy_state(left); rs = copy_state(right)
        current, _, _ = pack_bimanual(rs, ls)
        initial_delta = float(np.max(np.abs(actions[0] - current)))
        if initial_delta > 2.0:
            raise RuntimeError(f"Initial pose delta {initial_delta:.3f} rad is unsafe")

        left_cmd = arx5.JointState(left_robot.joint_dof)
        right_cmd = arx5.JointState(right_robot.joint_dof)
        right_cmd.pos()[:] = rs[0]; right_cmd.gripper_pos = float(rs[3])
        left_cmd.pos()[:] = ls[0]; left_cmd.gripper_pos = float(ls[3])
        right.set_joint_cmd(right_cmd); left.set_joint_cmd(left_cmd)
        set_reduced_gain(left, left_ctrl, left_robot.joint_dof)
        set_reduced_gain(right, right_ctrl, right_robot.joint_dof)

        print(f"ALIGNING initial_max_delta={initial_delta:.4f} duration={args.alignment_seconds:.1f}s", flush=True)
        align_steps = int(args.alignment_seconds * args.fps)
        for _ in range(align_steps):
            command_pair(left, right, actions[0], left_cmd, right_cmd, 0.004, 0.0003)
            time.sleep(1.0 / args.fps)

        print(f"REPLAY_ACTIVE chunks={len(chunks)} model_shape={chunks.shape}", flush=True)
        period = 1.0 / args.fps
        next_tick = time.monotonic()
        for index, action in enumerate(actions):
            # This is the same 32-D action-chunk interface returned by OpenPI;
            # the robot adapter consumes the first 14 dimensions.
            command_pair(left, right, action, left_cmd, right_cmd, 0.03, 0.004)
            ls = copy_state(left); rs = copy_state(right)
            measured, _, _ = pack_bimanual(rs, ls)
            errors.append(float(np.sqrt(np.mean((action - measured) ** 2))))
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0: time.sleep(delay)
            if (index + 1) % (args.fps * 10) == 0:
                print(f"replayed={index + 1}/{len(actions)} recent_rms={np.mean(errors[-args.fps*10:]):.5f}", flush=True)
        print(f"REPLAY_COMPLETE mean_rms={np.mean(errors):.5f} max_rms={np.max(errors):.5f}", flush=True)
    finally:
        if right is not None: del right
        if left is not None: del left
        print("CONTROLLERS_DISCONNECTED", flush=True)


if __name__ == "__main__":
    main()
