#!/usr/bin/env python3
"""ARX lifecycle using only vendor SDK modes and a single command owner."""

from __future__ import annotations

import argparse
from pathlib import Path
import signal
import time

import numpy as np
import arx5_interface as arx5

from arx_common import ACTION_DIM, FPS, GRIPPER_WIDTH, make_arm, pack_bimanual, copy_state


ROOT = Path(__file__).resolve().parent
INITIAL_POSE_PATH = ROOT / "shared_poses" / "collection_initial_pose.npy"
SHUTDOWN_POSE_PATH = ROOT / "shared_poses" / "shutdown_pose.npy"
stop_requested = False


def request_stop(signum, _frame) -> None:
    global stop_requested
    stop_requested = True
    print(
        f"收到信号 {signum}：不直接断开；当前阶段结束后执行停机位回归。",
        flush=True,
    )


def state_pair(right, left) -> np.ndarray:
    state, _, _ = pack_bimanual(copy_state(right), copy_state(left))
    return state


def official_gain(arm, robot, controller) -> None:
    gain = arx5.Gain(robot.joint_dof)
    gain.kp()[:] = np.asarray(controller.default_kp)
    gain.kd()[:] = np.asarray(controller.default_kd)
    gain.gripper_kp = float(controller.default_gripper_kp)
    gain.gripper_kd = float(controller.default_gripper_kd)
    arm.set_gain(gain)


def move_pair(
    left,
    left_robot,
    left_controller,
    right,
    right_robot,
    right_controller,
    target: np.ndarray,
    duration: float,
    label: str,
) -> None:
    """Send one smooth, fixed-duration trajectory without custom feedback gates."""
    start = state_pair(right, left).astype(np.float64)
    target = np.asarray(target, dtype=np.float64)
    left_cmd = arx5.JointState(left_robot.joint_dof)
    right_cmd = arx5.JointState(right_robot.joint_dof)
    right_cmd.pos()[:] = start[:6]
    right_cmd.gripper_pos = float(start[6])
    left_cmd.pos()[:] = start[7:13]
    left_cmd.gripper_pos = float(start[13])
    right.set_joint_cmd(right_cmd)
    left.set_joint_cmd(left_cmd)
    official_gain(right, right_robot, right_controller)
    official_gain(left, left_robot, left_controller)
    print(f"OFFICIAL_STAGE_MOVE_BEGIN label={label} duration={duration:.1f}s", flush=True)
    steps = max(1, int(round(duration * FPS)))
    next_tick = time.monotonic()
    for step in range(1, steps + 1):
        ratio = step / steps
        alpha = ratio * ratio * (3.0 - 2.0 * ratio)
        command = start + alpha * (target - start)
        right_cmd.pos()[:] = command[:6]
        right_cmd.gripper_pos = float(command[6])
        left_cmd.pos()[:] = command[7:13]
        left_cmd.gripper_pos = float(command[13])
        right.set_joint_cmd(right_cmd)
        left.set_joint_cmd(left_cmd)
        next_tick += 1.0 / FPS
        delay = next_tick - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            next_tick = time.monotonic()
    print(f"OFFICIAL_STAGE_MOVE_COMPLETE label={label}", flush=True)


def gravity_stage(left, right, duration: float, label: str) -> None:
    """Enter the vendor's native damping/gravity mode and leave it in charge."""
    right.set_to_damping()
    left.set_to_damping()
    print(
        f"OFFICIAL_STAGE_GRAVITY_BEGIN label={label} duration={duration:.1f}s",
        flush=True,
    )
    deadline = time.monotonic() + duration
    next_report = time.monotonic()
    while time.monotonic() < deadline:
        if stop_requested and label == "initial":
            print("已请求结束初始位重力补偿，转入停机位回归。", flush=True)
            break
        now = time.monotonic()
        if now >= next_report:
            print(
                f"OFFICIAL_STAGE_GRAVITY_ACTIVE label={label} "
                f"remaining={max(0.0, deadline-now):.0f}s",
                flush=True,
            )
            next_report = now + 5.0
        time.sleep(0.05)
    print(f"OFFICIAL_STAGE_GRAVITY_COMPLETE label={label}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--move-seconds", type=float, default=5.0)
    parser.add_argument("--gravity-seconds", type=float, default=30.0)
    args = parser.parse_args()
    if args.move_seconds <= 0 or args.gravity_seconds <= 0:
        raise ValueError("stage durations must be positive")
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, request_stop)

    initial = np.load(INITIAL_POSE_PATH).astype(np.float64)
    shutdown = np.load(SHUTDOWN_POSE_PATH).astype(np.float64)
    if initial.shape != (ACTION_DIM,) or shutdown.shape != (ACTION_DIM,):
        raise RuntimeError("共享姿态文件维度错误")
    initial[[6, 13]] = GRIPPER_WIDTH

    left = right = None
    left_robot = right_robot = None
    left_controller = right_controller = None
    shutdown_motion_complete = False
    failure: BaseException | None = None
    try:
        print("OFFICIAL_LIFECYCLE_CONNECT left=can0 then right=can1", flush=True)
        left, left_robot, left_controller = make_arm(
            "can0", gravity_compensation=True, shutdown_to_passive=True
        )
        right, right_robot, right_controller = make_arm(
            "can1", gravity_compensation=True, shutdown_to_passive=True
        )
        move_pair(
            left, left_robot, left_controller,
            right, right_robot, right_controller,
            initial, args.move_seconds, "collection_initial",
        )
        gravity_stage(left, right, args.gravity_seconds, "initial")
    except BaseException as exc:
        failure = exc
        print(f"OFFICIAL_LIFECYCLE_STAGE_FAILURE {type(exc).__name__}: {exc}", flush=True)
    finally:
        if left is not None and right is not None:
            try:
                move_pair(
                    left, left_robot, left_controller,
                    right, right_robot, right_controller,
                    shutdown, args.move_seconds, "shutdown",
                )
                shutdown_motion_complete = True
                gravity_stage(left, right, args.gravity_seconds, "shutdown")
                right.set_to_damping()
                left.set_to_damping()
            except BaseException as exc:
                failure = failure or exc
                print(
                    f"OFFICIAL_LIFECYCLE_SHUTDOWN_FAILURE {type(exc).__name__}: {exc}",
                    flush=True,
                )
                try:
                    right.set_to_damping()
                    left.set_to_damping()
                    print("OFFICIAL_LIFECYCLE_FALLBACK_DAMPING", flush=True)
                    time.sleep(args.gravity_seconds)
                except BaseException as damping_exc:
                    failure = failure or damping_exc
        if right is not None:
            del right
        if left is not None:
            del left
        print(
            "OFFICIAL_LIFECYCLE_DISCONNECTED_AFTER_SHUTDOWN"
            if shutdown_motion_complete
            else "OFFICIAL_LIFECYCLE_DISCONNECTED_AFTER_FALLBACK_DAMPING",
            flush=True,
        )
    if failure is not None:
        raise failure
    print("OFFICIAL_LIFECYCLE_TEST_COMPLETE", flush=True)


if __name__ == "__main__":
    main()
