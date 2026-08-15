#!/usr/bin/env python3
"""Two-cycle baseline of the pre-VR hand-collection arm lifecycle.

Uses only public arx5_interface APIs and the historical left-then-right
construction / right-then-left command order. No Quest, cameras, or VR code.
"""

from pathlib import Path
import signal
import time

import numpy as np
import arx5_interface as arx5

from arx_common import ACTION_DIM, FPS, copy_state, make_arm, pack_bimanual
from replay_pi05 import command_pair, set_reduced_gain
ROOT = Path(__file__).resolve().parent
SHARED_POSES = ROOT / "shared_poses"
COLLECTION_INITIAL_POSE_PATH = SHARED_POSES / "collection_initial_pose.npy"
GLOBAL_SHUTDOWN_POSE_PATH = SHARED_POSES / "shutdown_pose.npy"
GLOBAL_SHUTDOWN_BOOT_ID_PATH = SHARED_POSES / "shutdown_pose_boot_id.txt"
SYSTEM_BOOT_ID_PATH = Path("/proc/sys/kernel/random/boot_id")


class StopRequested(Exception):
    pass


stop_requested = False


def request_stop(signum, _frame) -> None:
    global stop_requested
    stop_requested = True
    print(f"收到信号 {signum}：停止后续阶段并优先返回停机位。", flush=True)


def packed_state(left, right) -> np.ndarray:
    state, _, _ = pack_bimanual(copy_state(right), copy_state(left))
    return state


def set_damping(left, right) -> None:
    # Public SDK transition; do not alter vendor code or config files.
    left.set_to_damping()
    right.set_to_damping()


def hold_until_manual_shutdown(left, right, shutdown: np.ndarray, cycle: int) -> None:
    """Keep SDK life support alive until both arms are manually at shutdown."""
    set_damping(left, right)
    joint_indices = np.asarray([*range(6), *range(7, 13)])
    print(
        f"第{cycle}轮自动回停失败：保持双臂SDK和重力补偿在线。"
        "请手动缓慢移动到停机位；达到后程序才允许断开。",
        flush=True,
    )
    settled = 0
    last_report = 0.0
    while settled < 20:
        try:
            current = packed_state(left, right)
            error = float(
                np.max(np.abs(current[joint_indices] - shutdown[joint_indices]))
            )
            settled = settled + 1 if error <= 0.08 else 0
            now = time.monotonic()
            if now - last_report >= 2.0:
                print(f"等待手动停机位：最大关节误差={error:.4f}rad", flush=True)
                last_report = now
        except BaseException as state_error:
            settled = 0
            print(f"等待停机位时状态读取异常：{state_error}", flush=True)
        time.sleep(0.05)
    print("已连续确认手动停机位，允许正常断开SDK。", flush=True)


def move_to(left, right, left_robot, right_robot, left_ctrl, right_ctrl,
            target: np.ndarray, label: str, *, honor_stop: bool = True) -> float:
    current = packed_state(left, right)
    target = np.asarray(target, dtype=np.float32).copy()
    joint_indices = np.asarray([*range(6), *range(7, 13)])
    initial_error = float(np.max(np.abs(target[joint_indices] - current[joint_indices])))
    if initial_error > 2.0:
        raise RuntimeError(f"{label}距离 {initial_error:.3f}rad 超过2.0rad安全限制")

    right_cmd = arx5.JointState(right_robot.joint_dof)
    left_cmd = arx5.JointState(left_robot.joint_dof)
    right_cmd.pos()[:] = current[:6]
    right_cmd.gripper_pos = float(current[6])
    left_cmd.pos()[:] = current[7:13]
    left_cmd.gripper_pos = float(current[13])
    right.set_joint_cmd(right_cmd)
    left.set_joint_cmd(left_cmd)
    # Historical replay gain and command order.
    set_reduced_gain(left, left_ctrl, left_robot.joint_dof)
    set_reduced_gain(right, right_ctrl, right_robot.joint_dof)

    next_tick = time.monotonic()
    previous = {"right": float("inf"), "left": float("inf")}
    stalled = {"right": 0, "left": 0}
    for step in range(12 * FPS):
        if honor_stop and stop_requested:
            raise StopRequested()
        command_pair(left, right, target, left_cmd, right_cmd, 0.006, 0.0005)
        next_tick += 1.0 / FPS
        delay = next_tick - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        if (step + 1) % FPS == 0:
            measured = packed_state(left, right)
            errors = {
                "right": float(np.max(np.abs(target[:6] - measured[:6]))),
                "left": float(np.max(np.abs(target[7:13] - measured[7:13]))),
            }
            print(
                f"{label}：t={(step + 1) / FPS:.0f}s "
                f"right={errors['right']:.4f}rad left={errors['left']:.4f}rad",
                flush=True,
            )
            for side in ("right", "left"):
                improvement = previous[side] - errors[side]
                stalled[side] = (
                    stalled[side] + 1
                    if previous[side] != float("inf")
                    and errors[side] > 0.08
                    and improvement < 0.003
                    else 0
                )
                previous[side] = errors[side]
            dead = [side for side, count in stalled.items() if count >= 2]
            if dead:
                raise RuntimeError(
                    f"{label}检测到无响应侧={dead}；"
                    f"right={errors['right']:.4f} left={errors['left']:.4f}rad"
                )
            if max(errors.values()) <= 0.05:
                break

    settled = 0
    final_error = float("inf")
    for _ in range(100):
        command_pair(left, right, target, left_cmd, right_cmd, 0.006, 0.0005)
        measured = packed_state(left, right)
        final_error = float(
            np.max(np.abs(target[joint_indices] - measured[joint_indices]))
        )
        if final_error <= 0.08:
            settled += 1
            if settled >= 10:
                break
        else:
            settled = 0
        time.sleep(1.0 / FPS)
    if final_error > 0.08:
        raise RuntimeError(f"{label}最终误差={final_error:.4f}rad > 0.08rad")
    print(f"{label}完成：最大关节误差={final_error:.4f}rad", flush=True)
    return final_error


def run_cycle(index: int, shutdown: np.ndarray, initial: np.ndarray) -> None:
    left = right = None
    left_robot = right_robot = left_ctrl = right_ctrl = None
    at_shutdown = False
    try:
        print(f"BASELINE_CYCLE_{index}_CONNECT left=can0 then right=can1", flush=True)
        left, left_robot, left_ctrl = make_arm(
            "can0", gravity_compensation=True, shutdown_to_passive=True
        )
        right, right_robot, right_ctrl = make_arm(
            "can1", gravity_compensation=True, shutdown_to_passive=True
        )
        set_damping(left, right)
        current = packed_state(left, right)
        joint_indices = np.asarray([*range(6), *range(7, 13)])
        startup_error = float(
            np.max(np.abs(current[joint_indices] - shutdown[joint_indices]))
        )
        if startup_error > 0.35:
            raise RuntimeError(
                f"第{index}轮启动不在停机位附近：误差={startup_error:.4f}rad"
            )
        move_to(
            left, right, left_robot, right_robot, left_ctrl, right_ctrl,
            shutdown, f"第{index}轮进入停机位"
        )
        at_shutdown = True
        at_shutdown = False
        move_to(
            left, right, left_robot, right_robot, left_ctrl, right_ctrl,
            initial, f"第{index}轮进入采集初始位"
        )
        time.sleep(1.0)
        at_shutdown = False
        move_to(
            left, right, left_robot, right_robot, left_ctrl, right_ctrl,
            shutdown, f"第{index}轮返回停机位", honor_stop=False
        )
        at_shutdown = True
        set_damping(left, right)
        print(f"BASELINE_CYCLE_{index}_PASS", flush=True)
    except BaseException:
        if left is not None and right is not None and not at_shutdown:
            print(f"第{index}轮异常：尝试返回停机位后再退出。", flush=True)
            try:
                move_to(
                    left, right, left_robot, right_robot, left_ctrl, right_ctrl,
                    shutdown, f"第{index}轮异常安全回停", honor_stop=False
                )
                at_shutdown = True
            except BaseException as cleanup_error:
                print(f"第{index}轮异常安全回停失败：{cleanup_error}", flush=True)
                hold_until_manual_shutdown(left, right, shutdown, index)
                at_shutdown = True
        raise
    finally:
        if left is not None and right is not None:
            set_damping(left, right)
        # Match the historical hand collection destructor order.
        if right is not None:
            del right
        if left is not None:
            del left
        print(f"BASELINE_CYCLE_{index}_DISCONNECTED at_shutdown={at_shutdown}", flush=True)


def main() -> None:
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, request_stop)
    boot_id = SYSTEM_BOOT_ID_PATH.read_text(encoding="utf-8").strip()
    saved_boot_id = GLOBAL_SHUTDOWN_BOOT_ID_PATH.read_text(encoding="utf-8").strip()
    if boot_id != saved_boot_id:
        raise RuntimeError("停机位不是本次系统开机记录，拒绝测试")
    shutdown = np.load(GLOBAL_SHUTDOWN_POSE_PATH).astype(np.float32)
    initial = np.load(COLLECTION_INITIAL_POSE_PATH).astype(np.float32)
    for name, pose in (("shutdown", shutdown), ("initial", initial)):
        if pose.shape != (ACTION_DIM,) or not np.isfinite(pose).all():
            raise RuntimeError(f"{name}位姿文件无效")

    for index in (1, 2):
        if stop_requested:
            break
        run_cycle(index, shutdown, initial)
        if index == 1:
            print("第一轮已正常析构；不复位CAN，2秒后创建第二轮SDK会话。", flush=True)
            time.sleep(2.0)
    print("HAND_COLLECTION_LIFECYCLE_TEST_COMPLETE", flush=True)


if __name__ == "__main__":
    main()
