#!/usr/bin/env python3
"""Quest bimanual control test with cameras and recording disabled."""

import argparse
import os
from pathlib import Path
import signal
import time

import numpy as np

from arx_common import ACTION_DIM, GRIPPER_WIDTH
from collect_workflow import PersistentArms, RobotFeedbackLost
from quest3_input import Quest3Receiver
from quest3_teleop import QuestTeleopController
from validate_persistent_session_motion import (
    confirm_physical_power_off,
    motion_harness,
    verified_shutdown_error,
    wait_for_manual_shutdown,
)


ROOT = Path(__file__).resolve().parent
INITIAL_POSE_PATH = ROOT / "shared_poses" / "collection_initial_pose.npy"
SHUTDOWN_POSE_PATH = ROOT / "shared_poses" / "shutdown_pose.npy"
SHUTDOWN_BOOT_ID_PATH = ROOT / "shared_poses" / "shutdown_pose_boot_id.txt"
SYSTEM_BOOT_ID_PATH = Path("/proc/sys/kernel/random/boot_id")
stop_requested = False


def request_stop(signum, _frame) -> None:
    global stop_requested
    stop_requested = True
    print(
        f"收到信号 {signum}：立即撤销Quest运动授权，随后安全返回停机位。",
        flush=True,
    )


def load_pose(path: Path, label: str) -> np.ndarray:
    pose = np.load(path).astype(np.float32)
    if pose.shape != (ACTION_DIM,) or not np.isfinite(pose).all():
        raise RuntimeError(f"{label}无效：{path}")
    return pose


def write_ready(path: Path | None) -> None:
    if path is None:
        return
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(f"{os.getpid()}\n", encoding="ascii")
    temporary.replace(path)


def main() -> None:
    global stop_requested
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--quest-host", default=None)
    parser.add_argument("--ready-file", type=Path, default=None)
    parser.add_argument(
        "--camera-mode", choices=("off", "external"), default="off",
        help="diagnostic label only; this process never owns camera devices",
    )
    args = parser.parse_args()
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, request_stop)

    boot_id = SYSTEM_BOOT_ID_PATH.read_text(encoding="utf-8").strip()
    saved_boot_id = SHUTDOWN_BOOT_ID_PATH.read_text(encoding="utf-8").strip()
    if boot_id != saved_boot_id:
        raise RuntimeError("停机位不属于本次系统开机，拒绝Quest运动测试")
    shutdown = load_pose(SHUTDOWN_POSE_PATH, "本次开机停机位")
    initial = load_pose(INITIAL_POSE_PATH, "统一采集初始位")
    initial[[6, 13]] = GRIPPER_WIDTH

    receiver = Quest3Receiver(allowed_sender=args.quest_host)
    arms = None
    harness = None
    controller = None
    at_shutdown = False
    powered_off = False
    failure = None
    try:
        receiver.start()
        receiver.ready.wait(2.0)
        if receiver.error:
            raise RuntimeError(receiver.error)
        print("QUEST_ROBOT_ONLY_CONNECT left=can0 then right=can1", flush=True)
        arms = PersistentArms(expected_shutdown_pose=shutdown)
        harness = motion_harness(arms)
        harness.safe_return_to_pose(
            initial, "进入统一采集初始位", require_open_grippers=True
        )
        arms.enter_position_hold()
        print(
            "控制模式切换：当前为双臂位姿PID闭环固定；等待Quest有效姿态。",
            flush=True,
        )
        controller = QuestTeleopController(
            receiver,
            arms,
            lambda: not stop_requested,
            ROOT / "quest3_teleop_config.json",
            initial_pose=initial,
        )
        controller.start()
        write_ready(args.ready_file)
        print(
            f"QUEST_ROBOT_ONLY_READY cameras={args.camera_mode} recording=off；"
            "分别按住左右Grip才允许对应机械臂跟随。",
            flush=True,
        )
        while not stop_requested and not receiver.snapshot().fresh():
            if controller.error:
                raise RuntimeError(controller.error)
            arms.wait_for_feedback_recovery("等待Quest阶段")
            time.sleep(0.05)
        if stop_requested:
            print("Quest控制尚未开始即收到停止请求。", flush=True)
        else:
            print(
                f"QUEST_ROBOT_ONLY_ACTIVE duration={args.duration:.0f}s "
                "translation_scale=1.25 rotation_scale=1.25 "
                f"cameras={args.camera_mode} recording=off",
                flush=True,
            )
            started = time.monotonic()
            next_report = started
            while not stop_requested and time.monotonic() - started < args.duration:
                if controller.error:
                    raise RuntimeError(controller.error)
                arms.wait_for_feedback_recovery("Quest控制阶段")
                now = time.monotonic()
                if now >= next_report:
                    packet = receiver.snapshot()
                    can_rx = arms.can_watchdog.snapshot()
                    print(
                        f"QUEST_ROBOT_ONLY_HEALTHY elapsed={now-started:.1f}s "
                        f"quest_fresh={packet.fresh()} "
                        f"left_rx={can_rx['left']} right_rx={can_rx['right']}",
                        flush=True,
                    )
                    next_report += 10.0
                time.sleep(0.05)
            print("QUEST_ROBOT_ONLY_CONTROL_COMPLETE", flush=True)
    except BaseException as exc:
        failure = exc
        print(f"QUEST_ROBOT_ONLY_FAILURE {type(exc).__name__}: {exc}", flush=True)
    finally:
        # Revoke the lowest-priority producer before any shutdown-pose command.
        stop_requested = True
        if controller is not None:
            controller.close()
        receiver.close()
        if args.ready_file is not None:
            args.ready_file.unlink(missing_ok=True)
        if arms is not None and arms.connected:
            try:
                if harness is None:
                    harness = motion_harness(arms)
                harness.safe_return_to_pose(shutdown, "安全返回停机位")
                error = verified_shutdown_error(arms, shutdown)
                if error > 0.08:
                    raise RuntimeError(f"停机位验证误差={error:.4f}rad")
                at_shutdown = True
            except RobotFeedbackLost as exc:
                failure = failure or exc
                confirm_physical_power_off()
                powered_off = True
            except BaseException as exc:
                failure = failure or exc
                try:
                    wait_for_manual_shutdown(arms, shutdown)
                    at_shutdown = True
                except BaseException as manual_error:
                    failure = failure or manual_error
                    confirm_physical_power_off()
                    powered_off = True
            if at_shutdown:
                arms.set_teach_mode()
                arms.close()
                print("QUEST_ROBOT_ONLY_DISCONNECTED_AT_SHUTDOWN", flush=True)
            elif powered_off:
                arms.close()
                print("QUEST_ROBOT_ONLY_RELEASED_AFTER_POWER_OFF", flush=True)
            else:
                raise RuntimeError("未确认停机位或物理断电，拒绝释放SDK")
    if failure is not None:
        raise failure
    print("QUEST_ROBOT_ONLY_TEST_COMPLETE", flush=True)


if __name__ == "__main__":
    main()
