#!/usr/bin/env python3
import argparse
import os
from pathlib import Path
import signal
import time

import arx5_interface as arx5
import numpy as np

from arx_common import FPS, GRIPPER_WIDTH, copy_state, pack_bimanual
from collect_workflow import PersistentArms
from quest3_input import Quest3Receiver
from quest3_teleop import QuestTeleopController
from replay_pi05 import command_pair, set_reduced_gain


ROOT = Path(__file__).resolve().parent
INITIAL_POSE_PATH = ROOT / "shared_poses" / "collection_initial_pose.npy"
SHUTDOWN_POSE_PATH = ROOT / "sessions" / "poses" / "shutdown_pose.npy"
SHUTDOWN_BOOT_ID_PATH = ROOT / "sessions" / "poses" / "shutdown_pose_boot_id.txt"
SYSTEM_BOOT_ID_PATH = Path("/proc/sys/kernel/random/boot_id")
_shutdown_signal_seen = False
_shutdown_cleanup_active = False


def request_interrupt(signum, frame):
    global _shutdown_signal_seen
    if _shutdown_cleanup_active:
        return
    if not _shutdown_signal_seen:
        _shutdown_signal_seen = True
        raise KeyboardInterrupt(f"received signal {signum}")


def return_to_start(arms, target):
    current, _, _ = arms.state()
    left_cmd = arx5.JointState(arms.left_robot.joint_dof)
    right_cmd = arx5.JointState(arms.right_robot.joint_dof)
    right_cmd.pos()[:] = current[:6]
    right_cmd.gripper_pos = float(current[6])
    left_cmd.pos()[:] = current[7:13]
    left_cmd.gripper_pos = float(current[13])
    with arms.sdk_lock:
        arms.left.set_joint_cmd(left_cmd)
        arms.right.set_joint_cmd(right_cmd)
        set_reduced_gain(arms.left, arms.left_ctrl, arms.left_robot.joint_dof)
        set_reduced_gain(arms.right, arms.right_ctrl, arms.right_robot.joint_dof)
    for _ in range(8 * FPS):
        fault = arms.health_fault()
        if fault is not None:
            raise RuntimeError(f"CAN反馈看门狗：{fault}")
        with arms.sdk_lock:
            command_pair(
                arms.left, arms.right, target, left_cmd, right_cmd, 0.004, 0.0003
            )
        time.sleep(1.0 / FPS)
    measured, _, _ = arms.state()
    error = float(np.max(np.abs(measured - target)))
    if error > 0.08:
        raise RuntimeError(f"return error {error:.4f} exceeds 0.08")
    print(f"已回到测试开始姿态，最大误差={error:.4f}", flush=True)


def main():
    signal.signal(signal.SIGTERM, request_interrupt)
    signal.signal(signal.SIGINT, request_interrupt)
    signal.signal(signal.SIGUSR1, request_interrupt)
    signal.signal(signal.SIGHUP, request_interrupt)
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--quest-host", default=None)
    parser.add_argument("--ready-file", type=Path, default=None)
    parser.add_argument("--shutdown-status-file", type=Path, default=None)
    parser.add_argument(
        "--durability-plan", action="store_true",
        help="Print the 180-second staged durability-test prompts",
    )
    args = parser.parse_args()
    receiver = Quest3Receiver(allowed_sender=args.quest_host)
    arms = controller = None
    shutdown_pose = None
    reached_shutdown = False
    try:
        receiver.start()
        receiver.ready.wait(2.0)
        if receiver.error:
            raise RuntimeError(receiver.error)
        if not INITIAL_POSE_PATH.exists():
            raise RuntimeError(f"未找到已注册的采集初始姿态：{INITIAL_POSE_PATH}")
        start = np.load(INITIAL_POSE_PATH).astype(np.float32)
        if start.shape != (14,) or not np.isfinite(start).all():
            raise RuntimeError(f"采集初始姿态无效：shape={start.shape}")
        start[[6, 13]] = GRIPPER_WIDTH
        if not SHUTDOWN_POSE_PATH.exists() or not SHUTDOWN_BOOT_ID_PATH.exists():
            raise RuntimeError("未找到本次开机捕获的停机姿态，拒绝启动测试")
        saved_boot_id = SHUTDOWN_BOOT_ID_PATH.read_text(encoding="utf-8").strip()
        current_boot_id = SYSTEM_BOOT_ID_PATH.read_text(encoding="utf-8").strip()
        if saved_boot_id != current_boot_id:
            raise RuntimeError("停机姿态不属于本次开机，拒绝启动测试")
        shutdown_pose = np.load(SHUTDOWN_POSE_PATH).astype(np.float32)
        if shutdown_pose.shape != (14,) or not np.isfinite(shutdown_pose).all():
            raise RuntimeError(f"停机姿态无效：shape={shutdown_pose.shape}")
        print("正在连接双臂；双臂和夹爪均已启用；Grip分别使能左右臂。", flush=True)
        arms = PersistentArms(expected_shutdown_pose=shutdown_pose)
        arms.verify_position_response()
        print("正在安全回归已注册的采集初始姿态；到位后才开放Quest控制。", flush=True)
        return_to_start(arms, start)
        print("已到达采集初始姿态，等待Quest有效数据。", flush=True)
        arms.enter_position_hold()
        controller = QuestTeleopController(
            receiver,
            arms,
            lambda: True,
            ROOT / "quest3_teleop_config.json",
            initial_pose=start,
        )
        controller.start()
        if args.ready_file is not None:
            args.ready_file.parent.mkdir(parents=True, exist_ok=True)
            temporary = args.ready_file.with_suffix(args.ready_file.suffix + ".tmp")
            temporary.write_text(f"{os.getpid()}\n", encoding="ascii")
            temporary.replace(args.ready_file)
        print("等待Quest有效数据；请保持两臂周围无障碍。", flush=True)
        while not receiver.snapshot().fresh():
            if controller.error:
                raise RuntimeError(controller.error)
            fault = arms.health_fault()
            if fault is not None:
                raise RuntimeError(f"CAN反馈看门狗：{fault}")
            time.sleep(0.05)
        print(
            "双臂测试开始：分别按住左右Grip移动；松开对应Grip立即冻结；"
            "Trigger松开张开、按下闭合。",
            flush=True,
        )
        if args.durability_plan:
            print(
                "耐久测试阶段：0-30秒静置；30-90秒双臂同时平移/旋转；"
                "90-120秒双夹爪开合；120-150秒Grip松开/重按；"
                "150-165秒Quest追踪遮挡后恢复；165-180秒连续双臂运动。",
                flush=True,
            )
        deadline = time.monotonic() + args.duration
        test_started = time.monotonic()
        next_can_report = test_started + 10.0
        previous_rx = arms.can_watchdog.snapshot()
        while time.monotonic() < deadline:
            if controller.error:
                raise RuntimeError(controller.error)
            fault = arms.health_fault()
            if fault is not None:
                raise RuntimeError(f"CAN反馈看门狗：{fault}")
            now = time.monotonic()
            if now >= next_can_report:
                current_rx = arms.can_watchdog.snapshot()
                elapsed = now - test_started
                print(
                    f"CAN_HEALTH t={elapsed:.1f}s "
                    f"left_rx=+{current_rx['left'] - previous_rx['left']} "
                    f"right_rx=+{current_rx['right'] - previous_rx['right']} "
                    "rx_errors=0",
                    flush=True,
                )
                previous_rx = current_rx
                next_can_report += 10.0
            time.sleep(0.1)
        print("测试计时结束，正在回归采集初始姿态。", flush=True)
        controller.close()
        controller = None
        return_to_start(arms, start)
    except KeyboardInterrupt:
        print("收到停止请求，开始执行安全停机流程。", flush=True)
    finally:
        global _shutdown_cleanup_active
        _shutdown_cleanup_active = True
        print("安全停机已锁定：后续 Ctrl-C 不会中断回停机位流程。", flush=True)
        if controller is not None:
            controller.close()
        receiver.close()
        if arms is not None:
            while arms.connected:
                try:
                    if shutdown_pose is None:
                        raise RuntimeError("没有可验证的本次开机停机姿态")
                    print("正在安全回归本次开机停机姿态；到位后才断开控制。", flush=True)
                    return_to_start(arms, shutdown_pose)
                    reached_shutdown = True
                    arms.set_teach_mode()
                    arms.close()
                except BaseException as exc:
                    try:
                        arms.set_teach_mode()
                    except BaseException:
                        pass
                    print(
                        f"拒绝断开：尚未确认停机姿态（{exc}）。"
                        "双臂保持重力补偿连接，2秒后自动重试。",
                        flush=True,
                    )
                    time.sleep(2.0)
        if reached_shutdown:
            print("机械臂已回到停机姿态、恢复低阻尼并断开控制。", flush=True)
            if args.shutdown_status_file is not None:
                temporary = args.shutdown_status_file.with_suffix(
                    args.shutdown_status_file.suffix + ".tmp"
                )
                temporary.write_text("SAFE_SHUTDOWN_COMPLETE\n", encoding="ascii")
                temporary.replace(args.shutdown_status_file)
            print("SAFE_SHUTDOWN_COMPLETE", flush=True)
        elif arms is None:
            print("机械臂控制器未成功建立；未执行运动或停机回归。", flush=True)


if __name__ == "__main__":
    main()
