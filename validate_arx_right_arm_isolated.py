#!/usr/bin/env python3
"""Isolated right-arm response probe using only public arx5_interface APIs."""

from pathlib import Path
import signal
import time

import numpy as np
import arx5_interface as arx5

from arx_common import make_arm
from collect_workflow import (
    ACTION_DIM,
    GLOBAL_SHUTDOWN_BOOT_ID_PATH,
    GLOBAL_SHUTDOWN_POSE_PATH,
    SYSTEM_BOOT_ID_PATH,
)


def main() -> None:
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, signal.SIG_IGN)
    boot_id = SYSTEM_BOOT_ID_PATH.read_text(encoding="utf-8").strip()
    saved_boot_id = GLOBAL_SHUTDOWN_BOOT_ID_PATH.read_text(encoding="utf-8").strip()
    if boot_id != saved_boot_id:
        raise RuntimeError("停机姿态不是本次系统开机记录，拒绝测试")
    shutdown = np.load(GLOBAL_SHUTDOWN_POSE_PATH).astype(np.float32)
    if shutdown.shape != (ACTION_DIM,) or not np.isfinite(shutdown).all():
        raise RuntimeError("停机姿态文件无效")

    arm = robot = controller = None
    baseline = None
    try:
        arm, robot, controller = make_arm(
            "can1", gravity_compensation=True, shutdown_to_passive=True
        )
        arm.set_to_damping()
        time.sleep(0.10)
        state = arm.get_joint_state()
        baseline = state.pos().copy()
        shutdown_error = float(np.max(np.abs(baseline - shutdown[:6])))
        if shutdown_error > 0.35:
            raise RuntimeError(
                f"右臂当前不在停机位置：误差={shutdown_error:.4f}rad > 0.35rad"
            )

        command = arx5.JointState(robot.joint_dof)
        command.pos()[:] = baseline
        command.gripper_pos = float(state.gripper_pos)
        arm.set_joint_cmd(command)

        gain = arx5.Gain(robot.joint_dof)
        gain.kp()[:] = np.asarray(controller.default_kp) * 0.75
        gain.kd()[:] = np.asarray(controller.default_kd)
        gain.gripper_kp = 0.0
        gain.gripper_kd = float(controller.default_gripper_kd)
        arm.set_gain(gain)
        time.sleep(0.10)
        applied_gain = arm.get_gain()
        print(
            "右臂隔离控制增益："
            f"kp={np.asarray(applied_gain.kp()).round(3).tolist()}，"
            f"kd={np.asarray(applied_gain.kd()).round(3).tolist()}",
            flush=True,
        )

        lower = float(robot.joint_pos_min[4])
        upper = float(robot.joint_pos_max[4])
        direction = 1.0 if baseline[4] < (lower + upper) * 0.5 else -1.0
        target = float(np.clip(baseline[4] + direction * 0.03, lower + 0.03, upper - 0.03))
        command.pos()[4] = target
        arm.set_joint_cmd(command)
        # Observe the whole response window. A single snapshot can miss a
        # delayed response and incorrectly report zero movement.
        moved = 0.0
        moved_state = None
        for _ in range(100):
            sample = arm.get_joint_state()
            sample_movement = float(abs(sample.pos()[4] - baseline[4]))
            if sample_movement >= moved:
                moved = sample_movement
                moved_state = sample
            time.sleep(0.02)
        command_state = arm.get_joint_cmd()
        commanded = float(abs(command_state.pos()[4] - baseline[4]))
        assert moved_state is not None
        velocity = float(moved_state.vel()[4])
        torque = float(moved_state.torque()[4])
        print(
            "右臂隔离微动结果："
            f"实测={moved:.4f}rad，命令={commanded:.4f}rad，"
            f"速度={velocity:.4f}rad/s，扭矩={torque:.4f}Nm",
            flush=True,
        )
        print(
            "右臂六关节诊断："
            f"位置变化={np.asarray(moved_state.pos() - baseline).round(5).tolist()}，"
            f"速度={np.asarray(moved_state.vel()).round(5).tolist()}，"
            f"扭矩={np.asarray(moved_state.torque()).round(5).tolist()}",
            flush=True,
        )

        command.pos()[:] = baseline
        arm.set_joint_cmd(command)
        restore_error = float("inf")
        settled_samples = 0
        for _ in range(150):
            restore_state = arm.get_joint_state()
            restored = restore_state.pos().copy()
            restore_error = float(np.max(np.abs(restored - baseline)))
            if restore_error <= 0.010:
                settled_samples += 1
                if settled_samples >= 10:
                    break
            else:
                settled_samples = 0
            time.sleep(0.02)
        if restore_error > 0.025:
            raise RuntimeError(f"右臂隔离测试未安全回位：误差={restore_error:.4f}rad")
        if moved < 0.008:
            raise RuntimeError(
                f"右臂在单SDK实例中仍无位置响应：实测={moved:.4f}/命令={commanded:.4f}rad"
            )
        print(
            f"ARX_RIGHT_ISOLATED_PASS movement={moved:.4f}rad restore={restore_error:.4f}rad",
            flush=True,
        )
    finally:
        if arm is not None:
            try:
                if baseline is not None:
                    command = arx5.JointState(robot.joint_dof)
                    command.pos()[:] = baseline
                    current = arm.get_joint_state()
                    command.gripper_pos = float(current.gripper_pos)
                    arm.set_joint_cmd(command)
                    time.sleep(0.50)
                arm.set_to_damping()
            finally:
                del arm
            print("右臂隔离诊断结束：已恢复阻尼并断开SDK。", flush=True)


if __name__ == "__main__":
    main()
