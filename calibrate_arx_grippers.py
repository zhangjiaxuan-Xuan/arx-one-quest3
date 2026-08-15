#!/usr/bin/env python3
"""Interactive per-arm gripper zeroing before the normal ARX session."""

from __future__ import annotations

import json
from pathlib import Path
import time

import numpy as np
import arx5_interface as arx5

from arx_common import (
    GRIPPER_CALIBRATION_PATH,
    GRIPPER_WIDTH,
    assert_operator_owned_hardware_session,
)


ROOT = Path(__file__).resolve().parent
SHUTDOWN_POSE_PATH = ROOT / "shared_poses" / "shutdown_pose.npy"
SHUTDOWN_BOOT_ID_PATH = ROOT / "shared_poses" / "shutdown_pose_boot_id.txt"
SYSTEM_BOOT_ID_PATH = Path("/proc/sys/kernel/random/boot_id")
# The constructor sanity check accepts -5 mm..width+5 mm.  A deliberately
# large temporary scale maps any normal DM motor angle close to zero while
# preserving the exact raw angle when the damping command is converted back.
# It is used only long enough to invoke the official reset-zero routine.
BOOTSTRAP_OPEN_READOUT = 1000.0
CLOSED_WIDTH_TOLERANCE_M = 0.004


def make_calibration_arm(interface: str):
    robot = arx5.RobotConfigFactory.get_instance().get_config("X5")
    robot.gripper_open_readout = BOOTSTRAP_OPEN_READOUT
    robot.gripper_width = GRIPPER_WIDTH
    controller = arx5.ControllerConfigFactory.get_instance().get_config(
        "joint_controller", robot.joint_dof
    )
    controller.background_send_recv = True
    controller.gravity_compensation = False
    controller.shutdown_to_passive = True
    arm = arx5.Arx5JointController(robot, controller, interface)
    arm.set_log_level(arx5.LogLevel.INFO)
    arm.set_to_damping()
    return arm


def calibrate_one(interface: str, label: str):
    print("\n" + "=" * 72, flush=True)
    print(f"开始{label}夹爪手动归零（{interface}）；机械臂仅保持阻尼。", flush=True)
    print("【第1步】完全闭合夹爪，看到SDK提示后按Enter写入零点。", flush=True)
    print("【第2步】随后必须完全张开夹爪，再按Enter记录最大开度。", flush=True)
    print("注意：第2次Enter之前不要把夹爪重新闭合。", flush=True)
    arm = make_calibration_arm(interface)
    arm.calibrate_gripper()
    time.sleep(0.25)
    state = arm.get_joint_state()
    open_readout = (
        float(state.gripper_pos) / GRIPPER_WIDTH * BOOTSTRAP_OPEN_READOUT
    )
    if not np.isfinite(open_readout) or not 0.5 <= abs(open_readout) <= 20.0:
        del arm
        raise RuntimeError(
            f"{label}完全张开读数异常：{open_readout:.6f}rad；"
            "第2次Enter时夹爪仍接近闭合或没有完全张开，标定不保存"
        )
    print(f"{label}夹爪标定读数={open_readout:.6f}rad", flush=True)
    print(
        f"【第3步】现在再次完全闭合{label}夹爪，按Enter验证闭合是否回到0。",
        flush=True,
    )
    input(f"{label}夹爪完全闭合后按 Enter > ")
    time.sleep(0.25)
    closed_state = arm.get_joint_state()
    closed_readout = (
        float(closed_state.gripper_pos) / GRIPPER_WIDTH * BOOTSTRAP_OPEN_READOUT
    )
    closed_width = closed_readout / open_readout * GRIPPER_WIDTH
    if not np.isfinite(closed_width) or abs(closed_width) > CLOSED_WIDTH_TOLERANCE_M:
        del arm
        raise RuntimeError(
            f"{label}二次闭合未回到零点：等效宽度={closed_width:.6f}m，"
            f"允许±{CLOSED_WIDTH_TOLERANCE_M:.3f}m；标定不保存"
        )
    print(
        f"{label}闭合零点复验通过：raw={closed_readout:.6f}rad，"
        f"等效宽度={closed_width:.6f}m",
        flush=True,
    )
    return arm, open_readout


def save_calibration(values: dict[str, float]) -> None:
    payload = {
        "schema": "arx.gripper_calibration.v1",
        "created_unix": time.time(),
        "can0": {"role": "left", "open_readout": values["can0"]},
        "can1": {"role": "right", "open_readout": values["can1"]},
    }
    GRIPPER_CALIBRATION_PATH.parent.mkdir(parents=True, exist_ok=True)
    temporary = GRIPPER_CALIBRATION_PATH.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(GRIPPER_CALIBRATION_PATH)


def capture_shutdown_pose(left, right) -> None:
    print("\n夹爪归零完成。现在把双臂摆到希望的停机姿态，然后按Enter。", flush=True)
    input("停机姿态准备好后按 Enter > ")
    time.sleep(0.15)
    right_state = right.get_joint_state()
    left_state = left.get_joint_state()
    pose = np.concatenate(
        [
            np.asarray(right_state.pos(), dtype=np.float32),
            [0.0],
            np.asarray(left_state.pos(), dtype=np.float32),
            [0.0],
        ]
    ).astype(np.float32)
    if pose.shape != (14,) or not np.isfinite(pose).all():
        raise RuntimeError("自动捕获的停机姿态无效")
    SHUTDOWN_POSE_PATH.parent.mkdir(parents=True, exist_ok=True)
    np.save(SHUTDOWN_POSE_PATH, pose)
    SHUTDOWN_BOOT_ID_PATH.write_text(
        SYSTEM_BOOT_ID_PATH.read_text(encoding="ascii").strip() + "\n",
        encoding="ascii",
    )
    print(f"本次开机停机姿态已自动更新：{SHUTDOWN_POSE_PATH}", flush=True)


def main() -> None:
    assert_operator_owned_hardware_session()
    print("ARX双夹爪启动标定：过程中不执行关节轨迹。", flush=True)
    print("请支撑双臂；夹爪移动全部由你手动完成。", flush=True)
    left = right = None
    try:
        # Preserve the only lifecycle that has remained stable on this AC One:
        # create can0/left first, keep it alive, then create can1/right.  Do not
        # destroy and reconstruct another controller in this Python process.
        left, left_readout = calibrate_one("can0", "左臂")
        right, right_readout = calibrate_one("can1", "右臂")
        values = {"can0": left_readout, "can1": right_readout}
        save_calibration(values)
        print(f"左右独立夹爪标定已保存：{GRIPPER_CALIBRATION_PATH}", flush=True)
        capture_shutdown_pose(left, right)
    finally:
        # Match the historical bimanual destructor order.
        if right is not None:
            del right
        if left is not None:
            del left


if __name__ == "__main__":
    main()
