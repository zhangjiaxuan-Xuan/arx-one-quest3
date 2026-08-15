from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import arx5_interface as arx5

from resolve_hardware import DEFAULT_REGISTRY, resolve

FPS = 50
ACTION_DIM = 14
MODEL_ACTION_DIM = 32
ACTION_HORIZON = 50
GRIPPER_WIDTH = 0.082
GRIPPER_OPEN_READOUT = -3.4
GRIPPER_CALIBRATION_PATH = (
    Path(__file__).resolve().parent / "shared_poses" / "gripper_calibration.json"
)


def calibrated_gripper_open_readout(interface: str) -> float:
    """Return the per-arm readout without mutating the installed ARX SDK."""
    if not GRIPPER_CALIBRATION_PATH.exists():
        return GRIPPER_OPEN_READOUT
    payload = json.loads(GRIPPER_CALIBRATION_PATH.read_text(encoding="utf-8"))
    if payload.get("schema") != "arx.gripper_calibration.v1":
        raise RuntimeError("夹爪标定文件schema不匹配")
    try:
        value = float(payload[interface]["open_readout"])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f"夹爪标定缺少{interface}开度读数") from exc
    if not np.isfinite(value) or not 0.5 <= abs(value) <= 20.0:
        raise RuntimeError(f"{interface}夹爪开度读数无效：{value}")
    return value


def assert_operator_owned_hardware_session() -> None:
    """Never load the arm SDK inside an agent-managed disposable process tree.

    Codex command sessions run under bwrap with a die-with-parent lifetime.
    Losing that outer session can SIGKILL every descendant and bypass Python's
    mandatory shutdown-pose cleanup. Hardware processes must therefore be
    started by the operator in a normal terminal.
    """
    try:
        init_command = Path("/proc/1/cmdline").read_bytes().replace(b"\0", b" ").lower()
    except OSError:
        init_command = b""
    if b"bwrap" in init_command or b"codex-linux-sandbox" in init_command:
        raise RuntimeError(
            "安全锁：检测到Codex/bwrap托管会话，拒绝加载ARX SDK。"
            "请由操作者在普通终端执行启动命令；托管会话结束会强杀子进程，"
            "可能绕过回停机位流程。"
        )


def resolve_registered_hardware():
    """Return current device nodes/interfaces using stable USB serial mappings."""
    return resolve(DEFAULT_REGISTRY)

NAMES = [
    "right_joint_1",
    "right_joint_2",
    "right_joint_3",
    "right_joint_4",
    "right_joint_5",
    "right_joint_6",
    "right_gripper",
    "left_joint_1",
    "left_joint_2",
    "left_joint_3",
    "left_joint_4",
    "left_joint_5",
    "left_joint_6",
    "left_gripper",
]


def make_arm(
    interface: str,
    *,
    gravity_compensation: bool = True,
    shutdown_to_passive: bool = True,
):
    assert_operator_owned_hardware_session()
    robot = arx5.RobotConfigFactory.get_instance().get_config("X5")
    robot.gripper_open_readout = calibrated_gripper_open_readout(interface)
    robot.gripper_width = GRIPPER_WIDTH
    controller = arx5.ControllerConfigFactory.get_instance().get_config(
        "joint_controller", robot.joint_dof
    )
    controller.background_send_recv = True
    controller.gravity_compensation = gravity_compensation
    controller.shutdown_to_passive = shutdown_to_passive
    arm = arx5.Arx5JointController(robot, controller, interface)
    arm.set_log_level(arx5.LogLevel.WARNING)
    return arm, robot, controller


def copy_state(arm):
    state = arm.get_joint_state()
    return (
        state.pos().copy().astype(np.float32),
        state.vel().copy().astype(np.float32),
        state.torque().copy().astype(np.float32),
        np.float32(state.gripper_pos),
        np.float32(state.gripper_vel),
        np.float32(state.gripper_torque),
    )


def pack_bimanual(right, left) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    state = np.concatenate([right[0], [right[3]], left[0], [left[3]]]).astype(np.float32)
    velocity = np.concatenate([right[1], [right[4]], left[1], [left[4]]]).astype(np.float32)
    effort = np.concatenate([right[2], [right[5]], left[2], [left[5]]]).astype(np.float32)
    return state, velocity, effort


def unpack_action(action: np.ndarray) -> tuple[np.ndarray, float, np.ndarray, float]:
    action = np.asarray(action, dtype=np.float32)
    if action.shape[-1] < ACTION_DIM:
        raise ValueError(f"Expected at least {ACTION_DIM} action dimensions, got {action.shape}")
    return action[:6], float(action[6]), action[7:13], float(action[13])
