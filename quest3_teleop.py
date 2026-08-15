#!/usr/bin/env python3
"""Guarded Quest 3 Touch-controller teleoperation for ARX AC One."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import threading
import time
from typing import Callable

import numpy as np
import arx5_interface as arx5
from scipy.spatial.transform import Rotation

from arx_common import FPS, GRIPPER_WIDTH, copy_state
from quest3_input import ControllerState, Quest3Receiver
from remote_delta_roundtrip import GripperGuard, solver


MAX_JOINT_JUMP_RAD = 0.35
# VR command slew limit only: 0.015 rad/tick at 50 Hz = 0.75 rad/s.
# Safe return, replay and model deployment keep their own validated limits.
MAX_JOINT_STEP_RAD = 0.015
MAX_JOINT_VELOCITY_RAD_S = MAX_JOINT_STEP_RAD * FPS
MAX_JOINT_ACCELERATION_RAD_S2 = 8.0
IK_TARGET_FILTER_TIME_CONSTANT_SECONDS = 0.08
MAX_GRIPPER_STEP_M = 0.003
# A per-arm return is allowed while the other arm is still being teleoperated.
# Keep it at the already validated safe-return slew rate (0.30 rad/s at 50 Hz),
# rather than the faster operator-following rate above.
SINGLE_ARM_RETURN_STEP_RAD = 0.006
SINGLE_ARM_RETURN_COMPLETE_RAD = 0.05
SINGLE_ARM_RETURN_GRIPPER_TOLERANCE_M = 0.002
PID_INTEGRAL_LIMIT = np.asarray([0.03] * 3 + [0.15] * 3, dtype=np.float64)
PID_INTEGRATION_ERROR_GATE = np.asarray(
    [0.04] * 3 + [0.25] * 3, dtype=np.float64
)
PID_CORRECTION_LIMIT = np.asarray([0.03] * 3 + [0.12] * 3, dtype=np.float64)
PID_INTEGRAL_LEAK = 0.998
IK_BACKOFF_SCALES = (1.0, 0.5, 0.25, 0.125)
IK_FAILURES_BEFORE_REANCHOR = 5


@dataclass
class SideConfig:
    enabled: bool
    gripper_enabled: bool
    translation_scale: float
    rotation_scale: float
    max_translation_m: float
    quest_to_robot: np.ndarray
    rotation_quest_to_eef: np.ndarray
    pid_kp: np.ndarray
    pid_ki: np.ndarray
    pid_kd: np.ndarray


@dataclass
class SideSession:
    engaged: bool = False
    controller_position: np.ndarray | None = None
    controller_rotation: Rotation | None = None
    robot_pose: np.ndarray | None = None
    command: arx5.JointState | None = None
    gripper_command: int = 0
    gripper_guard: GripperGuard | None = None
    ik_failures: int = 0
    pid_integral: np.ndarray | None = None
    pid_previous_error: np.ndarray | None = None
    pid_previous_time: float | None = None
    filtered_joint_target: np.ndarray | None = None
    joint_velocity_command: np.ndarray | None = None
    last_command_time: float | None = None
    last_command_step: np.ndarray | None = None
    last_command_interval: float = 1.0 / FPS
    needs_reanchor: bool = False
    reconnect_count: int = 0
    transient_faults: int = 0
    returning_to_initial: bool = False
    at_initial_position: bool = False


def advance_single_arm_return(
    command_position: np.ndarray,
    target_position: np.ndarray,
    measured_position: np.ndarray,
) -> tuple[np.ndarray, bool]:
    """Slew one arm toward its standard pose and report measured completion."""
    command = np.asarray(command_position, dtype=np.float64)
    target = np.asarray(target_position, dtype=np.float64)
    measured = np.asarray(measured_position, dtype=np.float64)
    if command.shape != (6,) or target.shape != (6,) or measured.shape != (6,):
        raise ValueError("single-arm return expects six joint positions")
    if not all(np.isfinite(value).all() for value in (command, target, measured)):
        raise ValueError("single-arm return received non-finite joint positions")
    next_command = command + np.clip(
        target - command,
        -SINGLE_ARM_RETURN_STEP_RAD,
        SINGLE_ARM_RETURN_STEP_RAD,
    )
    complete = bool(
        np.max(np.abs(target - measured)) <= SINGLE_ARM_RETURN_COMPLETE_RAD
    )
    return next_command, complete


def low_pass_ik_target(
    previous: np.ndarray,
    target: np.ndarray,
    dt: float,
) -> np.ndarray:
    """Filter numerical IK branch noise without changing the final target."""
    previous = np.asarray(previous, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    if previous.shape != (6,) or target.shape != (6,):
        raise ValueError("IK target filter expects six joints")
    if not np.isfinite(previous).all() or not np.isfinite(target).all():
        raise ValueError("IK target filter received non-finite joints")
    dt = max(1e-3, min(0.05, float(dt)))
    alpha = 1.0 - np.exp(-dt / IK_TARGET_FILTER_TIME_CONSTANT_SECONDS)
    return previous + alpha * (target - previous)


def acceleration_limited_joint_step(
    error: np.ndarray,
    previous_velocity: np.ndarray,
    dt: float,
    acceleration_dt: float | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Generate a velocity/acceleration-limited step with braking distance.

    A reversed IK target first decelerates the existing command velocity to
    zero before moving the opposite way.  This prevents individually legal
    position steps from alternating at full speed every 20 ms.
    """
    error = np.asarray(error, dtype=np.float64)
    previous_velocity = np.asarray(previous_velocity, dtype=np.float64)
    if error.shape != (6,) or previous_velocity.shape != (6,):
        raise ValueError("joint smoother expects six joints")
    if not np.isfinite(error).all() or not np.isfinite(previous_velocity).all():
        raise ValueError("joint smoother received non-finite values")
    dt = max(1e-3, min(0.05, float(dt)))
    acceleration_dt = (
        dt
        if acceleration_dt is None
        else max(1e-3, min(0.05, float(acceleration_dt)))
    )
    braking_velocity = np.sqrt(
        2.0 * MAX_JOINT_ACCELERATION_RAD_S2 * np.abs(error)
    )
    desired_velocity = np.sign(error) * np.minimum(
        np.minimum(braking_velocity, MAX_JOINT_VELOCITY_RAD_S),
        np.abs(error) / dt,
    )
    max_velocity_change = MAX_JOINT_ACCELERATION_RAD_S2 * acceleration_dt
    velocity = previous_velocity + np.clip(
        desired_velocity - previous_velocity,
        -max_velocity_change,
        max_velocity_change,
    )
    step = velocity * dt
    # Do not snap exactly onto the target: that would silently violate the
    # acceleration limit near zero. The braking profile permits a tiny,
    # bounded overshoot and decelerates it smoothly on subsequent ticks.
    return step, velocity


def trigger_gripper_command(trigger: float) -> int:
    """Map a newly engaged controller to the dataset's strict 0=open/1=close."""
    value = float(trigger)
    if not np.isfinite(value):
        raise ValueError("non-finite Quest Trigger value")
    # On first Grip engagement, anything short of a deliberate close press is
    # open. Never infer operator intent from measured gripper width: an open
    # gripper can settle below half-width and was previously misclassified as
    # closed, causing the observed inward rebound.
    return 1 if value >= 0.65 else 0


def forward_motion_calibration_matrix(displacement: np.ndarray) -> np.ndarray:
    """Rotate a measured horizontal forward stroke onto canonical Quest +X."""
    forward = np.asarray(displacement, dtype=np.float64).copy()
    if forward.shape != (3,) or not np.isfinite(forward).all():
        raise ValueError("正前方位移样本无效")
    forward[1] = 0.0
    distance = float(np.linalg.norm(forward))
    if distance < 0.05:
        raise ValueError("正前方校准移动不足 5 cm，请按住 Menu 明显向前移动 1 秒")
    forward /= distance
    up = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
    lateral = np.cross(forward, up)
    matrix = np.stack([forward, up, lateral], axis=0)
    if not np.allclose(matrix @ matrix.T, np.eye(3), atol=1e-6):
        raise ValueError("正前方偏航矩阵无效")
    return matrix


def load_config(path: Path) -> dict[str, SideConfig]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    if raw.get("schema") != "arx.quest3.teleop_config.v1":
        raise ValueError("Quest teleop configuration schema mismatch")
    result = {}
    for side in ("left", "right"):
        item = raw[side]
        basis = np.asarray(item["quest_to_robot"], dtype=np.float64)
        if basis.shape != (3, 3) or not np.isfinite(basis).all():
            raise ValueError(f"invalid {side} quest_to_robot matrix")
        if not np.allclose(basis @ basis.T, np.eye(3), atol=1e-5):
            raise ValueError(f"{side} quest_to_robot must be orthonormal")
        rotation_basis = np.asarray(item["rotation_quest_to_eef"], dtype=np.float64)
        if rotation_basis.shape != (3, 3) or not np.isfinite(rotation_basis).all():
            raise ValueError(f"invalid {side} rotation_quest_to_eef matrix")
        if not np.allclose(rotation_basis @ rotation_basis.T, np.eye(3), atol=1e-5):
            raise ValueError(f"{side} rotation_quest_to_eef must be orthonormal")
        if not np.isclose(np.linalg.det(rotation_basis), 1.0, atol=1e-5):
            raise ValueError(f"{side} rotation_quest_to_eef must be a proper rotation")
        result[side] = SideConfig(
            enabled=bool(item.get("enabled", False)),
            gripper_enabled=bool(item.get("gripper_enabled", False)),
            translation_scale=float(item.get("translation_scale", 1.0)),
            rotation_scale=float(item.get("rotation_scale", 1.0)),
            max_translation_m=float(item.get("max_translation_m", 0.03)),
            quest_to_robot=basis,
            rotation_quest_to_eef=rotation_basis,
            pid_kp=np.asarray(item.get("pid_kp", [1.0] * 6), dtype=np.float64),
            pid_ki=np.asarray(item.get("pid_ki", [0.0] * 6), dtype=np.float64),
            pid_kd=np.asarray(item.get("pid_kd", [0.0] * 6), dtype=np.float64),
        )
        for name in ("pid_kp", "pid_ki", "pid_kd"):
            value = getattr(result[side], name)
            if value.shape != (6,) or not np.isfinite(value).all() or (value < 0).any():
                raise ValueError(f"invalid {side} {name}")
    return result


def target_pose(
    anchor_pose: np.ndarray,
    anchor_controller_position: np.ndarray,
    anchor_controller_rotation: Rotation,
    controller: ControllerState,
    config: SideConfig,
    world_to_canonical: np.ndarray | None = None,
) -> np.ndarray:
    position = np.asarray(controller.position, dtype=np.float64)
    rotation = Rotation.from_quat(controller.orientation_xyzw)
    target = np.asarray(anchor_pose, dtype=np.float64).copy()
    yaw_correction = (
        np.eye(3, dtype=np.float64)
        if world_to_canonical is None
        else np.asarray(world_to_canonical, dtype=np.float64)
    )
    delta_position = (
        config.quest_to_robot
        @ yaw_correction
        @ (position - anchor_controller_position)
    )
    target[:3] += delta_position * config.translation_scale
    # Rotation is intentionally independent of the world-frame translation
    # basis.  Compute the controller delta in its Grip-anchor local frame,
    # map calibrated pitch/yaw/roll to the end-effector's local Y/Z/X axes,
    # then compose on the right of the anchored end-effector orientation. This
    # prevents axes from changing when the arm or controller starts at a
    # different world orientation.
    relative_local = anchor_controller_rotation.inv() * rotation
    relative_vector = relative_local.as_rotvec() * config.rotation_scale
    mapped_local = Rotation.from_rotvec(
        config.rotation_quest_to_eef @ relative_vector
    )
    target[3:6] = (
        Rotation.from_rotvec(anchor_pose[3:6]) * mapped_local
    ).as_rotvec()
    return target


def safe_pid_correction(
    error: np.ndarray,
    previous_error: np.ndarray,
    integral: np.ndarray,
    dt: float,
    kp: np.ndarray,
    ki: np.ndarray,
    kd: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Bounded task-space PID with leaky conditional integration.

    Integral action is only allowed near the target, cannot wind farther into
    a saturated output, and slowly decays whenever it is not useful.  The
    caller rolls it back if IK or the joint-jump guard rejects the correction.
    """
    error = np.asarray(error, dtype=np.float64)
    previous_error = np.asarray(previous_error, dtype=np.float64)
    leaked = np.asarray(integral, dtype=np.float64) * PID_INTEGRAL_LEAK
    integration_allowed = np.abs(error) <= PID_INTEGRATION_ERROR_GATE
    candidate = leaked + np.where(integration_allowed, error * dt, 0.0)
    candidate = np.clip(candidate, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT)
    derivative = (error - previous_error) / dt
    unsaturated = kp * error + ki * candidate + kd * derivative
    saturation = np.abs(unsaturated) > PID_CORRECTION_LIMIT
    drives_further_into_limit = np.sign(error) == np.sign(unsaturated)
    blocked = saturation & drives_further_into_limit
    candidate = np.where(blocked, leaked, candidate)
    correction = kp * error + ki * candidate + kd * derivative
    return np.clip(
        correction, -PID_CORRECTION_LIMIT, PID_CORRECTION_LIMIT
    ), candidate


def solve_ik_with_backoff(
    kinematics,
    measured_pose: np.ndarray,
    measured_joints: np.ndarray,
    correction: np.ndarray,
) -> tuple[int, np.ndarray | None, float | None]:
    """Preserve 6D IK while retreating from an overstated SDK joint boundary."""
    last_status = -1
    for scale in IK_BACKOFF_SCALES:
        candidate = np.asarray(measured_pose, dtype=np.float64).copy()
        candidate[:3] += correction[:3] * scale
        candidate[3:6] = (
            Rotation.from_rotvec(correction[3:] * scale)
            * Rotation.from_rotvec(measured_pose[3:6])
        ).as_rotvec()
        status, joints = kinematics.multi_trial_ik(candidate, measured_joints, 6)
        last_status = status
        if status == 0:
            return status, np.asarray(joints, dtype=np.float64), scale
    return last_status, None, None


class QuestTeleopController(threading.Thread):
    """50 Hz controller; each Grip independently clutches one robot arm."""

    def __init__(
        self,
        receiver: Quest3Receiver,
        arms,
        workflow_state: Callable[[], str | bool],
        config_path: Path,
        initial_pose: np.ndarray | None = None,
    ):
        super().__init__(daemon=True, name="quest3-teleop")
        self.receiver = receiver
        self.arms = arms
        self.workflow_state = workflow_state
        self.config = load_config(config_path)
        self.kinematics = solver()
        robot_config = arx5.RobotConfigFactory.get_instance().get_config("X5")
        self.joint_min = np.asarray(robot_config.joint_pos_min, dtype=np.float64)
        self.joint_max = np.asarray(robot_config.joint_pos_max, dtype=np.float64)
        self.stop_event = threading.Event()
        self.error: BaseException | None = None
        self.last_transient_error: dict[str, str | None] = {"left": None, "right": None}
        self.sessions = {"left": SideSession(), "right": SideSession()}
        self._status_lock = threading.Lock()
        self._forward_lock = threading.Lock()
        self._side_status = {"left": "待命", "right": "待命"}
        self._world_to_canonical = np.eye(3, dtype=np.float64)
        self._forward_motion_calibrated = False
        self._return_button_down = {"left": False, "right": False}
        self.initial_joint_targets: dict[str, np.ndarray] | None = None
        self.initial_gripper_targets: dict[str, float] | None = None
        if initial_pose is not None:
            pose = np.asarray(initial_pose, dtype=np.float64)
            if pose.shape != (14,) or not np.isfinite(pose).all():
                raise ValueError("Quest single-arm return requires a finite 14D initial pose")
            self.initial_joint_targets = {
                "right": pose[0:6].copy(),
                "left": pose[7:13].copy(),
            }
            self.initial_gripper_targets = {
                "right": GRIPPER_WIDTH,
                "left": GRIPPER_WIDTH,
            }

    def _set_status(self, side: str, status: str) -> None:
        with self._status_lock:
            self._side_status[side] = status

    def status(self, side: str) -> str:
        with self._status_lock:
            return self._side_status[side]

    def forward_motion_calibrated(self) -> bool:
        with self._forward_lock:
            return self._forward_motion_calibrated

    def calibrate_forward_motion(self, displacement) -> np.ndarray:
        matrix = forward_motion_calibration_matrix(displacement)
        with self._forward_lock:
            self._world_to_canonical = matrix
            self._forward_motion_calibrated = True
        self._set_status("left", "前向移动已校准")
        self._set_status("right", "前向移动已校准")
        return matrix.copy()

    def _state(self, side: str):
        return self.arms.side_state(side)

    def _controller(self, snapshot, side: str) -> ControllerState:
        return snapshot.left if side == "left" else snapshot.right

    def _single_arm_return_requests(self, snapshot) -> dict[str, bool]:
        """Return rising edges for X/A without stealing the shutdown chord."""
        shutdown_chord = snapshot.left.primary and snapshot.right.secondary
        requests: dict[str, bool] = {}
        for side in ("left", "right"):
            controller = self._controller(snapshot, side)
            pressed = bool(controller.primary)
            requests[side] = bool(
                pressed
                and not self._return_button_down[side]
                and not shutdown_chord
            )
            self._return_button_down[side] = pressed
        return requests

    def _arm(self, side: str):
        return self.arms.left if side == "left" else self.arms.right

    def _robot(self, side: str):
        return self.arms.left_robot if side == "left" else self.arms.right_robot

    def _disengage(self, side: str, reason: str) -> None:
        session = self.sessions[side]
        self._set_status(side, f"待命（{reason}）")
        self.sessions[side] = SideSession(ik_failures=session.ik_failures)

    def _hold_for_reconnect(self, side: str, reason: str) -> None:
        """Keep motor control and the last safe command during data loss."""
        session = self.sessions[side]
        if not session.engaged:
            self._set_status(side, f"等待连接（{reason}）")
            return
        session.needs_reanchor = True
        assert session.command is not None
        # The vendor background controller already keeps its last command.
        # Re-sending here created a second writer during packet loss.
        self._set_status(side, f"目标冻结（{reason}，等待重新锚定）")

    def _release_clutch(self, side: str) -> None:
        """Grip release freezes position without changing SDK/gain mode."""
        session = self.sessions[side]
        if not session.engaged:
            self._set_status(side, "等待 Grip 授权")
            return
        session.needs_reanchor = True
        assert session.command is not None
        if session.at_initial_position:
            self._set_status(side, "标准初始位保持")
        else:
            self._set_status(side, "位置保持（Grip 已松开）")

    def _mark_reconnect_pending(self, side: str, reason: str) -> None:
        """Preserve the session when the arm object is temporarily unavailable."""
        session = self.sessions[side]
        session.needs_reanchor = session.engaged
        self._set_status(side, f"连接保持（{reason}，等待自动恢复）")

    def _reanchor(self, side: str, controller: ControllerState) -> None:
        """Resume from current robot/controller poses without a recovery jump."""
        session = self.sessions[side]
        state = self._state(side)
        assert session.command is not None
        session.command.pos()[:] = state[0]
        session.command.gripper_pos = float(state[3])
        session.controller_position = np.asarray(controller.position, dtype=np.float64)
        session.controller_rotation = Rotation.from_quat(controller.orientation_xyzw)
        session.robot_pose = np.asarray(
            self.kinematics.forward_kinematics(state[0]), dtype=np.float64
        )
        session.pid_integral = np.zeros(6, dtype=np.float64)
        session.pid_previous_error = np.zeros(6, dtype=np.float64)
        session.pid_previous_time = time.monotonic()
        session.filtered_joint_target = state[0].astype(np.float64).copy()
        session.joint_velocity_command = np.zeros(6, dtype=np.float64)
        session.last_command_step = np.zeros(6, dtype=np.float64)
        session.last_command_interval = 1.0 / FPS
        session.needs_reanchor = False
        session.at_initial_position = False
        self.arms.set_side_command(side, session.command)
        session.last_command_time = time.monotonic()
        # Reconnection is pose-only.  Quest transport is not allowed to change
        # gains or rebuild the robot controller; the existing mode remains live.
        session.reconnect_count += 1
        session.transient_faults = 0
        self.last_transient_error[side] = None
        self._set_status(side, "已自动重连（重新锚定）")

    def _service_side(self, side: str, operation: Callable[[], None]) -> None:
        """Contain a transient SDK/transport failure to one side and retry."""
        try:
            operation()
        except Exception as exc:
            session = self.sessions[side]
            session.transient_faults += 1
            session.needs_reanchor = session.engaged
            self.last_transient_error[side] = f"{type(exc).__name__}: {exc}"
            self._set_status(
                side,
                f"控制保持（瞬时故障 {session.transient_faults}，自动重试）",
            )

    def _engage(self, side: str, controller: ControllerState) -> None:
        state = self._state(side)
        command = arx5.JointState(self._robot(side).joint_dof)
        command.pos()[:] = state[0]
        command.gripper_pos = float(state[3])
        pose = np.asarray(self.kinematics.forward_kinematics(state[0]), dtype=np.float64)
        # Latch the measured pose before raising stiffness.  Collection starts
        # in reduced position hold, so every Grip engagement must explicitly
        # enter the authorized payload-capable VR gains for this side.
        self.arms.set_side_command(side, command)
        self.arms.set_side_control_mode(side)
        self.sessions[side] = SideSession(
            engaged=True,
            controller_position=np.asarray(controller.position, dtype=np.float64),
            controller_rotation=Rotation.from_quat(controller.orientation_xyzw),
            robot_pose=np.asarray(pose, dtype=np.float64),
            command=command,
            gripper_command=trigger_gripper_command(controller.trigger),
            gripper_guard=GripperGuard(),
            pid_integral=np.zeros(6, dtype=np.float64),
            pid_previous_error=np.zeros(6, dtype=np.float64),
            pid_previous_time=time.monotonic(),
            filtered_joint_target=state[0].astype(np.float64).copy(),
            joint_velocity_command=np.zeros(6, dtype=np.float64),
            last_command_time=time.monotonic(),
            last_command_step=np.zeros(6, dtype=np.float64),
            last_command_interval=1.0 / FPS,
        )
        self._set_status(side, "跟随中")

    def _start_single_arm_return(self, side: str) -> None:
        """Latch current feedback, then start a non-blocking one-arm return."""
        if self.initial_joint_targets is None:
            self._set_status(side, "单臂回位不可用（未提供共享初始位）")
            return
        state = self._state(side)
        command = arx5.JointState(self._robot(side).joint_dof)
        command.pos()[:] = state[0]
        command.gripper_pos = float(state[3])
        previous = self.sessions[side]
        self.arms.set_side_command(side, command)
        self.arms.set_side_control_mode(side)
        self.sessions[side] = SideSession(
            engaged=True,
            command=command,
            gripper_command=0,
            gripper_guard=previous.gripper_guard or GripperGuard(),
            ik_failures=previous.ik_failures,
            reconnect_count=previous.reconnect_count,
            returning_to_initial=True,
        )
        self._set_status(side, "安全回共享初始位中")

    def _step_single_arm_return(self, side: str) -> None:
        session = self.sessions[side]
        assert self.initial_joint_targets is not None
        assert self.initial_gripper_targets is not None
        assert session.command is not None
        measured = self._state(side)
        next_position, complete = advance_single_arm_return(
            session.command.pos().copy(),
            self.initial_joint_targets[side],
            measured[0],
        )
        session.command.pos()[:] = next_position
        session.command.gripper_pos += float(np.clip(
            self.initial_gripper_targets[side] - session.command.gripper_pos,
            -MAX_GRIPPER_STEP_M,
            MAX_GRIPPER_STEP_M,
        ))
        complete = complete and bool(
            float(measured[3])
            >= self.initial_gripper_targets[side]
            - SINGLE_ARM_RETURN_GRIPPER_TOLERANCE_M
        )
        self.arms.set_side_command(side, session.command)
        if complete:
            # Keep the exact standard pose commanded. The next Grip engagement
            # reanchors from feedback, so there is no delayed controller jump.
            session.command.pos()[:] = self.initial_joint_targets[side]
            session.command.gripper_pos = self.initial_gripper_targets[side]
            self.arms.set_side_command(side, session.command)
            session.returning_to_initial = False
            session.at_initial_position = True
            session.needs_reanchor = True
            self._set_status(side, "已到共享初始位（保持中）")
        else:
            self._set_status(side, "安全回共享初始位中")

    def _step_side(
        self,
        side: str,
        controller: ControllerState,
        return_requested: bool = False,
    ) -> None:
        config = self.config[side]
        session = self.sessions[side]
        if not config.enabled:
            self._disengage(side, "config disabled")
            return
        if not controller.tracking:
            # Ignore button values while OpenXR says the controller is not
            # tracked: false Grip in such a frame is not a deliberate release.
            self._hold_for_reconnect(side, "手柄追踪丢失")
            return
        if return_requested:
            if controller.grip:
                self._set_status(side, "请松开 Grip 后重新按回位键")
            elif not session.returning_to_initial:
                self._start_single_arm_return(side)
                session = self.sessions[side]
        if session.returning_to_initial:
            # A return owns this side until completion. Grip input is ignored
            # for this arm, while the other controller remains fully usable.
            self._step_single_arm_return(side)
            return
        if not controller.grip:
            self._release_clutch(side)
            return
        if not session.engaged:
            self._engage(side, controller)
            return
        if session.needs_reanchor:
            self._reanchor(side, controller)
            return
        assert session.controller_position is not None
        assert session.controller_rotation is not None
        assert session.robot_pose is not None
        assert session.command is not None
        assert session.gripper_guard is not None
        try:
            with self._forward_lock:
                world_to_canonical = self._world_to_canonical.copy()
            controller_pose = target_pose(
                session.robot_pose,
                session.controller_position,
                session.controller_rotation,
                controller,
                config,
                world_to_canonical,
            )
        except ValueError as exc:
            self._disengage(side, str(exc))
            return
        measured = self._state(side)
        measured_pose = np.asarray(
            self.kinematics.forward_kinematics(measured[0]), dtype=np.float64
        )
        error = np.empty(6, dtype=np.float64)
        error[:3] = controller_pose[:3] - measured_pose[:3]
        error[3:] = (
            Rotation.from_rotvec(controller_pose[3:6])
            * Rotation.from_rotvec(measured_pose[3:6]).inv()
        ).as_rotvec()
        now = time.monotonic()
        dt = max(1e-3, min(0.05, now - (session.pid_previous_time or now)))
        integral_before = session.pid_integral.copy()
        correction, session.pid_integral = safe_pid_correction(
            error,
            session.pid_previous_error,
            session.pid_integral,
            dt,
            config.pid_kp,
            config.pid_ki,
            config.pid_kd,
        )
        session.pid_previous_error = error
        session.pid_previous_time = now
        status, joint_target, ik_scale = solve_ik_with_backoff(
            self.kinematics, measured_pose, measured[0], correction
        )
        if status != 0:
            session.pid_integral = integral_before * PID_INTEGRAL_LEAK
            session.ik_failures += 1
            if session.ik_failures >= IK_FAILURES_BEFORE_REANCHOR:
                # The SDK can report a narrower joint range than the physical
                # arm. Do not accumulate unreachable controller displacement:
                # rebase both poses at the current boundary and continue from
                # there without a delayed jump.
                session.controller_position = np.asarray(
                    controller.position, dtype=np.float64
                )
                session.controller_rotation = Rotation.from_quat(
                    controller.orientation_xyzw
                )
                session.robot_pose = measured_pose.copy()
                session.command.pos()[:] = measured[0]
                session.command.gripper_pos = float(measured[3])
                session.pid_integral[:] = 0.0
                session.pid_previous_error[:] = 0.0
                session.filtered_joint_target = measured[0].astype(np.float64).copy()
                session.joint_velocity_command = np.zeros(6, dtype=np.float64)
                session.last_command_step = np.zeros(6, dtype=np.float64)
                session.last_command_interval = 1.0 / FPS
                session.ik_failures = 0
                self.arms.set_side_command(side, session.command)
                session.last_command_time = time.monotonic()
                self._set_status(side, "IK边界已重新锚定（无积累跳动）")
                return
            self._set_status(
                side,
                "保持安全命令（整末端IK："
                f"{self.kinematics.get_ik_status_name(status)}）",
            )
            return
        session.ik_failures = 0
        assert joint_target is not None
        joint_target = np.clip(
            np.asarray(joint_target, dtype=np.float64), self.joint_min, self.joint_max
        )
        if not np.isfinite(joint_target).all() or np.max(np.abs(joint_target - measured[0])) > MAX_JOINT_JUMP_RAD:
            session.pid_integral = integral_before * PID_INTEGRAL_LEAK
            self._set_status(side, "保持安全命令（关节跳变保护）")
            return
        assert session.filtered_joint_target is not None
        assert session.joint_velocity_command is not None
        assert session.last_command_time is not None
        assert session.last_command_step is not None
        session.filtered_joint_target = low_pass_ik_target(
            session.filtered_joint_target,
            joint_target,
            dt,
        )
        gripper_target = session.command.gripper_pos
        if config.gripper_enabled:
            if controller.trigger >= 0.65:
                session.gripper_command = 1
            elif controller.trigger <= 0.35:
                session.gripper_command = 0
            gripper_target = session.gripper_guard.target(
                session.gripper_command, float(measured[3]), float(measured[5])
            )
        command_now = time.monotonic()
        command_dt = max(
            1e-3, min(0.05, command_now - session.last_command_time)
        )
        previous_output_velocity = (
            session.last_command_step / session.last_command_interval
        )
        acceleration_dt = 0.5 * (
            session.last_command_interval + command_dt
        )
        joint_step, session.joint_velocity_command = acceleration_limited_joint_step(
            session.filtered_joint_target - session.command.pos().copy(),
            previous_output_velocity,
            command_dt,
            acceleration_dt,
        )
        session.command.pos()[:] += joint_step
        session.command.gripper_pos += float(np.clip(
            gripper_target - session.command.gripper_pos,
            -MAX_GRIPPER_STEP_M,
            MAX_GRIPPER_STEP_M,
        ))
        self.arms.set_side_command(side, session.command)
        sent_time = time.monotonic()
        session.last_command_step = joint_step.copy()
        session.last_command_interval = max(
            1e-3, min(0.05, sent_time - session.last_command_time)
        )
        session.last_command_time = sent_time
        self._set_status(
            side,
            "跟随中" if ik_scale == 1.0 else f"跟随中（IK边界退让 {ik_scale:.3g}x）",
        )

    def run(self) -> None:
        next_tick = time.monotonic()
        try:
            while not self.stop_event.is_set():
                snapshot = self.receiver.snapshot()
                authority = self.workflow_state()
                recording = authority if isinstance(authority, bool) else authority == "采集中"
                if recording and snapshot.fresh() and self.arms.connected:
                    # Treat one Quest frame as one command transaction. Safety
                    # and workflow motion use the same gate, so their writes
                    # cannot interleave between the left and right updates.
                    with self.arms.command_gate:
                        return_requests = self._single_arm_return_requests(snapshot)
                        self._service_side(
                            "left",
                            lambda: self._step_side(
                                "left", snapshot.left, return_requests["left"]
                            ),
                        )
                        self._service_side(
                            "right",
                            lambda: self._step_side(
                                "right", snapshot.right, return_requests["right"]
                            ),
                        )
                elif recording and self.arms.connected:
                    # Require a release after transport recovery; a button held
                    # in the last stale packet must not start unexpected motion.
                    self._return_button_down = {"left": True, "right": True}
                    self._service_side(
                        "left", lambda: self._hold_for_reconnect("left", "Quest 数据中断")
                    )
                    self._service_side(
                        "right", lambda: self._hold_for_reconnect("right", "Quest 数据中断")
                    )
                elif recording:
                    # Do not call teach-mode APIs through a temporarily absent
                    # arm.  Preserve both sessions so an external SDK reconnect
                    # can resume by re-anchoring without restarting this thread.
                    self._return_button_down = {"left": True, "right": True}
                    self._mark_reconnect_pending("left", "机械臂接口暂不可用")
                    self._mark_reconnect_pending("right", "机械臂接口暂不可用")
                else:
                    self._return_button_down = {"left": False, "right": False}
                    reason = "not recording"
                    self._disengage("left", reason)
                    self._disengage("right", reason)
                next_tick += 1.0 / FPS
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.monotonic()
        except BaseException as exc:
            self.error = exc
            # Never switch to teach mode merely because the software loop hit
            # an unexpected fault.  The last safe command remains latched; the
            # workflow owner performs the normal safe-return shutdown path.
            self._set_status("left", "控制线程故障（保持最后安全命令）")
            self._set_status("right", "控制线程故障（保持最后安全命令）")

    def close(self) -> None:
        self.stop_event.set()
        self.join(timeout=2.0)
        self._disengage("left", "shutdown")
        self._disengage("right", "shutdown")
