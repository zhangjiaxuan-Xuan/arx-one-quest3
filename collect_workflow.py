#!/usr/bin/env python3
"""Interactive three-camera + bimanual collection workflow."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
import select
import shutil
import signal
import socket
import struct
import subprocess
import sys
import termios
import threading
import time
import tty
import unicodedata
from io import BytesIO

import numpy as np
from PIL import Image
import arx5_interface as arx5

from arx_common import (
    ACTION_DIM,
    FPS,
    NAMES,
    GRIPPER_WIDTH,
    GRIPPER_CALIBRATION_PATH,
    copy_state,
    make_arm,
    pack_bimanual,
    resolve_registered_hardware,
)
from replay_pi05 import command_pair, set_reduced_gain
from quest3_input import (
    DEFAULT_PORT,
    QUEST_SLEEP_SAFE_EXIT_SECONDS,
    Quest3Receiver,
    QuestWorkflowInput,
)
from quest3_teleop import QuestTeleopController
from control_authority import SafetyAuthority
from can_rx_watchdog import CanRxWatchdog


ROOT = Path(__file__).resolve().parent
SESSIONS = ROOT / "sessions"
SHARED_POSES = ROOT / "shared_poses"
COLLECTION_INITIAL_POSE_PATH = SHARED_POSES / "collection_initial_pose.npy"
GLOBAL_SHUTDOWN_POSE_PATH = SHARED_POSES / "shutdown_pose.npy"
GLOBAL_SHUTDOWN_BOOT_ID_PATH = SHARED_POSES / "shutdown_pose_boot_id.txt"
LEGACY_INITIAL_POSE_PATH = ROOT / "sessions" / "poses" / "latest_initial_pose.npy"
POSES = SESSIONS / "poses"
SHUTDOWN_POSE_PATH = POSES / "shutdown_pose.npy"
SHUTDOWN_BOOT_ID_PATH = POSES / "shutdown_pose_boot_id.txt"
_shutdown_signal_seen = False
_shutdown_cleanup_active = False
_power_off_confirmation_active = False
_hardware_control_acquired = False


def request_safe_shutdown(signum, frame):
    """First signal requests shutdown; later signals cannot interrupt return."""
    global _shutdown_signal_seen
    if _power_off_confirmation_active:
        # Feedback is already lost and trajectory output has stopped.  In the
        # explicit physical-power-off prompt Ctrl-C means "release now", not
        # "try another blind return".
        raise KeyboardInterrupt(f"power-off confirmation signal {signum}")
    if not _hardware_control_acquired:
        if not _shutdown_signal_seen:
            _shutdown_signal_seen = True
            print(
                "尚未取得机械臂控制对象；正在中断初始化。若厂商构造函数卡死，"
                "再次 Ctrl-C 将立即结束进程。",
                flush=True,
            )
            raise KeyboardInterrupt(f"initialization signal {signum}")
        print("厂商SDK初始化未返回且没有可清理控制对象；立即结束进程。", flush=True)
        os._exit(130)
    if _shutdown_cleanup_active:
        return
    if not _shutdown_signal_seen:
        _shutdown_signal_seen = True
        raise KeyboardInterrupt(f"received signal {signum}")


def write_lifecycle_status(path: Path | None, status: str) -> None:
    if path is None:
        return
    if status not in {
        "SAFE_SHUTDOWN_COMPLETE",
        "NO_CONTROL_ACQUIRED",
        "OPERATOR_POWER_OFF_CONFIRMED",
    }:
        raise ValueError(f"invalid lifecycle status: {status}")
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(status + "\n", encoding="ascii")
    temporary.replace(path)


def write_shutdown_complete(path: Path | None) -> None:
    write_lifecycle_status(path, "SAFE_SHUTDOWN_COMPLETE")


def write_no_control_acquired(path: Path | None) -> None:
    """Report that SDK construction failed before this process controlled an arm."""
    write_lifecycle_status(path, "NO_CONTROL_ACQUIRED")


class RobotFeedbackLost(RuntimeError):
    """The SDK object exists, but safe closed-loop motion is no longer provable."""


class WorkflowInputRejected(RuntimeError):
    """A correctable operator action that must not trigger safe shutdown."""


SYSTEM_BOOT_ID_PATH = Path("/proc/sys/kernel/random/boot_id")
WARMUP_SECONDS = 2.0
CAMERA_INITIALIZATION_SECONDS = 2.0
CAMERA_GAP_FACTOR = 1.5
ROBOT_MAX_SAMPLE_INTERVAL_SECONDS = 0.040
QUEST_MAX_PACKET_LOSS_SECONDS = 3.0
CAN_TRANSIENT_FEEDBACK_TIMEOUT_SECONDS = 1.00
CAN_FATAL_FEEDBACK_TIMEOUT_SECONDS = 5.0
SAFE_RETURN_COMPLETION_RAD = 0.050
SAFE_RETURN_NUMERICAL_MARGIN_RAD = 1e-6
INITIAL_GRIPPER_OPEN_TOLERANCE_M = 0.002
START_POSE_LIMIT_RAD = 0.25
# Payload-capable stiffness authorized for VR Grip-follow only. Safe return,
# replay/model deployment and teach mode keep their separately validated
# settings. The gripper scales below apply only while its VR side is engaged.
VR_JOINT_KP_SCALE = 0.75
VR_JOINT_KD_SCALE = 1.00
VR_GRIPPER_KP_SCALE = 0.85
VR_GRIPPER_KD_SCALE = 0.60
WORKFLOW_STATE_PATH = SESSIONS / "workflow_state.json"
REVIEW_BATCH_TARGET = 10
PREVIEW_WIDTH = 640
PREVIEW_HEIGHT = 360
PREVIEW_FRAME_BYTES = PREVIEW_WIDTH * PREVIEW_HEIGHT * 3
DASHBOARD_MAX_WIDTH = 108
DASHBOARD_MIN_WIDTH = 52


def display_width(value: object) -> int:
    """Terminal-cell width, including CJK full-width characters."""
    total = 0
    for char in str(value):
        if unicodedata.combining(char):
            continue
        total += 2 if unicodedata.east_asian_width(char) in ("W", "F") else 1
    return total


def clean_terminal_text(value: object) -> str:
    return " ".join(
        "".join(char for char in str(value) if ord(char) >= 32 and char != "\x7f").split()
    )


def fit_terminal_text(value: object, width: int, *, align: str = "left") -> str:
    text = clean_terminal_text(value)
    if display_width(text) > width:
        kept = []
        used = 0
        for char in text:
            char_width = display_width(char)
            if used + char_width > max(0, width - 1):
                break
            kept.append(char)
            used += char_width
        text = "".join(kept) + "…"
    padding = max(0, width - display_width(text))
    if align == "center":
        left = padding // 2
        return " " * left + text + " " * (padding - left)
    if align == "right":
        return " " * padding + text
    return text + " " * padding


def dashboard_width() -> int:
    columns = shutil.get_terminal_size(fallback=(100, 30)).columns
    return max(DASHBOARD_MIN_WIDTH, min(DASHBOARD_MAX_WIDTH, max(1, columns - 1)))


def dashboard_row(value: object, width: int, *, align: str = "left") -> str:
    return "┃" + fit_terminal_text(value, width - 2, align=align) + "┃"


def dashboard_pair(left: object, right: object, width: int) -> str:
    inner = width - 2
    separator = " │ "
    left_width = (inner - display_width(separator)) // 2
    right_width = inner - display_width(separator) - left_width
    return (
        "┃"
        + fit_terminal_text(left, left_width)
        + separator
        + fit_terminal_text(right, right_width)
        + "┃"
    )


class RecordingPreview:
    """Compose FFmpeg preview branches without opening cameras a second time."""

    def __init__(self, port: int = 10505):
        self.port = port
        self.stop_event = threading.Event()
        self.frames: dict[str, np.ndarray] = {}
        self.lock = threading.Lock()
        self.readers: list[threading.Thread] = []
        self.server_thread: threading.Thread | None = None

    def add(self, role: str, process: subprocess.Popen) -> None:
        if process.stdout is None:
            raise RuntimeError(f"{role} preview pipe unavailable")

        def read_frames() -> None:
            while not self.stop_event.is_set():
                payload = process.stdout.read(PREVIEW_FRAME_BYTES)
                if len(payload) != PREVIEW_FRAME_BYTES:
                    return
                frame = np.frombuffer(payload, dtype=np.uint8).reshape(
                    PREVIEW_HEIGHT, PREVIEW_WIDTH, 3
                ).copy()
                with self.lock:
                    self.frames[role] = frame

        thread = threading.Thread(target=read_frames, daemon=True, name=f"preview-{role}")
        thread.start()
        self.readers.append(thread)

    def start(self) -> None:
        def serve() -> None:
            order = ("left_arm_camera", "third_person_camera", "right_arm_camera")
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.settimeout(0.2)
            server.bind(("0.0.0.0", self.port))
            server.listen(1)
            try:
                while not self.stop_event.is_set():
                    try:
                        client, _ = server.accept()
                    except socket.timeout:
                        continue
                    client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    try:
                        next_frame = time.monotonic()
                        while not self.stop_event.is_set():
                            with self.lock:
                                frames = [self.frames.get(role) for role in order]
                            if any(frame is None for frame in frames):
                                time.sleep(0.01)
                                continue
                            panorama = np.hstack(frames)
                            # FFmpeg provides BGR frames; Pillow is already a
                            # dependency of the lerobot environment and avoids
                            # requiring cv2 in the launcher environment.
                            encoded = BytesIO()
                            Image.fromarray(panorama[:, :, ::-1]).save(
                                encoded, format="JPEG", quality=80
                            )
                            payload = encoded.getvalue()
                            client.sendall(struct.pack("!I", len(payload)) + payload)
                            next_frame += 1.0 / 20.0
                            time.sleep(max(0.0, next_frame - time.monotonic()))
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        pass
                    finally:
                        client.close()
            finally:
                server.close()

        self.server_thread = threading.Thread(target=serve, daemon=True, name="record-preview")
        self.server_thread.start()

    def close(self) -> None:
        self.stop_event.set()
        if self.server_thread is not None:
            self.server_thread.join(timeout=2.0)


def configure_sessions_root(path: Path) -> None:
    """Select one isolated collection domain before Workflow is constructed."""
    global SESSIONS, POSES, SHUTDOWN_POSE_PATH, SHUTDOWN_BOOT_ID_PATH
    global WORKFLOW_STATE_PATH
    sessions = path.expanduser().resolve()
    SESSIONS = sessions
    POSES = sessions / "poses"
    SHUTDOWN_POSE_PATH = POSES / "shutdown_pose.npy"
    SHUTDOWN_BOOT_ID_PATH = POSES / "shutdown_pose_boot_id.txt"
    WORKFLOW_STATE_PATH = sessions / "workflow_state.json"


def ensure_shared_collection_pose() -> None:
    if COLLECTION_INITIAL_POSE_PATH.exists():
        return
    if not LEGACY_INITIAL_POSE_PATH.exists():
        raise RuntimeError(
            "缺少共享采集起始姿态；请先用维护流程注册一次统一起始位置"
        )
    pose = np.load(LEGACY_INITIAL_POSE_PATH).astype(np.float32)
    if pose.shape != (ACTION_DIM,) or not np.isfinite(pose).all():
        raise RuntimeError("旧采集起始姿态无效，无法初始化共享起始姿态")
    pose[[6, 13]] = GRIPPER_WIDTH
    SHARED_POSES.mkdir(parents=True, exist_ok=True)
    np.save(COLLECTION_INITIAL_POSE_PATH, pose)
    print(f"已从旧采集配置初始化共享起始姿态：{COLLECTION_INITIAL_POSE_PATH}")


def episode_task(episode: Path) -> str:
    """Return an episode's task text without trusting optional pickle data."""
    raw_demo = episode / "raw_demo.npz"
    if not raw_demo.is_file():
        return ""
    try:
        with np.load(raw_demo, allow_pickle=False) as data:
            if "task" not in data.files:
                return ""
            return str(data["task"].item()).strip()
    except (OSError, ValueError, KeyError):
        return ""


def episode_numbers(episode: Path) -> tuple[int, int]:
    """Parse round/attempt numbers while tolerating unrelated session folders."""
    parts = episode.name.split("_")
    try:
        return int(parts[1]), int(parts[3])
    except (IndexError, ValueError):
        return 0, 0


def run_checked(command: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, check=True, text=True, capture_output=True)


def configure_camera(device: str) -> None:
    run_checked(
        [
            "v4l2-ctl",
            "-d",
            device,
            "--set-ctrl=auto_exposure=3",
            "--set-ctrl=exposure_dynamic_framerate=0",
        ]
    )


class PersistentArms:
    """One persistent vendor-SDK session with serialized application writes.

    The vendor controller already owns the continuous CAN send/receive loop.
    Application code must therefore write only when a real motion producer has
    a new command; idle/reconnect monitoring is strictly read-only.
    """

    def __init__(self, expected_shutdown_pose: np.ndarray | None = None):
        self.sdk_lock = threading.RLock()
        # Every gain/command mutation passes through this gate.  State readers
        # use sdk_lock only and remain concurrent with recording.
        self.command_gate = threading.RLock()
        self.left = None
        self.right = None
        self.left_robot = None
        self.right_robot = None
        self.left_ctrl = None
        self.right_ctrl = None
        self.can_watchdog = CanRxWatchdog()
        self.connect(expected_shutdown_pose=expected_shutdown_pose)

    @property
    def connected(self) -> bool:
        return self.left is not None and self.right is not None

    def connect(self, expected_shutdown_pose: np.ndarray | None = None) -> None:
        global _hardware_control_acquired
        if self.connected:
            # This persistent session already passed preflight when it was
            # created.  Rechecking against the shutdown pose here would reject
            # legitimate calls made from the collection start pose.
            return
        # X5 forcibly requires shutdown_to_passive=True. Keep both sessions
        # alive for the whole workflow and only destroy them after the verified
        # shutdown pose. The AC One has now been verified twice to require the
        # historical/official bimanual order: initialize can0/left first, then
        # can1/right. Right-first and right-only sessions lose can1 feedback
        # shortly after startup even though TX continues.
        self.left, self.left_robot, self.left_ctrl = make_arm(
            "can0", gravity_compensation=True, shutdown_to_passive=True
        )
        _hardware_control_acquired = True
        try:
            self.right, self.right_robot, self.right_ctrl = make_arm(
                "can1", gravity_compensation=True, shutdown_to_passive=True
            )
            self.preflight_at_shutdown(expected_shutdown_pose)
            self.can_watchdog.reset()
        except BaseException:
            self.close()
            raise

    def preflight_at_shutdown(
        self, expected_shutdown_pose: np.ndarray | None = None
    ) -> dict[str, float]:
        """Read back both arms in the constructor-configured damping mode.

        ``make_arm`` constructs each controller with gravity compensation and
        shutdown-to-passive enabled. Do not immediately repeat set_to_damping:
        on this hardware that redundant startup transition can race the vendor
        background receiver. Keep this preflight strictly read-only.
        """
        if not self.connected:
            raise RuntimeError("双臂SDK尚未完整连接")
        if not bool(self.left_ctrl.gravity_compensation) or not bool(
            self.right_ctrl.gravity_compensation
        ):
            raise RuntimeError("双臂控制器未启用重力补偿")
        samples = []
        self.can_watchdog.reset()
        start_counts = self.can_watchdog.snapshot()
        for _ in range(15):
            state, velocity, effort = self.state()
            if not all(np.isfinite(value).all() for value in (state, velocity, effort)):
                raise RuntimeError("机械臂状态包含非有限值")
            samples.append((state, velocity, effort))
            # Refresh packet counters without treating passive telemetry idle
            # as a fault. Active continuous-feedback requirements begin only
            # after the zero-displacement position-hold handshake.
            self.can_watchdog.fault(10.0)
            time.sleep(1.0 / FPS)
        end_counts = self.can_watchdog.snapshot()
        startup_rx = {
            side: end_counts[side] - start_counts[side]
            for side in ("left", "right")
        }
        verified, velocity, effort = samples[-1]
        if not np.isfinite(verified).all():
            raise RuntimeError("重力补偿只读预检后状态无效")
        shutdown_error = 0.0
        if expected_shutdown_pose is not None:
            expected = np.asarray(expected_shutdown_pose, dtype=np.float32)
            if expected.shape != (ACTION_DIM,) or not np.isfinite(expected).all():
                raise RuntimeError("停机姿态文件无效")
            # Joint positions are the safety-critical comparison; gripper width
            # may legitimately differ after a power cycle.
            joint_indices = np.asarray(list(range(6)) + list(range(7, 13)))
            shutdown_error = float(
                np.max(np.abs(verified[joint_indices] - expected[joint_indices]))
            )
            if shutdown_error > 0.35:
                raise RuntimeError(
                    f"当前不在停机位置：关节最大误差 {shutdown_error:.3f} rad > 0.350 rad"
                )
        report = {
            "shutdown_error_rad": shutdown_error,
            "max_velocity": float(np.max(np.abs(velocity))),
            "max_effort": float(np.max(np.abs(effort))),
        }
        print(
            "停机位重力补偿预检通过：双臂SDK状态读取有效，"
            f"构造后观察窗口left_rx=+{startup_rx['left']} "
            f"right_rx=+{startup_rx['right']}，"
            f"停机位误差={shutdown_error:.4f} rad；被动静止后允许遥测idle",
            flush=True,
        )
        return report

    def verify_active_feedback(self, context: str, duration_seconds: float = 0.5) -> None:
        """Prove feedback after a zero-displacement position hold is armed."""
        self.can_watchdog.reset()
        start_counts = self.can_watchdog.snapshot()
        deadline = time.monotonic() + duration_seconds
        while time.monotonic() < deadline:
            # Use a long timeout only to refresh kernel counters. The bounded
            # packet-delta test below is the actual active-mode criterion.
            self.can_watchdog.fault(duration_seconds + 1.0)
            self.state()
            time.sleep(1.0 / FPS)
        end_counts = self.can_watchdog.snapshot()
        advances = {
            side: end_counts[side] - start_counts[side]
            for side in ("left", "right")
        }
        minimum_packets = max(10, int(duration_seconds * 20))
        failed = [
            side for side, advance in advances.items() if advance < minimum_packets
        ]
        if failed:
            raise RobotFeedbackLost(
                f"{context}当前位置PID唤醒反馈失败："
                f"left_rx=+{advances['left']} right_rx=+{advances['right']}；"
                f"无反馈侧={failed}"
            )
        self.can_watchdog.reset()
        print(
            f"{context}当前位置PID唤醒通过："
            f"left_rx=+{advances['left']} right_rx=+{advances['right']}",
            flush=True,
        )

    def verify_position_response(self, attempts: int = 3) -> None:
        """Warm a cold SLCAN/SDK session and retry actuation without rebuilding it."""
        errors: list[str] = []
        for attempt in range(1, attempts + 1):
            self.set_teach_mode()
            state, _, _ = self.state()
            right_cmd = arx5.JointState(self.right_robot.joint_dof)
            left_cmd = arx5.JointState(self.left_robot.joint_dof)
            right_cmd.pos()[:] = state[:6]
            right_cmd.gripper_pos = float(state[6])
            left_cmd.pos()[:] = state[7:13]
            left_cmd.gripper_pos = float(state[13])
            # This is an explicit diagnostic, not a production heartbeat.
            # Keep the same SDK session and hold the measured pose before each
            # reversible response attempt.
            for _ in range(FPS):
                self.set_side_command("right", right_cmd)
                self.set_side_command("left", left_cmd)
                time.sleep(1.0 / FPS)
            try:
                self._verify_position_response_once()
                if attempt > 1:
                    print(
                        f"双臂位置控制在同一SDK会话第{attempt}次尝试恢复。",
                        flush=True,
                    )
                return
            except RuntimeError as exc:
                errors.append(str(exc))
                if attempt < attempts:
                    print(
                        f"位置闭环预检第{attempt}/{attempts}次未响应；"
                        "保持SDK与重力补偿在线并重新进入阻尼后重试。",
                        flush=True,
                    )
        raise RuntimeError(
            f"位置闭环响应在同一SDK会话连续{attempts}次失败：{errors[-1]}"
        )

    def _verify_position_response_once(self) -> None:
        """Prove both command paths with a tiny reversible wrist movement."""
        baseline, _, _ = self.state()
        target = baseline.copy()
        # Joint 5 is a low-load wrist joint. Move toward the center of each
        # configured range so the test cannot push farther into a hard limit.
        test_indices = {"right": 4, "left": 11}
        for side, packed_index in test_indices.items():
            robot = self.right_robot if side == "right" else self.left_robot
            local_index = 4
            lower = float(robot.joint_pos_min[local_index])
            upper = float(robot.joint_pos_max[local_index])
            direction = 1.0 if baseline[packed_index] < (lower + upper) * 0.5 else -1.0
            target[packed_index] = np.clip(
                baseline[packed_index] + direction * 0.030, lower + 0.03, upper - 0.03
            )
        right_cmd = arx5.JointState(self.right_robot.joint_dof)
        left_cmd = arx5.JointState(self.left_robot.joint_dof)
        right_cmd.pos()[:] = baseline[:6]
        right_cmd.gripper_pos = float(baseline[6])
        left_cmd.pos()[:] = baseline[7:13]
        left_cmd.gripper_pos = float(baseline[13])
        try:
            with self.command_gate, self.sdk_lock:
                # The controller starts in damping.  First latch the measured
                # pose, then use a firm (but still reduced) joint gain for this
                # low-load wrist test.  The normal 25% replay gain is too weak
                # to reliably overcome wrist stiction over only 0.02 rad and
                # produced false "motor offline" reports.
                self.right.set_joint_cmd(right_cmd)
                self.left.set_joint_cmd(left_cmd)
                for arm, robot, controller in (
                    (self.right, self.right_robot, self.right_ctrl),
                    (self.left, self.left_robot, self.left_ctrl),
                ):
                    gain = arx5.Gain(robot.joint_dof)
                    gain.kp()[:] = np.asarray(controller.default_kp) * 0.75
                    gain.kd()[:] = np.asarray(controller.default_kd)
                    gain.gripper_kp = 0.0
                    gain.gripper_kd = float(controller.default_gripper_kd)
                    arm.set_gain(gain)
            time.sleep(0.10)
            for _ in range(40):
                with self.command_gate, self.sdk_lock:
                    command_pair(
                        self.left, self.right, target, left_cmd, right_cmd, 0.002, 0.0002
                    )
                time.sleep(1.0 / FPS)
            moved, _, _ = self.state()
            movement = {
                side: float(abs(moved[index] - baseline[index]))
                for side, index in test_indices.items()
            }
            with self.command_gate, self.sdk_lock:
                right_command_state = self.right.get_joint_cmd()
                left_command_state = self.left.get_joint_cmd()
                command_progress = {
                    "right": float(
                        abs(right_command_state.pos()[4] - baseline[4])
                    ),
                    "left": float(
                        abs(left_command_state.pos()[4] - baseline[11])
                    ),
                }
            failed = [side for side, distance in movement.items() if distance < 0.008]
            for _ in range(50):
                with self.command_gate, self.sdk_lock:
                    command_pair(
                        self.left, self.right, baseline, left_cmd, right_cmd, 0.002, 0.0002
                    )
                time.sleep(1.0 / FPS)
            restored, _, _ = self.state()
            indices = np.asarray([*range(6), *range(7, 13)])
            restore_error = float(np.max(np.abs(restored[indices] - baseline[indices])))
            if restore_error > 0.025:
                raise RuntimeError(
                    f"位置闭环响应测试未回到原位：误差={restore_error:.4f}rad"
                )
            if failed:
                raise RuntimeError(
                    "位置闭环响应失败："
                    + "，".join(
                        f"{side}=实测{movement[side]:.4f}/命令{command_progress[side]:.4f}rad"
                        for side in movement
                    )
                    + f"；无响应侧={failed}"
                )
            print(
                "双臂位置闭环微动预检通过："
                f"right={movement['right']:.4f}rad，left={movement['left']:.4f}rad，"
                f"回位误差={restore_error:.4f}rad",
                flush=True,
            )
        finally:
            self.set_teach_mode()

    def state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if not self.connected:
            raise RuntimeError("机械臂控制器当前处于休息断开状态")
        with self.sdk_lock:
            return pack_bimanual(copy_state(self.right), copy_state(self.left))

    def side_state(self, side: str):
        if not self.connected:
            raise RuntimeError("机械臂控制器未连接")
        with self.sdk_lock:
            return copy_state(self.left if side == "left" else self.right)

    def set_side_command(self, side: str, command: arx5.JointState) -> None:
        if not self.connected:
            raise RuntimeError("机械臂控制器未连接")
        with self.command_gate, self.sdk_lock:
            (self.left if side == "left" else self.right).set_joint_cmd(command)

    def set_bimanual_commands(
        self,
        left_command: arx5.JointState,
        right_command: arx5.JointState,
    ) -> None:
        """Commit one paired motion command in right-then-left order."""
        if not self.connected:
            raise RuntimeError("机械臂控制器未连接")
        with self.command_gate, self.sdk_lock:
            self.right.set_joint_cmd(right_command)
            self.left.set_joint_cmd(left_command)

    def health_fault(self, timeout_seconds: float = 1.00) -> str | None:
        if not self.connected:
            return "双臂SDK连接对象不完整"
        return self.can_watchdog.fault(timeout_seconds)

    def wait_for_feedback_recovery(
        self,
        context: str,
        *,
        timeout_seconds: float = CAN_FATAL_FEEDBACK_TIMEOUT_SECONDS,
    ) -> bool:
        """Passively wait for kernel CAN feedback; never write recovery commands."""
        fault = self.health_fault(CAN_TRANSIENT_FEEDBACK_TIMEOUT_SECONDS)
        if fault is None:
            return False
        started = time.monotonic()
        print(
            f"{context}检测到CAN反馈停顿；停止生成新目标，由官方SDK保持当前模式并被动等待：{fault}",
            flush=True,
        )
        while True:
            time.sleep(0.05)
            fault = self.health_fault(CAN_TRANSIENT_FEEDBACK_TIMEOUT_SECONDS)
            if fault is None:
                elapsed = time.monotonic() - started
                print(
                    f"{context}CAN反馈已在{elapsed:.2f}s内恢复；沿用同一SDK会话。",
                    flush=True,
                )
                return True
            elapsed = time.monotonic() - started
            if elapsed >= timeout_seconds:
                raise RobotFeedbackLost(
                    f"{context}CAN反馈连续{elapsed:.2f}s未恢复，已停止发送轨迹：{fault}"
                )

    def set_teach_mode(self) -> None:
        if not self.connected:
            return
        with self.command_gate, self.sdk_lock:
            for arm in (self.left, self.right):
                # Keep the exact vendor damping behavior that passed the clean
                # baseline. Do not immediately overwrite it with custom gains.
                arm.set_to_damping()

    def set_side_teach_mode(self, side: str) -> None:
        arm, robot, controller = (
            (self.left, self.left_robot, self.left_ctrl)
            if side == "left"
            else (self.right, self.right_robot, self.right_ctrl)
        )
        if arm is None:
            return
        with self.command_gate, self.sdk_lock:
            arm.set_to_damping()

    def set_side_control_mode(self, side: str) -> None:
        arm, robot, controller = (
            (self.left, self.left_robot, self.left_ctrl)
            if side == "left"
            else (self.right, self.right_robot, self.right_ctrl)
        )
        if arm is not None:
            gain = arx5.Gain(robot.joint_dof)
            gain.kp()[:] = np.asarray(controller.default_kp) * VR_JOINT_KP_SCALE
            gain.kd()[:] = np.asarray(controller.default_kd) * VR_JOINT_KD_SCALE
            gain.gripper_kp = (
                float(controller.default_gripper_kp) * VR_GRIPPER_KP_SCALE
            )
            gain.gripper_kd = (
                float(controller.default_gripper_kd) * VR_GRIPPER_KD_SCALE
            )
            with self.command_gate, self.sdk_lock:
                arm.set_gain(gain)

    def enter_position_hold(self) -> None:
        """Serialized transition into closed-loop position hold."""
        if not self.connected:
            raise RuntimeError("双臂SDK未连接，不能进入位置保持")
        state, _, _ = self.state()
        right_cmd = arx5.JointState(self.right_robot.joint_dof)
        left_cmd = arx5.JointState(self.left_robot.joint_dof)
        right_cmd.pos()[:] = state[:6]
        right_cmd.gripper_pos = float(state[6])
        left_cmd.pos()[:] = state[7:13]
        left_cmd.gripper_pos = float(state[13])
        with self.command_gate, self.sdk_lock:
            self.right.set_joint_cmd(right_cmd)
            self.left.set_joint_cmd(left_cmd)
            set_reduced_gain(self.right, self.right_ctrl, self.right_robot.joint_dof)
            set_reduced_gain(self.left, self.left_ctrl, self.left_robot.joint_dof)
        self.verify_active_feedback("进入位置保持")

    def close(self) -> None:
        global _hardware_control_acquired
        with self.command_gate, self.sdk_lock:
            if self.right is not None:
                right, self.right = self.right, None
                del right
            if self.left is not None:
                left, self.left = self.left, None
                del left
            self.left_robot = None
            self.right_robot = None
            self.left_ctrl = None
            self.right_ctrl = None
            _hardware_control_acquired = False


class RobotRecorder(threading.Thread):
    def __init__(
        self,
        arms: PersistentArms,
        output: Path,
        initial_pose: np.ndarray,
        task: str,
        quest_receiver: Quest3Receiver | None = None,
    ):
        super().__init__(daemon=True)
        self.arms = arms
        self.output = output
        self.initial_pose = initial_pose
        self.task = task
        self.quest_receiver = quest_receiver
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.record_event = threading.Event()
        self.started_at = 0.0
        self.started_unix = 0.0
        self.error: BaseException | None = None

    def run(self) -> None:
        timestamps: list[float] = []
        timestamps_unix: list[float] = []
        states: list[np.ndarray] = []
        velocities: list[np.ndarray] = []
        efforts: list[np.ndarray] = []
        quest_sequences: list[int] = []
        quest_ages: list[float] = []
        quest_controller_states: list[np.ndarray] = []
        try:
            current, _, _ = self.arms.state()
            delta = float(np.max(np.abs(current - self.initial_pose)))
            if delta > START_POSE_LIMIT_RAD:
                raise RuntimeError(
                    f"current pose differs from registered pose by {delta:.3f} rad "
                    f"(limit {START_POSE_LIMIT_RAD:.3f})"
                )
            self.ready.set()
            while not self.record_event.wait(timeout=0.1):
                if self.stop_event.is_set():
                    return
            period = 1.0 / FPS
            self.started_at = time.monotonic()
            self.started_unix = time.time_ns() / 1e9
            next_tick = self.started_at
            while not self.stop_event.is_set():
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(next_tick - now)
                sample_time = time.monotonic()
                sample_time_unix = time.time_ns() / 1e9
                state, velocity, effort = self.arms.state()
                timestamps.append(sample_time - self.started_at)
                timestamps_unix.append(sample_time_unix)
                states.append(state)
                velocities.append(velocity)
                efforts.append(effort)
                if self.quest_receiver is not None:
                    snapshot = self.quest_receiver.snapshot()
                    quest_sequences.append(snapshot.sequence)
                    quest_ages.append(max(0.0, sample_time - snapshot.received_monotonic))
                    values = []
                    for controller in (snapshot.right, snapshot.left):
                        values.extend(controller.position)
                        values.extend(controller.orientation_xyzw)
                        values.extend(
                            [
                                float(controller.grip),
                                controller.trigger,
                                float(controller.primary),
                                float(controller.secondary),
                                float(controller.menu),
                                float(controller.tracking),
                            ]
                        )
                    quest_controller_states.append(np.asarray(values, dtype=np.float32))
                next_tick += period
            if len(states) < 2:
                raise RuntimeError("fewer than two robot samples captured")
            timestamp = np.asarray(timestamps, dtype=np.float64)
            observation_state = np.asarray(states, dtype=np.float32)
            action = np.concatenate([observation_state[1:], observation_state[-1:]], axis=0)
            payload = dict(
                timestamp=timestamp,
                timestamp_unix=np.asarray(timestamps_unix, dtype=np.float64),
                observation_state=observation_state,
                observation_velocity=np.asarray(velocities, dtype=np.float32),
                observation_effort=np.asarray(efforts, dtype=np.float32),
                action=action,
                fps=np.int32(FPS),
                task=np.asarray(self.task),
                names=np.asarray(NAMES),
            )
            if self.quest_receiver is not None:
                payload.update(
                    teleop_source=np.asarray("meta_quest_3_touch"),
                    quest_schema=np.asarray("arx.quest3.controllers.v1"),
                    quest_sequence=np.asarray(quest_sequences, dtype=np.int64),
                    quest_packet_age_seconds=np.asarray(quest_ages, dtype=np.float32),
                    quest_controller_state=np.asarray(quest_controller_states, dtype=np.float32),
                    quest_controller_names=np.asarray(
                        [
                            f"{side}_{name}"
                            for side in ("right", "left")
                            for name in (
                                "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw",
                                "grip", "trigger", "primary", "secondary", "menu", "tracking",
                            )
                        ]
                    ),
                )
            np.savez_compressed(self.output, **payload)
        except BaseException as exc:
            self.error = exc
            self.ready.set()


def start_camera(role: str, config: dict, output: Path, log_file) -> subprocess.Popen:
    return subprocess.Popen(
        [
            "ffmpeg",
            "-nostdin",
            "-hide_banner",
            "-loglevel",
            "warning",
            "-thread_queue_size",
            str(config.get("buffer_frames", 1)),
            "-f",
            "v4l2",
            "-timestamps",
            "mono2abs",
            "-input_format",
            "mjpeg",
            "-video_size",
            f"{config['width']}x{config['height']}",
            "-framerate",
            str(config["fps"]),
            "-i",
            config["device"],
            "-copyts",
            "-map",
            "0:v:0",
            "-an",
            "-c:v",
            "copy",
            "-f",
            "nut",
            "-y",
            str(output),
            "-map",
            "0:v:0",
            "-vf",
            f"scale={PREVIEW_WIDTH}:{PREVIEW_HEIGHT}:force_original_aspect_ratio=decrease,"
            f"pad={PREVIEW_WIDTH}:{PREVIEW_HEIGHT}:(ow-iw)/2:(oh-ih)/2:black",
            "-c:v",
            "rawvideo",
            "-pix_fmt",
            "bgr24",
            "-f",
            "rawvideo",
            "pipe:1",
        ],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=log_file,
        start_new_session=True,
    )


def signal_process_stop(process: subprocess.Popen) -> None:
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)


def wait_process(process: subprocess.Popen) -> int:
    try:
        return process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        return process.wait(timeout=5)


def probe_video(
    path: Path, expected: dict, robot_duration: float, capture_start: float, capture_end: float
) -> tuple[dict, np.ndarray, np.ndarray]:
    result = run_checked(
        [
            "ffprobe",
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=width,height:frame=best_effort_timestamp_time",
            "-of",
            "json",
            str(path),
        ]
    )
    payload = json.loads(result.stdout)
    stream = payload["streams"][0]
    all_timestamps = np.asarray(
        [float(frame["best_effort_timestamp_time"]) for frame in payload.get("frames", [])],
        dtype=np.float64,
    )
    # Keep one cadence before the formal start so the first robot sample can
    # use the latest frame exactly as online deployment does.
    selected_file_indices = np.flatnonzero(
        (all_timestamps >= capture_start - 1.5 / expected["fps"])
        & (all_timestamps <= capture_end)
    )
    timestamps = all_timestamps[selected_file_indices]
    dt = np.diff(timestamps)
    nominal = 1.0 / expected["fps"]
    gap_indices = np.flatnonzero(dt > nominal * CAMERA_GAP_FACTOR)
    non_monotonic = np.flatnonzero(dt <= 0)
    expected_frames = int(round(robot_duration * expected["fps"]))
    edge_delta = len(timestamps) - expected_frames
    video_duration = float(timestamps[-1] - timestamps[0]) if len(timestamps) > 1 else 0.0
    passed = (
        stream["width"] == expected["width"]
        and stream["height"] == expected["height"]
        and len(timestamps) > 1
        and len(gap_indices) == 0
        and len(non_monotonic) == 0
        and abs(video_duration - robot_duration) <= 0.25
    )
    report = {
        "passed": passed,
        "width": stream["width"],
        "height": stream["height"],
        "frames": len(timestamps),
        "expected_frames_from_robot_duration": expected_frames,
        "edge_frame_delta": edge_delta,
        "video_duration": video_duration,
        "duration_delta_from_robot": video_duration - robot_duration,
        "first_timestamp": float(timestamps[0]) if len(timestamps) else None,
        "last_timestamp": float(timestamps[-1]) if len(timestamps) else None,
        "internal_gaps": len(gap_indices),
        "gap_after_frame_indices": gap_indices.tolist(),
        "non_monotonic_intervals": len(non_monotonic),
        "max_interval_ms": float(dt.max() * 1000) if len(dt) else None,
        "buffer_frames": expected.get("buffer_frames", 1),
        "timestamp_mode": expected.get("timestamp_mode", "unknown"),
    }
    # Alignment may use any warmup frame as the latest frame at formal start.
    alignment_file_indices = np.flatnonzero(all_timestamps <= capture_end)
    return report, all_timestamps[alignment_file_indices], alignment_file_indices


def analyze_robot_quality(raw, timestamp: np.ndarray) -> dict:
    dt = np.diff(timestamp)
    robot_duration = float(timestamp[-1] - timestamp[0])
    return {
        "passed": bool(
            len(timestamp) > 1
            and np.isfinite(timestamp).all()
            and all(
                np.isfinite(raw[key]).all()
                for key in ("observation_state", "observation_velocity", "observation_effort", "action")
            )
            and float(dt.max()) <= ROBOT_MAX_SAMPLE_INTERVAL_SECONDS
        ),
        "frames": len(timestamp),
        "duration": robot_duration,
        "actual_fps": float((len(timestamp) - 1) / robot_duration),
        "mean_interval_ms": float(dt.mean() * 1000),
        "max_interval_ms": float(dt.max() * 1000),
        "max_allowed_interval_ms": ROBOT_MAX_SAMPLE_INTERVAL_SECONDS * 1000,
    }


def analyze_quest_quality(raw, timestamp: np.ndarray) -> dict:
    quest_sequence = raw["quest_sequence"].astype(np.int64)
    quest_age = raw["quest_packet_age_seconds"].astype(np.float64)
    controller_state = raw["quest_controller_state"].astype(np.float64)
    if (
        quest_sequence.shape != timestamp.shape
        or quest_age.shape != timestamp.shape
        or controller_state.shape != (len(timestamp), 26)
    ):
        raise RuntimeError("Quest tracking arrays do not match robot samples")
    sequence_delta = np.diff(quest_sequence)
    left_tracking = controller_state[:, 12] > 0.5
    right_tracking = controller_state[:, 25] > 0.5
    valid_packet_age = np.isfinite(quest_age) & (quest_age >= 0.0)
    max_packet_age = float(quest_age.max()) if len(quest_age) else float("inf")
    return {
        "passed": bool(
            len(quest_sequence) > 1
            and np.isfinite(controller_state).all()
            and valid_packet_age.all()
            and max_packet_age <= QUEST_MAX_PACKET_LOSS_SECONDS
            and left_tracking.all()
            and right_tracking.all()
            and (sequence_delta >= 0).all()
        ),
        "source": str(raw["teleop_source"].item()),
        "schema": str(raw["quest_schema"].item()),
        "samples": int(len(quest_sequence)),
        "max_packet_age_ms": max_packet_age * 1000,
        "packet_loss_failure_threshold_ms": QUEST_MAX_PACKET_LOSS_SECONDS * 1000,
        "sequence_rollbacks": int((sequence_delta < 0).sum()),
        "repeated_sequence_samples": int((sequence_delta == 0).sum()),
        "left_tracking_lost_samples": int((~left_tracking).sum()),
        "right_tracking_lost_samples": int((~right_tracking).sum()),
    }


def safe_return_joint_errors(target: np.ndarray, measured: np.ndarray) -> dict[str, float]:
    """Return the safety-relevant joint error for each arm; grippers are excluded."""
    return {
        "right": float(np.max(np.abs(target[:6] - measured[:6]))),
        "left": float(np.max(np.abs(target[7:13] - measured[7:13]))),
    }


def safe_return_complete(target: np.ndarray, measured: np.ndarray) -> bool:
    return all(
        error <= SAFE_RETURN_COMPLETION_RAD - SAFE_RETURN_NUMERICAL_MARGIN_RAD
        for error in safe_return_joint_errors(target, measured).values()
    )


def collection_initial_complete(target: np.ndarray, measured: np.ndarray) -> bool:
    """Collection start additionally requires both grippers physically open."""
    return safe_return_complete(target, measured) and bool(
        np.all(
            measured[[6, 13]]
            >= GRIPPER_WIDTH - INITIAL_GRIPPER_OPEN_TOLERANCE_M
        )
    )


def analyze_episode(episode: Path, camera_configs: dict) -> dict:
    raw = np.load(episode / "raw_demo.npz", allow_pickle=False)
    timing = json.loads((episode / "timing.json").read_text(encoding="utf-8"))
    timestamp = raw["timestamp"]
    robot_duration = float(timestamp[-1] - timestamp[0])
    robot_report = analyze_robot_quality(raw, timestamp)
    quest_report = analyze_quest_quality(raw, timestamp) if "teleop_source" in raw.files else None
    cameras = {}
    camera_timestamps = {}
    camera_file_indices = {}
    for role, config in camera_configs.items():
        camera_report, pts, file_indices = probe_video(
            episode / f"{role}.nut",
            config,
            robot_duration,
            timing["capture_start_unix"],
            timing["capture_end_unix"],
        )
        cameras[role] = camera_report
        camera_timestamps[role] = pts
        camera_file_indices[role] = file_indices

    robot_unix = raw["timestamp_unix"].astype(np.float64)
    first_camera_at_or_after_start = []
    for pts in camera_timestamps.values():
        index = int(np.searchsorted(pts, timing["capture_start_unix"], side="left"))
        if index >= len(pts):
            raise RuntimeError("camera has no frame at or after formal capture start")
        first_camera_at_or_after_start.append(float(pts[index]))
    common_start = max(
        timing["capture_start_unix"], float(robot_unix[0]), *first_camera_at_or_after_start
    )
    common_end = min(
        timing["capture_end_unix"],
        float(robot_unix[-1]),
        *(float(pts[-1]) for pts in camera_timestamps.values() if len(pts)),
    )
    model_valid_mask = (robot_unix >= common_start) & (robot_unix <= common_end)
    alignment = {
        "robot_timestamp_unix": robot_unix,
        "model_valid_mask": model_valid_mask,
        "common_start_unix": np.asarray(common_start),
        "common_end_unix": np.asarray(common_end),
    }
    for role, pts in camera_timestamps.items():
        latest = np.searchsorted(pts, robot_unix, side="right") - 1
        valid = latest >= 0
        file_index = np.full(len(robot_unix), -1, dtype=np.int64)
        age_seconds = np.full(len(robot_unix), np.nan, dtype=np.float64)
        file_index[valid] = camera_file_indices[role][latest[valid]]
        age_seconds[valid] = robot_unix[valid] - pts[latest[valid]]
        evaluation_age = age_seconds[model_valid_mask]
        finite_age = evaluation_age[np.isfinite(evaluation_age)]
        alignment[f"{role}_file_frame_index"] = file_index
        alignment[f"{role}_age_seconds"] = age_seconds
        alignment[f"{role}_lag_timesteps_at_{FPS}hz"] = age_seconds * FPS
        alignment[f"{role}_frame_timestamp_unix"] = pts
        cameras[role]["aligned_robot_samples"] = int(len(finite_age))
        cameras[role]["common_model_samples"] = int(model_valid_mask.sum())
        cameras[role]["mean_visual_age_ms"] = float(finite_age.mean() * 1000) if len(finite_age) else None
        cameras[role]["max_visual_age_ms"] = float(finite_age.max() * 1000) if len(finite_age) else None
        cameras[role]["mean_lag_timesteps_at_50hz"] = (
            float(finite_age.mean() * FPS) if len(finite_age) else None
        )
        cameras[role]["passed"] = cameras[role]["passed"] and bool(
            len(finite_age) == int(model_valid_mask.sum())
            and len(finite_age) > 0
            and finite_age.max() <= 1.5 / camera_configs[role]["fps"]
        )
    np.savez_compressed(episode / "time_alignment.npz", **alignment)
    passed = (
        robot_report["passed"]
        and all(report["passed"] for report in cameras.values())
        and (quest_report is None or quest_report["passed"])
    )
    report = {
        "passed": passed,
        "robot": robot_report,
        "quest": quest_report,
        "common_model_samples": int(model_valid_mask.sum()),
        "common_start_unix": common_start,
        "common_end_unix": common_end,
        "cameras": cameras,
    }
    (episode / "quality_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


class Workflow:
    def __init__(
        self,
        *,
        quest_port: int = DEFAULT_PORT,
        quest_host: str | None = None,
        task: str = "",
        input_mode: str = "quest",
    ):
        self.authority = SafetyAuthority()
        self.shutdown_result = "safe_shutdown"
        # Initialize every field consulted by status rendering and cleanup
        # before acquiring either arm.  Active motor verification below can
        # fail after PersistentArms has established a live, gravity-compensated
        # SDK session; emergency_cleanup_partial() must then be able to render
        # status and return to the shutdown pose without touching attributes
        # that have not been created yet.
        self.quest_receiver: Quest3Receiver | None = None
        self.quest_input: QuestWorkflowInput | None = None
        self.quest_teleop: QuestTeleopController | None = None
        self.hardware = resolve_registered_hardware()
        if not all(arm["can_exists"] for arm in self.hardware["arms"].values()):
            raise RuntimeError("can0/can1 未就绪，无法启动常驻重力补偿")
        boot_id = SYSTEM_BOOT_ID_PATH.read_text(encoding="utf-8").strip()
        saved_boot_id = (
            GLOBAL_SHUTDOWN_BOOT_ID_PATH.read_text(encoding="utf-8").strip()
            if GLOBAL_SHUTDOWN_BOOT_ID_PATH.exists()
            else ""
        )
        expected_shutdown_pose = None
        calibration_newer_than_shutdown = bool(
            GRIPPER_CALIBRATION_PATH.exists()
            and (
                not GLOBAL_SHUTDOWN_POSE_PATH.exists()
                or GRIPPER_CALIBRATION_PATH.stat().st_mtime
                > GLOBAL_SHUTDOWN_POSE_PATH.stat().st_mtime
            )
        )
        if calibration_newer_than_shutdown:
            print(
                "夹爪标定晚于现有停机姿态；忽略旧停机姿态，"
                "本次SDK建立后自动捕捉当前位置。",
                flush=True,
            )
        if (
            GLOBAL_SHUTDOWN_POSE_PATH.exists()
            and saved_boot_id == boot_id
            and not calibration_newer_than_shutdown
        ):
            expected_shutdown_pose = np.load(GLOBAL_SHUTDOWN_POSE_PATH).astype(np.float32)
            self.startup_pose = expected_shutdown_pose
            if self.startup_pose.shape != (ACTION_DIM,) or not np.isfinite(self.startup_pose).all():
                raise RuntimeError("本次开机停机姿态文件无效，拒绝启动")
            print("已加载本次系统开机时捕获的停机姿态；不会因进程重启覆盖。")
        self.arms = PersistentArms(expected_shutdown_pose=expected_shutdown_pose)
        time.sleep(0.1)
        current_pose, _, _ = self.arms.state()
        if expected_shutdown_pose is None:
            self.startup_pose = current_pose.astype(np.float32)
            SHARED_POSES.mkdir(parents=True, exist_ok=True)
            np.save(GLOBAL_SHUTDOWN_POSE_PATH, self.startup_pose)
            GLOBAL_SHUTDOWN_BOOT_ID_PATH.write_text(boot_id + "\n", encoding="utf-8")
            print("已快速捕获本次系统开机的初始静息/停机姿态。")
        self.shutdown_pose_reached = False
        ensure_shared_collection_pose()
        self.initial_pose = np.load(COLLECTION_INITIAL_POSE_PATH).astype(np.float32)
        if self.initial_pose.shape != (ACTION_DIM,) or not np.isfinite(self.initial_pose).all():
            raise RuntimeError("共享采集起始姿态文件无效，拒绝启动")
        # The shared collection pose has one canonical gripper state regardless
        # of what older registration files happened to contain.
        if not np.allclose(self.initial_pose[[6, 13]], GRIPPER_WIDTH):
            self.initial_pose[[6, 13]] = GRIPPER_WIDTH
            np.save(COLLECTION_INITIAL_POSE_PATH, self.initial_pose)
            print("已将共享采集初始姿态的双夹爪规范化为完全张开。")
        self.episode: Path | None = None
        self.camera_processes: dict[str, subprocess.Popen] = {}
        self.recording_preview: RecordingPreview | None = None
        self.idle_preview: subprocess.Popen | None = None
        self.camera_logs: list = []
        self.robot: RobotRecorder | None = None
        self.quality: dict | None = None
        self.capture_start_unix: float | None = None
        self.capture_end_unix: float | None = None
        self.state = "等待中"
        self.requested_task = task.strip()
        self.input_mode = input_mode
        self.target_accepted_rounds = 50
        self.accepted_rounds = 0
        self.attempt_number = 1
        self.last_task = ""
        self.current_task = ""
        self.visualized = False
        self.review_queue: list[Path] = []
        # Do not run the diagnostic wrist micro-motion probe in the production
        # startup path.  preflight_at_shutdown() has already verified fresh
        # state timestamps without writing motion commands;
        # safe_return_to_initial() below is the authoritative closed-loop
        # actuation check.  Running a second mode/gain transition here made a
        # healthy cold-start session lose feedback before collection began.
        self.restore_collection_progress()
        self.state = "采集阶段等待"
        if self.requested_task and self.requested_task != self.last_task:
            self.last_task = self.requested_task
            self.set_task_progress(self.last_task)
            self.save_collection_progress()
        print("启动后自动回归共享采集起始姿态。", flush=True)
        self.safe_return_to_initial()
        self.state = "采集阶段等待"
        # Quest is a lower-authority command producer. Do not even construct
        # its receiver/controller threads until the persistent SDK session owns
        # both arms and the computer workflow has completed the initial safe
        # return. This makes startup ordering structural rather than relying
        # only on the VR-authority boolean inside a live teleop thread.
        self.start_quest_stack(quest_port=quest_port, quest_host=quest_host)
        self.start_idle_preview()

    def start_quest_stack(
        self, *, quest_port: int, quest_host: str | None
    ) -> None:
        if self.input_mode != "quest" or self.quest_receiver is not None:
            return
        if self.state != "采集阶段等待" or self.authority.vr_allowed():
            raise RuntimeError("机械臂安全回归尚未完成，拒绝启动Quest输入层")
        self.quest_receiver = Quest3Receiver(port=quest_port, allowed_sender=quest_host)
        self.quest_receiver.start()
        if not self.quest_receiver.ready.wait(timeout=2.0) or self.quest_receiver.error:
            error = self.quest_receiver.error or "启动超时"
            self.quest_receiver.close()
            self.quest_receiver = None
            print(
                f"警告：Quest 输入层不可用（{error}）；机械臂SDK会话继续在线，"
                "本次仅保留键盘维护操作。",
                flush=True,
            )
            return
        self.quest_input = QuestWorkflowInput(self.quest_receiver)
        self.quest_teleop = QuestTeleopController(
            self.quest_receiver,
            self.arms,
            self.authority.vr_allowed,
            ROOT / "quest3_teleop_config.json",
            initial_pose=self.initial_pose,
        )
        self.quest_teleop.start()

    def start_idle_preview(self) -> None:
        """Show live cameras in the same Quest panel used by review playback."""
        if self.input_mode != "quest" or self.idle_preview is not None:
            return
        try:
            self.idle_preview = subprocess.Popen(
                [
                    str(Path(os.sys.executable)),
                    str(ROOT / "quest3_camera_stream.py"),
                    "--port", "10505",
                    "--fps", "20",
                ],
                stdin=subprocess.DEVNULL,
                start_new_session=True,
            )
            time.sleep(0.3)
            if self.idle_preview.poll() is not None:
                code = self.idle_preview.returncode
                self.idle_preview = None
                print(
                    f"警告：三视角预览不可用（退出码={code}）；机械臂控制继续保持，"
                    "请检查USB摄像头后重新启动预览。",
                    flush=True,
                )
        except (OSError, RuntimeError) as exc:
            self.idle_preview = None
            print(f"警告：三视角预览启动失败：{exc}；机械臂控制继续保持。", flush=True)

    def stop_idle_preview(self) -> None:
        if self.idle_preview is None:
            return
        signal_process_stop(self.idle_preview)
        wait_process(self.idle_preview)
        self.idle_preview = None

    def session_episodes(self) -> list[Path]:
        episodes: list[Path] = []
        for status in ("accepted", "rejected", "pending"):
            root = SESSIONS / status
            if root.is_dir():
                episodes.extend(path for path in root.iterdir() if path.is_dir())
        return episodes

    def set_task_progress(self, task: str) -> None:
        matching = [episode for episode in self.session_episodes() if episode_task(episode) == task]
        accepted_root = SESSIONS / "accepted"
        self.accepted_rounds = sum(episode.parent == accepted_root for episode in matching)
        self.attempt_number = max((episode_numbers(episode)[1] for episode in matching), default=0) + 1

    def pending_reviews(self) -> list[Path]:
        pending_root = SESSIONS / "pending"
        if not pending_root.is_dir():
            return []
        return sorted(
            (
                episode
                for episode in pending_root.iterdir()
                if episode.is_dir()
                and (episode / "raw_demo.npz").is_file()
                and (episode / "quality_report.json").is_file()
            ),
            key=lambda path: (path.stat().st_mtime, path.name),
        )

    def pending_audits(self) -> list[Path]:
        pending_root = SESSIONS / "pending"
        if not pending_root.is_dir():
            return []
        required = ("raw_demo.npz", "timing.json")
        camera_files = tuple(f"{role}.nut" for role in self.hardware["cameras"])
        return sorted(
            (
                episode
                for episode in pending_root.iterdir()
                if episode.is_dir()
                and all((episode / name).is_file() for name in required + camera_files)
                and not (episode / "quality_report.json").is_file()
            ),
            key=lambda path: (path.stat().st_mtime, path.name),
        )

    def save_collection_progress(self) -> None:
        SESSIONS.mkdir(parents=True, exist_ok=True)
        payload = {
            "version": 1,
            "last_task": self.last_task,
            "accepted_rounds": self.accepted_rounds,
            "attempt_number": self.attempt_number,
            "pending_episode": str(self.episode) if self.episode is not None else None,
            "pending_review_count": len(self.pending_reviews()),
            "pending_audit_count": len(self.pending_audits()),
            "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        }
        temporary = WORKFLOW_STATE_PATH.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        temporary.replace(WORKFLOW_STATE_PATH)

    def restore_collection_progress(self) -> None:
        saved: dict = {}
        if WORKFLOW_STATE_PATH.is_file():
            try:
                saved = json.loads(WORKFLOW_STATE_PATH.read_text(encoding="utf-8"))
            except (OSError, ValueError, TypeError):
                print("警告：采集进度文件无效，将从 episode 目录重建。")

        episodes = self.session_episodes()
        self.last_task = str(saved.get("last_task", "")).strip()
        if not self.last_task:
            task_episodes = [(episode.stat().st_mtime, episode_task(episode)) for episode in episodes]
            task_episodes = [(mtime, task) for mtime, task in task_episodes if task]
            if task_episodes:
                self.last_task = max(task_episodes)[1]
        if self.last_task:
            self.set_task_progress(self.last_task)

        if self.last_task:
            print(
                f"已恢复采集记录：任务={self.last_task!r}，"
                f"已接受={self.accepted_rounds}/{self.target_accepted_rounds}，"
                f"下一尝试={self.attempt_number}"
            )
        pending_count = len(self.pending_reviews()) + len(self.pending_audits())
        if pending_count:
            print(f"已恢复 {pending_count} 条待审阅数据；可在采集阶段按 v 进入审阅。")
        self.state = "采集阶段等待"
        self.save_collection_progress()

    @property
    def round_number(self) -> int:
        return self.accepted_rounds + 1

    def show_status(self) -> None:
        width = dashboard_width()
        top = "┏" + "━" * (width - 2) + "┓"
        middle = "┣" + "━" * (width - 2) + "┫"
        bottom = "┗" + "━" * (width - 2) + "┛"
        title = dashboard_row("ARX AC One · Quest 双臂数据采集", width, align="center")
        progress = f"进度：已接受 {self.accepted_rounds}/{self.target_accepted_rounds} · 尝试 {self.attempt_number}"
        queue = f"待审核：已质检 {len(self.pending_reviews())} · 待质检 {len(self.pending_audits())}"
        lines = [
            top,
            title,
            middle,
            dashboard_pair(f"状态：{self.state}", progress, width),
            dashboard_pair(f"任务：{self.last_task or '未设置'}", queue, width),
            dashboard_row(f"数据：{SESSIONS}", width),
        ]
        quest_text = "Quest：未启用"
        grip_text = "Grip：左-- · 右--"
        if self.quest_receiver is not None:
            snapshot = self.quest_receiver.snapshot()
            silence = self.quest_receiver.silence_seconds()
            age_ms = silence * 1000.0
            link = "在线" if snapshot.fresh() else "等待数据/超时"
            age = (
                f"{max(0.0, age_ms):.0f} ms"
                if snapshot.sequence >= 0
                else f"休眠 {silence:.0f}/{QUEST_SLEEP_SAFE_EXIT_SECONDS:.0f}s"
            )
            quest_text = (
                f"Quest：{link} · {self.quest_receiver.sender or '尚未锁定'} · {age} · "
                f"链路自愈 {self.quest_receiver.socket_recoveries}"
            )
            grip_text = (
                f"Grip：左{'按下' if snapshot.left.grip else '松开'} · "
                f"右{'按下' if snapshot.right.grip else '松开'}"
            )
        control_text = "双臂：控制器准备中"
        if self.quest_teleop is not None:
            control_text = (
                f"双臂：左 {self.quest_teleop.status('left')} · "
                f"右 {self.quest_teleop.status('right')} · "
                f"前向{'已校准' if self.quest_teleop.forward_motion_calibrated() else '默认映射'}"
            )
        lines.extend(
            [
                dashboard_pair(quest_text, grip_text, width),
                dashboard_row(control_text, width),
            ]
        )
        authority = self.authority.snapshot()
        authority_text = (
            "权限：SDK会话 > 异常安全 > 电脑 > VR · "
            + ("安全停机已锁定" if authority.shutdown_latched else (
                "VR运动已授权" if authority.vr_enabled else "VR运动未授权"
            ))
        )
        lines.append(dashboard_row(authority_text, width))
        if self.quest_teleop is not None and self.quest_teleop.error is not None:
            lines.append(dashboard_row(f"警告：{self.quest_teleop.error}", width))

        if self.state == "采集阶段等待":
            controller_hint = "手柄　按住左Menu并向前移动1秒 校准前向　摇杆上 开始　下 审阅"
            keyboard_hint = "键盘　s 开始　v 审阅　o 回起始位　r 注册　q 安全退出"
        elif self.state == "采集中":
            controller_hint = "手柄　Grip 跟随　Trigger 夹爪　左X 左臂回位　右A 右臂回位"
            keyboard_hint = "结束　摇杆下/右B/e 结束本轮　左X+右B长按2秒/q 安全退出"
        elif self.state == "审阅阶段":
            controller_hint = "手柄　摇杆上 接受　下 拒绝　左 回放　右 继续　左X+右B长按2秒 退出"
            keyboard_hint = "键盘　p 回放　a 接受　x 拒绝　b 继续采集　q 安全退出"
        else:
            controller_hint = "当前操作执行中，请等待；手柄运动授权已暂停"
            keyboard_hint = "停止请求仍会先回到停机姿态，确认后才断开控制"

        prompt_start = len(lines)
        lines.extend(
            [
                middle,
                dashboard_row("", width),
                dashboard_row("【 下一步操作 · 键盘与手柄均可 · 同时输入时键盘优先 】", width, align="center"),
                dashboard_row("", width),
                dashboard_row(controller_hint, width),
                dashboard_row(keyboard_hint, width),
                dashboard_row("", width),
                dashboard_row("安全规则：停机姿态确认后才允许断开机械臂控制", width),
                bottom,
            ]
        )
        # Color is applied after padding, so it never changes cell alignment.
        lines[1] = "\033[1;36m" + lines[1] + "\033[0m"
        for index in range(prompt_start + 1, len(lines) - 1):
            lines[index] = "\033[1;33m" + lines[index] + "\033[0m"
        print("\033[2J\033[H" + "\n".join(lines), end="\033[J", flush=True)

    def register_pose(self) -> None:
        if self.state != "采集阶段等待":
            raise RuntimeError("只有等待状态可以注册初始姿态")
        if self.robot is not None:
            raise RuntimeError("cannot register pose while recording")
        self.arms.connect(expected_shutdown_pose=self.startup_pose)
        self.shutdown_pose_reached = False
        self.arms.set_teach_mode()
        time.sleep(0.1)
        self.initial_pose, _, _ = self.arms.state()
        self.initial_pose[[6, 13]] = GRIPPER_WIDTH
        SHARED_POSES.mkdir(parents=True, exist_ok=True)
        np.save(COLLECTION_INITIAL_POSE_PATH, self.initial_pose)
        self.save_collection_progress()
        print(f"共享采集起始姿态已更新：{COLLECTION_INITIAL_POSE_PATH}")

    def calibrate_quest_forward_motion(self) -> None:
        if self.state != "采集阶段等待":
            raise WorkflowInputRejected("只有采集等待阶段可以校准前向")
        if self.quest_input is None or self.quest_teleop is None:
            raise WorkflowInputRejected("Quest 输入层未连接，不能校准前向")
        displacement = self.quest_input.mapper.consume_forward_displacement()
        if displacement is None:
            raise WorkflowInputRejected("没有完整的 Menu+前移 1 秒轨迹样本")
        try:
            matrix = self.quest_teleop.calibrate_forward_motion(displacement)
        except ValueError as exc:
            raise WorkflowInputRejected(str(exc)) from exc
        horizontal = np.asarray(displacement, dtype=np.float64).copy()
        horizontal[1] = 0.0
        yaw_degrees = float(np.degrees(np.arctan2(horizontal[2], horizontal[0])))
        print(
            f"Quest 前向移动校准完成：水平位移={np.linalg.norm(horizontal):.3f}m，"
            f"偏航补偿基准={yaw_degrees:+.1f}°，矩阵="
            f"{np.array2string(matrix, precision=3, suppress_small=True)}",
            flush=True,
        )

    def start(self) -> None:
        if self.state != "采集阶段等待":
            raise RuntimeError("当前状态不能开始采集")
        if self.initial_pose is None:
            raise RuntimeError("请先按 r 注册初始姿态")
        if self.robot is not None:
            raise RuntimeError("当前已经在采集")
        self.stop_idle_preview()
        try:
            self.hardware = resolve_registered_hardware()
            if not all(arm["can_exists"] for arm in self.hardware["arms"].values()):
                raise RuntimeError("can0/can1 未就绪")
        except BaseException:
            self.start_idle_preview()
            raise
        task = (
            self.requested_task
            if self.input_mode == "quest"
            else read_task_text(f"任务文本（回车沿用：{self.last_task or '尚未设置'}）：")
        )
        if task and task != self.last_task:
            self.last_task = task
            self.set_task_progress(self.last_task)
            print(
                f"已切换/恢复任务：已接受 {self.accepted_rounds}/"
                f"{self.target_accepted_rounds}，下一尝试 {self.attempt_number}"
            )
        if not self.last_task:
            raise RuntimeError("第一轮必须用 --task 指定任务文本；之后可沿用上次任务")
        if self.accepted_rounds >= self.target_accepted_rounds:
            raise RuntimeError("当前任务已经完成50轮有效数据；请输入新的任务文本")
        self.current_task = self.last_task
        self.visualized = False
        try:
            self.safe_return_to_initial()
        except RuntimeError:
            self.state = "采集阶段等待"
            raise
        self.state = "采集阶段等待"
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.episode = SESSIONS / "pending" / (
            f"round_{self.round_number:03d}_attempt_{self.attempt_number:03d}_{timestamp}"
        )
        self.episode.mkdir(parents=True, exist_ok=False)
        self.save_collection_progress()
        camera_configs = self.hardware["cameras"]
        # Share the exact Quest display endpoint used by review playback.
        self.recording_preview = RecordingPreview(port=10505)
        for role, config in camera_configs.items():
            configure_camera(config["device"])
            log = (self.episode / f"{role}.ffmpeg.log").open("w", encoding="utf-8")
            self.camera_logs.append(log)
            self.camera_processes[role] = start_camera(
                role, config, self.episode / f"{role}.nut", log
            )
            self.recording_preview.add(role, self.camera_processes[role])
        self.recording_preview.start()
        self.robot = RobotRecorder(
            self.arms,
            self.episode / "raw_demo.npz",
            self.initial_pose,
            self.current_task,
            self.quest_receiver,
        )
        self.robot.start()
        self.state = "摄像机预热和控制器准备中"
        self.show_status()
        print(
            f"三路摄像机初始化 {CAMERA_INITIALIZATION_SECONDS:.1f} 秒，随后自动曝光稳定 "
            f"{WARMUP_SECONDS:.1f} 秒……",
            flush=True,
        )
        time.sleep(CAMERA_INITIALIZATION_SECONDS + WARMUP_SECONDS)
        failed = [role for role, proc in self.camera_processes.items() if proc.poll() is not None]
        if failed:
            raise RuntimeError(f"摄像机启动失败: {failed}")
        if not self.robot.ready.wait(timeout=10) or self.robot.error:
            raise RuntimeError(f"机械臂采集启动失败: {self.robot.error}")
        # The workflow performs one position-hold transition. Quest receives motion authority
        # after both arms already hold their current measured positions.
        self.arms.enter_position_hold()
        # The recorder must be armed before VR receives motion authority.  This
        # prevents the first controller command from preceding the first robot
        # sample and silently shifting action/observation alignment.
        self.capture_start_unix = time.time_ns() / 1e9
        self.robot.record_event.set()
        self.state = "采集中"
        self.authority.enable_vr()
        self.show_status()
        print("数据采集已开始；按住对应 Grip 才会跟随，松开立即冻结；长按右 B 结束。", flush=True)

    def stop_and_analyze(self) -> None:
        if self.state != "采集中":
            raise RuntimeError("只有采集中状态可以按 e 结束")
        if self.robot is None or self.episode is None:
            raise RuntimeError("当前没有正在进行的采集")
        self.authority.disable_vr("collection stop requested")
        self.state = "正在结束采集并封装文件"
        self.show_status()
        self.capture_end_unix = time.time_ns() / 1e9
        self.robot.stop_event.set()
        for process in self.camera_processes.values():
            signal_process_stop(process)
        self.robot.join(timeout=15)
        if self.robot.is_alive():
            raise RuntimeError("机械臂采集线程未能停止")
        robot_error = self.robot.error
        self.robot = None
        statuses = {role: wait_process(proc) for role, proc in self.camera_processes.items()}
        self.camera_processes.clear()
        if self.recording_preview is not None:
            self.recording_preview.close()
            self.recording_preview = None
        for log in self.camera_logs:
            log.close()
        self.camera_logs.clear()
        if robot_error:
            raise RuntimeError(f"机械臂采集失败: {robot_error}")
        # FFmpeg returns 255 when SIGINT cleanly finalizes a live input.
        if any(code not in (0, 255) for code in statuses.values()):
            raise RuntimeError(f"摄像机退出异常: {statuses}")
        (self.episode / "timing.json").write_text(
            json.dumps(
                {
                    "warmup_seconds": WARMUP_SECONDS,
                    "camera_initialization_seconds": CAMERA_INITIALIZATION_SECONDS,
                    "capture_start_unix": self.capture_start_unix,
                    "capture_end_unix": self.capture_end_unix,
                    "camera_buffer_frames": 1,
                    "timestamp_basis": "V4L2 monotonic converted to absolute host clock",
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        completed_episode = self.episode
        self.attempt_number += 1
        self.episode = None
        self.quality = None
        self.current_task = ""
        self.visualized = False
        self.state = "采集阶段等待"
        print(f"已加入待审计队列：{completed_episode.name}")
        self.save_collection_progress()
        print("采集封装完成；已回共享采集起始位，继续等待下一轮。")
        self.safe_return_to_initial()
        self.state = "采集阶段等待"
        self.start_idle_preview()
        self.show_status()

    def audit_pending_parallel(self) -> None:
        episodes = self.pending_audits()
        if not episodes:
            return
        workers = min(4, len(episodes))
        print(f"开始并行审计 {len(episodes)} 条数据，工作线程={workers}……", flush=True)

        def audit_one(episode: Path) -> tuple[Path, dict]:
            try:
                report = analyze_episode(episode, self.hardware["cameras"])
            except Exception as exc:
                report = {"passed": False, "analysis_error": str(exc)}
                (episode / "quality_report.json").write_text(
                    json.dumps(report, indent=2), encoding="utf-8"
                )
            return episode, report

        completed = 0
        passed = 0
        discarded = 0
        with ThreadPoolExecutor(max_workers=workers, thread_name_prefix="episode-audit") as pool:
            futures = {pool.submit(audit_one, episode): episode for episode in episodes}
            for future in as_completed(futures):
                episode, report = future.result()
                completed += 1
                if report.get("passed"):
                    passed += 1
                    result = "通过"
                else:
                    shutil.rmtree(episode)
                    discarded += 1
                    result = "失败，已自动丢弃"
                print(f"审计进度 {completed}/{len(episodes)}：{episode.name} -> {result}")
        print(f"并行审计完成：通过 {passed}，失败并丢弃 {discarded}。")

    def load_review_episode(self) -> bool:
        self.review_queue = self.pending_reviews()
        if not self.review_queue:
            self.episode = None
            self.quality = None
            self.current_task = ""
            self.visualized = False
            return False
        self.episode = self.review_queue[0]
        try:
            quality = json.loads(
                (self.episode / "quality_report.json").read_text(encoding="utf-8")
            )
        except (OSError, ValueError, TypeError) as exc:
            raise RuntimeError(f"无法读取质检报告 {self.episode.name}: {exc}") from exc
        if not isinstance(quality, dict) or "passed" not in quality:
            raise RuntimeError(f"质检报告格式无效：{self.episode.name}")
        self.quality = quality
        self.current_task = episode_task(self.episode)
        self.visualized = False
        return True

    def enter_review(self) -> None:
        if self.state != "采集阶段等待":
            raise RuntimeError("只有采集等待阶段可以进入审阅")
        if not self.pending_reviews() and not self.pending_audits():
            raise RuntimeError("当前没有待审阅数据")
        self.stop_idle_preview()
        previous_state = self.state
        try:
            self.safe_return_to_startup()
            self.arms.set_teach_mode()
        except RuntimeError:
            self.state = previous_state
            raise
        print(
            "机械臂已回到停机姿态并进入低阻尼重力补偿；"
            "双臂SDK会话保持在线，审阅结束后无需重新初始化。"
        )
        self.state = "并行审计中"
        self.show_status()
        self.audit_pending_parallel()
        if not self.load_review_episode():
            self.state = "采集阶段等待"
            return
        self.state = "审阅阶段"
        print(f"开始审阅：{self.episode.name}")
        self.save_collection_progress()

    def leave_review(self) -> None:
        if self.state != "审阅阶段":
            raise RuntimeError("当前不在审阅阶段")
        self.episode = None
        self.quality = None
        self.current_task = ""
        self.visualized = False
        self.review_queue = []
        print("双臂常驻控制器保持在线，正在从停机位恢复采集流程……")
        self.arms.connect(expected_shutdown_pose=self.startup_pose)
        self.shutdown_pose_reached = False
        self.state = "采集阶段等待"
        self.safe_return_to_initial()
        self.state = "采集阶段等待"
        self.start_idle_preview()
        print("已沿用同一双臂SDK会话回到共享采集起始姿态，等待下一轮。")
        self.save_collection_progress()

    def replay(self) -> None:
        if self.state != "审阅阶段":
            raise RuntimeError("当前状态不能播放数据可视化")
        if self.episode is None or self.quality is None:
            raise RuntimeError("请先结束采集并完成质量检查")
        subprocess.run(
            [
                str(Path(os.sys.executable)),
                str(ROOT / "visualize_episode.py"),
                str(self.episode),
                "--quest-stream",
            ],
            check=True,
        )
        self.visualized = True

    def decide(self, accept: bool) -> None:
        if self.state != "审阅阶段":
            raise RuntimeError("当前状态没有等待决策的数据")
        if self.episode is None or self.quality is None:
            raise RuntimeError("没有等待决策的数据")
        if accept and not self.quality["passed"]:
            raise RuntimeError("质量检查失败，不能接受本轮数据")
        destination_root = SESSIONS / ("accepted" if accept else "rejected")
        destination_root.mkdir(parents=True, exist_ok=True)
        destination = destination_root / self.episode.name
        self.episode.rename(destination)
        (destination / "decision.json").write_text(
            json.dumps({"accepted": accept, "decided_at": time.strftime("%Y-%m-%dT%H:%M:%S")}, indent=2),
            encoding="utf-8",
        )
        print(f"本轮已{'接受' if accept else '拒绝'}：{destination}")
        reviewed_task = episode_task(destination)
        if reviewed_task == self.last_task:
            self.set_task_progress(self.last_task)
        self.episode = None
        self.quality = None
        self.current_task = ""
        self.visualized = False
        if self.load_review_episode():
            self.state = "审阅阶段"
            print(f"下一条待审阅数据：{self.episode.name}")
        else:
            self.state = "审阅阶段"
            print("本批审阅已完成；机械臂在停机位保持低阻尼在线，请选择继续采集或安全退出。")
        self.save_collection_progress()
        self.show_status()

    def manual_restore(self) -> None:
        if self.state != "采集阶段等待":
            raise RuntimeError("当前阶段不能手动回归初始位置")
        previous_state = self.state
        try:
            self.safe_return_to_initial()
        finally:
            self.state = previous_state
        self.show_status()

    def control_fault(self) -> str | None:
        """Escalate robot/control failures; Quest transport failure is isolated."""
        # A short CAN scheduling/USB stall is handled inside QuestTeleop by
        # freezing both targets and waiting for the same SDK session to resume.
        # Only a continuous five-second loss escalates to manual safe exit.
        fault = self.arms.health_fault(CAN_FATAL_FEEDBACK_TIMEOUT_SECONDS)
        if fault is not None:
            return fault
        # Quest sleep is normal and does not affect the persistent robot SDK session. During
        # collection/waiting, freeze VR immediately and allow wake/reconnect
        # for three minutes. Review explicitly permits the headset to be off.
        if (
            self.quest_receiver is not None
            and self.state in {"采集阶段等待", "采集中"}
            and self.quest_receiver.silence_seconds()
            > QUEST_SLEEP_SAFE_EXIT_SECONDS
        ):
            return (
                "Quest连续休眠/无有效数据超过"
                f"{QUEST_SLEEP_SAFE_EXIT_SECONDS:.0f}s"
            )
        if self.state == "采集中":
            if self.robot is None:
                return "采集中但机械臂记录线程不存在"
            if self.robot.error is not None:
                return f"机械臂记录线程异常：{self.robot.error}"
            if not self.robot.is_alive():
                return "机械臂记录线程意外退出"
            if self.quest_teleop is not None:
                if self.quest_teleop.error is not None:
                    return f"VR控制计算线程异常：{self.quest_teleop.error}"
                if not self.quest_teleop.is_alive():
                    return "VR控制计算线程意外退出"
                for side in ("left", "right"):
                    session = self.quest_teleop.sessions[side]
                    if session.transient_faults >= 25:
                        return f"{side} 连续机械臂SDK命令异常达到安全阈值"
        return None

    def safe_return_to_initial(self) -> None:
        if self.initial_pose is None:
            return
        self.arms.connect()
        self.safe_return_to_pose(
            self.initial_pose,
            "安全回归采集初始位置中",
            require_open_grippers=True,
        )
        self.shutdown_pose_reached = False

    def safe_return_to_startup(self) -> None:
        if not self.arms.connected:
            print("机械臂控制器已关闭；保持停机被动状态。")
            self.shutdown_pose_reached = True
            return
        self.safe_return_to_pose(self.startup_pose, "安全回归开机位置中")
        self.shutdown_pose_reached = True

    def disconnect_after_shutdown_pose(self, reason: str) -> None:
        """The only legal full-arm disconnect path in the collection workflow."""
        if not self.arms.connected:
            self.shutdown_pose_reached = True
            return
        self.safe_return_to_startup()
        if not self.shutdown_pose_reached:
            raise RuntimeError(f"{reason}：未确认到达停机姿态，拒绝断开控制")
        self.arms.set_teach_mode()
        print(f"{reason}：已确认停机姿态，现允许断开双臂控制器。", flush=True)
        self.arms.close()

    def safe_return_to_pose(
        self,
        target_pose: np.ndarray,
        status: str,
        *,
        require_open_grippers: bool = False,
    ) -> None:
        # Safety/computer motion always outranks Quest. Revoke VR before the
        # state transition and before reading or writing either arm, so no
        # controller packet can contend with a return trajectory.
        self.authority.disable_vr(f"computer safety motion: {status}")
        self.state = status
        self.show_status()
        self.arms.wait_for_feedback_recovery(status)
        current, _, _ = self.arms.state()
        target = np.asarray(target_pose, dtype=np.float32).copy()
        if target.shape != (ACTION_DIM,) or not np.isfinite(target).all():
            raise RuntimeError("目标姿态无效，拒绝执行安全回归")
        if require_open_grippers:
            target[[6, 13]] = GRIPPER_WIDTH
        complete = (
            collection_initial_complete(target, current)
            if require_open_grippers
            else safe_return_complete(target, current)
        )
        initial_errors = safe_return_joint_errors(target, current)
        initial_delta = max(initial_errors.values())
        if complete:
            self.arms.set_teach_mode()
            print(f"机械臂已在目标位置附近，最大误差={initial_delta:.4f} rad")
            return
        if initial_delta > 2.0:
            self.arms.set_teach_mode()
            raise RuntimeError(f"回归距离 {initial_delta:.3f} rad 超过安全限制，请人工调整")
        left_cmd = arx5.JointState(self.arms.left_robot.joint_dof)
        right_cmd = arx5.JointState(self.arms.right_robot.joint_dof)
        right_cmd.pos()[:] = current[:6]
        right_cmd.gripper_pos = float(current[6])
        left_cmd.pos()[:] = current[7:13]
        left_cmd.gripper_pos = float(current[13])
        target[[6, 13]] = np.clip(target[[6, 13]], 0.0, GRIPPER_WIDTH)
        feedback_lost = False
        try:
            with self.arms.command_gate, self.arms.sdk_lock:
                self.arms.right.set_joint_cmd(right_cmd)
                self.arms.left.set_joint_cmd(left_cmd)
                set_reduced_gain(
                    self.arms.left, self.arms.left_ctrl, self.arms.left_robot.joint_dof
                )
                set_reduced_gain(
                    self.arms.right, self.arms.right_ctrl, self.arms.right_robot.joint_dof
                )
            # The passive firmware may suppress telemetry while idle. Arm a
            # zero-displacement hold first and prove fresh kernel RX before the
            # interpolator is allowed to move toward its target.
            self.arms.verify_active_feedback(f"{status}运动前")
            next_tick = time.monotonic()
            previous_error = {
                **initial_errors,
            }
            stalled_checks = {"right": 0, "left": 0}
            converged = False
            for step in range(10 * FPS):
                try:
                    recovered = self.arms.wait_for_feedback_recovery(status)
                except RobotFeedbackLost:
                    feedback_lost = True
                    raise
                if recovered:
                    # The arm may have moved while transport was unavailable.
                    # Rebase both command interpolators from fresh feedback so
                    # recovery never catches up a stale trajectory in one jump.
                    resumed, _, _ = self.arms.state()
                    right_cmd.pos()[:] = resumed[:6]
                    right_cmd.gripper_pos = float(resumed[6])
                    left_cmd.pos()[:] = resumed[7:13]
                    left_cmd.gripper_pos = float(resumed[13])
                    with self.arms.command_gate, self.arms.sdk_lock:
                        self.arms.right.set_joint_cmd(right_cmd)
                        self.arms.left.set_joint_cmd(left_cmd)
                    previous_error = safe_return_joint_errors(target, resumed)
                    stalled_checks = {"right": 0, "left": 0}
                    next_tick = time.monotonic()
                with self.arms.command_gate, self.arms.sdk_lock:
                    command_pair(
                        self.arms.left,
                        self.arms.right,
                        target,
                        left_cmd,
                        right_cmd,
                        0.006,
                        0.0005,
                    )
                next_tick += 1.0 / FPS
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                if (step + 1) % FPS == 0:
                    progress, _, _ = self.arms.state()
                    errors = safe_return_joint_errors(target, progress)
                    for side in ("right", "left"):
                        improvement = previous_error[side] - errors[side]
                        stalled_checks[side] = (
                            stalled_checks[side] + 1
                            if errors[side] > 0.08 and improvement < 0.003
                            else 0
                        )
                        previous_error[side] = errors[side]
                    print(
                        f"回归进度：right={errors['right']:.4f}rad，"
                        f"left={errors['left']:.4f}rad",
                        flush=True,
                    )
                    complete = (
                        collection_initial_complete(target, progress)
                        if require_open_grippers
                        else safe_return_complete(target, progress)
                    )
                    if complete:
                        converged = True
                        break
                    stalled = [side for side, count in stalled_checks.items() if count >= 2]
                    if stalled:
                        raise RuntimeError(
                            f"安全回归检测到无响应侧：{stalled}；"
                            f"right={errors['right']:.4f}rad，left={errors['left']:.4f}rad"
                        )
            measured, _, _ = self.arms.state()
            final_errors = safe_return_joint_errors(target, measured)
            final_error = max(final_errors.values())
            complete = (
                collection_initial_complete(target, measured)
                if require_open_grippers
                else safe_return_complete(target, measured)
            )
            if not converged and not complete:
                raise RuntimeError(
                    "安全回归未达到断开精度："
                    f"right={final_errors['right']:.4f}rad，"
                    f"left={final_errors['left']:.4f}rad，"
                    f"要求双臂均 < {SAFE_RETURN_COMPLETION_RAD:.4f}rad"
                )
            print(f"安全回归完成，最大位置误差={final_error:.4f} rad")
        finally:
            if feedback_lost:
                print(
                    "机械臂反馈已丢失，无法确认低阻尼或重力补偿仍然生效。",
                    flush=True,
                )
            else:
                self.arms.set_teach_mode()
                print("已恢复低阻尼重力补偿模式。")

    def wait_for_operator_power_off(self, reason: str) -> None:
        """Hold the SDK object until the operator physically removes arm power."""
        global _power_off_confirmation_active
        self.state = "硬件反馈失联，等待断电确认"
        print("\n" + "!" * 88, flush=True)
        print(f"机械臂反馈链路失效：{reason}", flush=True)
        print("软件无法验证当前位置，已停止自动轨迹与VR目标。", flush=True)
        try:
            self.arms.set_teach_mode()
            print("已请求双臂进入零Kp低阻尼重力补偿；请手动扶正到稳定位置。", flush=True)
        except BaseException as exc:
            print(f"无法确认低阻尼指令是否送达（{exc}）；请持续物理支撑双臂。", flush=True)
        print("扶正后请关闭机械臂控制电源。", flush=True)
        print("确认机械臂控制电源已经关闭后，直接按 Enter；Ctrl-C 也可确认释放。", flush=True)
        print("在确认前程序保留SDK对象、不再发送回归轨迹，也不会重复刷屏。", flush=True)
        print("!" * 88, flush=True)
        _power_off_confirmation_active = True
        try:
            try:
                read_task_text("已支撑双臂并关闭控制电源后按 Enter > ")
            except KeyboardInterrupt:
                print("\n已通过 Ctrl-C 确认释放。", flush=True)
            except (EOFError, OSError):
                print("终端输入已关闭，按断电确认处理并释放失联SDK对象。", flush=True)
            self.shutdown_result = "operator_power_off"
            print("已收到断电确认，允许释放失联SDK对象。", flush=True)
        finally:
            _power_off_confirmation_active = False

    def cleanup(self) -> None:
        self.authority.request_safe_shutdown("process cleanup / terminal safety stop")
        # Stop command producers first, but never let peripheral cleanup skip
        # the mandatory shutdown-pose transition.
        if self.quest_teleop is not None:
            try:
                self.quest_teleop.close()
            except BaseException as exc:
                print(f"警告：停止Quest遥控异常：{exc}", flush=True)
        if self.robot is not None:
            self.robot.stop_event.set()
        for process in self.camera_processes.values():
            try:
                signal_process_stop(process)
            except BaseException as exc:
                print(f"警告：停止相机采集异常：{exc}", flush=True)

        # Do not close the SDK unless the shutdown pose is positively reached.
        # On failure, remain connected in low damping so the operator can move
        # closer to the saved pose; retry until the invariant is satisfied.
        while self.arms.connected:
            try:
                self.disconnect_after_shutdown_pose("程序退出")
            except RobotFeedbackLost as exc:
                self.wait_for_operator_power_off(str(exc))
                self.arms.close()
                break
            except BaseException as exc:
                try:
                    self.arms.set_teach_mode()
                except BaseException:
                    pass
                print(
                    f"拒绝断开：尚未安全到达停机姿态（{exc}）。"
                    "双臂保持低阻尼连接；请人工靠近停机位，程序将在2秒后重试。",
                    flush=True,
                )
                time.sleep(2.0)

        # Arms are now safely disconnected; remaining peripheral cleanup is
        # best effort and cannot affect the shutdown invariant.
        try:
            self.stop_idle_preview()
        except BaseException as exc:
            print(f"警告：停止待机预览异常：{exc}", flush=True)
        if self.robot is not None:
            self.robot.join(timeout=5)
            self.robot = None
        for process in self.camera_processes.values():
            try:
                wait_process(process)
            except BaseException as exc:
                print(f"警告：等待相机退出异常：{exc}", flush=True)
        self.camera_processes.clear()
        if self.recording_preview is not None:
            try:
                self.recording_preview.close()
            except BaseException as exc:
                print(f"警告：关闭录制预览异常：{exc}", flush=True)
            self.recording_preview = None
        for log in self.camera_logs:
            try:
                log.close()
            except BaseException:
                pass
        self.camera_logs.clear()
        if self.quest_receiver is not None:
            try:
                self.quest_receiver.close()
            except BaseException as exc:
                print(f"警告：关闭Quest接收器异常：{exc}", flush=True)

    def emergency_cleanup_partial(self) -> None:
        """Clean a partially constructed workflow after initialization fails."""
        global _shutdown_cleanup_active
        _shutdown_cleanup_active = True
        try:
            idle = getattr(self, "idle_preview", None)
            if idle is not None:
                signal_process_stop(idle)
                wait_process(idle)
            arms = getattr(self, "arms", None)
            startup_pose = getattr(self, "startup_pose", None)
            if arms is not None and arms.connected:
                if startup_pose is None:
                    arms.set_teach_mode()
                    raise RuntimeError("没有可验证的停机姿态，拒绝断开已连接控制器")
                while arms.connected:
                    try:
                        self.safe_return_to_pose(
                            startup_pose, "初始化失败，安全回归开机位置中"
                        )
                        self.shutdown_pose_reached = True
                        self.disconnect_after_shutdown_pose("初始化失败清理")
                    except RobotFeedbackLost as exc:
                        # Feedback loss makes every further trajectory blind.
                        # Stop retrying immediately and use the same explicit
                        # physical-power-off handoff as normal cleanup.
                        self.wait_for_operator_power_off(str(exc))
                        arms.close()
                        break
                    except BaseException as exc:
                        try:
                            arms.set_teach_mode()
                        except BaseException:
                            pass
                        print(
                            f"拒绝断开：初始化失败且尚未到达停机姿态（{exc}）。"
                            "双臂保持低阻尼连接；2秒后重试。",
                            flush=True,
                        )
                        time.sleep(2.0)
        except BaseException as exc:
            print(f"警告：初始化失败后的控制器清理异常：{exc}", flush=True)


def print_menu(workflow: Workflow) -> None:
    workflow.show_status()


def read_stage_key() -> str:
    """Read one stage command immediately; task text still uses normal input()."""
    if not sys.stdin.isatty():
        return input("\n请输入当前阶段指令 > ").strip().lower()
    fd = sys.stdin.fileno()
    previous = termios.tcgetattr(fd)
    print("\n按下当前阶段指令键（无需回车） > ", end="", flush=True)
    try:
        tty.setcbreak(fd)
        while True:
            key = sys.stdin.read(1)
            if key and not key.isspace():
                print(key, flush=True)
                return key.lower()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, previous)


def read_task_text(prompt: str) -> str:
    """Read a normal echoed line even after the console used cbreak key input."""
    if not sys.stdin.isatty():
        return input(prompt).strip()
    fd = sys.stdin.fileno()
    previous = termios.tcgetattr(fd)
    canonical = termios.tcgetattr(fd)
    canonical[0] |= termios.ICRNL
    canonical[1] |= termios.OPOST
    canonical[3] |= termios.ICANON | termios.ECHO | termios.ISIG
    canonical[6][termios.VMIN] = 1
    canonical[6][termios.VTIME] = 0
    try:
        termios.tcsetattr(fd, termios.TCSANOW, canonical)
        print(prompt, end="", flush=True)
        return sys.stdin.readline().rstrip("\r\n").strip()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, previous)


def read_interactive_command(workflow: Workflow, use_quest: bool) -> str:
    """Read both inputs concurrently, resolving a same-cycle conflict for keyboard."""
    valid = {
        "采集阶段等待": {"c", "r", "s", "v", "o", "q"},
        "采集中": {"e", "q"},
        "审阅阶段": {"p", "a", "x", "b", "q"},
    }.get(workflow.state, set())
    fd = sys.stdin.fileno() if sys.stdin.isatty() else None
    previous = termios.tcgetattr(fd) if fd is not None else None
    if fd is not None:
        tty.setcbreak(fd)
    last_refresh = 0.0
    try:
        while True:
            fault = workflow.control_fault()
            if fault is not None:
                raise RuntimeError(f"机械臂SDK会话检测到故障：{fault}")
            # The computer is above VR in the authority hierarchy. Poll it
            # first and never flush pending keys during a stage transition.
            if fd is not None and select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1).lower()
                if key in valid:
                    return key
            if use_quest:
                assert workflow.quest_input is not None
                # Quest is the lowest authority.  Its failure only disables VR
                # input; keyboard and the persistent SDK session continue normally.
                if workflow.quest_input.receiver.error is None:
                    command = workflow.quest_input.mapper.update(
                        workflow.state, workflow.quest_input.receiver.snapshot()
                    )
                    if command is not None:
                        return command
            if fd is None and not use_quest:
                command = input("当前阶段指令 > ").strip().lower()
                if command in valid:
                    return command
            now = time.monotonic()
            if now - last_refresh >= 0.25:
                workflow.show_status()
                last_refresh = now
            time.sleep(0.01)
    finally:
        if fd is not None and previous is not None:
            termios.tcsetattr(fd, termios.TCSADRAIN, previous)


def main() -> int:
    # The shell launcher starts this process as a background job while it waits
    # for readiness. Explicitly restore signal handling so Ctrl+C/TERM always
    # enters the normal workflow.cleanup() path.
    signal.signal(signal.SIGINT, request_safe_shutdown)
    signal.signal(signal.SIGTERM, request_safe_shutdown)
    signal.signal(signal.SIGUSR1, request_safe_shutdown)
    signal.signal(signal.SIGHUP, request_safe_shutdown)
    parser = argparse.ArgumentParser()
    parser.add_argument("--quest-port", type=int, default=DEFAULT_PORT)
    parser.add_argument(
        "--quest-host", default=None,
        help="Expected Quest IPv4 address; when omitted the first valid sender is latched",
    )
    parser.add_argument("--task", default="", help="新任务文本；省略时沿用上次任务")
    parser.add_argument(
        "--sessions-root",
        type=Path,
        default=SESSIONS,
        help="独立的数据、进度和姿态根目录",
    )
    parser.add_argument(
        "--input", choices=("quest", "keyboard"), default="quest",
        help="Enable Quest alongside keyboard; simultaneous commands resolve to keyboard",
    )
    parser.add_argument("--shutdown-status-file", type=Path, default=None)
    args = parser.parse_args()
    configure_sessions_root(args.sessions_root)
    workflow = Workflow.__new__(Workflow)
    try:
        Workflow.__init__(
            workflow,
            quest_port=args.quest_port,
            quest_host=args.quest_host,
            task=args.task,
            input_mode=args.input,
        )
    except (OSError, RuntimeError, KeyboardInterrupt) as exc:
        print(f"启动失败：{exc}")
        if workflow is not None:
            workflow.emergency_cleanup_partial()
            arms = getattr(workflow, "arms", None)
            if arms is None or not arms.connected:
                if getattr(workflow, "shutdown_result", "") == "operator_power_off":
                    write_lifecycle_status(
                        args.shutdown_status_file, "OPERATOR_POWER_OFF_CONFIRMED"
                    )
                    print("OPERATOR_POWER_OFF_CONFIRMED", flush=True)
                elif getattr(workflow, "shutdown_pose_reached", False):
                    write_shutdown_complete(args.shutdown_status_file)
                    print("SAFE_SHUTDOWN_COMPLETE", flush=True)
                else:
                    write_no_control_acquired(args.shutdown_status_file)
                    print("NO_CONTROL_ACQUIRED", flush=True)
                if "None of the motors are initialized" in str(exc):
                    # The vendor extension has already stopped and destroyed its
                    # partially constructed SocketCAN receiver at this point.
                    # Returning through Python finalization triggers a known
                    # duplicate-free abort in that failed-constructor path.
                    # There is no live controller to clean up, so bypass only
                    # process-global extension finalizers for this exact error.
                    sys.stdout.flush()
                    sys.stderr.flush()
                    os._exit(1)
        return 1
    print("ARX AC One Quest 3 三视觉双臂采集控制台")
    print(f"数据域：{SESSIONS}")
    print("硬件序列号检查通过；键盘与手柄均可操作；同周期冲突时电脑键盘优先。")
    try:
        while True:
            print_menu(workflow)
            command = read_interactive_command(
                workflow, args.input == "quest" and workflow.quest_input is not None
            )
            if command == "q":
                if workflow.state == "采集中":
                    try:
                        workflow.stop_and_analyze()
                    except (OSError, RuntimeError, subprocess.SubprocessError, ValueError) as exc:
                        print(
                            f"当前轮结束时回归异常：{exc}；"
                            "不返回交互界面，立即转入最高优先级安全停机。",
                            flush=True,
                        )
                        workflow.authority.request_safe_shutdown(
                            f"collection exit return failed: {exc}"
                        )
                        break
                previous_state = workflow.state
                try:
                    workflow.safe_return_to_startup()
                except RuntimeError as exc:
                    workflow.state = previous_state
                    print(f"拒绝退出：无法安全回到开机姿态：{exc}")
                    print("请在低阻尼重力补偿下手动靠近开机姿态后再次按 q。")
                    continue
                print("正在退出并关闭重力补偿及机械臂控制器……")
                break
            commands = {
                "采集阶段等待": {
                    "c": workflow.calibrate_quest_forward_motion,
                    "r": workflow.register_pose,
                    "s": workflow.start,
                    "v": workflow.enter_review,
                    "o": workflow.manual_restore,
                },
                "采集中": {"e": workflow.stop_and_analyze},
                "审阅阶段": {
                    "p": workflow.replay,
                    "a": lambda: workflow.decide(True),
                    "x": lambda: workflow.decide(False),
                    "b": workflow.leave_review,
                },
            }
            action = commands.get(workflow.state, {}).get(command)
            if action is None:
                print("该指令在当前状态不可用，请按面板提示操作。")
                continue
            try:
                action()
            except WorkflowInputRejected as exc:
                print(f"校准未生效：{exc}；保持等待状态，可重新操作。", flush=True)
                continue
            except (OSError, RuntimeError, subprocess.SubprocessError, ValueError) as exc:
                print(f"操作异常，进入自动安全停机：{exc}")
                workflow.authority.request_safe_shutdown(f"workflow exception: {exc}")
                break
    except (KeyboardInterrupt, EOFError):
        print("\n收到终止请求。")
        workflow.authority.request_safe_shutdown("terminal signal or terminal closed")
    except BaseException as exc:
        print(f"\n未处理异常，进入自动安全停机：{exc}", flush=True)
        workflow.authority.request_safe_shutdown(f"unhandled exception: {exc}")
    finally:
        global _shutdown_cleanup_active
        _shutdown_cleanup_active = True
        print("安全停机已锁定：后续 Ctrl-C 不会中断回停机位流程。", flush=True)
        workflow.cleanup()
        if workflow.shutdown_result == "operator_power_off":
            write_lifecycle_status(
                args.shutdown_status_file, "OPERATOR_POWER_OFF_CONFIRMED"
            )
            print("OPERATOR_POWER_OFF_CONFIRMED", flush=True)
        else:
            write_shutdown_complete(args.shutdown_status_file)
            print("SAFE_SHUTDOWN_COMPLETE", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
