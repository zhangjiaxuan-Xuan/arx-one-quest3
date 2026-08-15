#!/usr/bin/env python3
"""Software-only Quest-to-ARX command stability test.

No ARX controller is constructed.  The real Quest teleoperation loop, mapping,
PID, IK, gripper logic and 50 Hz scheduler write into an instrumented virtual
arm boundary so unsafe interpolation can be rejected before hardware is used.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from typing import Callable

import numpy as np
from scipy.spatial.transform import Rotation

from arx_common import ACTION_DIM, GRIPPER_WIDTH
from quest3_input import (
    DEFAULT_PORT,
    TRACKING_TIMEOUT_SECONDS,
    ControllerState,
    Quest3Receiver,
    QuestSnapshot,
)
from quest3_teleop import (
    MAX_JOINT_ACCELERATION_RAD_S2,
    MAX_JOINT_VELOCITY_RAD_S,
    MAX_GRIPPER_STEP_M,
    MAX_JOINT_STEP_RAD,
    QuestTeleopController,
)


ROOT = Path(__file__).resolve().parent
CONFIG_PATH = ROOT / "quest3_teleop_config.json"
INITIAL_POSE_PATH = ROOT / "shared_poses" / "collection_initial_pose.npy"
REPORT_PATH = ROOT / "logs" / "quest3" / "no_arm_stability_latest.json"
SYNTHETIC_MINIMUM_SECONDS = 16.0
# A position slew limit alone still permits +0.015 -> -0.015 rad in one 20 ms
# tick.  Reject that high-frequency reversal before hardware is connected.
MAX_COMMAND_ACCEL_RAD_S2 = MAX_JOINT_ACCELERATION_RAD_S2 + 0.5
CONTIGUOUS_INTERVAL_SECONDS = 0.060


@dataclass(frozen=True)
class CommandEvent:
    timestamp: float
    side: str
    sequence: int
    phase: str
    thread_id: int
    joints: tuple[float, ...]
    gripper: float


class InstrumentedCommandGate:
    """Track writer overlap while preserving the RLock API used in production."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._counter_lock = threading.Lock()
        self.active = 0
        self.max_active = 0

    def __enter__(self):
        self._lock.acquire()
        with self._counter_lock:
            self.active += 1
            self.max_active = max(self.max_active, self.active)
        return self

    def __exit__(self, exc_type, exc, traceback):
        with self._counter_lock:
            self.active -= 1
        self._lock.release()


class VirtualArms:
    """Ideal measured plant plus an audit log; contains no vendor controller."""

    def __init__(
        self,
        initial_pose: np.ndarray,
        context: Callable[[str], tuple[int, str]],
    ) -> None:
        pose = np.asarray(initial_pose, dtype=np.float64)
        if pose.shape != (ACTION_DIM,) or not np.isfinite(pose).all():
            raise ValueError("virtual arms require a finite 14D initial pose")
        self.connected = True
        self.command_gate = InstrumentedCommandGate()
        self.left = SimpleNamespace(name="virtual-left")
        self.right = SimpleNamespace(name="virtual-right")
        self.left_robot = SimpleNamespace(joint_dof=6)
        self.right_robot = SimpleNamespace(joint_dof=6)
        self._position = {"right": pose[:6].copy(), "left": pose[7:13].copy()}
        self._gripper = {"right": float(pose[6]), "left": float(pose[13])}
        self._context = context
        self._lock = threading.RLock()
        self.events: list[CommandEvent] = []
        self.gain_events: list[tuple[float, str, int]] = []

    def side_state(self, side: str):
        with self._lock:
            zeros = np.zeros(6, dtype=np.float32)
            return (
                self._position[side].astype(np.float32).copy(),
                zeros.copy(),
                zeros.copy(),
                np.float32(self._gripper[side]),
                np.float32(0.0),
                np.float32(0.0),
            )

    def set_side_command(self, side: str, command) -> None:
        joints = np.asarray(command.pos(), dtype=np.float64).copy()
        gripper = float(command.gripper_pos)
        if joints.shape != (6,) or not np.isfinite(joints).all() or not np.isfinite(gripper):
            raise RuntimeError(f"{side} generated a non-finite virtual command")
        sequence, phase = self._context(side)
        with self._lock:
            self.events.append(
                CommandEvent(
                    timestamp=time.monotonic(),
                    side=side,
                    sequence=sequence,
                    phase=phase,
                    thread_id=threading.get_ident(),
                    joints=tuple(float(value) for value in joints),
                    gripper=gripper,
                )
            )
            # Ideal position feedback isolates command-generation continuity
            # from physical tracking error and motor dynamics.
            self._position[side] = joints
            self._gripper[side] = gripper

    def set_side_control_mode(self, side: str) -> None:
        sequence, _ = self._context(side)
        with self._lock:
            self.gain_events.append((time.monotonic(), side, sequence))


class SyntheticReceiver:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._snapshot = QuestSnapshot()
        self.phase = {"left": "idle", "right": "idle"}

    def snapshot(self) -> QuestSnapshot:
        with self._lock:
            return self._snapshot

    def update(self, snapshot: QuestSnapshot, phase: str) -> None:
        with self._lock:
            self._snapshot = snapshot
            self.phase = {"left": phase, "right": phase}

    def set_phase_without_packet(self, phase: str) -> None:
        with self._lock:
            self.phase = {"left": phase, "right": phase}

    def context(self, side: str) -> tuple[int, str]:
        with self._lock:
            return self._snapshot.sequence, self.phase[side]


def load_initial_pose() -> np.ndarray:
    if not INITIAL_POSE_PATH.is_file():
        raise RuntimeError(f"missing shared initial pose: {INITIAL_POSE_PATH}")
    pose = np.load(INITIAL_POSE_PATH).astype(np.float64)
    if pose.shape != (ACTION_DIM,) or not np.isfinite(pose).all():
        raise RuntimeError("shared collection initial pose is invalid")
    pose[[6, 13]] = GRIPPER_WIDTH
    return pose


def controller_state(
    elapsed: float,
    *,
    side_sign: float,
    grip: bool,
    tracking: bool,
    world_offset: float = 0.0,
) -> ControllerState:
    phase = elapsed * 0.55
    position = (
        world_offset + 0.025 * np.sin(phase),
        1.20 + 0.020 * np.sin(phase * 0.71),
        side_sign * 0.24 + 0.025 * np.cos(phase * 0.83),
    )
    rotation = Rotation.from_euler(
        "xyz",
        [0.12 * np.sin(phase * 0.67), 0.15 * np.sin(phase * 0.53), 0.18 * np.cos(phase * 0.61)],
    )
    trigger = 1.0 if int(elapsed // 2.0) % 2 else 0.0
    return ControllerState(
        position=tuple(float(value) for value in position),
        orientation_xyzw=tuple(float(value) for value in rotation.as_quat()),
        grip=grip,
        trigger=trigger,
        tracking=tracking,
    )


def synthetic_phase(cycle_time: float) -> str:
    if cycle_time < 1.0:
        return "idle"
    if cycle_time < 5.0:
        return "smooth"
    if cycle_time < 5.0 + TRACKING_TIMEOUT_SECONDS + 0.05:
        return "dropout_grace"
    if cycle_time < 6.0:
        return "stale_blocked"
    if cycle_time < 10.0:
        return "recovered"
    if cycle_time < 11.0:
        return "tracking_lost"
    if cycle_time < 13.0:
        return "tracking_recovered"
    if cycle_time < 14.0:
        return "released"
    return "reengaged"


def run_synthetic(receiver: SyntheticReceiver, duration: float) -> None:
    started = time.monotonic()
    sequence = 0
    next_packet = started
    while time.monotonic() - started < duration:
        now = time.monotonic()
        elapsed = now - started
        cycle_time = elapsed % SYNTHETIC_MINIMUM_SECONDS
        phase = synthetic_phase(cycle_time)
        if phase in {"dropout_grace", "stale_blocked"}:
            receiver.set_phase_without_packet(phase)
        elif now >= next_packet:
            grip = phase not in {"idle", "released"}
            tracking = phase != "tracking_lost"
            # A large world-space relocation after each outage deliberately
            # proves that re-anchoring suppresses catch-up motion.
            world_offset = 0.65 if phase in {"recovered", "tracking_recovered"} else 0.0
            left = controller_state(
                elapsed, side_sign=-1.0, grip=grip, tracking=tracking,
                world_offset=world_offset,
            )
            right = controller_state(
                elapsed, side_sign=1.0, grip=grip, tracking=tracking,
                world_offset=world_offset,
            )
            receiver.update(
                QuestSnapshot(
                    sequence=sequence,
                    received_monotonic=now,
                    left=left,
                    right=right,
                ),
                phase,
            )
            sequence += 1
            next_packet += 1.0 / 90.0
        time.sleep(0.001)


def live_context(receiver: Quest3Receiver, side: str) -> tuple[int, str]:
    snapshot = receiver.snapshot()
    controller = snapshot.left if side == "left" else snapshot.right
    if not snapshot.fresh():
        phase = "stale_blocked"
    elif not controller.tracking:
        phase = "tracking_lost"
    elif not controller.grip:
        phase = "released"
    else:
        phase = "live_active"
    return snapshot.sequence, phase


def percentile(values: np.ndarray, value: float) -> float:
    return float(np.percentile(values, value)) if len(values) else 0.0


def analyze(
    arms: VirtualArms,
    *,
    mode: str,
    duration: float,
    controller_error: BaseException | None,
) -> dict:
    failures: list[str] = []
    if controller_error is not None:
        failures.append(f"teleop thread failed: {type(controller_error).__name__}: {controller_error}")
    if arms.command_gate.max_active > 1:
        failures.append(f"overlapping command transactions: {arms.command_gate.max_active}")
    events = arms.events
    if not events:
        failures.append("no virtual arm commands were generated")

    forbidden_phases = {"stale_blocked", "tracking_lost", "released", "idle"}
    forbidden = [event for event in events if event.phase in forbidden_phases]
    if forbidden:
        failures.append(f"{len(forbidden)} commands were written in blocked phases")

    side_reports = {}
    all_thread_ids = {event.thread_id for event in events}
    if len(all_thread_ids) > 1:
        failures.append(f"multiple command writer threads observed: {sorted(all_thread_ids)}")
    for side in ("left", "right"):
        side_events = [event for event in events if event.side == side]
        joints = np.asarray([event.joints for event in side_events], dtype=np.float64)
        gripper = np.asarray([event.gripper for event in side_events], dtype=np.float64)
        timestamps = np.asarray([event.timestamp for event in side_events], dtype=np.float64)
        joint_steps = (
            np.max(np.abs(np.diff(joints, axis=0)), axis=1)
            if len(joints) > 1 else np.zeros(0)
        )
        gripper_steps = np.abs(np.diff(gripper)) if len(gripper) > 1 else np.zeros(0)
        intervals = np.diff(timestamps) if len(timestamps) > 1 else np.zeros(0)
        nonpositive_intervals = int(np.count_nonzero(intervals <= 0.0))
        velocities = np.zeros((len(intervals), 6), dtype=np.float64)
        if len(intervals):
            np.divide(
                np.diff(joints, axis=0),
                intervals[:, None],
                out=velocities,
                where=intervals[:, None] > 0.0,
            )
        contiguous_pairs = (
            (intervals[:-1] > 0.0)
            & (intervals[1:] > 0.0)
            & (intervals[:-1] < CONTIGUOUS_INTERVAL_SECONDS)
            & (intervals[1:] < CONTIGUOUS_INTERVAL_SECONDS)
            if len(intervals) > 1 else np.zeros(0, dtype=bool)
        )
        acceleration = np.zeros((max(0, len(velocities) - 1), 6), dtype=np.float64)
        if len(velocities) > 1:
            acceleration_dt = (intervals[:-1] + intervals[1:]) * 0.5
            np.divide(
                np.abs(np.diff(velocities, axis=0)),
                acceleration_dt[:, None],
                out=acceleration,
                where=acceleration_dt[:, None] > 0.0,
            )
        contiguous_acceleration = acceleration[contiguous_pairs]
        max_acceleration = float(contiguous_acceleration.max(initial=0.0))
        p99_acceleration = (
            percentile(contiguous_acceleration.reshape(-1), 99)
            if contiguous_acceleration.size else 0.0
        )
        full_speed_reversals = 0
        if len(velocities) > 1:
            previous_velocity = velocities[:-1]
            next_velocity = velocities[1:]
            full_speed_reversals = int(np.count_nonzero(
                contiguous_pairs[:, None]
                & (previous_velocity * next_velocity < 0.0)
                & (np.abs(previous_velocity) > 0.5)
                & (np.abs(next_velocity) > 0.5)
            ))
        max_joint_step = float(joint_steps.max(initial=0.0))
        max_joint_velocity = float(np.max(np.abs(velocities), initial=0.0))
        max_gripper_step = float(gripper_steps.max(initial=0.0))
        if max_joint_velocity > MAX_JOINT_VELOCITY_RAD_S + 0.03:
            failures.append(
                f"{side} command velocity {max_joint_velocity:.3f}rad/s "
                f"> {MAX_JOINT_VELOCITY_RAD_S:.3f}rad/s"
            )
        if nonpositive_intervals:
            failures.append(
                f"{side} has {nonpositive_intervals} non-monotonic command timestamps"
            )
        if max_gripper_step > MAX_GRIPPER_STEP_M + 1e-6:
            failures.append(
                f"{side} gripper discontinuity {max_gripper_step:.6f}m > {MAX_GRIPPER_STEP_M:.6f}m"
            )
        if max_acceleration > MAX_COMMAND_ACCEL_RAD_S2 + 1e-6:
            failures.append(
                f"{side} command acceleration {max_acceleration:.3f}rad/s^2 "
                f"> {MAX_COMMAND_ACCEL_RAD_S2:.3f}rad/s^2"
            )
        if full_speed_reversals:
            failures.append(
                f"{side} has {full_speed_reversals} single-tick full-speed reversals"
            )
        if len(side_events) < 10:
            failures.append(f"{side} generated too few commands: {len(side_events)}")
        side_reports[side] = {
            "commands": len(side_events),
            "nonpositive_intervals": nonpositive_intervals,
            "max_joint_step_rad": max_joint_step,
            "max_joint_velocity_rad_s": max_joint_velocity,
            "max_gripper_step_m": max_gripper_step,
            "median_interval_ms": percentile(intervals, 50) * 1000.0,
            "p99_interval_ms": percentile(intervals, 99) * 1000.0,
            "minimum_interval_ms": (
                float(intervals.min()) * 1000.0 if len(intervals) else 0.0
            ),
            "max_command_acceleration_rad_s2": max_acceleration,
            "p99_command_acceleration_rad_s2": p99_acceleration,
            "single_tick_full_speed_reversals": full_speed_reversals,
            "reconnect_commands": sum(
                event.phase in {"recovered", "tracking_recovered"}
                for event in side_events
            ),
        }

    if mode == "synthetic":
        counts_by_sequence: dict[int, set[str]] = {}
        for event in events:
            if event.phase in {"smooth", "recovered", "tracking_recovered", "reengaged"}:
                counts_by_sequence.setdefault(event.sequence, set()).add(event.side)
        paired = sum(sides == {"left", "right"} for sides in counts_by_sequence.values())
        pairing_ratio = paired / max(1, len(counts_by_sequence))
        if pairing_ratio < 0.95:
            failures.append(f"paired-frame ratio too low: {pairing_ratio:.3f}")
    else:
        pairing_ratio = None

    source_files = [
        Path(__file__),
        ROOT / "quest3_teleop.py",
        ROOT / "quest3_input.py",
        CONFIG_PATH,
    ]
    digest = hashlib.sha256()
    for path in source_files:
        digest.update(path.name.encode("utf-8"))
        digest.update(path.read_bytes())
    return {
        "schema": "arx.quest3.no_arm_stability.v1",
        "passed": not failures,
        "mode": mode,
        "duration_seconds": duration,
        "generated_unix_time": time.time(),
        "source_fingerprint_sha256": digest.hexdigest(),
        "hardware_sdk_constructed": False,
        "command_gate_max_overlap": arms.command_gate.max_active,
        "writer_thread_count": len(all_thread_ids),
        "forbidden_phase_writes": len(forbidden),
        "paired_frame_ratio": pairing_ratio,
        "gain_transitions": len(arms.gain_events),
        "sides": side_reports,
        "failures": failures,
    }


def save_report(report: dict, events: list[CommandEvent], report_path: Path) -> None:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    trace_path = report_path.with_suffix(".npz")
    np.savez_compressed(
        trace_path,
        timestamp=np.asarray([event.timestamp for event in events], dtype=np.float64),
        side=np.asarray([event.side for event in events]),
        sequence=np.asarray([event.sequence for event in events], dtype=np.int64),
        phase=np.asarray([event.phase for event in events]),
        joints=np.asarray([event.joints for event in events], dtype=np.float64),
        gripper=np.asarray([event.gripper for event in events], dtype=np.float64),
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("synthetic", "live"), default="synthetic")
    parser.add_argument("--duration", type=float, default=32.0)
    parser.add_argument("--quest-host")
    parser.add_argument("--quest-port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--report", type=Path, default=REPORT_PATH)
    args = parser.parse_args()
    if args.mode == "synthetic" and args.duration < SYNTHETIC_MINIMUM_SECONDS:
        raise ValueError(f"synthetic duration must be at least {SYNTHETIC_MINIMUM_SECONDS:.0f}s")
    if args.duration <= 0:
        raise ValueError("duration must be positive")

    pose = load_initial_pose()
    receiver = None
    controller = None
    try:
        if args.mode == "synthetic":
            receiver = SyntheticReceiver()
            context = receiver.context
        else:
            receiver = Quest3Receiver(port=args.quest_port, allowed_sender=args.quest_host)
            receiver.start()
            if not receiver.ready.wait(timeout=3.0) or receiver.error is not None:
                raise RuntimeError(f"Quest receiver unavailable: {receiver.error or 'startup timeout'}")
            context = lambda side: live_context(receiver, side)

        arms = VirtualArms(pose, context)
        controller = QuestTeleopController(
            receiver,
            arms,
            lambda: True,
            CONFIG_PATH,
            initial_pose=pose,
        )
        controller.start()
        print(
            f"NO_ARM_STABILITY_ACTIVE mode={args.mode} duration={args.duration:.0f}s "
            "ARX_SDK=forbidden robot_power=not_required",
            flush=True,
        )
        if args.mode == "synthetic":
            run_synthetic(receiver, args.duration)
        else:
            started = time.monotonic()
            next_report = started
            while time.monotonic() - started < args.duration:
                now = time.monotonic()
                if now >= next_report:
                    snapshot = receiver.snapshot()
                    print(
                        f"LIVE elapsed={now - started:.0f}s seq={snapshot.sequence} "
                        f"fresh={snapshot.fresh()} commands={len(arms.events)}",
                        flush=True,
                    )
                    next_report = now + 5.0
                time.sleep(0.05)
        controller.close()
        report = analyze(
            arms,
            mode=args.mode,
            duration=args.duration,
            controller_error=controller.error,
        )
        save_report(report, arms.events, args.report)
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        print(f"NO_ARM_STABILITY_REPORT={args.report}", flush=True)
        if not report["passed"]:
            raise SystemExit(1)
        print("NO_ARM_STABILITY_PASS", flush=True)
    finally:
        if controller is not None and controller.is_alive():
            controller.close()
        if isinstance(receiver, Quest3Receiver):
            receiver.close()


if __name__ == "__main__":
    main()
