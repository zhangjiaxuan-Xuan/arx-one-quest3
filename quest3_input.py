#!/usr/bin/env python3
"""Quest 3 controller transport and guarded workflow-button mapping.

The receiver intentionally has no robot dependency.  A Unity/OpenXR client sends
one JSON datagram per tracking frame.  Robot motion authorization (Grip) is kept
separate from workflow actions, which are generated only while both Grips are
released.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
import socket
import threading
import time
from typing import Any


SCHEMA = "arx.quest3.controllers.v1"
DISCOVERY_REQUEST = b"arx.quest3.discover.v1"
DISCOVERY_RESPONSE = b"arx.quest3.host.v1"
DEFAULT_PORT = 7447
# Quest/USB scheduling occasionally introduces a frame gap above 100 ms.  A
# 250 ms transport window avoids treating that jitter as a robot disconnect;
# Grip release remains an immediate, independent motion stop.
TRACKING_TIMEOUT_SECONDS = 0.25
# Operational policy is intentionally separate from tracking freshness and
# episode quality. A sleeping headset freezes VR input immediately but may
# wake and resume for up to three minutes before the robot workflow exits.
QUEST_SLEEP_SAFE_EXIT_SECONDS = 180.0
WORKFLOW_HOLD_SECONDS = 0.70
QUIT_HOLD_SECONDS = 2.0
FORWARD_MOTION_CALIBRATION_SECONDS = 1.0


@dataclass(frozen=True)
class ControllerState:
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    thumbstick_xy: tuple[float, float] = (0.0, 0.0)
    grip: bool = False
    trigger: float = 0.0
    primary: bool = False
    secondary: bool = False
    menu: bool = False
    tracking: bool = False


@dataclass(frozen=True)
class QuestSnapshot:
    sequence: int = -1
    received_monotonic: float = 0.0
    left: ControllerState = field(default_factory=ControllerState)
    right: ControllerState = field(default_factory=ControllerState)

    def fresh(self, now: float | None = None) -> bool:
        current = time.monotonic() if now is None else now
        return self.sequence >= 0 and current - self.received_monotonic <= TRACKING_TIMEOUT_SECONDS


def _finite_vector(value: Any, size: int, name: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f"{name} must contain {size} values")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} contains non-finite values")
    return result


def _controller(payload: Any) -> ControllerState:
    if not isinstance(payload, dict):
        raise ValueError("controller payload must be an object")
    position = _finite_vector(payload.get("position_m"), 3, "position_m")
    orientation = _finite_vector(payload.get("orientation_xyzw"), 4, "orientation_xyzw")
    thumbstick = _finite_vector(payload.get("thumbstick_xy", [0.0, 0.0]), 2, "thumbstick_xy")
    norm = math.sqrt(sum(item * item for item in orientation))
    if not 0.8 <= norm <= 1.2:
        raise ValueError("controller quaternion norm is invalid")
    orientation = tuple(item / norm for item in orientation)
    buttons = payload.get("buttons", {})
    if not isinstance(buttons, dict):
        raise ValueError("buttons must be an object")
    trigger = float(buttons.get("trigger", 0.0))
    if not math.isfinite(trigger):
        raise ValueError("trigger is non-finite")
    return ControllerState(
        position=position,
        orientation_xyzw=orientation,
        thumbstick_xy=thumbstick,
        grip=bool(buttons.get("grip", False)),
        trigger=min(1.0, max(0.0, trigger)),
        primary=bool(buttons.get("primary", False)),
        secondary=bool(buttons.get("secondary", False)),
        menu=bool(buttons.get("menu", False)),
        tracking=bool(payload.get("tracking", True)),
    )


def decode_datagram(data: bytes, received_monotonic: float | None = None) -> QuestSnapshot:
    if len(data) > 16_384:
        raise ValueError("Quest datagram is too large")
    payload = json.loads(data.decode("utf-8"))
    if payload.get("schema") != SCHEMA:
        raise ValueError("Quest schema mismatch")
    sequence = int(payload["sequence"])
    if sequence < 0:
        raise ValueError("sequence must be non-negative")
    return QuestSnapshot(
        sequence=sequence,
        received_monotonic=time.monotonic() if received_monotonic is None else received_monotonic,
        left=_controller(payload.get("left")),
        right=_controller(payload.get("right")),
    )


class Quest3Receiver(threading.Thread):
    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = DEFAULT_PORT,
        allowed_sender: str | None = None,
    ):
        super().__init__(daemon=True, name="quest3-receiver")
        self.host = host
        self.port = port
        self.allowed_sender = allowed_sender
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.error: BaseException | None = None
        self.invalid_packets = 0
        self.socket_recoveries = 0
        self.last_socket_error: BaseException | None = None
        self.started_monotonic = time.monotonic()
        self._lock = threading.Lock()
        self._snapshot = QuestSnapshot()
        self._socket: socket.socket | None = None
        self._latched_sender: str | None = allowed_sender

    @property
    def sender(self) -> str | None:
        return self._latched_sender

    def snapshot(self) -> QuestSnapshot:
        with self._lock:
            return self._snapshot

    def silence_seconds(self, now: float | None = None) -> float:
        """Time since the latest valid controller packet, or receiver startup."""
        current = time.monotonic() if now is None else now
        snapshot = self.snapshot()
        origin = (
            snapshot.received_monotonic
            if snapshot.sequence >= 0
            else self.started_monotonic
        )
        return max(0.0, current - origin)

    def run(self) -> None:
        bound_once = False
        while not self.stop_event.is_set():
            sock: socket.socket | None = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._socket = sock
                sock.settimeout(0.1)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.bind((self.host, self.port))
                bound_once = True
                self.ready.set()
                while not self.stop_event.is_set():
                    try:
                        data, address = sock.recvfrom(16_384)
                    except socket.timeout:
                        continue
                    if data == DISCOVERY_REQUEST:
                        sock.sendto(DISCOVERY_RESPONSE, address)
                        continue
                    with self._lock:
                        link_was_stale = (
                            time.monotonic() - self._snapshot.received_monotonic
                            > TRACKING_TIMEOUT_SECONDS
                        )
                        # An automatically discovered Quest may obtain a new IP
                        # after Wi-Fi/USB recovery.  Accept it only after the old
                        # stream is demonstrably stale; explicit allowed_sender
                        # remains pinned.
                        if (
                            self.allowed_sender is None
                            and link_was_stale
                            and self._latched_sender != address[0]
                        ):
                            self._latched_sender = address[0]
                    if self._latched_sender is not None and address[0] != self._latched_sender:
                        self.invalid_packets += 1
                        continue
                    try:
                        candidate = decode_datagram(data)
                    except (UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError, ValueError):
                        self.invalid_packets += 1
                        continue
                    with self._lock:
                        # Quest app restart resets sequence to zero.  Once the
                        # previous stream is stale, accept the first recovered
                        # packet immediately instead of forcing a script restart.
                        can_reset_sequence = (
                            candidate.received_monotonic - self._snapshot.received_monotonic
                            > TRACKING_TIMEOUT_SECONDS
                        )
                        if candidate.sequence > self._snapshot.sequence or can_reset_sequence:
                            if self._latched_sender is None:
                                self._latched_sender = address[0]
                            self._snapshot = candidate
            except OSError as exc:
                self.last_socket_error = exc
                if not bound_once:
                    self.error = exc
                    self.ready.set()
                    return
                self.socket_recoveries += 1
                self.stop_event.wait(0.20)
            except BaseException as exc:
                self.error = exc
                self.ready.set()
                return
            finally:
                if sock is not None:
                    sock.close()
                if self._socket is sock:
                    self._socket = None

    def close(self) -> None:
        self.stop_event.set()
        self.join(timeout=2.0)


class WorkflowButtonMapper:
    """Convert held controller buttons into one-shot workflow commands."""

    def __init__(self):
        self._held_since: dict[str, float] = {}
        self._latched: set[str] = set()
        self._stick_latched = False
        self._forward_start_position: tuple[float, float, float] | None = None
        self._forward_displacement: tuple[float, float, float] | None = None

    def consume_forward_displacement(self) -> tuple[float, float, float] | None:
        displacement = self._forward_displacement
        self._forward_displacement = None
        return displacement

    @staticmethod
    def _stick_direction(snapshot: QuestSnapshot) -> str | None:
        candidates = (snapshot.left.thumbstick_xy, snapshot.right.thumbstick_xy)
        x, y = max(candidates, key=lambda value: max(abs(value[0]), abs(value[1])))
        if max(abs(x), abs(y)) < 0.75:
            return None
        if abs(y) >= abs(x):
            return "up" if y > 0 else "down"
        return "right" if x > 0 else "left"

    @staticmethod
    def _stick_command(state: str, direction: str) -> str | None:
        mappings = {
            "采集阶段等待": {"up": "s", "down": "v", "left": "o", "right": "r"},
            "采集中": {"down": "e"},
            "审阅阶段": {"up": "a", "down": "x", "left": "p", "right": "b"},
        }
        return mappings.get(state, {}).get(direction)

    @staticmethod
    def _candidates(state: str, snapshot: QuestSnapshot) -> dict[str, tuple[bool, float]]:
        # Quest reserves the right Oculus button, so use a cross-controller
        # chord that OpenXR exposes on every Touch pair.  While held, suppress
        # the shorter individual X/B actions until the quit threshold expires.
        shutdown_chord = snapshot.left.primary and snapshot.right.secondary
        if state in ("采集阶段等待", "采集中", "审阅阶段") and shutdown_chord:
            return {"q": (True, QUIT_HOLD_SECONDS)}
        if state == "采集阶段等待":
            return {
                "c": (snapshot.left.menu, FORWARD_MOTION_CALIBRATION_SECONDS),
                "r": (snapshot.left.primary, WORKFLOW_HOLD_SECONDS),
                "o": (snapshot.left.secondary, WORKFLOW_HOLD_SECONDS),
                "s": (snapshot.right.primary, WORKFLOW_HOLD_SECONDS),
                "v": (snapshot.right.secondary, WORKFLOW_HOLD_SECONDS),
            }
        if state == "采集中":
            return {
                "e": (snapshot.right.secondary, WORKFLOW_HOLD_SECONDS),
            }
        if state == "审阅阶段":
            return {
                "x": (snapshot.left.primary, WORKFLOW_HOLD_SECONDS),
                "p": (snapshot.left.secondary, WORKFLOW_HOLD_SECONDS),
                "a": (snapshot.right.primary, WORKFLOW_HOLD_SECONDS),
                "b": (snapshot.right.secondary, WORKFLOW_HOLD_SECONDS),
            }
        return {}

    def update(self, state: str, snapshot: QuestSnapshot, now: float | None = None) -> str | None:
        current = time.monotonic() if now is None else now
        # Workflow transitions are forbidden while either arm is motion-enabled.
        if not snapshot.fresh(current) or snapshot.left.grip or snapshot.right.grip:
            self._held_since.clear()
            self._latched.clear()
            self._stick_latched = False
            self._forward_start_position = None
            return None
        if state == "采集阶段等待" and snapshot.left.menu:
            if self._forward_start_position is None:
                self._forward_start_position = snapshot.left.position
        else:
            self._forward_start_position = None
        stick_magnitude = max(
            abs(value)
            for controller in (snapshot.left, snapshot.right)
            for value in controller.thumbstick_xy
        )
        if stick_magnitude <= 0.35:
            self._stick_latched = False
        direction = self._stick_direction(snapshot)
        if direction is not None and not self._stick_latched:
            command = self._stick_command(state, direction)
            if command is not None:
                self._stick_latched = True
                self._held_since.clear()
                self._latched.clear()
                return command
        candidates = self._candidates(state, snapshot)
        active_names = {name for name, (active, _) in candidates.items() if active}
        for name in tuple(self._held_since):
            if name not in active_names:
                self._held_since.pop(name, None)
                self._latched.discard(name)
        for name, (active, duration) in candidates.items():
            if not active:
                self._held_since.pop(name, None)
                self._latched.discard(name)
                continue
            started = self._held_since.setdefault(name, current)
            if name not in self._latched and current - started >= duration:
                self._latched.add(name)
                if name == "c" and self._forward_start_position is not None:
                    self._forward_displacement = tuple(
                        current_value - start_value
                        for current_value, start_value in zip(
                            snapshot.left.position, self._forward_start_position
                        )
                    )
                return name
        return None


class QuestWorkflowInput:
    def __init__(self, receiver: Quest3Receiver):
        self.receiver = receiver
        self.mapper = WorkflowButtonMapper()

    def read(self, workflow_state: str) -> str:
        while True:
            if self.receiver.error is not None:
                raise RuntimeError(f"Quest receiver failed: {self.receiver.error}")
            command = self.mapper.update(workflow_state, self.receiver.snapshot())
            if command is not None:
                print(f"Quest workflow command: {command}", flush=True)
                return command
            time.sleep(0.02)
