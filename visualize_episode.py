#!/usr/bin/env python3
"""Synchronized desktop visualization for one collected episode."""

from __future__ import annotations

import argparse
from pathlib import Path
import socket
import struct
import subprocess
import time

import av
import cv2
import numpy as np
import arx5_interface as arx5


WIDTH = 1920
PANEL_WIDTH = 640
PANEL_HEIGHT = 360
PLAYBACK_SPEED = 1.5
DISPLAY_FPS = 30.0 * PLAYBACK_SPEED
GRIPPER_WIDTH = 0.082


class OptionalQuestStream:
    def __init__(self, enabled: bool, port: int = 10505):
        self.server = None
        self.client = None
        if not enabled:
            return
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("0.0.0.0", port))
        self.server.listen(1)
        self.server.setblocking(False)
        print(f"Quest审阅流已就绪，端口={port}；VR可连接或断开，不影响电脑回放。", flush=True)

    def send(self, frame: np.ndarray) -> None:
        if self.server is None:
            return
        if self.client is None:
            try:
                self.client, _ = self.server.accept()
                self.client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except BlockingIOError:
                return
        preview = cv2.resize(frame, (1920, 720), interpolation=cv2.INTER_AREA)
        ok, encoded = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        payload = encoded.tobytes()
        try:
            self.client.sendall(struct.pack("!I", len(payload)) + payload)
        except (BrokenPipeError, ConnectionResetError, OSError):
            self.client.close()
            self.client = None

    def close(self) -> None:
        if self.client is not None:
            self.client.close()
        if self.server is not None:
            self.server.close()


class FrameReader:
    def __init__(self, path: Path):
        self.container = av.open(str(path))
        self.stream = self.container.streams.video[0]
        self.frames = iter(self.container.decode(self.stream))
        self.index = -1
        self.image = None

    def get(self, target_index: int):
        if target_index < 0:
            return self.image
        while self.index < target_index:
            try:
                frame = next(self.frames)
            except StopIteration:
                break
            self.index += 1
            self.image = frame.to_ndarray(format="bgr24")
        return self.image

    def close(self):
        self.container.close()


def model_path() -> Path:
    module = Path(arx5.__file__).resolve()
    candidates = [
        module.parent.parent / "models" / "X5.urdf",
        module.parent / "models" / "X5.urdf",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError("X5.urdf not found beside arx5_interface")


def compute_xyz(joints: np.ndarray) -> np.ndarray:
    config = arx5.RobotConfigFactory.get_instance().get_config("X5")
    solver = arx5.Arx5Solver(
        str(model_path()),
        6,
        config.joint_pos_min,
        config.joint_pos_max,
    )
    return np.asarray(
        [solver.forward_kinematics(q.astype(np.float64))[:3] for q in joints],
        dtype=np.float32,
    )


def fit_xy(points: np.ndarray, origin: tuple[int, int], size: tuple[int, int]) -> np.ndarray:
    xy = points[:, :2].astype(np.float64)
    low = xy.min(axis=0)
    high = xy.max(axis=0)
    span = np.maximum(high - low, 0.02)
    pad = span * 0.12
    low -= pad
    high += pad
    width, height = size
    px = origin[0] + (xy[:, 0] - low[0]) / (high[0] - low[0]) * width
    py = origin[1] + height - (xy[:, 1] - low[1]) / (high[1] - low[1]) * height
    return np.column_stack([px, py]).astype(np.int32)


def put_text(image, text, position, scale=0.7, color=(255, 255, 255), thickness=2):
    cv2.putText(image, text, position, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


def camera_panel(image, label: str) -> np.ndarray:
    if image is None:
        panel = np.zeros((PANEL_HEIGHT, PANEL_WIDTH, 3), dtype=np.uint8)
        put_text(panel, "FRAME UNAVAILABLE", (180, 190), 0.8, (0, 0, 255))
    else:
        panel = cv2.resize(image, (PANEL_WIDTH, PANEL_HEIGHT), interpolation=cv2.INTER_AREA)
    cv2.rectangle(panel, (0, 0), (PANEL_WIDTH, 45), (0, 0, 0), -1)
    put_text(panel, label, (16, 31), 0.8)
    return panel


def trajectory_panel(
    left_xyz: np.ndarray,
    right_xyz: np.ndarray,
    left_gripper: np.ndarray,
    right_gripper: np.ndarray,
    current: int,
    total: int,
    task: str,
) -> np.ndarray:
    panel = np.full((PANEL_HEIGHT, WIDTH, 3), 22, dtype=np.uint8)
    margin = 48
    plot_w = PANEL_WIDTH - 2 * margin
    plot_h = PANEL_HEIGHT - 100
    left_pixels = fit_xy(left_xyz, (margin, 58), (plot_w, plot_h))
    right_pixels = fit_xy(right_xyz, (PANEL_WIDTH + margin, 58), (plot_w, plot_h))
    cv2.rectangle(panel, (margin, 58), (margin + plot_w, 58 + plot_h), (80, 80, 80), 1)
    cv2.rectangle(
        panel,
        (PANEL_WIDTH + margin, 58),
        (PANEL_WIDTH + margin + plot_w, 58 + plot_h),
        (80, 80, 80),
        1,
    )
    if current > 0:
        cv2.polylines(panel, [left_pixels[: current + 1]], False, (255, 170, 40), 3)
        cv2.polylines(panel, [right_pixels[: current + 1]], False, (40, 190, 255), 3)
    cv2.circle(panel, tuple(left_pixels[current]), 7, (255, 255, 255), -1)
    cv2.circle(panel, tuple(right_pixels[current]), 7, (255, 255, 255), -1)
    put_text(panel, "LEFT EEF XY TRAJECTORY", (margin, 35), 0.65, (255, 170, 40))
    put_text(panel, "RIGHT EEF XY TRAJECTORY", (PANEL_WIDTH + margin, 35), 0.65, (40, 190, 255))

    info_x = 2 * PANEL_WIDTH + 45
    put_text(panel, "ROBOT STATE", (info_x, 35), 0.75, (120, 255, 120))
    put_text(panel, f"Task: {task[:46]}", (info_x, 72), 0.58)
    put_text(panel, f"Frame: {current + 1}/{total}", (info_x, 105), 0.62)
    put_text(panel, "Left XYZ:  " + " ".join(f"{v:+.3f}" for v in left_xyz[current]), (info_x, 142), 0.57)
    put_text(panel, "Right XYZ: " + " ".join(f"{v:+.3f}" for v in right_xyz[current]), (info_x, 174), 0.57)

    for row, (name, value, color) in enumerate(
        [
            ("LEFT GRIPPER", left_gripper[current], (255, 170, 40)),
            ("RIGHT GRIPPER", right_gripper[current], (40, 190, 255)),
        ]
    ):
        y = 220 + row * 62
        fraction = float(np.clip(value / GRIPPER_WIDTH, 0.0, 1.0))
        put_text(panel, f"{name}: {value * 1000:.1f} mm", (info_x, y), 0.58, color)
        cv2.rectangle(panel, (info_x, y + 12), (info_x + 480, y + 32), (75, 75, 75), 1)
        cv2.rectangle(panel, (info_x, y + 12), (info_x + int(480 * fraction), y + 32), color, -1)

    progress = (current + 1) / total
    cv2.rectangle(panel, (45, 342), (WIDTH - 45, 353), (70, 70, 70), 1)
    cv2.rectangle(panel, (45, 342), (45 + int((WIDTH - 90) * progress), 353), (90, 220, 120), -1)
    return panel


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=Path)
    parser.add_argument("--quest-stream", action="store_true")
    args = parser.parse_args()
    episode = args.episode.resolve()
    raw = np.load(episode / "raw_demo.npz", allow_pickle=False)
    alignment = np.load(episode / "time_alignment.npz", allow_pickle=False)
    state = raw["observation_state"]
    robot_time = alignment["robot_timestamp_unix"]
    valid = alignment["model_valid_mask"].astype(bool)
    valid_indices = np.flatnonzero(valid)
    if len(valid_indices) < 2:
        raise RuntimeError("episode has fewer than two aligned model samples")

    target_times = np.arange(robot_time[valid_indices[0]], robot_time[valid_indices[-1]], 1.0 / DISPLAY_FPS)
    sample_indices = np.searchsorted(robot_time, target_times, side="left")
    sample_indices = np.clip(sample_indices, valid_indices[0], valid_indices[-1])
    sample_indices = np.unique(sample_indices)
    display_state = state[sample_indices]
    left_xyz = compute_xyz(display_state[:, 7:13])
    right_xyz = compute_xyz(display_state[:, :6])
    left_gripper = np.clip(display_state[:, 13], 0.0, GRIPPER_WIDTH)
    right_gripper = np.clip(display_state[:, 6], 0.0, GRIPPER_WIDTH)
    task = str(raw["task"])

    roles = ["left_arm_camera", "third_person_camera", "right_arm_camera"]
    labels = ["LEFT ARM CAMERA", "THIRD-PERSON CAMERA", "RIGHT ARM CAMERA"]
    readers = {role: FrameReader(episode / f"{role}.nut") for role in roles}
    player = subprocess.Popen(
        [
            "ffplay",
            "-hide_banner",
            "-loglevel",
            "warning",
            "-f",
            "rawvideo",
            "-pixel_format",
            "bgr24",
            "-video_size",
            f"{WIDTH}x{PANEL_HEIGHT * 2}",
            "-framerate",
            str(DISPLAY_FPS),
            "-window_title",
            "ARX AC One - 3 Views | EEF Trajectories | Grippers",
            "-autoexit",
            "-",
        ],
        stdin=subprocess.PIPE,
    )
    start_wall = time.monotonic()
    quest_stream = OptionalQuestStream(args.quest_stream)
    try:
        for display_index, robot_index in enumerate(sample_indices):
            images = []
            for role, label in zip(roles, labels):
                file_index = int(alignment[f"{role}_file_frame_index"][robot_index])
                images.append(camera_panel(readers[role].get(file_index), label))
            top = np.hstack(images)
            bottom = trajectory_panel(
                left_xyz,
                right_xyz,
                left_gripper,
                right_gripper,
                display_index,
                len(sample_indices),
                task,
            )
            canvas = np.vstack([top, bottom])
            quest_stream.send(canvas)
            try:
                player.stdin.write(canvas.tobytes())
            except BrokenPipeError:
                return 0
            deadline = start_wall + (display_index + 1) / DISPLAY_FPS
            delay = deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)
    finally:
        for reader in readers.values():
            reader.close()
        quest_stream.close()
        if player.stdin is not None:
            try:
                player.stdin.close()
            except BrokenPipeError:
                pass
        try:
            player.wait(timeout=3)
        except subprocess.TimeoutExpired:
            player.terminate()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
