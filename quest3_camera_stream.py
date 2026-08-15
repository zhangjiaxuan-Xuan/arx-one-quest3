#!/usr/bin/env python3
"""Low-latency three-camera Quest preview using the recording FFmpeg path."""

import argparse
from concurrent.futures import ThreadPoolExecutor
from io import BytesIO
import os
import signal
import socket
import struct
import subprocess
import threading
import time

import numpy as np
from PIL import Image

from resolve_hardware import resolve, DEFAULT_REGISTRY


ORDER = ("left_arm_camera", "third_person_camera", "right_arm_camera")
WIDTH = 640
HEIGHT = 360
FRAME_BYTES = WIDTH * HEIGHT * 3


def request_stop(signum, _frame):
    """Route parent shutdown through main's camera cleanup path."""
    signal.signal(signum, signal.SIG_IGN)
    raise KeyboardInterrupt(f"camera publisher stop signal {signum}")


def open_camera(role, config, input_profile="registered"):
    device = str(config["device"])
    if input_profile == "preview-low":
        input_width, input_height, input_fps = 640, 480, 30
    else:
        input_width = int(config["width"])
        input_height = int(config["height"])
        input_fps = int(config["fps"])
    command = [
        "ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "warning",
        "-thread_queue_size", str(config.get("buffer_frames", 1)),
        "-f", "v4l2", "-input_format", "mjpeg",
        "-video_size", f"{input_width}x{input_height}",
        "-framerate", str(input_fps), "-i", device,
        "-an", "-vf",
        f"scale={WIDTH}:{HEIGHT}:force_original_aspect_ratio=decrease,"
        f"pad={WIDTH}:{HEIGHT}:(ow-iw)/2:(oh-ih)/2:black",
        "-c:v", "rawvideo", "-pix_fmt", "bgr24", "-f", "rawvideo", "pipe:1",
    ]
    process = subprocess.Popen(
        command,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    print(
        f"摄像机FFmpeg启动: {role}={device} "
        f"input={input_width}x{input_height}@{input_fps} profile={input_profile}",
        flush=True,
    )
    return process


def latest_reader(role, process, slot, stop):
    assert process.stdout is not None
    while not stop.is_set():
        payload = process.stdout.read(FRAME_BYTES)
        if len(payload) != FRAME_BYTES:
            detail = ""
            if process.stderr is not None:
                detail = process.stderr.read().decode("utf-8", errors="replace").strip()
            slot[0] = RuntimeError(
                f"{role} FFmpeg stopped (code={process.poll()}): {detail[-1000:]}"
            )
            return
        slot[0] = np.frombuffer(payload, dtype=np.uint8).reshape(HEIGHT, WIDTH, 3).copy()


def stop_camera(process):
    """Best-effort, idempotent FFmpeg teardown that never masks robot exit."""
    if process.poll() is not None:
        return True
    # This process owns preview-only raw pipes, not episode containers. There
    # is no muxer trailer to preserve, so terminate promptly. The three camera
    # processes are stopped concurrently by stop_cameras().
    stages = (
        (signal.SIGTERM, 0.5, "SIGTERM"),
        (signal.SIGKILL, 3.0, "SIGKILL"),
    )
    for signum, timeout, label in stages:
        try:
            os.killpg(process.pid, signum)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=timeout)
            return True
        except subprocess.TimeoutExpired:
            if signum == signal.SIGKILL:
                print(
                    f"摄像机FFmpeg在{label}后{timeout:.0f}秒仍未退出，升级清理。",
                    flush=True,
                )
    # A V4L2 process can remain in uninterruptible kernel sleep even after
    # SIGKILL. Do not turn that peripheral condition into a traceback after
    # the robot has already completed its verified shutdown.
    print(
        f"警告：摄像机FFmpeg pid={process.pid} 在SIGKILL后仍未回收；"
        "忽略外设清理超时，机械臂安全状态不受影响。",
        flush=True,
    )
    return False


def stop_cameras(processes):
    """Bound total three-camera cleanup latency to one parallel timeout path."""
    values = list(processes)
    if not values:
        return []
    with ThreadPoolExecutor(max_workers=len(values)) as executor:
        return list(executor.map(stop_camera, values))


def encode_panorama(frames, quality):
    panorama = np.hstack(frames)
    output = BytesIO()
    Image.fromarray(panorama[:, :, ::-1]).save(output, format="JPEG", quality=quality)
    return output.getvalue()


def main():
    for signum in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, request_stop)
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=10505)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--quality", type=int, default=80)
    parser.add_argument(
        "--input-profile",
        choices=("registered", "preview-low"),
        default="registered",
        help="preview-low reads 640x480 MJPEG30 without changing capture registry",
    )
    parser.add_argument(
        "--snapshot", type=str, default="",
        help="write one verified three-camera panorama and exit without opening a TCP port",
    )
    parser.add_argument("--snapshot-timeout", type=float, default=10.0)
    args = parser.parse_args()
    hardware = resolve(DEFAULT_REGISTRY)
    cameras = {
        role: open_camera(role, hardware["cameras"][role], args.input_profile)
        for role in ORDER
    }
    slots = {role: [None] for role in ORDER}
    stop = threading.Event()
    threads = [
        threading.Thread(
            target=latest_reader, args=(role, cameras[role], slots[role], stop), daemon=True
        )
        for role in ORDER
    ]
    for thread in threads:
        thread.start()
    if args.snapshot:
        deadline = time.monotonic() + args.snapshot_timeout
        try:
            while time.monotonic() < deadline:
                values = [slots[role][0] for role in ORDER]
                failure = next(
                    (value for value in values if isinstance(value, BaseException)), None
                )
                if failure is not None:
                    raise failure
                if all(isinstance(value, np.ndarray) for value in values):
                    panorama = np.hstack(values)
                    Image.fromarray(panorama[:, :, ::-1]).save(
                        args.snapshot, format="JPEG", quality=args.quality
                    )
                    stats = ", ".join(
                        f"{role}:mean={float(frame.mean()):.1f},std={float(frame.std()):.1f}"
                        for role, frame in zip(ORDER, values)
                    )
                    if any(float(frame.std()) < 2.0 for frame in values):
                        raise RuntimeError(f"blank or near-constant camera frame: {stats}")
                    print(
                        f"三视角拼图验证通过: {args.snapshot} "
                        f"size={panorama.shape[1]}x{panorama.shape[0]} | {stats}",
                        flush=True,
                    )
                    return
                time.sleep(0.02)
            raise RuntimeError("timed out waiting for all three camera frames")
        finally:
            stop.set()
            stop_cameras(cameras.values())
    server = None
    try:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", args.port))
        server.listen(1)
        print(
            f"VR三相机预览等待连接，端口={args.port} 顺序=左臂|第三视角|右臂",
            flush=True,
        )
        while True:
            values = [slots[role][0] for role in ORDER]
            failure = next((value for value in values if isinstance(value, BaseException)), None)
            if failure is not None:
                raise failure
            server.settimeout(0.2)
            try:
                client, address = server.accept()
            except socket.timeout:
                continue
            client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"Quest视觉已连接: {address[0]}", flush=True)
            try:
                next_frame = time.monotonic()
                while True:
                    values = [slots[role][0] for role in ORDER]
                    failure = next(
                        (value for value in values if isinstance(value, BaseException)), None
                    )
                    if failure is not None:
                        raise failure
                    if any(value is None for value in values):
                        time.sleep(0.01)
                        continue
                    payload = encode_panorama(values, args.quality)
                    client.sendall(struct.pack("!I", len(payload)) + payload)
                    next_frame += 1.0 / args.fps
                    time.sleep(max(0.0, next_frame - time.monotonic()))
            except (BrokenPipeError, ConnectionResetError, OSError):
                print("Quest视觉断开，等待自动重连", flush=True)
            finally:
                client.close()
    finally:
        stop.set()
        if server is not None:
            server.close()
        stop_cameras(cameras.values())


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # Normal parent-requested preview shutdown. Camera cleanup has already
        # completed in main()'s finally block; do not emit a false traceback.
        pass
