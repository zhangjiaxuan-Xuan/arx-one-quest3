#!/usr/bin/env python3
"""Reject unsafe camera/CAN concentration on one USB 2.0 root bus."""

from __future__ import annotations

from collections import Counter
from pathlib import Path
import re

from resolve_hardware import DEFAULT_REGISTRY, resolve


SYS = Path("/sys")


def usb_branch_for_device(device: str, sys_root: Path = SYS) -> tuple[int, int, float]:
    node = Path(device).resolve()
    if node.name.startswith("tty"):
        current = (sys_root / "class" / "tty" / node.name / "device").resolve()
    elif node.name.startswith("video"):
        current = (sys_root / "class" / "video4linux" / node.name / "device").resolve()
    else:
        raise RuntimeError(f"不支持的USB设备节点：{device}")
    for component in current.parts:
        match = re.fullmatch(r"(\d+)-(\d+)", component)
        if not match:
            continue
        bus = int(match.group(1))
        root_port = int(match.group(2))
        root_speed = sys_root / "bus" / "usb" / "devices" / f"usb{bus}" / "speed"
        speed = float(root_speed.read_text(encoding="ascii").strip())
        return bus, root_port, speed
    raise RuntimeError(f"无法确定USB根总线/根端口：{device}")


def main() -> None:
    hardware = resolve(DEFAULT_REGISTRY)
    cameras = {
        role: usb_branch_for_device(config["device"])
        for role, config in hardware["cameras"].items()
    }
    arms = {
        role: usb_branch_for_device(config["tty"])
        for role, config in hardware["arms"].items()
    }
    cameras_per_branch = Counter((bus, port) for bus, port, _ in cameras.values())
    unsafe = []
    for role, (bus, port, speed) in arms.items():
        camera_count = cameras_per_branch[(bus, port)]
        if speed <= 480.0 and camera_count >= 2:
            unsafe.append(
                f"{role}=Bus {bus}/Root Port {port}"
                f"({speed:.0f}M，与{camera_count}路相机共享同一分支)"
            )
    summary = "；".join(
        [
            "CAN " + ", ".join(
                f"{role}=Bus {bus}/Root Port {port}/{speed:.0f}M"
                for role, (bus, port, speed) in arms.items()
            ),
            "相机 " + ", ".join(
                f"{role}=Bus {bus}/Root Port {port}/{speed:.0f}M"
                for role, (bus, port, speed) in cameras.items()
            ),
        ]
    )
    if unsafe:
        raise SystemExit(
            "USB拓扑安全预检失败："
            + "，".join(unsafe)
            + "。三路MJPEG持续采集可能饿死同分支USB-CAN反馈；"
            "请把两只CANable移到不同的主机USB根端口分支，再启动。\n"
            + summary
        )
    print("USB拓扑安全预检通过：" + summary, flush=True)


if __name__ == "__main__":
    main()
