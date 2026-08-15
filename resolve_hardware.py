#!/usr/bin/env python3
"""Resolve logical ARX AC One hardware roles from immutable USB serials."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path


ROOT = Path(__file__).resolve().parent
DEFAULT_REGISTRY = ROOT / "hardware_registry.json"


def _resolve_link(directory: Path, serial: str, suffix: str | None = None) -> str:
    matches = []
    if directory.exists():
        for link in directory.iterdir():
            if serial in link.name and (suffix is None or link.name.endswith(suffix)):
                matches.append(link)
    if len(matches) != 1:
        detail = "not found" if not matches else f"ambiguous: {[p.name for p in matches]}"
        raise RuntimeError(f"serial {serial} {detail} in {directory}")
    return str(matches[0].resolve())


def resolve(registry_path: Path) -> dict:
    registry = json.loads(registry_path.read_text())
    result = {"cameras": {}, "arms": {}, "excluded_camera_serials": registry.get("excluded_camera_serials", [])}

    claimed = set()
    for role, config in registry["cameras"].items():
        serial = config["serial"]
        if serial in claimed:
            raise RuntimeError(f"camera serial {serial} is registered more than once")
        claimed.add(serial)
        node = _resolve_link(Path("/dev/v4l/by-id"), serial, f"video-index{config.get('video_index', 0)}")
        result["cameras"][role] = {**config, "device": node}

    excluded = set(result["excluded_camera_serials"])
    overlap = claimed & excluded
    if overlap:
        raise RuntimeError(f"registered cameras are also excluded: {sorted(overlap)}")

    for role, config in registry["arms"].items():
        tty = _resolve_link(Path("/dev/serial/by-id"), config["canable_serial"])
        result["arms"][role] = {**config, "tty": tty, "can_exists": Path(f"/sys/class/net/{config['can_interface']}").exists()}
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--registry", type=Path, default=DEFAULT_REGISTRY)
    parser.add_argument("--shell", action="store_true", help="print shell-safe device exports")
    args = parser.parse_args()
    try:
        hardware = resolve(args.registry)
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"hardware resolution failed: {exc}", file=os.sys.stderr)
        return 1

    if args.shell:
        values = {
            "ARX_LEFT_CAMERA": hardware["cameras"]["left_arm_camera"]["device"],
            "ARX_RIGHT_CAMERA": hardware["cameras"]["right_arm_camera"]["device"],
            "ARX_THIRD_CAMERA": hardware["cameras"]["third_person_camera"]["device"],
            "ARX_LEFT_CAN_TTY": hardware["arms"]["left_arm"]["tty"],
            "ARX_RIGHT_CAN_TTY": hardware["arms"]["right_arm"]["tty"],
            "ARX_LEFT_CAN": hardware["arms"]["left_arm"]["can_interface"],
            "ARX_RIGHT_CAN": hardware["arms"]["right_arm"]["can_interface"],
        }
        for key, value in values.items():
            print(f"export {key}={value!r}")
    else:
        print(json.dumps(hardware, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
