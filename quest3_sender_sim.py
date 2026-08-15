#!/usr/bin/env python3
"""Send safe synthetic Quest packets for receiver/workflow testing (no Grip by default)."""

from __future__ import annotations

import argparse
import json
import socket
import time

from quest3_input import SCHEMA


def controller(primary=False, secondary=False, menu=False):
    return {
        "position_m": [0.0, 0.0, 0.0],
        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "tracking": True,
        "buttons": {
            "grip": False,
            "trigger": 0.0,
            "primary": primary,
            "secondary": secondary,
            "menu": menu,
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7447)
    parser.add_argument(
        "--command",
        choices=("idle", "register", "start", "review-stop", "restore", "quit"),
        default="idle",
    )
    parser.add_argument("--seconds", type=float, default=1.0)
    args = parser.parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    started = time.monotonic()
    sequence = 0
    while time.monotonic() - started < args.seconds:
        left = controller(
            primary=args.command == "register",
            secondary=args.command == "restore",
            menu=args.command == "quit",
        )
        right = controller(
            primary=args.command == "start",
            secondary=args.command == "review-stop",
            menu=args.command == "quit",
        )
        packet = {
            "schema": SCHEMA,
            "sequence": sequence,
            "client_time_ns": time.time_ns(),
            "left": left,
            "right": right,
        }
        sock.sendto(json.dumps(packet, separators=(",", ":")).encode(), (args.host, args.port))
        sequence += 1
        time.sleep(1.0 / 90.0)


if __name__ == "__main__":
    main()

