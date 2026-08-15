#!/usr/bin/env python3
import argparse
import time

import numpy as np
from scipy.spatial.transform import Rotation

from quest3_input import Quest3Receiver


def delta(controller, anchor):
    if anchor is None:
        return np.zeros(3), np.zeros(3)
    p0, q0 = anchor
    dp = np.asarray(controller.position) - p0
    q = Rotation.from_quat(controller.orientation_xyzw)
    dr = (q * q0.inv()).as_rotvec(degrees=True)
    return dp, dr


def main():
    parser = argparse.ArgumentParser(description="Quest controller calibration monitor; never opens ARX hardware")
    parser.add_argument("--duration", type=float, default=90.0)
    args = parser.parse_args()
    receiver = Quest3Receiver()
    receiver.start()
    receiver.ready.wait(2)
    if receiver.error:
        raise RuntimeError(receiver.error)
    anchors = {"left": None, "right": None}
    was_grip = {"left": False, "right": False}
    started = time.monotonic()
    last_print = 0.0
    print("Grip按下沿锁定零点；按住并移动。单位：平移m，旋转deg。", flush=True)
    try:
        while time.monotonic() - started < args.duration:
            snap = receiver.snapshot()
            if snap.fresh():
                for side, state in (("left", snap.left), ("right", snap.right)):
                    if state.grip and not was_grip[side] and state.tracking:
                        anchors[side] = (
                            np.asarray(state.position, dtype=float),
                            Rotation.from_quat(state.orientation_xyzw),
                        )
                    if not state.grip:
                        anchors[side] = None
                    was_grip[side] = state.grip
                now = time.monotonic()
                if now - last_print >= 0.1:
                    values = []
                    for side, state in (("L", snap.left), ("R", snap.right)):
                        key = "left" if side == "L" else "right"
                        dp, dr = delta(state, anchors[key])
                        values.append(
                            f"{side}[G={int(state.grip)} d=({dp[0]:+.3f},{dp[1]:+.3f},{dp[2]:+.3f}) "
                            f"r=({dr[0]:+.1f},{dr[1]:+.1f},{dr[2]:+.1f})]"
                        )
                    print(" ".join(values), flush=True)
                    last_print = now
            time.sleep(0.005)
    finally:
        receiver.close()


if __name__ == "__main__":
    main()
