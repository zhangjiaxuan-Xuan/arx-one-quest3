#!/usr/bin/env python3
"""Tiny reversible two-arm actuation check; no Quest or camera processes."""

from pathlib import Path
import signal
import time

import numpy as np

from collect_workflow import (
    ACTION_DIM,
    GLOBAL_SHUTDOWN_BOOT_ID_PATH,
    GLOBAL_SHUTDOWN_POSE_PATH,
    PersistentArms,
    SYSTEM_BOOT_ID_PATH,
)


def main() -> None:
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, signal.SIG_IGN)
    boot_id = SYSTEM_BOOT_ID_PATH.read_text(encoding="utf-8").strip()
    saved_boot_id = GLOBAL_SHUTDOWN_BOOT_ID_PATH.read_text(encoding="utf-8").strip()
    if boot_id != saved_boot_id:
        raise RuntimeError("停机姿态不是本次系统开机记录，拒绝测试")
    shutdown = np.load(GLOBAL_SHUTDOWN_POSE_PATH).astype(np.float32)
    if shutdown.shape != (ACTION_DIM,) or not np.isfinite(shutdown).all():
        raise RuntimeError("停机姿态文件无效")
    arms = None
    try:
        arms = PersistentArms(expected_shutdown_pose=shutdown)
        arms.verify_position_response()
        rx_before = arms.can_watchdog.snapshot()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            fault = arms.health_fault()
            if fault is not None:
                raise RuntimeError(f"CAN反馈看门狗预检失败：{fault}")
            time.sleep(0.05)
        rx_after = arms.can_watchdog.snapshot()
        print(
            "CAN反馈看门狗预检通过："
            f"left +{rx_after['left'] - rx_before['left']}帧，"
            f"right +{rx_after['right'] - rx_before['right']}帧/1s，"
            "停滞阈值=0.20s",
            flush=True,
        )
        measured, _, _ = arms.state()
        indices = np.asarray([*range(6), *range(7, 13)])
        error = float(np.max(np.abs(measured[indices] - shutdown[indices])))
        if error > 0.08:
            raise RuntimeError(f"微动测试后停机位误差 {error:.4f}rad > 0.08rad")
        print(f"ARX_ACTUATION_PREFLIGHT_PASS shutdown_error={error:.4f}rad", flush=True)
    finally:
        if arms is not None and arms.connected:
            arms.set_teach_mode()
            arms.close()
            print("微动预检结束：已在停机位恢复低阻尼并断开SDK。", flush=True)


if __name__ == "__main__":
    main()
