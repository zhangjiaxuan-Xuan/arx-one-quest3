#!/usr/bin/env python3
"""Hardware lifecycle validation: shutdown -> collection -> dropout -> shutdown."""

from dataclasses import replace
from pathlib import Path
import signal
import threading
import time

import numpy as np

from collect_workflow import (
    ACTION_DIM,
    FPS,
    PersistentArms,
    SYSTEM_BOOT_ID_PATH,
    GLOBAL_SHUTDOWN_BOOT_ID_PATH,
    GLOBAL_SHUTDOWN_POSE_PATH,
    COLLECTION_INITIAL_POSE_PATH,
)
from quest3_bimanual_test import return_to_start
from quest3_input import ControllerState, QuestSnapshot
from quest3_teleop import QuestTeleopController


ROOT = Path(__file__).resolve().parent
_cleanup_active = False


def request_stop(signum, frame):
    if not _cleanup_active:
        raise KeyboardInterrupt(f"received signal {signum}")


class SyntheticReceiver:
    def __init__(self):
        self._lock = threading.Lock()
        self._snapshot = QuestSnapshot()

    def snapshot(self) -> QuestSnapshot:
        with self._lock:
            return self._snapshot

    def set(self, snapshot: QuestSnapshot) -> None:
        with self._lock:
            self._snapshot = snapshot


def controller(tracking: bool = True) -> ControllerState:
    return ControllerState(
        position=(0.0, 0.0, 0.0),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        grip=True,
        trigger=0.0,
        tracking=tracking,
    )


def can_packets(interface: str, direction: str) -> int:
    return int(
        (Path("/sys/class/net") / interface / "statistics" / f"{direction}_packets")
        .read_text(encoding="ascii")
        .strip()
    )


def load_poses() -> tuple[np.ndarray, np.ndarray]:
    boot = SYSTEM_BOOT_ID_PATH.read_text(encoding="utf-8").strip()
    saved = GLOBAL_SHUTDOWN_BOOT_ID_PATH.read_text(encoding="utf-8").strip()
    if boot != saved:
        raise RuntimeError("停机姿态不属于本次系统开机")
    shutdown = np.load(GLOBAL_SHUTDOWN_POSE_PATH).astype(np.float32)
    initial = np.load(COLLECTION_INITIAL_POSE_PATH).astype(np.float32)
    for name, pose in (("shutdown", shutdown), ("initial", initial)):
        if pose.shape != (ACTION_DIM,) or not np.isfinite(pose).all():
            raise RuntimeError(f"{name} pose invalid: {pose.shape}")
    return shutdown, initial


def main() -> None:
    global _cleanup_active
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    shutdown, initial = load_poses()
    arms = teleop = None
    reached_shutdown = False
    try:
        print("[1/5] 停机位建立双臂常驻SDK并验证重力补偿", flush=True)
        arms = PersistentArms(expected_shutdown_pose=shutdown)
        arms.verify_position_response()
        current, _, _ = arms.state()
        delta = float(np.max(np.abs(initial - current)))
        if delta > 2.0:
            raise RuntimeError(f"初始位距离 {delta:.3f} rad 超过安全限制")

        print("[2/5] 安全移动到共享采集初始位", flush=True)
        return_to_start(arms, initial)

        print("[3/5] 模拟Quest整链路断开并自动恢复", flush=True)
        receiver = SyntheticReceiver()
        live = QuestSnapshot(
            sequence=100,
            received_monotonic=time.monotonic(),
            left=controller(),
            right=controller(),
        )
        receiver.set(live)
        teleop = QuestTeleopController(
            receiver,
            arms,
            lambda: "采集中",
            ROOT / "quest3_teleop_config.json",
            initial_pose=initial,
        )
        teleop.start()
        time.sleep(0.30)
        if not all(teleop.sessions[side].engaged for side in ("left", "right")):
            raise RuntimeError("双臂未进入Quest跟随控制模式")
        tx_before = (can_packets("can0", "tx"), can_packets("can1", "tx"))
        receiver.set(replace(live, received_monotonic=time.monotonic() - 2.0))
        time.sleep(0.70)
        tx_after = (can_packets("can0", "tx"), can_packets("can1", "tx"))
        if any(after <= before + 10 for before, after in zip(tx_before, tx_after)):
            raise RuntimeError(f"断链期间控制心跳不足：{tx_before} -> {tx_after}")
        if not all(teleop.sessions[side].engaged for side in ("left", "right")):
            raise RuntimeError("整链路断开导致控制会话退出")
        receiver.set(replace(live, sequence=0, received_monotonic=time.monotonic()))
        time.sleep(0.25)
        if not all(teleop.sessions[side].reconnect_count >= 1 for side in ("left", "right")):
            raise RuntimeError("整链路恢复后未自动重新锚定")

        print("[4/5] 模拟右手柄单侧追踪丢失并自动恢复", flush=True)
        receiver.set(
            QuestSnapshot(
                sequence=1,
                received_monotonic=time.monotonic(),
                left=controller(),
                right=controller(tracking=False),
            )
        )
        time.sleep(0.40)
        if not teleop.sessions["right"].engaged:
            raise RuntimeError("右手柄追踪丢失导致右臂控制会话退出")
        receiver.set(
            QuestSnapshot(
                sequence=2,
                received_monotonic=time.monotonic(),
                left=controller(),
                right=controller(),
            )
        )
        time.sleep(0.25)
        if teleop.sessions["right"].reconnect_count < 2:
            raise RuntimeError("右手柄恢复后未自动重新锚定")
        print(
            f"断链保持心跳通过：can0 +{tx_after[0]-tx_before[0]}，"
            f"can1 +{tx_after[1]-tx_before[1]}；双臂自动恢复",
            flush=True,
        )
    finally:
        _cleanup_active = True
        if teleop is not None:
            teleop.close()
        if arms is not None:
            while arms.connected:
                try:
                    print("[5/5] 安全回到本次开机停机位", flush=True)
                    return_to_start(arms, shutdown)
                    reached_shutdown = True
                    arms.set_teach_mode()
                    arms.close()
                except BaseException as exc:
                    try:
                        arms.set_teach_mode()
                    except BaseException:
                        pass
                    print(
                        f"拒绝断开：停机位尚未确认（{exc}）；保持重力补偿，2秒后重试",
                        flush=True,
                    )
                    time.sleep(2.0)
        if reached_shutdown:
            print("LIFECYCLE_VALIDATION_PASS：已确认停机位后断开SDK", flush=True)


if __name__ == "__main__":
    main()
