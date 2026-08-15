#!/usr/bin/env python3
"""Validate the project SDK wrapper without Quest, cameras, or motion commands."""

from __future__ import annotations

import argparse
import time

import numpy as np

from arx_common import ACTION_DIM
from collect_workflow import (
    CAN_FATAL_FEEDBACK_TIMEOUT_SECONDS,
    GLOBAL_SHUTDOWN_BOOT_ID_PATH,
    GLOBAL_SHUTDOWN_POSE_PATH,
    SAFE_RETURN_COMPLETION_RAD,
    SYSTEM_BOOT_ID_PATH,
    PersistentArms,
)


JOINT_INDICES = np.asarray(list(range(6)) + list(range(7, 13)))


def load_current_boot_shutdown_pose() -> np.ndarray:
    if not GLOBAL_SHUTDOWN_POSE_PATH.is_file() or not GLOBAL_SHUTDOWN_BOOT_ID_PATH.is_file():
        raise RuntimeError("缺少本次开机停机姿态；请先运行正式维护流程注册")
    boot_id = SYSTEM_BOOT_ID_PATH.read_text(encoding="utf-8").strip()
    saved_boot_id = GLOBAL_SHUTDOWN_BOOT_ID_PATH.read_text(encoding="utf-8").strip()
    if saved_boot_id != boot_id:
        raise RuntimeError("停机姿态不是本次系统开机记录，拒绝加载SDK")
    pose = np.load(GLOBAL_SHUTDOWN_POSE_PATH).astype(np.float32)
    if pose.shape != (ACTION_DIM,) or not np.isfinite(pose).all():
        raise RuntimeError("停机姿态文件无效")
    return pose


def shutdown_error(arms: PersistentArms, expected: np.ndarray) -> float:
    state, _, _ = arms.state()
    return float(np.max(np.abs(state[JOINT_INDICES] - expected[JOINT_INDICES])))


def sdk_timestamps(arms: PersistentArms) -> tuple[float, float]:
    with arms.sdk_lock:
        return float(arms.left.get_timestamp()), float(arms.right.get_timestamp())


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=180.0)
    args = parser.parse_args()
    if args.duration <= 0:
        raise ValueError("duration must be positive")

    shutdown = load_current_boot_shutdown_pose()
    arms: PersistentArms | None = None
    feedback_valid = True
    try:
        print("WRAPPER_PASSIVE_CONNECT left=can0 then right=can1", flush=True)
        arms = PersistentArms(expected_shutdown_pose=shutdown)
        print(
            f"WRAPPER_PASSIVE_ACTIVE duration={args.duration:.0f}s "
            "Quest=off cameras=off app_heartbeat=off position_cmd=off",
            flush=True,
        )
        started = time.monotonic()
        next_report = started
        while time.monotonic() - started < args.duration:
            state, velocity, effort = arms.state()
            if not all(np.isfinite(value).all() for value in (state, velocity, effort)):
                raise RuntimeError("项目SDK会话返回非有限状态")
            fault = arms.health_fault(CAN_FATAL_FEEDBACK_TIMEOUT_SECONDS)
            if fault is not None:
                feedback_valid = False
                left_timestamp, right_timestamp = sdk_timestamps(arms)
                raise RuntimeError(
                    f"项目SDK会话CAN反馈失效：{fault}；"
                    f"SDK时间戳 left={left_timestamp:.3f} right={right_timestamp:.3f}"
                )
            now = time.monotonic()
            if now >= next_report:
                left_timestamp, right_timestamp = sdk_timestamps(arms)
                print(
                    f"WRAPPER_PASSIVE_HEALTHY elapsed={now - started:.0f}s "
                    f"shutdown_error={shutdown_error(arms, shutdown):.4f}rad "
                    f"left_state_t={left_timestamp:.3f} "
                    f"right_state_t={right_timestamp:.3f}",
                    flush=True,
                )
                next_report = now + 10.0
            time.sleep(0.02)
        print("WRAPPER_PASSIVE_DURATION_COMPLETE", flush=True)
    except KeyboardInterrupt:
        print("\n收到Ctrl-C：停止计时，不发送自动回归轨迹。", flush=True)
    except BaseException as exc:
        # An unexpected SDK/state failure means we cannot prove that another
        # mode write is safe. The finally block will require physical support
        # and power-off confirmation before releasing the SDK objects.
        feedback_valid = False
        print(
            f"WRAPPER_PASSIVE_FAILURE {type(exc).__name__}: {exc}",
            flush=True,
        )
        raise
    finally:
        if arms is not None and feedback_valid:
            try:
                arms.set_teach_mode()
            except BaseException as exc:
                feedback_valid = False
                print(f"无法确认厂商阻尼模式：{exc}", flush=True)

        if arms is not None and not feedback_valid:
            print(
                "反馈不可验证。请物理支撑双臂并关闭机械臂控制电源，"
                "然后直接按Enter释放SDK对象。",
                flush=True,
            )
            try:
                input("断电确认 > ")
            except (KeyboardInterrupt, EOFError):
                pass
            arms.close()
            print("WRAPPER_PASSIVE_DISCONNECTED_AFTER_POWER_OFF", flush=True)
        elif arms is not None:
            while True:
                error = shutdown_error(arms, shutdown)
                if error <= SAFE_RETURN_COMPLETION_RAD:
                    print(
                        f"已确认停机姿态，最大关节误差={error:.4f}rad；允许断开SDK。",
                        flush=True,
                    )
                    break
                print(
                    f"当前停机误差={error:.4f}rad。双臂保持厂商阻尼模式；"
                    "请手动扶回停机位置后按Enter重新检查。",
                    flush=True,
                )
                try:
                    input("停机检查 > ")
                except (KeyboardInterrupt, EOFError):
                    continue
            arms.close()
            print("WRAPPER_PASSIVE_DISCONNECTED_AT_SHUTDOWN", flush=True)


if __name__ == "__main__":
    main()
