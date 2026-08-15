#!/usr/bin/env python3
"""Exercise production PersistentArms motion with every peripheral disabled."""

import argparse
from pathlib import Path
import signal
import time

import numpy as np
import arx5_interface as arx5

from arx_common import ACTION_DIM, GRIPPER_WIDTH
from collect_workflow import PersistentArms, RobotFeedbackLost, Workflow
from control_authority import SafetyAuthority


ROOT = Path(__file__).resolve().parent
INITIAL_POSE_PATH = ROOT / "shared_poses" / "collection_initial_pose.npy"
SHUTDOWN_POSE_PATH = ROOT / "shared_poses" / "shutdown_pose.npy"
SHUTDOWN_BOOT_ID_PATH = ROOT / "shared_poses" / "shutdown_pose_boot_id.txt"
SYSTEM_BOOT_ID_PATH = Path("/proc/sys/kernel/random/boot_id")
JOINT_INDICES = np.asarray([*range(6), *range(7, 13)])
stop_requested = False


def request_stop(signum, _frame) -> None:
    global stop_requested
    stop_requested = True
    print(
        f"收到信号 {signum}：停止后续测试；当前原子运动结束后安全回停。",
        flush=True,
    )


def load_pose(path: Path, label: str) -> np.ndarray:
    pose = np.load(path).astype(np.float32)
    if pose.shape != (ACTION_DIM,) or not np.isfinite(pose).all():
        raise RuntimeError(f"{label}无效：{path}")
    return pose


def motion_harness(arms: PersistentArms):
    """Reuse the exact production safe-return implementation without Workflow I/O."""
    harness = Workflow.__new__(Workflow)
    harness.arms = arms
    harness.authority = SafetyAuthority()
    harness.state = "常驻会话运动测试"
    harness.show_status = lambda: None
    return harness


def verified_shutdown_error(arms: PersistentArms, shutdown: np.ndarray) -> float:
    current, _, _ = arms.state()
    return float(np.max(np.abs(current[JOINT_INDICES] - shutdown[JOINT_INDICES])))


def current_hold_commands(arms: PersistentArms):
    """Build a zero-displacement command pair for the single owning loop."""
    state, _, _ = arms.state()
    right_cmd = arx5.JointState(arms.right_robot.joint_dof)
    left_cmd = arx5.JointState(arms.left_robot.joint_dof)
    right_cmd.pos()[:] = state[:6]
    right_cmd.gripper_pos = float(state[6])
    left_cmd.pos()[:] = state[7:13]
    left_cmd.gripper_pos = float(state[13])
    return left_cmd, right_cmd


def confirm_physical_power_off() -> None:
    print(
        "反馈无法验证，软件不会释放SDK。请支撑双臂并关闭机械臂控制电源；"
        "断电完成后输入 off。",
        flush=True,
    )
    warned = False
    while True:
        try:
            answer = input("断电确认 > ")
            if answer == "":
                print("未确认断电；请输入 off，SDK对象继续保留。", flush=True)
                continue
            if answer.strip().lower() == "off":
                return
            print("未确认断电；请输入 off，SDK对象继续保留。", flush=True)
        except EOFError:
            # Hardware launchers bind stdin to /dev/tty.  If the controlling
            # terminal still disappears, EOF must never unwind the stack and
            # trigger vendor-controller destructors.
            if not warned:
                print("控制终端暂不可读；继续保留SDK并等待，不会因EOF退出。", flush=True)
                warned = True
            time.sleep(1.0)
        except KeyboardInterrupt:
            if not warned:
                print("仍未确认断电；SDK对象继续保留。", flush=True)
                warned = True
            time.sleep(1.0)


def wait_for_manual_shutdown(arms: PersistentArms, shutdown: np.ndarray) -> None:
    arms.set_teach_mode()
    print("自动回停失败但反馈有效；请在低阻尼下手动扶到停机位。", flush=True)
    settled = 0
    last_report = 0.0
    while settled < 20:
        error = verified_shutdown_error(arms, shutdown)
        settled = settled + 1 if error <= 0.08 else 0
        now = time.monotonic()
        if now - last_report >= 2.0:
            print(f"等待停机位：最大关节误差={error:.4f}rad", flush=True)
            last_report = now
        time.sleep(0.05)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--hold-seconds", type=float, default=30.0)
    parser.add_argument("--refresh-hz", type=float, default=50.0)
    args = parser.parse_args()
    if args.refresh_hz <= 0:
        raise ValueError("--refresh-hz must be positive")
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, request_stop)

    boot_id = SYSTEM_BOOT_ID_PATH.read_text(encoding="utf-8").strip()
    saved_boot_id = SHUTDOWN_BOOT_ID_PATH.read_text(encoding="utf-8").strip()
    if boot_id != saved_boot_id:
        raise RuntimeError("停机位不属于本次系统开机，拒绝运动测试")
    shutdown = load_pose(SHUTDOWN_POSE_PATH, "本次开机停机位")
    initial = load_pose(INITIAL_POSE_PATH, "统一采集初始位")
    initial[[6, 13]] = GRIPPER_WIDTH

    arms = None
    harness = None
    at_shutdown = False
    powered_off = False
    failure = None
    try:
        print("PERSISTENT_MOTION_CONNECT left=can0 then right=can1", flush=True)
        arms = PersistentArms(expected_shutdown_pose=shutdown)
        harness = motion_harness(arms)
        harness.safe_return_to_pose(shutdown, "进入停机位")
        at_shutdown = True
        # Revoke disconnect eligibility before issuing any command that leaves
        # the shutdown pose. If this motion fails halfway, the finally block
        # must not mistake the previous state for the current physical pose.
        at_shutdown = False
        harness.safe_return_to_pose(
            initial, "进入统一采集初始位", require_open_grippers=True
        )
        arms.enter_position_hold()
        left_hold_cmd, right_hold_cmd = current_hold_commands(arms)
        print(
            "控制模式切换：已离开低阻尼重力补偿，当前为双臂位姿PID闭环固定；"
            "这是本阶段需要验证的主动控制模式。",
            flush=True,
        )
        print(
            f"PERSISTENT_MOTION_HOLD_ACTIVE duration={args.hold_seconds:.0f}s "
            f"mode=single_owner_position_hold refresh={args.refresh_hz:.1f}Hz "
            "Quest=off cameras=off recording=off",
            flush=True,
        )
        started = time.monotonic()
        next_report = started
        next_command = started
        while time.monotonic() - started < args.hold_seconds:
            if stop_requested:
                print("测试提前停止，开始安全回停。", flush=True)
                break
            # This loop is the only command owner in this phase. Re-submit the
            # same zero-displacement target to test the cadence required by the
            # vendor controller; this is not a competing background heartbeat.
            arms.set_bimanual_commands(left_hold_cmd, right_hold_cmd)
            arms.wait_for_feedback_recovery("闭环保持阶段")
            now = time.monotonic()
            if now >= next_report:
                can_rx = arms.can_watchdog.snapshot()
                print(
                    f"PERSISTENT_MOTION_HOLD_HEALTHY elapsed={now-started:.1f}s "
                    f"left_rx={can_rx['left']} right_rx={can_rx['right']}",
                    flush=True,
                )
                next_report += 10.0
            next_command += 1.0 / args.refresh_hz
            delay = next_command - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_command = time.monotonic()
        print("PERSISTENT_MOTION_HOLD_COMPLETE", flush=True)
    except BaseException as exc:
        failure = exc
        print(f"PERSISTENT_MOTION_FAILURE {type(exc).__name__}: {exc}", flush=True)
    finally:
        if arms is not None and arms.connected:
            try:
                if harness is None:
                    harness = motion_harness(arms)
                harness.safe_return_to_pose(shutdown, "安全返回停机位")
                error = verified_shutdown_error(arms, shutdown)
                if error > 0.08:
                    raise RuntimeError(f"停机位验证误差={error:.4f}rad")
                at_shutdown = True
            except RobotFeedbackLost as exc:
                failure = failure or exc
                confirm_physical_power_off()
                powered_off = True
            except BaseException as exc:
                failure = failure or exc
                try:
                    wait_for_manual_shutdown(arms, shutdown)
                    at_shutdown = True
                except BaseException as manual_error:
                    failure = failure or manual_error
                    confirm_physical_power_off()
                    powered_off = True
            if at_shutdown:
                arms.set_teach_mode()
                arms.close()
                print("PERSISTENT_MOTION_DISCONNECTED_AT_SHUTDOWN", flush=True)
            elif powered_off:
                arms.close()
                print("PERSISTENT_MOTION_RELEASED_AFTER_POWER_OFF", flush=True)
            else:
                raise RuntimeError("未确认停机位或物理断电，拒绝释放SDK")
    if failure is not None:
        raise failure
    print("PERSISTENT_MOTION_TEST_COMPLETE", flush=True)


if __name__ == "__main__":
    main()
