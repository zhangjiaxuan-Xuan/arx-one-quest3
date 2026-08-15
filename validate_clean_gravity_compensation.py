#!/usr/bin/env python3
"""Pre-VR ARX baseline: vendor SDK background I/O plus passive state reads only.

This intentionally does not import collect_workflow/PersistentArms and does not
send position commands, start Quest, or open cameras. A read-only kernel RX
watchdog is used because vendor state timestamps advance even after replies stop.
"""

from __future__ import annotations

import argparse
import time

import numpy as np
import arx5_interface as arx5

from can_rx_watchdog import CanRxWatchdog


GRIPPER_WIDTH = 0.082
GRIPPER_OPEN_READOUT = -3.4


class KernelFeedbackLost(RuntimeError):
    pass


def confirm_power_off() -> None:
    print(
        "真实CAN反馈已失效。请物理支撑双臂并关闭机械臂控制电源；"
        "断电完成后输入 off，之前不会释放SDK。",
        flush=True,
    )
    while True:
        try:
            if input("断电确认 > ").strip().lower() == "off":
                return
            print("未确认断电；请输入 off，SDK对象继续保留。", flush=True)
        except EOFError:
            print("控制终端暂不可读；继续保留SDK，不会因EOF退出。", flush=True)
            time.sleep(1.0)
        except KeyboardInterrupt:
            print("仍未确认断电；SDK对象继续保留。", flush=True)
            time.sleep(1.0)


def make_official_baseline_arm(interface: str):
    """Reproduce the original hand-collection controller construction."""
    robot = arx5.RobotConfigFactory.get_instance().get_config("X5")
    # Official README-required hardware adaptation for the reversed 2025
    # AC One gripper. No installed SDK files are modified.
    robot.gripper_open_readout = GRIPPER_OPEN_READOUT
    robot.gripper_width = GRIPPER_WIDTH
    controller = arx5.ControllerConfigFactory.get_instance().get_config(
        "joint_controller", robot.joint_dof
    )
    controller.background_send_recv = True
    controller.gravity_compensation = True
    controller.shutdown_to_passive = True
    arm = arx5.Arx5JointController(robot, controller, interface)
    arm.set_log_level(arx5.LogLevel.INFO)
    return arm


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=180.0)
    parser.add_argument("--observe-after-loss", type=float, default=10.0)
    args = parser.parse_args()
    if args.duration <= 0:
        raise ValueError("duration must be positive")

    left = None
    right = None
    feedback_lost = False
    first_fault = None
    loss_started = None
    fault_active = False
    watchdog = CanRxWatchdog()
    try:
        print("OFFICIAL_BASELINE_CONNECT left=can0 then right=can1", flush=True)
        left = make_official_baseline_arm("can0")
        right = make_official_baseline_arm("can1")

        # The controllers were constructed with gravity_compensation=True and
        # already start in damping. Do not immediately repeat set_to_damping:
        # the redundant transition is the variable isolated by this baseline.
        watchdog.reset()
        print(
            f"OFFICIAL_BASELINE_ACTIVE duration={args.duration:.0f}s "
            "Quest=off cameras=off watchdog=kernel_readonly "
            "heartbeat=off position_cmd=off startup_mode_write=off "
            f"observe_after_loss={args.observe_after_loss:.0f}s",
            flush=True,
        )

        started = time.monotonic()
        next_report = started
        while time.monotonic() - started < args.duration:
            fault = watchdog.fault(0.20)
            if fault is not None:
                feedback_lost = True
                if first_fault is None:
                    first_fault = fault
                    loss_started = time.monotonic()
                    print(
                        "OFFICIAL_BASELINE_RX_GAP_DETECTED："
                        f"{fault}；继续纯读取观察{args.observe_after_loss:.0f}秒，"
                        "不发送任何恢复或位置命令。",
                        flush=True,
                    )
                elif not fault_active:
                    print(f"OFFICIAL_BASELINE_RX_GAP_RECURRED：{fault}", flush=True)
                fault_active = True
            elif fault_active:
                fault_active = False
                can_rx = watchdog.snapshot()
                print(
                    "OFFICIAL_BASELINE_RX_RECOVERED："
                    f"left_rx={can_rx['left']} right_rx={can_rx['right']}",
                    flush=True,
                )
            left_state = left.get_joint_state()
            right_state = right.get_joint_state()
            arrays = (
                left_state.pos(), left_state.vel(), left_state.torque(),
                right_state.pos(), right_state.vel(), right_state.torque(),
            )
            scalars = (
                left_state.gripper_pos, left_state.gripper_vel,
                left_state.gripper_torque, right_state.gripper_pos,
                right_state.gripper_vel, right_state.gripper_torque,
            )
            if not all(np.isfinite(value).all() for value in arrays) or not all(
                np.isfinite(value) for value in scalars
            ):
                raise RuntimeError("官方SDK返回非有限状态")
            now = time.monotonic()
            if now >= next_report:
                elapsed = now - started
                can_rx = watchdog.snapshot()
                print(
                    f"OFFICIAL_BASELINE_HEALTHY elapsed={elapsed:.0f}s "
                    f"left_rx={can_rx['left']} right_rx={can_rx['right']}",
                    flush=True,
                )
                next_report = now + 10.0
            if (
                loss_started is not None
                and now - loss_started >= args.observe_after_loss
            ):
                can_rx = watchdog.snapshot()
                print(
                    "OFFICIAL_BASELINE_LOSS_OBSERVATION_COMPLETE："
                    f"left_rx={can_rx['left']} right_rx={can_rx['right']} "
                    f"currently_faulted={fault_active}",
                    flush=True,
                )
                break
            time.sleep(0.02)
        if feedback_lost:
            raise KernelFeedbackLost(
                "官方基线观察期内发生真实CAN反馈中断："
                f"{first_fault}"
            )
        print("OFFICIAL_BASELINE_DURATION_COMPLETE", flush=True)
    except KernelFeedbackLost as exc:
        print(f"OFFICIAL_BASELINE_FAILURE {exc}", flush=True)
        raise
    except KeyboardInterrupt:
        print("\n收到Ctrl-C：停止计时，保持官方阻尼模式。", flush=True)
    finally:
        if left is None and right is None:
            return
        if feedback_lost:
            confirm_power_off()
        else:
            try:
                for arm in (left, right):
                    if arm is not None:
                        arm.set_to_damping()
            except BaseException as exc:
                print(f"设置官方阻尼模式失败：{exc}", flush=True)
                confirm_power_off()
            else:
                print(
                    "真实CAN反馈正常，双臂保持官方低阻尼。"
                    "请手动扶到稳定停机位置，然后按Enter断开。",
                    flush=True,
                )
                while True:
                    try:
                        input("停机确认 > ")
                        break
                    except EOFError:
                        print("控制终端暂不可读；继续保留SDK。", flush=True)
                        time.sleep(1.0)
                    except KeyboardInterrupt:
                        print("请先稳定双臂，再按Enter确认。", flush=True)
        if right is not None:
            del right
        if left is not None:
            del left
        print("OFFICIAL_BASELINE_DISCONNECTED", flush=True)


if __name__ == "__main__":
    main()
