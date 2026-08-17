import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import collect_workflow
from quest3_input import QUEST_SLEEP_SAFE_EXIT_SECONDS


class CollectionInputAuthorityTest(unittest.TestCase):
    def test_pose_registration_updates_live_quest_return_target(self):
        source = Path(collect_workflow.__file__).read_text(encoding="utf-8")
        register = source.split("    def register_pose(self) -> None:", 1)[1].split(
            "    def calibrate_quest_forward_motion", 1
        )[0]
        save = register.index("np.save(COLLECTION_INITIAL_POSE_PATH, self.initial_pose)")
        update = register.index("self.quest_teleop.update_initial_pose(self.initial_pose)")
        self.assertLess(save, update)

    def test_launcher_explicitly_preserves_terminal_input(self):
        launcher = (
            Path(__file__).parent / "tools" / "start_quest3_collection_test.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("exec {WORKFLOW_STDIN_FD}</dev/tty", launcher)
        self.assertIn('<&"$WORKFLOW_STDIN_FD"', launcher)

    def test_sleeping_quest_is_started_by_nonfatal_background_retry(self):
        launcher = (
            Path(__file__).parent / "tools" / "start_quest3_collection_test.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("start_quest_when_available &", launcher)
        self.assertIn("Quest当前休眠/离线", launcher)
        self.assertIn("trap - EXIT INT TERM HUP", launcher)
        retry_start = launcher.index("start_quest_when_available()")
        workflow_start = launcher.index('PYTHONUNBUFFERED=1 "$PYTHON"')
        background_start = launcher.index("start_quest_when_available &")
        self.assertLess(retry_start, workflow_start)
        self.assertLess(workflow_start, background_start)

    def test_usb_topology_is_checked_before_sdk_workflow(self):
        launcher = (
            Path(__file__).parent / "tools" / "start_quest3_collection_test.sh"
        ).read_text(encoding="utf-8")
        topology = launcher.index("preflight_arx_usb_topology.py")
        workflow = launcher.index("collect_workflow.py")
        self.assertLess(topology, workflow)

    def test_keyboard_is_polled_before_quest(self):
        workflow = Mock()
        workflow.state = "采集中"
        workflow.control_fault.return_value = None
        workflow.quest_input.mapper.update.return_value = "q"
        with (
            patch.object(collect_workflow.sys.stdin, "isatty", return_value=True),
            patch.object(collect_workflow.sys.stdin, "fileno", return_value=42),
            patch.object(collect_workflow.termios, "tcgetattr", return_value=["terminal"]),
            patch.object(collect_workflow.termios, "tcsetattr"),
            patch.object(collect_workflow.tty, "setcbreak"),
            patch.object(collect_workflow.select, "select", return_value=([collect_workflow.sys.stdin], [], [])),
            patch.object(collect_workflow.sys.stdin, "read", return_value="e"),
        ):
            command = collect_workflow.read_interactive_command(workflow, use_quest=True)
        self.assertEqual(command, "e")
        workflow.quest_input.mapper.update.assert_not_called()

    def test_stage_entry_does_not_flush_pending_keyboard_input(self):
        source = Path(collect_workflow.__file__).read_text(encoding="utf-8")
        function = source.split("def read_interactive_command", 1)[1].split("\ndef main", 1)[0]
        self.assertNotIn("tcflush", function)

    def test_quest_sleep_waits_three_minutes_then_requests_safe_exit(self):
        workflow = collect_workflow.Workflow.__new__(collect_workflow.Workflow)
        workflow.arms = Mock()
        workflow.arms.health_fault.return_value = None
        workflow.state = "采集阶段等待"
        workflow.quest_receiver = Mock()
        workflow.quest_receiver.silence_seconds.return_value = (
            QUEST_SLEEP_SAFE_EXIT_SECONDS
        )
        self.assertIsNone(workflow.control_fault())
        workflow.quest_receiver.silence_seconds.return_value = (
            QUEST_SLEEP_SAFE_EXIT_SECONDS + 0.001
        )
        self.assertIn("Quest连续休眠", workflow.control_fault())

    def test_quest_may_remain_off_during_review(self):
        workflow = collect_workflow.Workflow.__new__(collect_workflow.Workflow)
        workflow.arms = Mock()
        workflow.arms.health_fault.return_value = None
        workflow.state = "审阅阶段"
        workflow.quest_receiver = Mock()
        workflow.quest_receiver.silence_seconds.return_value = 999.0
        self.assertIsNone(workflow.control_fault())

    def test_can_feedback_loss_requires_five_continuous_seconds(self):
        workflow = collect_workflow.Workflow.__new__(collect_workflow.Workflow)
        workflow.arms = Mock()
        workflow.arms.health_fault.return_value = "right can1 stalled"
        workflow.state = "采集阶段等待"
        workflow.quest_receiver = None
        self.assertEqual(
            collect_workflow.CAN_FATAL_FEEDBACK_TIMEOUT_SECONDS,
            5.0,
        )
        self.assertEqual(workflow.control_fault(), "right can1 stalled")
        workflow.arms.health_fault.assert_called_once_with(5.0)

    def test_teleop_transport_loss_freezes_targets_without_sdk_writes(self):
        source = (Path(__file__).parent / "quest3_teleop.py").read_text(
            encoding="utf-8"
        )
        run = source.split("    def run(self) -> None:", 1)[1].split(
            "    def close(self) -> None:", 1
        )[0]
        stale = run.split("elif recording and self.arms.connected:", 1)[1].split(
            "elif recording:", 1
        )[0]
        self.assertIn('_hold_for_reconnect("left", "Quest 数据中断")', stale)
        self.assertIn('_hold_for_reconnect("right", "Quest 数据中断")', stale)
        self.assertNotIn("set_side_command", stale)
        self.assertNotIn("set_bimanual_commands", stale)
        self.assertNotIn("heartbeat", stale.lower())

    def test_teleop_fresh_frame_is_one_serialized_transaction(self):
        source = (Path(__file__).parent / "quest3_teleop.py").read_text(
            encoding="utf-8"
        )
        run = source.split("    def run(self) -> None:", 1)[1].split(
            "    def close(self) -> None:", 1
        )[0]
        fresh = run.split("if recording and snapshot.fresh()", 1)[1].split(
            "elif recording and self.arms.connected", 1
        )[0]
        self.assertIn("with self.arms.command_gate:", fresh)
        self.assertNotIn("_heartbeat_bimanual", fresh)

    def test_bimanual_commit_uses_historical_right_then_left_order(self):
        source = Path(collect_workflow.__file__).read_text(encoding="utf-8")
        method = source.split("    def set_bimanual_commands(", 1)[1].split(
            "    def health_fault", 1
        )[0]
        self.assertLess(
            method.index("self.right.set_joint_cmd(right_command)"),
            method.index("self.left.set_joint_cmd(left_command)"),
        )

    def test_persistent_session_has_no_application_heartbeat_thread(self):
        source = Path(collect_workflow.__file__).read_text(encoding="utf-8")
        persistent = source.split("class PersistentArms:", 1)[1].split(
            "class RobotRecorder", 1
        )[0]
        self.assertNotIn("start_life_support", persistent)
        self.assertNotIn("life_support_thread", persistent)
        self.assertNotIn("life_support_stop", persistent)

    def test_launcher_distinguishes_no_control_from_safe_shutdown(self):
        launcher = (
            Path(__file__).parent / "tools" / "start_quest3_collection_test.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("NO_CONTROL_ACQUIRED", launcher)
        self.assertIn("OPERATOR_POWER_OFF_CONFIRMED", launcher)
        self.assertIn("工作流从未获得机械臂控制权", launcher)
        self.assertIn("不要为此反复使用 --usb-reset", launcher)

    def test_slcan_recovery_uses_validated_stock_transport(self):
        recovery = (
            Path(__file__).parent / "tools" / "recover_arx_can.sh"
        ).read_text(encoding="utf-8")
        self.assertIn('slcand -o -f -s8 "$LEFT" can0', recovery)
        self.assertIn('slcand -o -f -s8 "$RIGHT" can1', recovery)
        self.assertNotIn("-S 3000000", recovery)
        self.assertNotIn("txqueuelen 1000", recovery)

    def test_can_preflight_preserves_serial_mapping_without_baud_assumption(self):
        source = (
            Path(__file__).parent / "arx_can_preflight.py"
        ).read_text(encoding="utf-8")
        self.assertIn("verify_canable_mapping(interface)", source)
        self.assertIn("CANABLE_SERIALS", source)
        self.assertNotIn("MINIMUM_SLCAN_BAUD", source)


if __name__ == "__main__":
    unittest.main()
