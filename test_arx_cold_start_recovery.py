import ast
from pathlib import Path
import unittest


class ArxColdStartRecoveryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = Path("collect_workflow.py").read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.source)

    def method(self, name):
        node = next(
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.FunctionDef) and node.name == name
        )
        return ast.get_source_segment(self.source, node)

    def class_method(self, class_name, method_name):
        class_node = next(
            node
            for node in self.tree.body
            if isinstance(node, ast.ClassDef) and node.name == class_name
        )
        method_node = next(
            node
            for node in class_node.body
            if isinstance(node, ast.FunctionDef) and node.name == method_name
        )
        return ast.get_source_segment(self.source, method_node)

    def test_connect_does_not_destroy_session_for_actuation_probe(self):
        self.assertNotIn("verify_position_response", self.method("connect"))

    def test_workflow_startup_does_not_run_diagnostic_micro_motion(self):
        init = self.class_method("Workflow", "__init__")
        self.assertNotIn("self.arms.verify_position_response()", init)

    def test_quest_stack_starts_only_after_initial_safe_return(self):
        init = self.class_method("Workflow", "__init__")
        safe_return = init.index("self.safe_return_to_initial()")
        quest_start = init.index("self.start_quest_stack(", safe_return)
        self.assertLess(safe_return, quest_start)
        before_return = init[:safe_return]
        self.assertNotIn("Quest3Receiver(", before_return)
        self.assertNotIn("QuestTeleopController(", before_return)

    def test_every_safe_return_revokes_vr_before_robot_io(self):
        safe_return = self.class_method("Workflow", "safe_return_to_pose")
        revoke = safe_return.index("self.authority.disable_vr")
        recovery = safe_return.index("self.arms.wait_for_feedback_recovery")
        self.assertLess(revoke, recovery)

    def test_cleanup_fields_exist_before_active_verification(self):
        init = self.class_method("Workflow", "__init__")
        arm_acquisition = init.index(
            "self.arms = PersistentArms(expected_shutdown_pose=expected_shutdown_pose)"
        )
        for field in ("self.quest_receiver", "self.quest_input", "self.quest_teleop"):
            with self.subTest(field=field):
                self.assertLess(init.index(field), arm_acquisition)

    def test_cold_start_retries_same_sdk_session(self):
        verification = self.method("verify_position_response")
        self.assertIn("for attempt in range(1, attempts + 1)", verification)
        self.assertIn("self._verify_position_response_once()", verification)
        self.assertNotIn("self.close()", verification)

    def test_vendor_background_io_is_not_shadowed_by_application_heartbeat(self):
        connect = self.class_method("PersistentArms", "connect")
        close = self.class_method("PersistentArms", "close")
        persistent = self.source.split("class PersistentArms:", 1)[1].split(
            "class RobotRecorder", 1
        )[0]
        self.assertNotIn("start_life_support", connect)
        self.assertNotIn("def start_life_support", persistent)
        self.assertNotIn("life_support_thread", close)
        self.assertIn("self.command_gate", persistent)

    def test_all_persistent_session_mutations_use_command_gate(self):
        for method_name in (
            "set_side_command",
            "set_bimanual_commands",
            "set_teach_mode",
            "set_side_teach_mode",
            "set_side_control_mode",
            "enter_position_hold",
            "close",
        ):
            with self.subTest(method=method_name):
                method = self.class_method("PersistentArms", method_name)
                self.assertIn("self.command_gate", method)

    def test_diagnostic_micro_motion_also_uses_command_gate(self):
        method = self.class_method("PersistentArms", "_verify_position_response_once")
        first_write = method.index("self.right.set_joint_cmd(right_cmd)")
        gate = method.rfind("with self.command_gate, self.sdk_lock:", 0, first_write)
        self.assertGreaterEqual(gate, 0)

    def test_initialization_failure_reports_no_control_only_after_disconnect(self):
        main = self.method("main")
        cleanup = main.index("workflow.emergency_cleanup_partial()")
        disconnected = main.index("arms is None or not arms.connected", cleanup)
        confirmation = main.index(
            "write_no_control_acquired(args.shutdown_status_file)", disconnected
        )
        self.assertLess(cleanup, disconnected)
        self.assertLess(disconnected, confirmation)

    def test_initialization_failure_preserves_lifecycle_outcome(self):
        main = self.method("main")
        self.assertIn('== "operator_power_off"', main)
        self.assertIn('"OPERATOR_POWER_OFF_CONFIRMED"', main)
        self.assertIn('getattr(workflow, "shutdown_pose_reached", False)', main)
        self.assertIn("write_shutdown_complete", main)

    def test_vendor_failed_constructor_bypasses_only_finalizers(self):
        main = self.method("main")
        self.assertIn('"None of the motors are initialized" in str(exc)', main)
        self.assertIn("os._exit(1)", main)

    def test_ctrl_c_before_control_acquisition_can_escape_stuck_constructor(self):
        handler = self.method("request_safe_shutdown")
        connect = self.class_method("PersistentArms", "connect")
        close = self.class_method("PersistentArms", "close")
        main = self.method("main")
        self.assertIn("not _hardware_control_acquired", handler)
        self.assertIn("os._exit(130)", handler)
        self.assertIn("_hardware_control_acquired = True", connect)
        self.assertIn("_hardware_control_acquired = False", close)
        self.assertIn("RuntimeError, KeyboardInterrupt", main)

    def test_feedback_loss_requires_explicit_physical_power_off(self):
        safe_return = self.class_method("Workflow", "safe_return_to_pose")
        recovery = self.class_method("PersistentArms", "wait_for_feedback_recovery")
        cleanup = self.class_method("Workflow", "cleanup")
        partial_cleanup = self.class_method("Workflow", "emergency_cleanup_partial")
        confirmation = self.class_method("Workflow", "wait_for_operator_power_off")
        self.assertIn("wait_for_feedback_recovery", safe_return)
        self.assertIn("raise RobotFeedbackLost", recovery)
        self.assertIn("except RobotFeedbackLost", cleanup)
        self.assertIn("except RobotFeedbackLost", partial_cleanup)
        self.assertIn("self.wait_for_operator_power_off", partial_cleanup)
        self.assertIn("_power_off_confirmation_active = True", confirmation)
        self.assertIn("except KeyboardInterrupt", confirmation)
        self.assertIn("直接按 Enter", confirmation)
        self.assertIn("不再发送回归轨迹", confirmation)

    def test_safe_return_uses_five_second_same_session_recovery(self):
        recovery = self.class_method("PersistentArms", "wait_for_feedback_recovery")
        safe_return = self.class_method("Workflow", "safe_return_to_pose")
        self.assertIn("CAN_FATAL_FEEDBACK_TIMEOUT_SECONDS", recovery)
        self.assertIn("CAN_TRANSIENT_FEEDBACK_TIMEOUT_SECONDS", recovery)
        self.assertNotIn("set_bimanual_commands", recovery)
        self.assertNotIn("set_side_command", recovery)
        self.assertNotIn("set_joint_cmd", recovery)
        self.assertIn("沿用同一SDK会话", recovery)
        self.assertGreaterEqual(
            safe_return.count("self.arms.wait_for_feedback_recovery"), 2
        )
        self.assertIn("right_cmd.pos()[:] = resumed[:6]", safe_return)
        self.assertIn("left_cmd.pos()[:] = resumed[7:13]", safe_return)

    def test_preflight_is_read_only_after_official_damping_transition(self):
        connect = self.class_method("PersistentArms", "connect")
        preflight = self.class_method("PersistentArms", "preflight_at_shutdown")
        self.assertNotIn("self.set_teach_mode()", connect)
        self.assertNotIn("self.set_teach_mode()", preflight)
        self.assertNotIn("set_joint_cmd", preflight)
        self.assertNotIn("set_side_command", preflight)
        self.assertNotIn("get_timestamp", preflight)
        self.assertNotIn("fault(0.20)", preflight)
        self.assertIn("fault(10.0)", preflight)
        self.assertNotIn("settle_vendor_session", connect)
        self.assertNotIn("SDK构造后未收到初始CAN反馈", preflight)

    def test_active_feedback_is_proved_after_zero_displacement_hold(self):
        verifier = self.class_method("PersistentArms", "verify_active_feedback")
        hold = self.class_method("PersistentArms", "enter_position_hold")
        safe_return = self.class_method("Workflow", "safe_return_to_pose")
        self.assertIn("当前位置PID唤醒反馈失败", verifier)
        self.assertNotIn("set_joint_cmd", verifier)
        self.assertIn("self.verify_active_feedback", hold)
        self.assertIn("self.arms.verify_active_feedback", safe_return)


if __name__ == "__main__":
    unittest.main()
