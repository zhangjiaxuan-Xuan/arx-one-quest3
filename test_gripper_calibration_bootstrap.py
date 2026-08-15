import ast
from pathlib import Path
import unittest


class GripperCalibrationBootstrapTest(unittest.TestCase):
    def test_normal_adapter_uses_per_interface_calibration(self):
        source = Path("arx_common.py").read_text(encoding="utf-8")
        self.assertIn("calibrated_gripper_open_readout(interface)", source)
        self.assertIn("arx.gripper_calibration.v1", source)

    def test_bootstrap_is_damping_only_and_uses_official_calibration(self):
        source = Path("calibrate_arx_grippers.py").read_text(encoding="utf-8")
        tree = ast.parse(source)
        self.assertIn("controller.gravity_compensation = False", source)
        self.assertIn("arm.set_to_damping()", source)
        self.assertIn("arm.calibrate_gripper()", source)
        self.assertNotIn("set_joint_cmd", source)
        self.assertIn("【第3步】", source)
        self.assertIn("CLOSED_WIDTH_TOLERANCE_M", source)
        self.assertIn('left, left_readout = calibrate_one("can0", "左臂")', source)
        self.assertIn('right, right_readout = calibrate_one("can1", "右臂")', source)
        self.assertIsNotNone(tree)

    def test_shutdown_capture_reuses_calibration_session(self):
        source = Path("calibrate_arx_grippers.py").read_text(encoding="utf-8")
        self.assertIn("capture_shutdown_pose(left, right)", source)
        self.assertNotIn("make_arm(", source)
        self.assertIn("[0.0]", source)

    def test_one_key_launcher_calibrates_before_collection(self):
        source = Path("tools/start_all_quest3_collection.sh").read_text(
            encoding="utf-8"
        )
        calibration = source.index("run_arx_gripper_calibration.sh")
        workflow = source.index("start_quest3_collection_test.sh")
        self.assertLess(calibration, workflow)
        self.assertIn('[[ ! -s "$GRIPPER_CALIBRATION"', source)
        self.assertIn("ARX_RECALIBRATE_GRIPPERS", source)
        self.assertIn("跳过电机零点重置", source)
        self.assertIn("关闭10秒后重新上电", source)
        self.assertIn('"$GRIPPER_CALIBRATION" -nt "$SHUTDOWN_POSE"', source)
        self.assertIn("把双臂摆到希望的停机位置", source)

    def test_calibration_emergency_has_bounded_operator_exit(self):
        source = Path("tools/run_arx_gripper_calibration.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("grep -q 'Emergency state entered'", source)
        self.assertIn("按Enter或Ctrl-C结束进程", source)
        self.assertIn('kill -KILL "$calibration_pid"', source)

    def test_startup_emergency_never_requests_blind_return(self):
        source = Path("tools/start_quest3_collection_test.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("grep -q 'Emergency state entered'", source)
        self.assertIn("禁止盲目自动回位", source)
        self.assertIn("OPERATOR_POWER_OFF_CONFIRMED", source)
        self.assertIn("trap 'emergency_poweroff_confirmed=1'", source)

    def test_new_calibration_invalidates_older_shutdown_pose(self):
        source = Path("collect_workflow.py").read_text(encoding="utf-8")
        self.assertIn("calibration_newer_than_shutdown", source)
        self.assertIn("GRIPPER_CALIBRATION_PATH.stat().st_mtime", source)


if __name__ == "__main__":
    unittest.main()
