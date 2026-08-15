import ast
from pathlib import Path
import unittest


class VrControlGainScopeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = Path("collect_workflow.py").read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.source)

    def constant(self, name):
        for node in self.tree.body:
            if isinstance(node, ast.Assign):
                if any(isinstance(target, ast.Name) and target.id == name for target in node.targets):
                    return ast.literal_eval(node.value)
        self.fail(f"missing {name}")

    def method_source(self, name):
        for node in ast.walk(self.tree):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name:
                return ast.get_source_segment(self.source, node)
        self.fail(f"missing method {name}")

    def test_authorized_vr_joint_scales(self):
        self.assertEqual(self.constant("VR_JOINT_KP_SCALE"), 0.75)
        self.assertEqual(self.constant("VR_JOINT_KD_SCALE"), 1.00)

    def test_gripper_scales_remain_conservative(self):
        self.assertEqual(self.constant("VR_GRIPPER_KP_SCALE"), 0.85)
        self.assertEqual(self.constant("VR_GRIPPER_KD_SCALE"), 0.60)

    def test_only_vr_control_transition_uses_new_scales(self):
        method = self.method_source("set_side_control_mode")
        self.assertIn("VR_JOINT_KP_SCALE", method)
        self.assertIn("VR_JOINT_KD_SCALE", method)
        source_without_method = self.source.replace(method, "")
        self.assertEqual(source_without_method.count("VR_JOINT_KP_SCALE"), 1)
        self.assertEqual(source_without_method.count("VR_JOINT_KD_SCALE"), 1)

    def test_grip_engagement_really_enters_vr_control_gain(self):
        teleop_source = Path("quest3_teleop.py").read_text(encoding="utf-8")
        teleop_tree = ast.parse(teleop_source)
        engage = next(
            node
            for node in ast.walk(teleop_tree)
            if isinstance(node, ast.FunctionDef) and node.name == "_engage"
        )
        method = ast.get_source_segment(teleop_source, engage)
        latch = method.index("self.arms.set_side_command(side, command)")
        raise_gain = method.index("self.arms.set_side_control_mode(side)")
        self.assertLess(latch, raise_gain)

    def test_vr_joint_slew_is_limited_to_point_75_rad_per_second(self):
        teleop_source = Path("quest3_teleop.py").read_text(encoding="utf-8")
        teleop_tree = ast.parse(teleop_source)
        value = next(
            ast.literal_eval(node.value)
            for node in teleop_tree.body
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name) and target.id == "MAX_JOINT_STEP_RAD"
                for target in node.targets
            )
        )
        self.assertEqual(value, 0.015)
        self.assertAlmostEqual(value * 50, 0.75)


if __name__ == "__main__":
    unittest.main()
