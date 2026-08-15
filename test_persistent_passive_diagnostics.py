import ast
from pathlib import Path
import unittest


class PersistentPassiveDiagnosticsTest(unittest.TestCase):
    def test_finally_never_swallows_the_original_failure(self):
        source = Path("validate_persistent_session_passive.py").read_text(
            encoding="utf-8"
        )
        tree = ast.parse(source)
        main = next(
            node for node in tree.body
            if isinstance(node, ast.FunctionDef) and node.name == "main"
        )
        guarded = next(node for node in ast.walk(main) if isinstance(node, ast.Try))
        returns = [node for node in ast.walk(ast.Module(body=guarded.finalbody, type_ignores=[])) if isinstance(node, ast.Return)]
        self.assertEqual(returns, [])
        self.assertIn("WRAPPER_PASSIVE_FAILURE", source)
        self.assertIn("SDK时间戳", source)


if __name__ == "__main__":
    unittest.main()
