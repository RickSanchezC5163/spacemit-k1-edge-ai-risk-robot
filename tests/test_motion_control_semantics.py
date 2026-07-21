import ast
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from k1_calibrated_motion import list_semantics
from replay_k1_semantic_events import build_replay


class MotionSemanticTest(unittest.TestCase):
    def test_registry_uses_direct_odom_targets(self) -> None:
        semantics = list_semantics()

        self.assertEqual(len(semantics), 34)
        for semantic in semantics:
            self.assertEqual(semantic.requested_value, semantic.odom_cutoff)

    def test_replay_is_built_only_from_runtime_events(self) -> None:
        events = [
            {"wall_time": 10.0, "event": "semantic_start", "code": "FORWARD_20"},
            {
                "wall_time": 11.5,
                "event": "semantic_result",
                "semantic": "FORWARD_20",
                "result": "distance_reached",
                "progress": 0.2,
            },
        ]

        replay = build_replay(events)

        self.assertEqual(len(replay), 1)
        self.assertEqual(replay[0]["semantic"], "FORWARD_20")
        self.assertEqual(replay[0]["recorded_duration_s"], 1.5)


class SecurityFrameTest(unittest.TestCase):
    def test_security_enable_is_not_sent_from_control_tick(self) -> None:
        path = (
            ROOT
            / "ros2_ws"
            / "src"
            / "turn_on_wheeltec_robot"
            / "scripts"
            / "wheeltec_tank_base_safe.py"
        )
        tree = ast.parse(path.read_text(encoding="utf-8"))
        methods = {
            node.name: node
            for node in ast.walk(tree)
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        }

        control_tick = methods["control_tick"]
        control_calls = {
            node.func.id
            for node in ast.walk(control_tick)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        }
        startup_calls = [
            node
            for node in ast.walk(methods["__init__"])
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "make_security_frame"
        ]
        self.assertNotIn("make_security_frame", control_calls)
        self.assertNotIn("send_security_enable_frames", methods)
        self.assertEqual(len(startup_calls), 2)


if __name__ == "__main__":
    unittest.main()
