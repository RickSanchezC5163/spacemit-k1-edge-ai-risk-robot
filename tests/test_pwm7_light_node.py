import importlib.util
from pathlib import Path


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "ros2_ws"
    / "src"
    / "k1_light_control"
    / "k1_light_control"
    / "pwm7_light_node.py"
)


def test_pwm7_brightness_is_binary_without_importing_ros() -> None:
    source = MODULE_PATH.read_text(encoding="utf-8")
    function_source = source.split("\n\nclass Pwm7LightNode", 1)[0]
    namespace: dict[str, object] = {}
    start = function_source.index("def action_for_brightness")
    exec(function_source[start:], namespace)
    action = namespace["action_for_brightness"]
    assert action(-1) == "off"
    assert action(0) == "off"
    assert action(1) == "on"
    assert action(100) == "on"
