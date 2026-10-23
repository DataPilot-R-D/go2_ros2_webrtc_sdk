"""T5.9: in gate mode no raw topic reaches the Go2's motion (source boundary of go2_driver_node.py; offline).

The real driver imports WebRTC / CvBridge / rclpy, so its construction is checked from source: every subscription
is recorded with the actuator-mode branches that enclose it."""
import ast
from pathlib import Path

import pytest

DRIVER = Path(__file__).resolve().parents[1] / "go2_robot_sdk" / "go2_driver_node.py"
POSTURE = {"stand_up", "balance_stand", "stand_down", "recovery_stand", "wake_up"}


def _subscriptions():
    tree = ast.parse(DRIVER.read_text())
    found = []

    def visit(node, branches):
        if isinstance(node, ast.If):
            test = ast.unparse(node.test)
            for child in node.body:
                visit(child, branches + [(test, True)])
            for child in node.orelse:
                visit(child, branches + [(test, False)])
            return
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == "create_subscription":
            kind, topic, callback = (ast.unparse(a) for a in node.args[:3])
            found.append({"type": kind, "topic": topic.strip("'\""), "callback": callback, "branches": branches})
        for child in ast.iter_child_nodes(node):
            visit(child, branches)

    visit(tree, [])
    return found


def _in_gate_mode(sub):
    """Could this subscription be created when actuator_mode == 'gate'?"""
    for test, taken in sub["branches"]:
        if test == "self.actuator_mode == 'gate'" and not taken:
            return False
        if test == "self.actuator_mode == 'legacy'" and taken:
            return False
    return True


def test_gate_mode_has_exactly_the_gate_output_as_its_velocity_input():
    velocity = [s for s in _subscriptions() if s["type"] == "Twist" and _in_gate_mode(s)]
    assert [s["topic"] for s in velocity] == ["cmd_vel_mux"]


def test_the_raw_velocity_topics_and_the_joystick_exist_only_in_legacy_mode():
    raw = [s for s in _subscriptions() if s["type"] in ("Twist", "Joy") and s["topic"] != "cmd_vel_mux"]
    assert raw and not [s for s in raw if _in_gate_mode(s)], raw


def test_every_posture_command_goes_through_the_gate_check():
    posture = [s for s in _subscriptions() if s["topic"] in POSTURE]
    assert {s["topic"] for s in posture} == POSTURE
    assert all(s["callback"].startswith("lambda _msg: self._posture(") for s in posture), posture


def test_gate_mode_reads_the_gate_status():
    status = [s for s in _subscriptions() if s["topic"] == "cmd_vel_mux_status"]
    assert len(status) == 1 and _in_gate_mode(status[0]) and status[0]["type"] == "String"


def test_the_joystick_state_is_only_ever_set_by_its_callback():
    """joy_cmd's stand up / down branches read joy_state; in gate mode nothing may ever fill it."""
    tree = ast.parse(DRIVER.read_text())
    writers = [node for node in ast.walk(tree) if isinstance(node, ast.Assign)
               and any(ast.unparse(target) == "self.joy_state" for target in node.targets)]
    sources = sorted(ast.unparse(node.value) for node in writers)
    assert sources == ["Joy()", "msg"], sources
