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


# ------------------------------------------------ posture re-checked at send time (security review, T5.9)

def _method(name, **globals_):
    """Compile ONE production method from the driver source, without its heavy imports."""
    import time
    import threading
    from go2_robot_sdk.actuator_gate_input import POSTURE_LABEL
    tree = ast.parse(DRIVER.read_text())
    node = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    module = ast.Module(body=[node], type_ignores=[])
    scope = {"time": time, "threading": threading, "POSTURE_LABEL": POSTURE_LABEL, **globals_}
    exec(compile(module, str(DRIVER), "exec"), scope)
    return scope[name]


class _Gate:
    def __init__(self, allowed):
        self.allowed = allowed

    def motion_allowed(self):
        return self.allowed if not callable(self.allowed) else self.allowed()


class _Driver:
    def __init__(self, gate):
        import threading
        self._gate_input = gate
        self._dc_send_lock = threading.Lock()
        self._dc_outbox, self._dc_delayed_outbox, self.sent, self.warnings = [], [], [], []

    def _send_dc_now(self, robot, payload):
        self.sent.append(payload)
        return True

    def get_logger(self):
        driver = self

        class _Log:
            def info(self, *_):
                pass

            def warning(self, message):
                driver.warnings.append(message)
        return _Log()


def test_a_posture_payload_queued_before_a_lock_is_dropped_at_send_time():
    import sys
    sys.path.insert(0, str(DRIVER.parents[1]))
    from go2_robot_sdk.actuator_gate_input import POSTURE_LABEL
    drain = _method("_drain_dc_outbox")
    driver = _Driver(_Gate(False))                      # the gate locked after the payload was queued
    driver._dc_outbox = [("0", "stand-up", POSTURE_LABEL), ("0", "video-on", None)]
    drain(driver, "0")
    assert driver.sent == ["video-on"] and driver.warnings


def test_the_wake_up_sequence_stops_when_the_gate_locks_between_its_steps():
    import sys
    sys.path.insert(0, str(DRIVER.parents[1]))
    steps = _method("_wake_up_steps")
    states = iter([True, False])                        # allowed for RecoveryStand, locked before BalanceStand
    driver = _Driver(_Gate(lambda: next(states)))
    sent = []
    driver._send_sport_cmd = lambda cmd, label=None: sent.append((cmd, label))
    steps(driver, sleep=lambda _s: None)
    assert [cmd for cmd, _ in sent] == ["RecoveryStand"] and driver.warnings
    assert sent[0][1] == "posture"


def test_legacy_mode_keeps_the_whole_wake_up_sequence():
    import sys
    sys.path.insert(0, str(DRIVER.parents[1]))
    steps = _method("_wake_up_steps")
    driver = _Driver(None)
    sent = []
    driver._send_sport_cmd = lambda cmd, label=None: sent.append(cmd)
    steps(driver, sleep=lambda _s: None)
    assert sent == ["RecoveryStand", "BalanceStand"]
