"""T5.9 Go2 actuator gate, driver side (offline: no ROS, no robot).

In gate mode the driver obeys ONLY the shared gate's output (cmd_vel_mux). Priority, the Safety latch and the
Bridge watchdog live in the gate (infra/actuator/actuator_mux.py, the same arbitration the simulator runs); the
driver guards the gate output with its own monotonic TTL so a dead or never-started gate is an explicit zero."""
import json
from pathlib import Path
import sys

import pytest

SDK_PKG = Path(__file__).resolve().parents[1]
MONOREPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(SDK_PKG))
CANONICAL = MONOREPO / "infra" / "actuator"
if not (CANONICAL / "actuator_mux.py").is_file():
    pytest.skip("monorepo infra/actuator not present", allow_module_level=True)
sys.path.insert(0, str(CANONICAL))

from actuator_mux import OutputGuard  # noqa: E402
from go2_robot_sdk import actuator_gate_input as gate  # noqa: E402

ZERO_STICKS = {"lx": 0, "ly": 0, "rx": 0, "ry": 0}


class Clock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t


def sticks(payload):
    message = json.loads(payload)
    assert message["topic"] == "rt/wirelesscontroller"
    return message["data"]


def make(clock):
    return gate.GateInput(OutputGuard(gate.GATE_OUTPUT_TTL_S, clock=clock), clock=clock)


def test_a_never_started_gate_is_an_explicit_zero():
    clock = Clock()
    assert sticks(make(clock).payload_due()) == ZERO_STICKS


def test_a_fresh_gate_output_drives_with_the_unitree_stick_mapping():
    clock = Clock()
    gi = make(clock)
    gi.on_twist(0.3, 0.1, 0.5)
    assert sticks(gi.payload_due()) == {"lx": -0.1, "ly": 0.3, "rx": -0.5, "ry": 0}


def test_a_gate_that_dies_mid_motion_turns_into_an_explicit_zero_at_its_ttl():
    clock = Clock()
    gi = make(clock)
    gi.on_twist(0.3, 0.0, 0.0)
    assert sticks(gi.payload_due())["ly"] == 0.3
    clock.t += gate.GATE_OUTPUT_TTL_S                     # no gate message since: exactly at the TTL
    assert sticks(gi.payload_due()) == ZERO_STICKS


def test_the_gate_zero_itself_is_sent_not_only_a_timeout():
    clock = Clock()
    gi = make(clock)
    gi.on_twist(0.3, 0.0, 0.0)
    gi.payload_due()
    clock.t += 0.05
    gi.on_twist(0.0, 0.0, 0.0)                            # the gate's explicit zero (latch, watchdog, halt)
    assert sticks(gi.payload_due()) == ZERO_STICKS


def test_identical_commands_are_not_resent_every_tick_but_refreshed():
    """WebRTC data channel load (the Go2 starves cmd_vel under load): an unchanged command is resent only every
    REFRESH_S, so a lost message can never leave the robot on an old stick state for long."""
    clock = Clock()
    gi = make(clock)
    gi.on_twist(0.2, 0.0, 0.0)
    assert gi.payload_due() is not None
    clock.t += 0.1
    gi.on_twist(0.2, 0.0, 0.0)
    assert gi.payload_due() is None                       # unchanged, refresh not due
    clock.t += gate.REFRESH_S
    gi.on_twist(0.2, 0.0, 0.0)
    assert gi.payload_due() is not None                   # refresh
    clock.t += 0.05
    gi.on_twist(0.25, 0.0, 0.0)
    assert sticks(gi.payload_due())["ly"] == 0.25         # a change is sent at once


def test_the_legacy_mapping_is_the_same_function():
    assert gate.joystick_payload(0.3, 0.1, 0.5) == gate.joystick_payload(0.3, 0.1, 0.5)
    assert sticks(gate.joystick_payload(0.0, 0.0, 0.0)) == ZERO_STICKS


@pytest.mark.parametrize("value, mode", [(None, "legacy"), ("legacy", "legacy"), ("gate", "gate")])
def test_the_actuator_mode_defaults_to_legacy(value, mode):
    environ = {} if value is None else {"GO2_ACTUATOR_MODE": value}
    assert gate.actuator_mode(environ) == mode


@pytest.mark.parametrize("value", ["", "GATE", "mux", "off"])
def test_an_unknown_actuator_mode_refuses_to_start(value):
    with pytest.raises(ValueError):
        gate.actuator_mode({"GO2_ACTUATOR_MODE": value})


def test_the_driver_ttl_is_the_simulator_consumer_ttl():
    """One consumer rule for both environments (repos/sim ros2_bridge MUX_COMMAND_TIMEOUT_S)."""
    import re
    bridge = MONOREPO / "repos" / "sim" / "ros2_bridge.py"
    if not bridge.is_file():
        pytest.skip("sim submodule not checked out")
    sim_ttl = float(re.search(r"^MUX_COMMAND_TIMEOUT_S = ([0-9.]+)", bridge.read_text(), re.M).group(1))
    assert gate.GATE_OUTPUT_TTL_S == sim_ttl


def _status(locked_by, source=None):
    return json.dumps({"source": source, "locked_by": locked_by})


def test_posture_commands_need_a_fresh_gate_status_without_any_lock():
    """stand_up / stand_down / wake_up move the body too: in gate mode only while the gate itself says nothing is
    locked (Safety latch released, Bridge alive), from its own fresh status - never on a raw topic alone."""
    clock = Clock()
    gi = make(clock)
    assert not gi.motion_allowed()                                  # no gate status ever: refused
    gi.on_twist(0.0, 0.0, 0.0)
    gi.on_status(_status(["safety_latch"]))
    assert not gi.motion_allowed()
    gi.on_status(_status([]))
    assert gi.motion_allowed()
    clock.t += gate.STATUS_TTL_S                                    # the gate went quiet: refused again
    assert not gi.motion_allowed()


def test_an_unlocked_status_without_any_gate_output_refuses_posture():
    """Codex #191: the status alone is not a live gate."""
    clock = Clock()
    gi = make(clock)
    gi.on_status(_status([]))
    assert not gi.motion_allowed()


def test_a_gate_output_that_expires_while_its_status_is_still_young_refuses_posture():
    """Codex #191: a stopped gate cannot authorize stand / wake for the rest of the status TTL."""
    clock = Clock()
    gi = make(clock)
    gi.on_twist(0.0, 0.0, 0.0)
    gi.on_status(_status([]))
    assert gi.motion_allowed()
    clock.t += gate.GATE_OUTPUT_TTL_S                               # status still young (< STATUS_TTL_S)
    assert clock.t - 100.0 < gate.STATUS_TTL_S and not gi.motion_allowed()


@pytest.mark.parametrize("raw", ["not json", "[]", json.dumps({"locked_by": "none"}), json.dumps({})])
def test_a_malformed_gate_status_refuses_posture(raw):
    clock = Clock()
    gi = make(clock)
    gi.on_status(_status([]))
    gi.on_status(raw)
    assert not gi.motion_allowed()


def test_the_status_ttl_outlives_the_gate_status_interval():
    """The gate publishes its status on every change and at least once a second (repos/sim actuator_mux_node)."""
    assert gate.STATUS_TTL_S > 1.0
