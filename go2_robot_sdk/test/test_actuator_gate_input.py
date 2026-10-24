"""T5.9 / C4 Go2 actuator gate, driver side (offline: no ROS, no robot).

In gate mode the driver obeys ONLY the collision monitor's filtered command (cmd_vel_actuate), under the shared
StampedOutputGuard (infra/actuator/actuator_mux.py, the same core the simulator runs): the Safety latch with downstream
authority, source age and receipt TTL at use, strict stamps after a release. Posture uses GateLiveness on the gate's
own stamped stream (cmd_vel_gate) and the gate's fresh status."""
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

from actuator_mux import (COMMAND_FUTURE_TOLERANCE_S, COMMAND_MAX_AGE_S, COMMAND_TTL_S, GateLiveness,  # noqa: E402
                          StampedOutputGuard)
from go2_robot_sdk import actuator_gate_input as gate  # noqa: E402
from go2_robot_sdk.actuator_units import JoystickCalibration  # noqa: E402

# A unit-scale calibration measured in both directions: these tests exercise the guard / refresh / posture rules;
# the SI-to-axis conversion itself is test_actuator_units'.
UNIT = JoystickCalibration({"ly": 1.0, "lx": 1.0, "rx": 1.0}, {"ly": 1.0, "lx": 1.0, "rx": 1.0}, "test", "test", "0" * 64)

ZERO_STICKS = {"lx": 0, "ly": 0, "rx": 0, "ry": 0}


class Clock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t


class Clocks:
    """The ROS clock (the stamps' clock) and the driver's monotonic clock, advanced together."""
    def __init__(self):
        self.ros, self.mono = Clock(1000.0), Clock(100.0)

    def advance(self, dt):
        self.ros.t += dt
        self.mono.t += dt


def sticks(payload):
    message = json.loads(payload)
    assert message["topic"] == "rt/wirelesscontroller"
    return message["data"]


def make(c, released=True):
    timing = (COMMAND_TTL_S, COMMAND_MAX_AGE_S, COMMAND_FUTURE_TOLERANCE_S, c.ros)
    gi = gate.GateInput(StampedOutputGuard(*timing, clock=c.mono), GateLiveness(*timing, clock=c.mono), UNIT,
                        clock=c.mono)
    if released:
        gi.on_latch(False)
        c.advance(0.01)
    return gi


def drive(gi, c, x=0.0, y=0.0, z=0.0):
    assert gi.on_actuate(c.ros.t, x, y, z)


def test_a_never_started_monitor_is_an_explicit_zero():
    c = Clocks()
    assert sticks(make(c).payload_due()) == ZERO_STICKS


def test_a_never_heard_latch_refuses_every_command():
    c = Clocks()
    gi = make(c, released=False)
    assert not gi.on_actuate(c.ros.t, 0.3, 0.0, 0.0) and sticks(gi.payload_due()) == ZERO_STICKS


def test_a_fresh_filtered_command_drives_with_the_unitree_stick_mapping():
    c = Clocks()
    gi = make(c)
    drive(gi, c, 0.3, 0.1, 0.5)
    assert sticks(gi.payload_due()) == {"lx": -0.1, "ly": 0.3, "rx": -0.5, "ry": 0}


def test_a_monitor_that_dies_mid_motion_turns_into_an_explicit_zero():
    c = Clocks()
    gi = make(c)
    drive(gi, c, 0.3)
    assert sticks(gi.payload_due())["ly"] == 0.3
    c.advance(COMMAND_MAX_AGE_S + 0.01)                     # no filtered message since: its source age ran out
    assert sticks(gi.payload_due()) == ZERO_STICKS


def test_engaging_the_latch_erases_the_command_at_once():
    c = Clocks()
    gi = make(c)
    drive(gi, c, 0.3)
    gi.payload_due()
    gi.on_latch(True)
    assert sticks(gi.payload_due()) == ZERO_STICKS


def test_the_zero_itself_is_sent_not_only_a_timeout():
    c = Clocks()
    gi = make(c)
    drive(gi, c, 0.3)
    gi.payload_due()
    c.advance(0.05)
    drive(gi, c, 0.0)                                       # the monitor's or the gate's explicit zero
    assert sticks(gi.payload_due()) == ZERO_STICKS


def test_identical_commands_are_not_resent_every_tick_but_refreshed():
    """WebRTC data channel load (the Go2 starves cmd_vel under load): an unchanged command is resent only every
    REFRESH_S, so a lost message can never leave the robot on an old stick state for long."""
    c = Clocks()
    gi = make(c)
    drive(gi, c, 0.2)
    assert gi.payload_due() is not None
    c.advance(0.1); drive(gi, c, 0.2)
    assert gi.payload_due() is None                         # unchanged, refresh not due
    for _ in range(int(gate.REFRESH_S / 0.05)):
        c.advance(0.05); drive(gi, c, 0.2)
    assert gi.payload_due() is not None                     # refresh
    c.advance(0.05); drive(gi, c, 0.25)
    assert sticks(gi.payload_due())["ly"] == 0.25           # a change is sent at once


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


def test_the_driver_and_the_simulator_share_one_command_timing():
    """Codex #401: both consumers build their guards from actuator_mux.COMMAND_* - no second safety setting."""
    driver = (SDK_PKG / "go2_robot_sdk" / "go2_driver_node.py").read_text()
    assert "COMMAND_TTL_S, COMMAND_MAX_AGE_S, COMMAND_FUTURE_TOLERANCE_S" in driver
    bridge = MONOREPO / "repos" / "sim" / "ros2_bridge.py"
    if bridge.is_file():
        assert "StampedOutputGuard(COMMAND_TTL_S, COMMAND_MAX_AGE_S, COMMAND_FUTURE_TOLERANCE_S" in bridge.read_text()


def test_the_driver_never_subscribes_the_unfiltered_gate_output_or_raw_velocity():
    """C4 boundary (security review): in gate mode the only velocity is cmd_vel_actuate - never cmd_vel_mux."""
    import re
    driver = (SDK_PKG / "go2_robot_sdk" / "go2_driver_node.py").read_text()
    gate_block = driver[driver.index("if self.actuator_mode == 'gate':"):driver.index("# Legacy (default)")]
    topics = re.findall(r"create_subscription\(\s*\w+,\s*'([^']+)'", gate_block)
    assert topics == ["cmd_vel_actuate", "cmd_vel_gate", "cmd_vel_lock", "cmd_vel_mux_status"], topics
    assert "'cmd_vel_mux'," not in gate_block


def _status(locked_by, source=None):
    return json.dumps({"source": source, "locked_by": locked_by})


def alive(gi, c):
    assert gi.on_gate(c.ros.t, 0.0, 0.0, 0.0)


def test_posture_needs_a_live_gate_stream_and_its_fresh_unlocked_status():
    """stand_up / stand_down / wake_up move the body too: only while the gate's own stamped stream is live after the
    latch release and its fresh status says nothing is locked - never the monitor's output, never a status alone."""
    c = Clocks()
    gi = make(c)
    assert not gi.motion_allowed()                                  # nothing heard: refused
    alive(gi, c)
    gi.on_status(_status(["safety_latch"]))
    assert not gi.motion_allowed()
    gi.on_status(_status([]))
    assert gi.motion_allowed()


def test_an_idle_robot_keeps_posture_while_the_monitor_is_silent():
    """Codex #395: the monitor stops publishing zero after 2 s at rest; the live gate still authorises posture."""
    c = Clocks()
    gi = make(c)
    gi.on_status(_status([]))
    for _ in range(80):                                             # 4 s idle at 20 Hz, no filtered message at all
        c.advance(0.05); alive(gi, c)
        if _ % 20 == 0:
            gi.on_status(_status([]))
    assert gi.motion_allowed() and sticks(gi.payload_due()) == ZERO_STICKS


def test_an_unlocked_status_without_a_live_gate_stream_refuses_posture():
    """Codex #191: the status alone is not a live gate."""
    c = Clocks()
    gi = make(c)
    gi.on_status(_status([]))
    assert not gi.motion_allowed()


def test_a_gate_stream_that_goes_quiet_refuses_posture_while_its_status_is_still_young():
    c = Clocks()
    gi = make(c)
    alive(gi, c)
    gi.on_status(_status([]))
    assert gi.motion_allowed()
    c.advance(COMMAND_MAX_AGE_S + 0.01)                             # status still young (< STATUS_TTL_S)
    assert not gi.motion_allowed()


def test_an_old_gate_message_never_reauthorises_posture_after_an_unlock():
    """Codex #397: a pre-lock gate message, queued and delivered after the release, is refused."""
    c = Clocks()
    gi = make(c)
    old = c.ros.t
    alive(gi, c)
    gi.on_status(_status([]))
    c.advance(0.02); gi.on_latch(True)
    c.advance(0.02); gi.on_latch(False)
    assert not gi.on_gate(old, 0.0, 0.0, 0.0) and not gi.motion_allowed()


@pytest.mark.parametrize("raw", ["not json", "[]", json.dumps({"locked_by": "none"}), json.dumps({})])
def test_a_malformed_gate_status_refuses_posture(raw):
    c = Clocks()
    gi = make(c)
    alive(gi, c)
    gi.on_status(_status([]))
    gi.on_status(raw)
    assert not gi.motion_allowed()


def test_the_status_ttl_outlives_the_gate_status_interval():
    """The gate publishes its status on every change and at least once a second (repos/sim actuator_mux_node)."""
    assert gate.STATUS_TTL_S > 1.0
