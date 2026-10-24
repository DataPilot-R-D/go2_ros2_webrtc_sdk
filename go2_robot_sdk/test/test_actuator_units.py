"""T5.9 SI-to-actuator units fence of the Go2 gate mode (Codex #228 / #237 / #242; offline, no robot).

The gate speaks SI (m/s, rad/s); the Go2's wireless-controller axes are normalised to [-1, 1]. Copying SI values
into axes is not a unit conversion, so autonomous gate mode needs a MEASURED calibration and refuses to start
without one; every command goes through it, and an axis that would leave [-1, 1] - or a direction that was never
measured (reverse) - is an explicit zero, counted, never silently clipped. Legacy mode is unchanged and distinct."""
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


def gate_input(cal, mono, ros):
    """A released C4 gate input (filtered-command guard + gate liveness) on the given clocks."""
    timing = (COMMAND_TTL_S, COMMAND_MAX_AGE_S, COMMAND_FUTURE_TOLERANCE_S, ros)
    gi = gate.GateInput(StampedOutputGuard(*timing, clock=mono), GateLiveness(*timing, clock=mono), cal, clock=mono)
    gi.on_latch(False)
    return gi
from go2_robot_sdk import actuator_gate_input as gate  # noqa: E402
from go2_robot_sdk import actuator_units as units  # noqa: E402

ZERO = {"lx": 0, "ly": 0, "rx": 0, "ry": 0}


def calibration(**axes):
    body = {"schema": units.SCHEMA, "status": "measured", "robot": {"serial": "B42D2000P4CAF505", "firmware": "1.1.15"},
            "measured_at": "2026-10-24T09:00:00Z", "method": "straight / spin runs, ground truth from mocap",
            "axes": {"ly": {"unit": "m/s", "positive_si_per_unit": 0.8},
                     "lx": {"unit": "m/s", "positive_si_per_unit": 0.4, "negative_si_per_unit": 0.4},
                     "rx": {"unit": "rad/s", "positive_si_per_unit": 1.5, "negative_si_per_unit": 1.5}}}
    for axis, value in axes.items():
        body["axes"][axis] = value
    return body


def write(tmp_path, body):
    path = tmp_path / "calibration.json"
    path.write_text(json.dumps(body))
    return str(path)


class Clock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t


def sticks(payload):
    return json.loads(payload)["data"]


# ---------------------------------------------------------------- the gate mode refuses to start uncalibrated

def test_gate_mode_without_a_calibration_refuses_to_start():
    with pytest.raises(units.UnitsError, match="measured SI-to-joystick calibration"):
        units.gate_calibration({"GO2_ACTUATOR_MODE": "gate"})


@pytest.mark.parametrize("mutate", [
    lambda b: b.update(status="assumed"),                                   # not measured
    lambda b: b.update(schema="other/1"),
    lambda b: b["robot"].update(serial=""),
    lambda b: b.pop("measured_at"),
    lambda b: b["axes"]["ly"].update(positive_si_per_unit=0.0),
    lambda b: b["axes"]["ly"].update(positive_si_per_unit=float("nan")),
    lambda b: b["axes"]["ly"].update(positive_si_per_unit=True),
    lambda b: b["axes"]["rx"].update(unit="deg/s"),                         # the unit is exact
    lambda b: b["axes"]["lx"].update(unit="rad/s"),
    lambda b: b["axes"].pop("rx"),
    lambda b: b["axes"]["ly"].pop("positive_si_per_unit"),                  # forward must be measured
])
def test_a_calibration_that_is_not_a_measured_si_contract_is_refused(tmp_path, mutate):
    body = calibration()
    mutate(body)
    with pytest.raises(units.UnitsError):
        units.load_calibration(write(tmp_path, body))


def test_a_measured_calibration_loads_with_its_identity(tmp_path):
    path = write(tmp_path, calibration())
    loaded = units.gate_calibration({units.CALIBRATION_ENV: path})
    assert loaded.robot_serial == "B42D2000P4CAF505" and len(loaded.sha256) == 64


# ---------------------------------------------------------------- every command goes through the calibration

def test_si_commands_are_converted_never_copied(tmp_path):
    cal = units.load_calibration(write(tmp_path, calibration()))
    assert cal.sticks(0.4, 0.0, 1.2) == {"ly": 0.5, "lx": 0.0, "rx": -0.8, "ry": 0}     # 0.4/0.8, -(1.2/1.5)
    assert cal.sticks(0.0, 0.2, -0.75) == {"ly": 0.0, "lx": -0.5, "rx": 0.5, "ry": 0}


def test_an_axis_beyond_the_documented_range_is_refused_not_clipped(tmp_path):
    cal = units.load_calibration(write(tmp_path, calibration()))
    assert cal.sticks(0.81, 0.0, 0.0) is None                # 0.81 / 0.8 > 1
    assert cal.sticks(0.0, 0.0, 1.6) is None
    assert cal.sticks(float("nan"), 0.0, 0.0) is None


def test_an_unmeasured_direction_is_refused(tmp_path):
    """Reverse is explicit at the adapter (Codex #239 / #237): no negative ly scale, no reverse command."""
    cal = units.load_calibration(write(tmp_path, calibration()))
    assert cal.sticks(-0.1, 0.0, 0.0) is None
    measured = units.load_calibration(write(tmp_path, calibration(
        ly={"unit": "m/s", "positive_si_per_unit": 0.8, "negative_si_per_unit": 0.5})))
    assert measured.sticks(-0.2, 0.0, 0.0) == {"ly": -0.4, "lx": 0.0, "rx": 0.0, "ry": 0}


def test_the_gate_input_sends_calibrated_sticks_and_zero_on_a_refused_command(tmp_path):
    clock, ros = Clock(), Clock(1000.0)
    cal = units.load_calibration(write(tmp_path, calibration()))
    gi = gate_input(cal, clock, ros)

    def at(dt, x, y, z):
        clock.t += dt
        ros.t += dt
        assert gi.on_actuate(ros.t, x, y, z)
    at(0.01, 0.4, 0.0, 1.2)
    assert sticks(gi.payload_due()) == {"ly": 0.5, "lx": 0.0, "rx": -0.8, "ry": 0}
    at(0.05, -0.1, 0.0, 0.0)                                  # reverse was never measured
    assert sticks(gi.payload_due()) == ZERO
    assert gi.refused == 1 and gi.last_refused == (-0.1, 0.0, 0.0)
    at(0.05, 0.0, 0.0, 0.0)
    assert gi.payload_due() is None and gi.refused == 1      # the same explicit zero is already out (refresh rule)
    clock.t += gate.REFRESH_S
    assert sticks(gi.payload_due()) == ZERO


def test_the_gate_input_cannot_be_built_without_a_calibration():
    with pytest.raises(TypeError):
        gate.GateInput(object(), object())                    # noqa - the calibration is a required argument


def test_the_driver_loads_the_calibration_before_building_the_gate_input():
    """Source boundary: the gate branch of go2_driver_node.py calls gate_calibration(os.environ) and passes it on."""
    source = (SDK_PKG / "go2_robot_sdk" / "go2_driver_node.py").read_text()
    branch = source[source.index("if self.actuator_mode == 'gate':"):source.index("# Legacy (default)")]
    assert "gate_calibration(os.environ)" in branch
    assert branch.index("gate_calibration(os.environ)") < branch.index("GateInput(")
    assert "GateInput(StampedOutputGuard(*timing), GateLiveness(*timing), calibration)" in branch


def test_legacy_mode_is_unchanged_and_distinct():
    assert sticks(gate.joystick_payload(0.3, 0.1, 0.5)) == {"lx": -0.1, "ly": 0.3, "rx": -0.5, "ry": 0}
    assert units.gate_calibration.__module__ == "go2_robot_sdk.actuator_units"



def test_every_new_refusal_is_reported_once_and_the_driver_logs_it_bounded(tmp_path):
    """Codex #244: a refused command is an explicit zero AND a warning; the driver throttles the log."""
    clock, ros = Clock(), Clock(1000.0)
    cal = units.load_calibration(write(tmp_path, calibration()))
    gi = gate_input(cal, clock, ros)
    ros.t += 0.01
    gi.on_actuate(ros.t, 0.4, 0.0, 0.0)
    gi.payload_due()
    assert gi.refusal_report() is None                        # nothing refused
    ros.t += 0.01
    gi.on_actuate(ros.t, -0.1, 0.0, 0.0)
    gi.payload_due()
    report = gi.refusal_report()
    assert report and "refused 1 command" in report and "(-0.1, 0.0, 0.0)" in report
    assert gi.refusal_report() is None                        # reported once
    source = (SDK_PKG / "go2_robot_sdk" / "go2_driver_node.py").read_text()
    tick = source[source.index("payload = self._gate_input.payload_due()"):][:600]
    assert "refusal_report()" in tick and "throttle_duration_sec=2.0" in tick


@pytest.mark.parametrize("axis", [0, 1, 2])
@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_a_non_finite_filtered_command_right_after_a_moving_one_sends_zero(tmp_path, axis, bad):
    """Codex #409: the shared guard refuses the vector at the boundary and erases the moving command, so the driver
    sends an explicit zero - the same outcome as the simulator - and only a newer finite command drives again."""
    clock, ros = Clock(), Clock(1000.0)
    gi = gate_input(units.load_calibration(write(tmp_path, calibration())), clock, ros)
    ros.t += 0.01
    assert gi.on_actuate(ros.t, 0.4, 0.0, 0.0)
    assert sticks(gi.payload_due())["ly"] == 0.5
    command = [0.4, 0.0, 0.0]
    command[axis] = bad
    ros.t += 0.01
    clock.t += 0.01
    assert gi.on_actuate(ros.t, *command) is False
    assert sticks(gi.payload_due()) == ZERO
    ros.t += 0.01
    clock.t += gate.REFRESH_S
    assert gi.on_actuate(ros.t, 0.4, 0.0, 0.0) and sticks(gi.payload_due())["ly"] == 0.5
