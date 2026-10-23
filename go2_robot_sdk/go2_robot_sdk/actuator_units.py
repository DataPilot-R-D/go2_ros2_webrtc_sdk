"""SI-to-actuator units contract of the Go2 gate mode (T5.9, Codex #228 / #237 / #242).

The shared gate speaks SI (m/s, rad/s). The Go2's wireless-controller axes (rt/wirelesscontroller lx / ly / rx) are
normalised to [-1, 1] (Unitree unitree_ros2 README, section 3), so copying SI values into them is not a conversion.
Autonomous gate mode therefore needs a MEASURED calibration (GO2_JOYSTICK_CALIBRATION, a JSON file of schema
``SCHEMA``) and refuses to start without one. Each axis states the SI it moves per unit, separately for each
direction; a direction without a measured scale (e.g. reverse) is refused, as is any axis that would leave [-1, 1]:
the caller sends an explicit zero instead and reports it - never a silent clip. A verified SI velocity API (sport
API 1008 Move) would replace this contract, not bypass it. Legacy mode (actuator_gate_input.joystick_payload) is
unchanged and is never autonomous-calibrated.

No ROS import: tested offline.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math

SCHEMA = "paic2.go2.joystick_calibration/1"
CALIBRATION_ENV = "GO2_JOYSTICK_CALIBRATION"
# axis -> SI unit; ly = forward (linear.x), lx = strafe (-linear.y), rx = turn (-angular.z), as joystick_payload.
AXES = {"ly": "m/s", "lx": "m/s", "rx": "rad/s"}


class UnitsError(ValueError):
    """The gate mode's SI-to-actuator contract is missing or not a measured one."""


def _scale(axis, body, key, required):
    value = body.get(key)
    if value is None and not required:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise UnitsError(f"axes.{axis}.{key} must be a finite SI-per-unit scale > 0")
    return float(value)


@dataclass(frozen=True)
class JoystickCalibration:
    positive: dict          # axis -> SI per unit for a positive axis value
    negative: dict          # axis -> SI per unit for a negative axis value, None = never measured (refused)
    robot_serial: str
    measured_at: str
    sha256: str

    def _axis(self, axis, value):
        if value == 0.0:
            return 0.0
        scale = (self.positive if value > 0 else self.negative)[axis]
        return None if scale is None else value / scale

    def sticks(self, x: float, y: float, z: float) -> dict | None:
        """Stick axes for the SI twist, or None when a direction is unmeasured or an axis leaves [-1, 1]."""
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (x, y, z)):
            return None
        values = {"ly": self._axis("ly", x), "lx": self._axis("lx", -y), "rx": self._axis("rx", -z)}
        if any(v is None or abs(v) > 1.0 for v in values.values()):
            return None
        return {"ly": round(values["ly"], 3), "lx": round(values["lx"], 3), "rx": round(values["rx"], 3), "ry": 0}


def load_calibration(path: str) -> JoystickCalibration:
    raw = open(path, "rb").read()
    try:
        body = json.loads(raw)
    except ValueError as error:
        raise UnitsError(f"calibration is not JSON: {error}") from None
    if not isinstance(body, dict) or body.get("schema") != SCHEMA:
        raise UnitsError(f"calibration schema must be {SCHEMA}")
    if body.get("status") != "measured":
        raise UnitsError("calibration status must be 'measured' (an assumed or guessed gain is no calibration)")
    robot = body.get("robot") if isinstance(body.get("robot"), dict) else {}
    serial, measured_at = robot.get("serial"), body.get("measured_at")
    if not (isinstance(serial, str) and serial and isinstance(measured_at, str) and measured_at
            and isinstance(body.get("method"), str) and body["method"]):
        raise UnitsError("calibration must name the robot serial, measured_at and the method")
    axes = body.get("axes") if isinstance(body.get("axes"), dict) else {}
    positive, negative = {}, {}
    for axis, unit in AXES.items():
        spec = axes.get(axis)
        if not isinstance(spec, dict) or spec.get("unit") != unit:
            raise UnitsError(f"axes.{axis} must be measured in {unit}")
        positive[axis] = _scale(axis, spec, "positive_si_per_unit", True)
        negative[axis] = _scale(axis, spec, "negative_si_per_unit", False)
    return JoystickCalibration(positive, negative, serial, measured_at, hashlib.sha256(raw).hexdigest())


def gate_calibration(environ) -> JoystickCalibration:
    """The calibration autonomous gate mode drives with; refuses to start without a measured one."""
    path = environ.get(CALIBRATION_ENV)
    if not path:
        raise UnitsError(f"GO2_ACTUATOR_MODE=gate needs a measured SI-to-joystick calibration ({CALIBRATION_ENV}); "
                         "uncalibrated joystick emulation is legacy mode only")
    return load_calibration(path)
