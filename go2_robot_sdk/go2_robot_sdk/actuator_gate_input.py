"""Driver side of the shared actuator gate (T5.9, C4): in gate mode the Go2 obeys ONLY the collision monitor's filtered
command (cmd_vel_actuate), which carries the gate's decision stamp.

The gate (monorepo infra/actuator/actuator_mux.py - the same arbitration the simulator runs) owns input priority, the
Safety latch and the Bridge-heartbeat watchdog, and publishes an explicit zero when nothing may move the robot; the
collision monitor filters that decision. Both can die, so the driver guards the filtered command with the shared
``StampedOutputGuard``: the source age and receipt TTL at every use, strict stamp order after the last latch release,
and the Safety latch with downstream authority (engaging it erases the command). It sends an explicit zero stick
state otherwise. Posture commands use ``GateLiveness``, the gate's own stamped stream, not the monitor, which falls
silent at rest. An unchanged command is not resent every tick (the WebRTC data channel starves under load) but at
least every ``REFRESH_S``, so a lost message can never leave the robot on an old stick state for long. The timing is
the shared core's ``COMMAND_*`` - one definition with the simulator (Codex #401).

No ROS import: the node wires it (go2_driver_node.py, GO2_ACTUATOR_MODE=gate) and tests run offline.
"""
from __future__ import annotations

import json
import time

ACTUATOR_MODES = ("legacy", "gate")
REFRESH_S = 0.5
# The gate publishes cmd_vel_mux_status on every change and at least once a second; posture commands need it fresh.
STATUS_TTL_S = 2.0
# Outbox label of posture payloads: the driver re-checks the gate when it SENDS them, not only when they were queued.
POSTURE_LABEL = "posture"


def actuator_mode(environ) -> str:
    """``GO2_ACTUATOR_MODE``: ``legacy`` (default, unchanged driver) or ``gate`` (cmd_vel_mux only)."""
    mode = environ.get("GO2_ACTUATOR_MODE", "legacy")
    if mode not in ACTUATOR_MODES:
        raise ValueError(f"GO2_ACTUATOR_MODE must be one of {ACTUATOR_MODES}")
    return mode


def joystick_payload(x: float, y: float, z: float) -> str:
    """LEGACY ROS twist -> Unitree rt/wirelesscontroller (joystick emulation): the values are copied, not converted.

    ly = forward/back (linear.x), lx = strafe (-linear.y), rx = turn (-angular.z); all zero = explicit stop. Gate
    mode never uses this mapping: it converts through a measured calibration (actuator_units, Codex #228).
    """
    data = ({"lx": round(-y, 2), "ly": round(x, 2), "rx": round(-z, 2), "ry": 0}
            if (x != 0.0 or y != 0.0 or z != 0.0) else {"lx": 0, "ly": 0, "rx": 0, "ry": 0})
    return json.dumps({"type": "msg", "topic": "rt/wirelesscontroller", "data": data})


def _sticks_payload(data: dict) -> str:
    return json.dumps({"type": "msg", "topic": "rt/wirelesscontroller", "data": data})


ZERO_STICKS = {"lx": 0, "ly": 0, "rx": 0, "ry": 0}


class GateInput:
    """The one velocity input of a gate-mode driver: SI commands through the measured calibration (required).
    ``guard``: actuator_mux.StampedOutputGuard over the filtered command; ``liveness``: actuator_mux.GateLiveness over
    the gate's own stamped stream (posture)."""

    def __init__(self, guard, liveness, calibration, refresh_s: float = REFRESH_S, clock=time.monotonic):
        self._guard = guard
        self._liveness = liveness
        self._calibration = calibration
        self.refused = 0                 # commands refused by the units contract (sent as an explicit zero)
        self.last_refused = None
        self._reported = 0
        self._refresh_s = refresh_s
        self._clock = clock
        self._last_payload: str | None = None
        self._last_sent = float("-inf")
        self._status: tuple[float, bool] | None = None   # (received_at, no lock engaged)

    def on_actuate(self, stamp: float, linear_x: float, linear_y: float, angular_z: float) -> bool:
        """The collision monitor's filtered command (the only velocity); False when the guard refuses it."""
        return self._guard.on_output(stamp, (linear_x, linear_y, 0.0, angular_z))

    def on_gate(self, stamp: float, linear_x: float, linear_y: float, angular_z: float) -> bool:
        """The gate's own stamped decision: its liveness for posture, never a velocity."""
        return self._liveness.on_gate_output(stamp, (linear_x, linear_y, 0.0, angular_z))

    def on_latch(self, engaged: bool) -> None:
        """The Safety latch, with downstream authority over both the command and posture."""
        self._guard.on_latch(engaged)
        self._liveness.on_latch(engaged)

    def on_status(self, raw: str) -> None:
        """The gate's JSON status ({source, locked_by}); anything malformed counts as locked."""
        try:
            status = json.loads(raw)
            locked_by = status["locked_by"] if isinstance(status, dict) else None
            free = isinstance(locked_by, list) and not locked_by
        except (TypeError, ValueError, KeyError):
            free = False
        self._status = (self._clock(), free)

    def refusal_report(self) -> str | None:
        """A warning for every NEW refusal since the last report, None otherwise (Codex #244: a refused command is
        an explicit zero, never silent). The driver logs it with a throttle, so it stays bounded."""
        if self.refused == self._reported:
            return None
        self._reported = self.refused
        return (f"actuator units refused {self.refused} command(s) so far, last {self.last_refused} "
                "(an unmeasured direction or an axis beyond [-1, 1]); sent an explicit zero")

    def motion_allowed(self) -> bool:
        """Posture commands (stand up / down, wake up) move the body: only while the gate is ALIVE - its own stamped
        stream fresh after the last latch release (GateLiveness, independent of the monitor's silence at rest, Codex
        #395 / #397) - and its own fresh status says no lock is engaged (Codex #191)."""
        return (self._liveness.allowed() and self._status is not None and self._status[1]
                and self._clock() - self._status[0] < STATUS_TTL_S)

    def payload_due(self) -> str | None:
        """The stick state to send this tick (explicit zero when the gate is stale), or None if nothing is due."""
        x, y, _, z = self._guard.command()
        if x == 0.0 and y == 0.0 and z == 0.0:
            data = ZERO_STICKS
        else:
            data = self._calibration.sticks(x, y, z)
            if data is None:             # unmeasured direction or outside [-1, 1]: an explicit zero, reported
                if self.last_refused != (x, y, z):
                    self.refused += 1
                self.last_refused = (x, y, z)
                data = ZERO_STICKS
        payload = _sticks_payload(data)
        now = self._clock()
        if payload == self._last_payload and now - self._last_sent < self._refresh_s:
            return None
        self._last_payload, self._last_sent = payload, now
        return payload
