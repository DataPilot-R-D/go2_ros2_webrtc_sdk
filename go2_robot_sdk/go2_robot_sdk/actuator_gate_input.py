"""Driver side of the shared actuator gate (T5.9): in gate mode the Go2 obeys ONLY the gate output (cmd_vel_mux).

The gate (monorepo infra/actuator/actuator_mux.py - the same arbitration the simulator runs) owns input priority,
the Safety latch and the Bridge-heartbeat watchdog, and publishes an explicit zero when nothing may move the robot.
A gate that dies cannot send that zero, so the driver guards the gate output with its OWN monotonic TTL
(``OutputGuard``) and sends an explicit zero stick state when the output is stale or was never received. An
unchanged command is not resent every tick (the WebRTC data channel starves under load) but at least every
``REFRESH_S``, so a lost message can never leave the robot on an old stick state for long.

No ROS import: the node wires it (go2_driver_node.py, GO2_ACTUATOR_MODE=gate) and tests run offline.
"""
from __future__ import annotations

import json
import time

ACTUATOR_MODES = ("legacy", "gate")
# The simulator's consumer TTL for the same gate output (repos/sim ros2_bridge MUX_COMMAND_TIMEOUT_S; test-bound).
GATE_OUTPUT_TTL_S = 0.5
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
    """ROS twist -> Unitree rt/wirelesscontroller (joystick emulation, the reliable locomotion path).

    ly = forward/back (linear.x), lx = strafe (-linear.y), rx = turn (-angular.z); all zero = explicit stop.
    """
    data = ({"lx": round(-y, 2), "ly": round(x, 2), "rx": round(-z, 2), "ry": 0}
            if (x != 0.0 or y != 0.0 or z != 0.0) else {"lx": 0, "ly": 0, "rx": 0, "ry": 0})
    return json.dumps({"type": "msg", "topic": "rt/wirelesscontroller", "data": data})


class GateInput:
    """The one velocity input of a gate-mode driver."""

    def __init__(self, guard, refresh_s: float = REFRESH_S, clock=time.monotonic):
        self._guard = guard
        self._refresh_s = refresh_s
        self._clock = clock
        self._last_payload: str | None = None
        self._last_sent = float("-inf")
        self._status: tuple[float, bool] | None = None   # (received_at, no lock engaged)

    def on_twist(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        self._guard.on_output((linear_x, linear_y, 0.0, angular_z))

    def on_status(self, raw: str) -> None:
        """The gate's JSON status ({source, locked_by}); anything malformed counts as locked."""
        try:
            status = json.loads(raw)
            locked_by = status["locked_by"] if isinstance(status, dict) else None
            free = isinstance(locked_by, list) and not locked_by
        except (TypeError, ValueError, KeyError):
            free = False
        self._status = (self._clock(), free)

    def motion_allowed(self) -> bool:
        """Posture commands (stand up / down, wake up) move the body: only while the gate's own fresh status says
        no lock is engaged."""
        return self._status is not None and self._status[1] and self._clock() - self._status[0] < STATUS_TTL_S

    def payload_due(self) -> str | None:
        """The stick state to send this tick (explicit zero when the gate is stale), or None if nothing is due."""
        x, y, _, z = self._guard.command()
        payload = joystick_payload(x, y, z)
        now = self._clock()
        if payload == self._last_payload and now - self._last_sent < self._refresh_s:
            return None
        self._last_payload, self._last_sent = payload, now
        return payload
