"""Body-frame velocity of the Go2 derived from consecutive received poses (T5.9, Codex #279 / #282).

The Go2's ROBOTODOM (rt/utlidar/robot_pose) carries a pose only. The driver used to publish /odom with an all-zero
twist, so Nav2's controller (MPPI takes robot_speed from odometry) and the Bridge's halt acknowledgement saw a robot
that never moved - unlike the simulator. The twist is DERIVED here, never echoed from the commanded velocity and never
taken from a vendor field of assumed frame:

- validation: a pose is used only with a finite position and a finite, near-unit quaternion (``QUATERNION_TOLERANCE``);
  anything else is refused BEFORE the driver publishes either the transform or the odometry, and breaks the velocity
  chain (the next valid pose is a first pose again) - a malformed pose is never standstill evidence;
- source: two consecutive poses the driver PUBLISHES (the newest received per timer tick, not every received pose),
  in the pose's own (odom) frame;
- time: the driver's receive times (the clock its outputs are stamped with, Codex #271 / #274) - the vendor's stamps
  are not a verified contract, so receive jitter is part of this estimate (PROVISIONAL, no physical accuracy claim);
- estimate: the SE(2) logarithm of the relative pose - EXACT for a constant body twist over the interval (a chord at
  the mid heading under-reads turning motion, Codex #282); the result is in the CHILD (body) frame at the interval's
  start, as nav_msgs/Odometry requires; the turn must stay within ``MAX_TURN_RAD`` (beyond it the wrapped heading
  change is ambiguous at this sampling);
- validity: no previous pose, an interval outside [MIN_DT_S, MAX_DT_S] or a turn beyond the limit gives NO velocity
  (``valid`` False with the reason) - the caller then publishes no Odometry. MAX_DT_S is PROVISIONAL: the physical
  robot_pose rate is not yet measured.
The twist covariance is not estimated: the driver leaves it zero and no consumer in this stack reads it.
No ROS import: tested offline.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

MIN_DT_S = 0.005
MAX_DT_S = 1.0
MAX_TURN_RAD = math.pi / 2
QUATERNION_TOLERANCE = 0.01          # |norm - 1|; a float32 vendor quaternion is unit to ~1e-6


@dataclass(frozen=True)
class PoseSample:
    received_ns: int
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class BodyVelocity:
    valid: bool
    reason: str = ""
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    dt_s: float = 0.0


def _number(value) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return None
    return float(value)


def sample(received_ns: int, pose) -> tuple[PoseSample | None, str]:
    """The validated pose, or (None, why): finite x / y / z and a finite near-unit quaternion are required."""
    if not isinstance(pose, dict) or not isinstance(pose.get("position"), dict) \
            or not isinstance(pose.get("orientation"), dict):
        return None, "pose without position / orientation"
    p = [_number(pose["position"].get(k)) for k in ("x", "y", "z")]
    q = [_number(pose["orientation"].get(k)) for k in ("x", "y", "z", "w")]
    if None in p:
        return None, "non-finite or missing position"
    if None in q:
        return None, "non-finite or missing orientation"
    norm = math.sqrt(sum(c * c for c in q))
    if abs(norm - 1.0) > QUATERNION_TOLERANCE:
        return None, f"orientation quaternion norm {norm:.4f} is not unit"
    x, y, z, w = (c / norm for c in q)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return PoseSample(int(received_ns), p[0], p[1], yaw), ""


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _left_jacobian_inverse(theta: float) -> tuple[float, float]:
    """(a, b) of V(theta)^-1 = [[a, b], [-b, a]] for the SE(2) logarithm; series near zero."""
    half = theta / 2.0
    if abs(theta) < 1e-6:
        return 1.0 - theta * theta / 12.0, half
    return half * math.cos(half) / math.sin(half), half


def body_velocity(previous: PoseSample | None, current: PoseSample) -> BodyVelocity:
    if previous is None:
        return BodyVelocity(False, "first pose: no velocity yet")
    dt = (current.received_ns - previous.received_ns) / 1e9
    if not MIN_DT_S <= dt <= MAX_DT_S:
        return BodyVelocity(False, f"pose interval {dt:.3f} s outside [{MIN_DT_S}, {MAX_DT_S}] s", dt_s=dt)
    theta = wrap(current.yaw - previous.yaw)
    if abs(theta) > MAX_TURN_RAD:
        return BodyVelocity(False, f"turn {theta:.3f} rad beyond +-{MAX_TURN_RAD:.3f} rad in one interval", dt_s=dt)
    c, s = math.cos(previous.yaw), math.sin(previous.yaw)
    dx, dy = current.x - previous.x, current.y - previous.y
    bx, by = c * dx + s * dy, -s * dx + c * dy          # the displacement in the start body frame
    a, b = _left_jacobian_inverse(theta)
    return BodyVelocity(True, "", vx=(a * bx + b * by) / dt, vy=(-b * bx + a * by) / dt, wz=theta / dt, dt_s=dt)
