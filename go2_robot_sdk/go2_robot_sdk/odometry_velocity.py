"""Body-frame velocity of the Go2 derived from consecutive received poses (T5.9, Codex #279).

The Go2's ROBOTODOM (rt/utlidar/robot_pose) carries a pose only. The driver used to publish /odom with an all-zero
twist, so Nav2's controller (MPPI takes robot_speed from odometry) saw a robot that never moved - unlike the
simulator. The twist is DERIVED here, never echoed from the commanded velocity and never taken from a vendor field of
assumed frame:

- source: two consecutive received poses of the same robot, in the pose's own (odom) frame;
- time: the driver's receive times (the clock its outputs are stamped with, Codex #271 / #274) - the vendor's own
  stamps are not a verified contract, so receive jitter is part of this measurement;
- frame: the displacement is rotated into the CHILD (body) frame at the mid-interval heading, the heading change
  wrapped to (-pi, pi], as nav_msgs/Odometry requires (twist in child_frame_id);
- validity: no previous sample, a non-finite pose, or an interval outside [MIN_DT_S, MAX_DT_S] gives NO velocity
  (``valid`` False with the reason) - the caller then publishes no Odometry at all, so a first or gapped sample never
  claims a measured standstill. MAX_DT_S is PROVISIONAL: the physical robot_pose rate is not yet measured.
No ROS import: tested offline.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

MIN_DT_S = 0.005
MAX_DT_S = 1.0


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


def yaw_of(q: dict) -> float:
    x, y, z, w = (float(q.get(k, 0.0)) for k in ("x", "y", "z", "w"))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def sample(received_ns: int, pose: dict) -> PoseSample:
    p = pose.get("position", {})
    return PoseSample(int(received_ns), float(p.get("x", math.nan)), float(p.get("y", math.nan)),
                      yaw_of(pose.get("orientation", {})))


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def body_velocity(previous: PoseSample | None, current: PoseSample) -> BodyVelocity:
    if previous is None:
        return BodyVelocity(False, "first pose: no velocity yet")
    values = (previous.x, previous.y, previous.yaw, current.x, current.y, current.yaw)
    if not all(math.isfinite(v) for v in values):
        return BodyVelocity(False, "non-finite pose")
    dt = (current.received_ns - previous.received_ns) / 1e9
    if not MIN_DT_S <= dt <= MAX_DT_S:
        return BodyVelocity(False, f"pose interval {dt:.3f} s outside [{MIN_DT_S}, {MAX_DT_S}] s", dt_s=dt)
    turn = wrap(current.yaw - previous.yaw)
    heading = previous.yaw + turn / 2.0
    wx, wy = (current.x - previous.x) / dt, (current.y - previous.y) / dt
    c, s = math.cos(heading), math.sin(heading)
    return BodyVelocity(True, "", vx=c * wx + s * wy, vy=-s * wx + c * wy, wz=turn / dt, dt_s=dt)
