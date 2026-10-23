"""The Go2 point cloud -> LaserScan contract (T5.9 robot host, Codex #263 / #264). No ROS import: tested offline.

A LaserScan range means one of three things, and the shared Nav2 profile (``inf_is_valid: true``) treats them
differently, so the adapter must never blur them:

* a finite range - a MEASURED return (marks the hit cell, clears the ray up to it);
* ``+inf`` - a VERIFIED no-hit: the sensor sampled that direction and saw nothing within range (clears the whole ray).
  A voxel / point cloud cannot verify that - an empty bin may be unsampled, height-filtered or simply sparse - so this
  adapter NEVER emits +inf;
* ``NaN`` - UNOBSERVED: Nav2 neither marks nor clears.

A cloud that yields no measured return, or that is older than ``max_age_s``, produces NO scan at all: the obstacle
layer's expected_update_rate then makes the source not current and the controller stops (the sensor-stall
contract) - an empty or stale observation never refreshes the costmap as free space.

Coverage (Codex #271 / #275): 360 output bins are not 360 observed directions, and an aggregate percentage is no
bound - a scan observed everywhere EXCEPT a central +-20 deg corridor would refresh Nav2 with the travel path unseen.
The forward-only controller still rotates in place, so every direction can become the travel direction: a scan is
published only if the angular SEPARATION of every pair of neighbouring observed rays around the robot (wrapping at
+-pi) is at most ``MAX_RAY_SEPARATION`` - the opening an obstacle could hide in is the unobserved bins PLUS one
increment (Codex #276), and a fully observed scan sampled coarser than the bound is refused too. The default bounds the angle the smallest obstacle the robot must see (``MIN_OBSTACLE_M``)
subtends at the distance it must see it (``SEE_AT_M``): both are STATED ASSUMPTIONS, UNCALIBRATED until a measured
sensor contract of the physical Go2 cloud replaces them - the bound is strict enough that a holed cloud stops the
robot rather than refreshing free space.
"""
from __future__ import annotations

import math

UNOBSERVED = float("nan")


def bins(angle_min: float, angle_max: float, angle_increment: float) -> int:
    return int((angle_max - angle_min) / angle_increment)


def ranges_from_points(points_xyz, *, min_height, max_height, range_min, range_max, angle_min, angle_max,
                       angle_increment) -> list[float]:
    """Points ALREADY in the scan frame -> one range per bin: the nearest measured return, else NaN (unobserved)."""
    out = [UNOBSERVED] * bins(angle_min, angle_max, angle_increment)
    for x, y, z in points_xyz:
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)) or not min_height <= z <= max_height:
            continue
        r = math.hypot(x, y)
        if not range_min <= r <= range_max:
            continue
        angle = math.atan2(y, x)
        if not angle_min <= angle <= angle_max:
            continue
        index = int((angle - angle_min) / angle_increment)
        if 0 <= index < len(out) and (math.isnan(out[index]) or r < out[index]):
            out[index] = r
    return out


MIN_OBSTACLE_M = 0.10      # ASSUMPTION: the narrowest obstacle the robot must perceive (a chair leg)
SEE_AT_M = 1.5             # ASSUMPTION: the distance it must be perceived at (controller horizon at 0.4 m/s)
MAX_RAY_SEPARATION = 2.0 * math.atan((MIN_OBSTACLE_M / 2.0) / SEE_AT_M)     # ~3.8 deg


def _quat_rotate(q, v):
    """Rotate v by the unit quaternion q (x, y, z, w)."""
    qx, qy, qz, qw = q
    x, y, z = v
    t2, t3, t4 = qw * qx, qw * qy, qw * qz
    t5, t6, t7 = -qx * qx, qx * qy, qx * qz
    t8, t9, t10 = -qy * qy, qy * qz, -qz * qz
    return (2 * ((t8 + t10) * x + (t6 - t4) * y + (t3 + t7) * z) + x,
            2 * ((t4 + t6) * x + (t5 + t10) * y + (t9 - t2) * z) + y,
            2 * ((t7 - t3) * x + (t2 + t9) * y + (t5 + t8) * z) + z)


def to_frame(points, translation, rotation):
    """tf2 semantics of lookup_transform(target, source): p_target = R(rotation) * p_source + translation, so after
    the transform into the robot's base frame, angle 0 of the scan is the body heading."""
    tx, ty, tz = translation
    for point in points:
        rx, ry, rz = _quat_rotate(rotation, point)
        yield rx + tx, ry + ty, rz + tz


def max_observed_separation(ranges, angle_increment: float) -> float:
    """Widest angular separation between neighbouring OBSERVED rays, in radians, around the circle: an opening of k
    unobserved bins between two observed rays is (k + 1) increments wide; an all-observed scan is one increment; a
    single observed ray (or none) leaves the whole circle open."""
    observed = [i for i, r in enumerate(ranges) if math.isfinite(r)]
    if len(observed) < 2:
        return 2.0 * math.pi
    steps = [b - a for a, b in zip(observed, observed[1:])] + [observed[0] + len(ranges) - observed[-1]]
    return max(steps) * angle_increment


def publishable(ranges, cloud_age_s: float, max_age_s: float, *, angle_increment: float = math.pi / 180.0,
                max_gap: float = MAX_RAY_SEPARATION) -> tuple[bool, str]:
    """Whether to publish this scan at all: never a stale cloud, never one without a measured return, never one whose
    neighbouring observed rays are more than ``max_gap`` apart anywhere around the robot (rotation sweeps every
    direction)."""
    if not math.isfinite(cloud_age_s) or cloud_age_s > max_age_s or cloud_age_s < -max_age_s:
        return False, f"cloud age {cloud_age_s:.3f} s outside +-{max_age_s} s"
    if not any(math.isfinite(r) for r in ranges):
        return False, "no measured return in the cloud"
    separation = max_observed_separation(ranges, angle_increment)
    if separation > max_gap + 1e-9:
        return False, (f"observed rays {math.degrees(separation):.1f} deg apart > {math.degrees(max_gap):.2f} deg "
                       "(an obstacle could hide between them)")
    return True, ""
