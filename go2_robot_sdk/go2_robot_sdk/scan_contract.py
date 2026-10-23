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

Coverage (Codex #271): 360 output bins are not 360 observed directions - one steady rear point would keep the
obstacle source "current" while the travel corridor is blind. A scan is published only if at least
``min_forward_coverage`` of the bins within ``forward_half_angle`` of the robot's heading hold a measured return (the
stack is forward-only; enabling reverse must add the rear sector). The defaults are FAIL-CLOSED and UNCALIBRATED: the
physical Go2 cloud's coverage has not been measured; set them from that measurement, never from a guess that passes.
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


MIN_FORWARD_COVERAGE = 0.5            # UNCALIBRATED fail-closed default (see the module docstring)
FORWARD_HALF_ANGLE = math.radians(45.0)


def forward_coverage(ranges, angle_min: float, angle_increment: float,
                     half_angle: float = FORWARD_HALF_ANGLE) -> float:
    """Fraction of the bins within ``half_angle`` of the heading (angle 0) that hold a measured return."""
    sector = [r for i, r in enumerate(ranges) if abs(angle_min + (i + 0.5) * angle_increment) <= half_angle]
    return sum(1 for r in sector if math.isfinite(r)) / len(sector) if sector else 0.0


def publishable(ranges, cloud_age_s: float, max_age_s: float, *, angle_min: float = -math.pi,
                angle_increment: float = math.pi / 180.0, min_forward_coverage: float = MIN_FORWARD_COVERAGE,
                forward_half_angle: float = FORWARD_HALF_ANGLE) -> tuple[bool, str]:
    """Whether to publish this scan at all: never a stale cloud, never one without a measured return, never one that
    leaves the forward travel sector unobserved."""
    if not math.isfinite(cloud_age_s) or cloud_age_s > max_age_s or cloud_age_s < -max_age_s:
        return False, f"cloud age {cloud_age_s:.3f} s outside +-{max_age_s} s"
    if not any(math.isfinite(r) for r in ranges):
        return False, "no measured return in the cloud"
    coverage = forward_coverage(ranges, angle_min, angle_increment, forward_half_angle)
    if coverage < min_forward_coverage:
        return False, f"forward coverage {coverage:.2f} < {min_forward_coverage:.2f} (the travel sector is blind)"
    return True, ""
