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


def publishable(ranges, cloud_age_s: float, max_age_s: float) -> tuple[bool, str]:
    """Whether to publish this scan at all: never a stale cloud, never one without a single measured return."""
    if not math.isfinite(cloud_age_s) or cloud_age_s > max_age_s or cloud_age_s < -max_age_s:
        return False, f"cloud age {cloud_age_s:.3f} s outside +-{max_age_s} s"
    if not any(math.isfinite(r) for r in ranges):
        return False, "no measured return in the cloud"
    return True, ""
