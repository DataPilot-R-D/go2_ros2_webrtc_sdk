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

Coverage (Codex #271 / #275 / #276): 360 output bins are not 360 observed directions, and an aggregate percentage is
no bound - a scan observed everywhere EXCEPT a central +-20 deg corridor would refresh Nav2 with the travel path
unseen. The forward-only controller still rotates in place, so every direction can become the travel direction:
EVERY gap between neighbouring observed rays around the robot (wrapping at +-pi) must be closed one of two ways -

* ANGLE: the rays are at most ``MAX_RAY_SEPARATION`` apart (unobserved bins + 1 increments, Codex #276) - the angle
  the smallest obstacle the robot must see (``MIN_OBSTACLE_M``) subtends at the distance it must see it
  (``SEE_AT_M``), so no such obstacle fits between them within that distance; or
* CHORD: the two measured hit points are at most ``MIN_OBSTACLE_M`` apart, taken conservatively between the bins'
  OUTER edges. The triangle sensor-hit-hit then has no room for a disk that wide (its base is the widest part), and
  whatever lies behind the segment between the hits is shielded by them (Nav2 marks and inflates both). This is what
  a 5 cm voxel map gives near a wall - returns 3 to 5 deg apart at 0.85 m - and it is geometrically closed.

Anything else is refused: a sparse or holed cloud stops the robot rather than refreshing free space. Both numbers are
STATED, PROVISIONAL assumptions until a measured sensor contract of the physical Go2 cloud replaces them; the voxel
map's own free-vs-unknown ambiguity (an unoccupied voxel may be unobserved) is a separate, open physical contract.
"""
from __future__ import annotations

import math

UNOBSERVED = float("nan")


def bins(angle_min: float, angle_max: float, angle_increment: float) -> int:
    return int((angle_max - angle_min) / angle_increment)


def hits_from_points(points_xyz, *, min_height, max_height, range_min, range_max, angle_min, angle_max,
                     angle_increment) -> list:
    """Points ALREADY in the scan frame -> per bin the nearest measured return as (range, angle), else None."""
    out = [None] * bins(angle_min, angle_max, angle_increment)
    for x, y, z in points_xyz:
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)) or not min_height <= z <= max_height:
            continue
        r = math.hypot(x, y)
        if not range_min <= r <= range_max:
            continue
        angle = math.atan2(y, x)
        index = int((angle - angle_min) / angle_increment)
        if 0 <= index < len(out) and (out[index] is None or r < out[index][0]):
            out[index] = (r, angle)
    return out


def ranges_from_points(points_xyz, **params) -> list[float]:
    """Points ALREADY in the scan frame -> one range per bin: the nearest measured return, else NaN (unobserved)."""
    return [UNOBSERVED if hit is None else hit[0] for hit in hits_from_points(points_xyz, **params)]


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


def gaps(ranges, angle_increment: float):
    """(i, j, steps) for each pair of neighbouring observed bins around the circle: j follows i after ``steps``
    increments (unobserved bins + 1); j may wrap past the end."""
    observed = [i for i, r in enumerate(ranges) if math.isfinite(r)]
    pairs = list(zip(observed, observed[1:])) + ([(observed[-1], observed[0])] if observed else [])
    return [(i, j, (j - i) % len(ranges) or len(ranges)) for i, j in pairs]


def max_observed_separation(ranges, angle_increment: float) -> float:
    """Widest angular separation between neighbouring OBSERVED rays, in radians (an all-observed scan is one
    increment; one observed ray or none leaves the whole circle open)."""
    found = gaps(ranges, angle_increment)
    if len(found) < 2:
        return 2.0 * math.pi
    return max(steps for _, _, steps in found) * angle_increment


def outer_chord(r1: float, r2: float, steps: int, angle_increment: float) -> float:
    """The largest possible distance between two hits known only by bin: their bins' OUTER edges ((steps + 1)
    increments apart)."""
    spread = min(math.pi, (steps + 1) * angle_increment)
    return math.sqrt(max(0.0, r1 * r1 + r2 * r2 - 2.0 * r1 * r2 * math.cos(spread)))


def hit_chord(a, b) -> float:
    """The distance between two measured hit points given as (range, angle)."""
    return math.dist((a[0] * math.cos(a[1]), a[0] * math.sin(a[1])), (b[0] * math.cos(b[1]), b[0] * math.sin(b[1])))


def open_gaps(ranges, angle_increment: float, max_gap: float = MAX_RAY_SEPARATION,
              min_obstacle: float = MIN_OBSTACLE_M, hits=None) -> list[dict]:
    """Every gap closed neither by ANGLE nor by CHORD (module docstring). With ``hits`` (per bin (range, angle) of the
    actual nearest point, as the adapter knows them) the chord is exact; without, it is the conservative outer-edge
    bound."""
    found = gaps(ranges, angle_increment)
    if len(found) < 2:
        return [{"separation_deg": 360.0, "chord_m": None, "bins": found[0][:2] if found else None}]
    out = []
    for i, j, steps in found:
        separation = steps * angle_increment
        if separation <= max_gap + 1e-9:
            continue
        chord = (hit_chord(hits[i], hits[j]) if hits is not None and hits[i] and hits[j]
                 else outer_chord(ranges[i], ranges[j], steps, angle_increment))
        if chord <= min_obstacle + 1e-9:
            continue
        out.append({"separation_deg": round(math.degrees(separation), 3), "chord_m": round(chord, 3), "bins": (i, j)})
    return out


def publishable(ranges, cloud_age_s: float, max_age_s: float, *, angle_increment: float = math.pi / 180.0,
                max_gap: float = MAX_RAY_SEPARATION, min_obstacle: float = MIN_OBSTACLE_M,
                hits=None) -> tuple[bool, str]:
    """Whether to publish this scan at all: never a stale cloud, never one without a measured return, never one with
    a gap between neighbouring observed rays that an obstacle of ``min_obstacle`` could hide in (module docstring)."""
    if not math.isfinite(cloud_age_s) or cloud_age_s > max_age_s or cloud_age_s < -max_age_s:
        return False, f"cloud age {cloud_age_s:.3f} s outside +-{max_age_s} s"
    if not any(math.isfinite(r) for r in ranges):
        return False, "no measured return in the cloud"
    holes = open_gaps(ranges, angle_increment, max_gap, min_obstacle, hits)
    if holes:
        worst = max(holes, key=lambda h: h["separation_deg"])
        chord = "open" if worst["chord_m"] is None else f"{worst['chord_m']:.3f} m"
        return False, (f"observed rays {worst['separation_deg']:.1f} deg apart > {math.degrees(max_gap):.2f} deg with "
                       f"hits {chord} apart > {min_obstacle:.2f} m (an obstacle could hide between them; "
                       f"{len(holes)} such gap(s))")
    return True, ""
