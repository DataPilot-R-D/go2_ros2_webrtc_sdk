"""T5.9 Go2 scan contract (Codex #263 / #264; offline): measured return, never-claimed no-hit, unobserved.

The inherited converter initialised every bin to +inf; with the shared profile's inf_is_valid that turned an empty,
sparse or height-filtered cloud into a fresh all-free 360-degree scan that cleared real obstacles, and it placed
clouds with the LATEST transform. Here an empty bin is NaN (unobserved: Nav2 neither marks nor clears), +inf is never
emitted, and an empty or stale cloud is not converted at all (the sensor-stall contract stops the controller)."""
import math
from pathlib import Path
import sys

import pytest

SDK_PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SDK_PKG))
from go2_robot_sdk import scan_contract as sc  # noqa: E402

PARAMS = dict(min_height=-0.1, max_height=0.5, range_min=0.15, range_max=10.0, angle_min=-math.pi, angle_max=math.pi,
              angle_increment=math.pi / 180.0)


def test_an_empty_cloud_gives_no_measured_return_and_is_not_published():
    ranges = sc.ranges_from_points([], **PARAMS)
    assert len(ranges) == 360 and all(math.isnan(r) for r in ranges)          # unobserved, never +inf
    assert sc.publishable(ranges, 0.05, 0.5) == (False, "no measured return in the cloud")


def ring(radius=3.0, skip=lambda deg: False, z=0.2):
    """One return per degree on a circle around the robot, except the skipped bearings."""
    out = []
    for d in range(-180, 180):
        bearing = d + 0.5
        if not skip(bearing):
            out.append((radius * math.cos(math.radians(bearing)), radius * math.sin(math.radians(bearing)), z))
    return out


def test_a_sparse_cloud_marks_only_its_bins_and_never_claims_free_space():
    ranges = sc.ranges_from_points([(2.0, 0.0, 0.2)], **PARAMS)
    finite = [r for r in ranges if math.isfinite(r)]
    assert finite == [2.0] and sum(math.isnan(r) for r in ranges) == 359 and not any(math.isinf(r) for r in ranges)
    ok, why = sc.publishable(ranges, 0.05, 0.5)                          # one steady point is no coverage
    assert not ok and "apart" in why


def test_a_fully_observed_ring_is_published_and_two_missing_bins_are_tolerated():
    full = sc.ranges_from_points(ring(), **PARAMS)
    assert math.degrees(sc.max_observed_separation(full, math.pi / 180)) == pytest.approx(1.0)
    assert sc.publishable(full, 0.05, 0.5)[0]
    hits = sc.hits_from_points(ring(skip=lambda b: 10 < b < 12), **PARAMS)          # 2 bins -> returns 3 deg apart
    ranges = [float("nan") if h is None else h[0] for h in hits]
    assert math.degrees(sc.max_observed_separation(ranges, math.pi / 180)) == pytest.approx(3.0)
    assert sc.publishable(ranges, 0.05, 0.5, hits=hits)[0]                         # exact opening 3 deg
    assert not sc.publishable(ranges, 0.05, 0.5)[0]           # known only by bin: outer edges 4 deg (Codex #289)


def test_codex_276_three_unobserved_bins_open_four_degrees_and_hide_the_obstacle():
    """Observed rays at -2 and +2 deg, NaN at -1 / 0 / +1: the opening is 4 deg > the 3.82 deg a 10 cm obstacle at
    1.5 m subtends - refused (an unobserved-bin count of 3 deg would have passed it)."""
    ranges = [3.0] * 360                       # bin i covers bearing -180 + i .. -179 + i deg
    ranges[179] = ranges[180] = ranges[181] = float("nan")                 # -1 / 0 / +1 deg unobserved
    assert math.degrees(sc.max_observed_separation(ranges, math.pi / 180)) == pytest.approx(4.0)
    ok, why = sc.publishable(ranges, 0.05, 0.5)
    assert not ok and "5.0 deg apart" in why                  # known only by bin: the outer edges


def test_a_fully_observed_scan_sampled_coarser_than_the_bound_is_refused():
    coarse = dict(PARAMS, angle_increment=math.radians(5.0))
    ranges = sc.ranges_from_points(ring(), **coarse)
    assert all(math.isfinite(r) for r in ranges) and len(ranges) == 72
    assert not sc.publishable(ranges, 0.05, 0.5, angle_increment=math.radians(5.0))[0]


def test_the_separation_wraps_across_the_seam():
    ranges = [float("nan")] * 360
    ranges[2], ranges[357] = 1.0, 1.0                                     # 5 bins apart across +-180
    assert math.degrees(sc.max_observed_separation(ranges, math.pi / 180)) == pytest.approx(355.0)
    ranges = [1.0] * 360
    ranges[0] = ranges[359] = float("nan")                                # the seam bins unobserved
    assert math.degrees(sc.max_observed_separation(ranges, math.pi / 180)) == pytest.approx(3.0)


@pytest.mark.parametrize("hole", [
    lambda b: -20 < b < 20,          # Codex #275: the central travel corridor, 0.73 m wide at 1 m
    lambda b: 70 < b < 110,          # a turn-side hole: in-place rotation sweeps it into the travel direction
    lambda b: abs(b) > 170,          # wrapping across +-180
    lambda b: 30 < b < 35,           # five degrees, just above the ~3.8 deg bound
])
def test_an_unobserved_sector_anywhere_around_the_robot_refuses_the_scan(hole):
    ranges = sc.ranges_from_points(ring(skip=hole), **PARAMS)
    ok, why = sc.publishable(ranges, 0.05, 0.5)
    assert not ok and "apart" in why


def test_the_gap_bound_is_the_stated_geometric_assumption():
    assert sc.MAX_RAY_SEPARATION == pytest.approx(2 * math.atan(0.05 / 1.5))
    assert math.degrees(sc.MAX_RAY_SEPARATION) < 4.0


def test_after_the_transform_bin_zero_is_the_body_heading():
    """A robot at (1, 1) facing +y in odom: a point 2 m ahead of it is at bearing 0 in base_link."""
    yaw = math.pi / 2
    rotation = (0.0, 0.0, math.sin(-yaw / 2), math.cos(-yaw / 2))            # odom -> base_link rotates by -yaw
    rx, ry, _ = next(sc.to_frame([(-1.0, -1.0, 0.0)], (0.0, 0.0, 0.0), rotation))
    translation = (rx, ry, 0.0)                                              # -R * robot position
    ahead = list(sc.to_frame([(1.0, 3.0, 0.2)], translation, rotation))
    assert ahead[0][0] == pytest.approx(2.0, abs=1e-9) and ahead[0][1] == pytest.approx(0.0, abs=1e-9)
    ranges = sc.ranges_from_points(ahead, **PARAMS)
    index = next(i for i, r in enumerate(ranges) if math.isfinite(r))
    assert abs(-math.pi + (index + 0.5) * math.pi / 180) < math.pi / 180      # the bin straddling bearing 0


def test_a_fully_height_filtered_cloud_is_not_free_space():
    ranges = sc.ranges_from_points([(2.0, 0.0, 1.5), (1.0, 1.0, -0.5)], **PARAMS)
    assert all(math.isnan(r) for r in ranges) and not sc.publishable(ranges, 0.05, 0.5)[0]


def test_the_nearest_return_wins_and_non_finite_points_are_ignored():
    ranges = sc.ranges_from_points([(3.0, 0.0, 0.2), (1.5, 0.0, 0.2), (float("nan"), 0.0, 0.2)], **PARAMS)
    assert [r for r in ranges if math.isfinite(r)] == [1.5]


def test_a_stale_or_future_cloud_is_never_converted():
    ranges = sc.ranges_from_points(ring(), **PARAMS)
    assert not sc.publishable(ranges, 100.0, 0.5)[0]                         # Codex #264: a 100 s old transform/cloud
    assert not sc.publishable(ranges, -3.0, 0.5)[0]
    assert not sc.publishable(ranges, float("nan"), 0.5)[0]


def _ring_at(radius, gap_bins=(), n=360):
    ranges = [radius] * n
    for i in gap_bins:
        ranges[i] = float("nan")
    return ranges


def test_a_gap_whose_hits_are_close_is_closed_by_the_chord():
    """A wall 0.85 m away at 5 cm voxel spacing: returns ~3.4-5 deg apart, hits <= 0.10 m apart - no 10 cm obstacle
    fits in the triangle they make with the sensor, so the scan is published (Codex #276 follow-up)."""
    ranges = _ring_at(0.85, gap_bins=(179, 180, 181))                       # rays 4 deg apart
    assert sc.max_observed_separation(ranges, math.pi / 180) == pytest.approx(math.radians(4.0))
    assert sc.outer_chord(0.85, 0.85, 4, math.pi / 180) == pytest.approx(2 * 0.85 * math.sin(math.radians(2.5)))
    assert sc.publishable(ranges, 0.05, 0.5)[0]                # bin-only: outer chord 0.074 m, a minor sector


def test_the_same_angular_gap_in_front_of_a_far_wall_is_refused():
    ok, why = sc.publishable(_ring_at(3.0, gap_bins=(179, 180, 181)), 0.05, 0.5)
    assert not ok and "5.0 deg apart" in why and "0.262 m apart" in why


def test_a_wedge_between_a_near_post_and_a_far_wall_is_refused():
    ranges = _ring_at(3.0, gap_bins=(180, 181, 182, 183))
    ranges[179] = 0.4                                                       # a post right of the gap, wall to its left
    ok, why = sc.publishable(ranges, 0.05, 0.5)
    assert not ok and "apart" in why


def test_the_chord_is_taken_between_the_bins_outer_edges():
    # 0.08 m at the bin centres, but up to (steps + 1) increments apart at the edges -> refused
    r = 0.08 / (2 * math.sin(math.radians(2.5)))                            # centre chord exactly 0.08 at 5 deg
    ranges = _ring_at(r, gap_bins=(179, 180, 181, 182))                     # 5 increments
    assert sc.outer_chord(r, r, 5, math.pi / 180) > 0.08
    assert sc.outer_chord(r, r, 5, math.pi / 180) == pytest.approx(2 * r * math.sin(math.radians(3.0)))


def test_coarse_sampling_is_refused_far_and_accepted_where_its_hits_touch():
    far = sc.ranges_from_points(ring(radius=3.0), **dict(PARAMS, angle_increment=math.radians(5.0)))
    assert not sc.publishable(far, 0.05, 0.5, angle_increment=math.radians(5.0))[0]
    near = sc.ranges_from_points(ring(radius=0.4), **dict(PARAMS, angle_increment=math.radians(5.0)))
    assert sc.publishable(near, 0.05, 0.5, angle_increment=math.radians(5.0))[0]     # outer chord 0.07 m


def test_with_the_actual_hit_points_the_chord_is_exact():
    """Bins 177 (-3..-2 deg) and 182 (2..3 deg) observed, 178-181 empty: 5 increments apart. The actual returns sit at
    -2.05 / +2.05 deg on a wall 1 m away, 0.0715 m apart - closed by the exact chord; known only by bin, the outer
    edges are 6 deg apart (0.105 m) and the same scan is refused."""
    bearings = [d + 0.5 for d in range(-180, 180) if not -3 <= d <= 2] + [-2.05, 2.05]
    points = [(math.cos(math.radians(b)), math.sin(math.radians(b)), 0.2) for b in bearings]
    hits = sc.hits_from_points(points, **PARAMS)
    assert [hits[i] is None for i in range(177, 183)] == [False, True, True, True, True, False]
    ranges = [float("nan") if h is None else h[0] for h in hits]
    assert sc.hit_chord(hits[177], hits[182]) == pytest.approx(2 * math.sin(math.radians(2.05)))
    assert sc.open_gaps(ranges, math.pi / 180, hits=hits) == []
    assert sc.publishable(ranges, 0.05, 0.5, hits=hits)[0] and not sc.publishable(ranges, 0.05, 0.5)[0]



def _hits(*returns, n=360):
    """Per-bin hits from explicit (range, bearing_deg) returns, as hits_from_points makes them."""
    points = [(r * math.cos(math.radians(b)), r * math.sin(math.radians(b)), 0.2) for r, b in returns]
    hits = sc.hits_from_points(points, **PARAMS)
    return [float("nan") if h is None else h[0] for h in hits], hits


@pytest.mark.parametrize("returns", [
    [(0.5, 0.2), (0.5, 1.2)],                  # Codex #289: two returns 1 deg apart, 359 deg blind
    [(0.5, 179.6), (0.5, -179.6)],             # the same across the +-180 seam
    [(0.2, 10.0), (0.2, 40.0)],                # a short chord (0.10 m) whose OTHER gap is a 330 deg reflex
])
def test_a_reflex_gap_is_never_closed_by_its_endpoints_chord(returns):
    ranges, hits = _hits(*returns)
    ok, why = sc.publishable(ranges, 0.05, 0.5, hits=hits)
    assert not ok and "reflex" in why
    assert not sc.publishable(ranges, 0.05, 0.5)[0]


def test_codex_289_exact_hit_angles_beat_the_bin_count():
    """Every other direction sampled at 1 deg; the gap's returns at 0.01 and 3.99 deg on a 5 m wall: bins only 3 apart,
    but the measured opening is 3.98 deg > 3.82 - refused with hits, and refused without (outer edges 5 deg)."""
    returns = [(5.0, d + 0.5) for d in range(-180, 180) if not 0 <= d <= 3] + [(5.0, 0.01), (5.0, 3.99)]
    ranges, hits = _hits(*returns)
    assert hits[180] is not None and hits[183] is not None and all(hits[i] is None for i in (181, 182))
    assert math.degrees(sc.hit_opening(hits[180], hits[183])) == pytest.approx(3.98)
    ok, why = sc.publishable(ranges, 0.05, 0.5, hits=hits)
    assert not ok and "4.0 deg apart" in why
    assert not sc.publishable(ranges, 0.05, 0.5)[0]


def test_the_opening_is_oriented_counter_clockwise():
    a, b = (1.0, math.radians(179.0)), (1.0, math.radians(-179.0))
    assert math.degrees(sc.hit_opening(a, b)) == pytest.approx(2.0)                # across the seam
    assert math.degrees(sc.hit_opening(b, a)) == pytest.approx(358.0)
    assert sc.hit_opening(a, a) == pytest.approx(2 * math.pi)
