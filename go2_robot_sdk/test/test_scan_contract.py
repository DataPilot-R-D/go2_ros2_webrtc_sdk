"""T5.9 Go2 scan contract (Codex #263 / #264; offline): measured return, never-claimed no-hit, unobserved.

The inherited converter initialised every bin to +inf; with the shared profile's inf_is_valid that turned an empty,
sparse or height-filtered cloud into a fresh all-free 360-degree scan that cleared real obstacles, and it placed
clouds with the LATEST transform. Here an empty bin is NaN (unobserved: Nav2 neither marks nor clears), +inf is never
emitted, and an empty or stale cloud is not converted at all (the sensor-stall contract stops the controller)."""
import ast
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
    assert not ok and "unobserved gap" in why


def test_a_fully_observed_ring_is_published_and_one_missing_bin_is_tolerated():
    assert sc.publishable(sc.ranges_from_points(ring(), **PARAMS), 0.05, 0.5)[0]
    ranges = sc.ranges_from_points(ring(skip=lambda b: 10 < b < 12), **PARAMS)     # 2 bins = 2 deg < the bound
    assert math.degrees(sc.max_unobserved_gap(ranges, math.pi / 180)) == pytest.approx(2.0)
    assert sc.publishable(ranges, 0.05, 0.5)[0]


@pytest.mark.parametrize("hole", [
    lambda b: -20 < b < 20,          # Codex #275: the central travel corridor, 0.73 m wide at 1 m
    lambda b: 70 < b < 110,          # a turn-side hole: in-place rotation sweeps it into the travel direction
    lambda b: abs(b) > 170,          # wrapping across +-180
    lambda b: 30 < b < 35,           # five degrees, just above the ~3.8 deg bound
])
def test_an_unobserved_sector_anywhere_around_the_robot_refuses_the_scan(hole):
    ranges = sc.ranges_from_points(ring(skip=hole), **PARAMS)
    ok, why = sc.publishable(ranges, 0.05, 0.5)
    assert not ok and "unobserved gap" in why


def test_the_gap_bound_is_the_stated_geometric_assumption():
    assert sc.MAX_UNOBSERVED_GAP == pytest.approx(2 * math.atan(0.05 / 1.5))
    assert math.degrees(sc.MAX_UNOBSERVED_GAP) < 4.0


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


def test_the_node_places_each_cloud_at_its_own_stamp_and_never_builds_an_all_inf_scan():
    """Source boundary of pointcloud_to_scan.cloud_cb (the node imports rclpy, so it is checked from source)."""
    source = (SDK_PKG / "go2_robot_sdk" / "pointcloud_to_scan.py").read_text()
    callback = next(n for n in ast.walk(ast.parse(source)) if isinstance(n, ast.FunctionDef) and n.name == "cloud_cb")
    body = ast.unparse(callback)
    assert "lookup_transform(self.output_frame, source_frame, rclpy.time.Time.from_msg(msg.header.stamp)" in body
    assert "float('inf')" not in body and "rclpy.time.Time()" not in body
    assert "publishable(ranges" in body and "ranges_from_points(" in body and "max_gap=" in body
    assert "to_frame(points" in body
    assert body.index("publishable(ranges") < body.index("self.pub.publish(scan)")
