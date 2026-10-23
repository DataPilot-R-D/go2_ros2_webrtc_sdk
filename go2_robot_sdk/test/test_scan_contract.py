"""T5.9 Go2 scan contract (Codex #263 / #264; offline): measured return, never-claimed no-hit, unobserved.

The inherited converter initialised every bin to +inf; with the shared profile's inf_is_valid that turned an empty,
sparse or height-filtered cloud into a fresh all-free 360-degree scan that cleared real obstacles, and it placed
clouds with the LATEST transform. Here an empty bin is NaN (unobserved: Nav2 neither marks nor clears), +inf is never
emitted, and an empty or stale cloud is not converted at all (the sensor-stall contract stops the controller)."""
import ast
import math
from pathlib import Path
import sys

SDK_PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SDK_PKG))
from go2_robot_sdk import scan_contract as sc  # noqa: E402

PARAMS = dict(min_height=-0.1, max_height=0.5, range_min=0.15, range_max=10.0, angle_min=-math.pi, angle_max=math.pi,
              angle_increment=math.pi / 180.0)


def test_an_empty_cloud_gives_no_measured_return_and_is_not_published():
    ranges = sc.ranges_from_points([], **PARAMS)
    assert len(ranges) == 360 and all(math.isnan(r) for r in ranges)          # unobserved, never +inf
    assert sc.publishable(ranges, 0.05, 0.5) == (False, "no measured return in the cloud")


def test_a_sparse_cloud_marks_only_its_bins_and_never_claims_free_space():
    ranges = sc.ranges_from_points([(2.0, 0.0, 0.2)], **PARAMS)
    finite = [r for r in ranges if math.isfinite(r)]
    assert finite == [2.0] and sum(math.isnan(r) for r in ranges) == 359 and not any(math.isinf(r) for r in ranges)
    assert sc.publishable(ranges, 0.05, 0.5)[0]


def test_a_fully_height_filtered_cloud_is_not_free_space():
    ranges = sc.ranges_from_points([(2.0, 0.0, 1.5), (1.0, 1.0, -0.5)], **PARAMS)
    assert all(math.isnan(r) for r in ranges) and not sc.publishable(ranges, 0.05, 0.5)[0]


def test_the_nearest_return_wins_and_non_finite_points_are_ignored():
    ranges = sc.ranges_from_points([(3.0, 0.0, 0.2), (1.5, 0.0, 0.2), (float("nan"), 0.0, 0.2)], **PARAMS)
    assert [r for r in ranges if math.isfinite(r)] == [1.5]


def test_a_stale_or_future_cloud_is_never_converted():
    ranges = sc.ranges_from_points([(2.0, 0.0, 0.2)], **PARAMS)
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
    assert "publishable(ranges" in body and "ranges_from_points(" in body
    assert body.index("publishable(ranges") < body.index("self.pub.publish(scan)")
