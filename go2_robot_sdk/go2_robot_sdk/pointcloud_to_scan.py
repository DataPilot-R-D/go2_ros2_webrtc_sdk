"""Convert PointCloud2 to LaserScan for SLAM Toolbox and Nav2.

The Go2 driver publishes /point_cloud2 with points expressed in the
`odom` frame (points are world-space coordinates coming out of Go2's
internal utlidar SLAM). A naive range/angle computation treats them
as sensor-relative which makes the apparent obstacles slide around
in odom while the robot moves — SLAM Toolbox then double-applies the
pose change and the map smears. Transform every point from the cloud's
frame into `output_frame` (base_link via the static tf chain) first.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from go2_robot_sdk.scan_contract import MAX_RAY_SEPARATION, publishable, ranges_from_points, to_frame


class PointCloudToScan(Node):
    def __init__(self):
        super().__init__('pointcloud_to_scan')

        self.declare_parameter('min_height', -0.1)
        self.declare_parameter('max_height', 0.5)
        self.declare_parameter('range_min', 0.15)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', math.pi / 180.0)
        self.declare_parameter('input_topic', '/point_cloud2')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('output_frame', 'radar')
        # T5.9 scan contract: a cloud older than this (vs this node's clock) is never turned into a scan.
        self.declare_parameter('max_cloud_age_s', 0.5)
        # Coverage (Codex #271 / #275 / #276): the widest separation of neighbouring observed rays anywhere around the robot (rotation
        # sweeps every direction); the default is scan_contract's stated, UNCALIBRATED assumption.
        self.declare_parameter('max_ray_separation_deg', math.degrees(MAX_RAY_SEPARATION))

        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.max_cloud_age_s = self.get_parameter('max_cloud_age_s').value
        self.max_ray_separation = math.radians(self.get_parameter('max_ray_separation_deg').value)

        self.num_ranges = int(
            (self.angle_max - self.angle_min) / self.angle_increment)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PointCloud2, input_topic, self.cloud_cb, 10)
        self.pub = self.create_publisher(LaserScan, output_topic, 10)

        # Rate-limit the "waiting for tf" log so it doesn't flood.
        self._tf_warn_last_ns = 0

        self.get_logger().info(
            f"Converting {input_topic} -> {output_topic} "
            f"(frame={self.output_frame}, {self.num_ranges} rays)")

    def cloud_cb(self, msg):
        source_frame = msg.header.frame_id
        # Look up transform from cloud frame into the sensor/base frame AT THE CLOUD'S STAMP (Codex #264: the latest
        # transform would place an old cloud with the robot's newer pose).
        try:
            tf = self.tf_buffer.lookup_transform(
                self.output_frame, source_frame,
                rclpy.time.Time.from_msg(msg.header.stamp),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._tf_warn_last_ns > 2_000_000_000:
                self.get_logger().warning(
                    f"No transform {source_frame} -> {self.output_frame}: {exc}"
                )
                self._tf_warn_last_ns = now_ns
            return

        t = tf.transform.translation
        r_q = tf.transform.rotation
        tx, ty, tz = t.x, t.y, t.z
        q = (r_q.x, r_q.y, r_q.z, r_q.w)

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.output_frame
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.time_increment = 0.0
        scan.scan_time = 0.1

        points = ((p[0], p[1], p[2]) for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        ranges = ranges_from_points(
            to_frame(points, (tx, ty, tz), q), min_height=self.min_height, max_height=self.max_height, range_min=self.range_min,
            range_max=self.range_max, angle_min=self.angle_min, angle_max=self.angle_max,
            angle_increment=self.angle_increment)
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
        ok, why = publishable(ranges, age, self.max_cloud_age_s, angle_increment=self.angle_increment,
                              max_gap=self.max_ray_separation)
        if not ok:
            # No scan: the obstacle layer's expected_update_rate stops the controller (sensor-stall contract).
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._tf_warn_last_ns > 2_000_000_000:
                self.get_logger().warning(f"cloud not converted: {why}")
                self._tf_warn_last_ns = now_ns
            return

        scan.ranges = [float(r) for r in ranges]
        self.pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
