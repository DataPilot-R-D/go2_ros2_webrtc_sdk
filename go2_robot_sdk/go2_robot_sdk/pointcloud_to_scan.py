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


def _quat_rotate(q, v):
    # Rotate 3-vector v by unit quaternion q (x,y,z,w).
    qx, qy, qz, qw = q
    x, y, z = v
    # q * v * q^-1, standard formula
    t2 = qw * qx; t3 = qw * qy; t4 = qw * qz
    t5 = -qx * qx; t6 = qx * qy; t7 = qx * qz
    t8 = -qy * qy; t9 = qy * qz; t10 = -qz * qz
    return (
        2 * ((t8 + t10) * x + (t6 - t4) * y + (t3 + t7) * z) + x,
        2 * ((t4 + t6) * x + (t5 + t10) * y + (t9 - t2) * z) + y,
        2 * ((t7 - t3) * x + (t2 + t9) * y + (t5 + t8) * z) + z,
    )


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
        # Look up transform from cloud frame into the sensor/base frame
        # so we can express every point relative to the robot.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.output_frame, source_frame,
                rclpy.time.Time(),  # latest available
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

        ranges = [float('inf')] * self.num_ranges

        for point in point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True):
            # Rotate then translate: target = q * src + t
            rx, ry, rz = _quat_rotate(q, (point[0], point[1], point[2]))
            x = rx + tx
            y = ry + ty
            z = rz + tz

            if z < self.min_height or z > self.max_height:
                continue

            r = math.sqrt(x * x + y * y)
            if r < self.range_min or r > self.range_max:
                continue

            angle = math.atan2(y, x)
            if angle < self.angle_min or angle > self.angle_max:
                continue

            idx = int((angle - self.angle_min) / self.angle_increment)
            if 0 <= idx < self.num_ranges:
                if r < ranges[idx]:
                    ranges[idx] = r

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
