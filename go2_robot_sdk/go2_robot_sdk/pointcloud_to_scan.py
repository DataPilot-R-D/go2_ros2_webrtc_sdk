"""Convert PointCloud2 to LaserScan for SLAM Toolbox and Nav2."""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2


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

        self.sub = self.create_subscription(
            PointCloud2, input_topic, self.cloud_cb, 10)
        self.pub = self.create_publisher(LaserScan, output_topic, 10)

        self.get_logger().info(
            f"Converting {input_topic} -> {output_topic} "
            f"(frame={self.output_frame}, {self.num_ranges} rays)")

    def cloud_cb(self, msg):
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
            x, y, z = point

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
