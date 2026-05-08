"""Publish a synthetic /map so Nav2's static_layer can activate.

When we use the robot's onboard SLAM (GO2_ONBOARD_SLAM=1), there is no
`slam_toolbox` producing a `nav_msgs/OccupancyGrid` on /map. Nav2's
`static_layer` (declared in both global and local costmaps) blocks
costmap updates until it receives a map, so planner_server and
controller_server log "Can't update static costmap layer, no map
received" and every NavigateToPose goal hangs in "started" forever.

This node publishes a 100m × 100m all-unknown (value 0 / free) map once,
with transient-local durability so any late Nav2 subscriber still
latches it. Nav2 then treats the global cost map as "all free, except
what voxel_layer reports from /scan". In practice: local obstacle
avoidance works, global plans are straight lines inside the
all-free region.

This is a shim for closed-source onboard-SLAM robots. The long-term
fix is to decompress the robot's internal voxel map (e.g.
rt/utlidar/voxel_map_compressed on Go2) into a real OccupancyGrid —
but Go2's voxel format is proprietary and not worth implementing for
the PoC. The shim keeps Nav2 unblocked so we can validate the rest
of the dispatch chain.
"""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class EmptyMapPublisher(Node):
    def __init__(self) -> None:
        super().__init__("empty_map_publisher")

        self.declare_parameter("width_m", 100.0)
        self.declare_parameter("resolution", 0.1)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate_hz", 0.5)

        width_m: float = self.get_parameter("width_m").value
        resolution: float = self.get_parameter("resolution").value
        frame_id: str = self.get_parameter("frame_id").value
        rate_hz: float = self.get_parameter("publish_rate_hz").value

        cells = int(math.ceil(width_m / resolution))

        msg = OccupancyGrid()
        msg.header.frame_id = frame_id
        msg.info.resolution = resolution
        msg.info.width = cells
        msg.info.height = cells
        # Center the grid on the origin so robot(0,0) is in the middle
        msg.info.origin.position.x = -width_m / 2.0
        msg.info.origin.position.y = -width_m / 2.0
        msg.info.origin.orientation.w = 1.0
        # 0 = free. Nav2's voxel_layer will stamp obstacles on top from /scan.
        msg.data = [0] * (cells * cells)
        self._msg = msg

        # transient-local lets a late Nav2 subscriber (starting after us)
        # still receive the map — matches the QoS `slam_toolbox` uses.
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(OccupancyGrid, "/map", qos)
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f"Publishing empty {cells}×{cells} cell map "
            f"({width_m:.0f}m × {width_m:.0f}m @ {resolution}m/cell) on /map "
            f"in frame '{frame_id}'"
        )

    def _tick(self) -> None:
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EmptyMapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
