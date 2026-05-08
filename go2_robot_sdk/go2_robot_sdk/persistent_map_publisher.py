"""Accumulate Go2's rolling voxel windows into a persistent global map.

Go2 streams only a 6.4 m × 6.4 m voxel window centered on the robot at
~2 Hz. The window REPLACES itself each tick — once the robot walks out
of a room, the points for that room are gone from `/point_cloud2`. Nav2
then sees only the current room as obstacles + "empty free space" beyond
lidar range, so the planner can never route between rooms that the
robot has already traversed.

This node accumulates every occupied voxel into a fixed 20 m × 20 m
grid @ 5 cm resolution, centered on the robot's starting origin in the
odom frame. Cells never seen stay `-1` (unknown); any cell that had a
lidar return at least once is marked `100` (occupied). With Nav2's
`allow_unknown: true` planner setting, unknown cells are traversable,
so the robot can plan through hallways it has walked through.

Trade-offs accepted for PoC:
  * No ray-cast clearing — moved objects stay as obstacles forever.
  * Requires the robot to physically traverse the path first (map builds
    as you teleop around).
  * Tied to Go2's odom frame. If Go2's onboard SLAM rebases (rare but
    possible), accumulated cells drift. Session-scoped map only.
"""

from __future__ import annotations

import atexit
import os
import signal
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PersistentMapPublisher(Node):
    def __init__(self) -> None:
        super().__init__("persistent_map_publisher")

        self.declare_parameter("width_m", 20.0)
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("input_topic", "/point_cloud2")
        self.declare_parameter("output_topic", "/map")
        self.declare_parameter("publish_rate_hz", 1.0)
        # Exclude floor returns (below min_z) and ceiling fixtures (above
        # max_z) — they create false obstacles everywhere the robot has
        # walked. Go2's lidar is mounted ~0.3 m off the ground so min_z
        # in odom frame (flat floor at z=0) maps to ~0.1 to skip it.
        # Z filter — keep walls, exclude floor/body (below) and door
        # frames + ceiling fixtures (above). Door frame tops and lamps
        # at z > 1.0 otherwise accumulate as ghost walls stretched
        # across doorway openings when the laser grazes them.
        self.declare_parameter("min_z", 0.35)
        self.declare_parameter("max_z", 1.0)
        # Radius around robot (in meters) to force-mark as free each
        # cloud. Any occupied cell within this radius is cleared back
        # to 0 (free). Needed because robot's own body still produces
        # some returns that would otherwise accumulate as lethal cells
        # directly under the footprint → planner fails with "Starting
        # point in lethal space". Also covers cells the robot has just
        # walked through so they stay known-free.
        self.declare_parameter("self_clear_radius", 0.8)
        # On-disk persistence — survives supervisor restarts so we don't
        # lose the map every time we tweak a Nav2 parameter. Saved as a
        # plain numpy .npy file with the grid dimensions baked in (if
        # dimensions change at next startup we discard and start fresh).
        self.declare_parameter("state_file", "/tmp/go2_persistent_map.npy")
        # Minimum connected-component size for an occupied cluster to
        # survive filtering. Single-cell returns (dust, glare, Go2 body
        # reflections that escaped self-clear) get stripped before /map
        # is published, so ghost walls can't inflate into path barriers.
        self.declare_parameter("min_cluster_cells", 3)

        self.width_m: float = self.get_parameter("width_m").value
        self.res: float = self.get_parameter("resolution").value
        self.frame_id: str = self.get_parameter("frame_id").value
        self.min_z: float = self.get_parameter("min_z").value
        self.max_z: float = self.get_parameter("max_z").value
        self.self_clear_radius: float = self.get_parameter("self_clear_radius").value
        self.state_file: Path = Path(self.get_parameter("state_file").value)
        self.min_cluster_cells: int = int(self.get_parameter("min_cluster_cells").value)
        # Robot pose in odom, updated from /odom. None until first odom msg.
        self._robot_xy: tuple[float, float] | None = None
        input_topic: str = self.get_parameter("input_topic").value
        output_topic: str = self.get_parameter("output_topic").value
        rate_hz: float = self.get_parameter("publish_rate_hz").value

        self.cells = int(self.width_m / self.res)
        # Origin of the grid in odom frame — bottom-left corner at
        # (-width/2, -width/2). Cell (cells/2, cells/2) sits at (0, 0).
        self.origin_x = -self.width_m / 2.0
        self.origin_y = -self.width_m / 2.0

        # int8 array: -1 unknown, 0 free (raytraced), 100 occupied.
        # Free cells come from raytrace clearing + self-clear; occupied
        # from lidar hits; unknown = never seen. Try to reload from disk
        # if a compatible snapshot exists, otherwise fresh.
        self.grid = self._load_or_init_grid()

        signal.signal(signal.SIGTERM, self._on_signal)
        signal.signal(signal.SIGINT, self._on_signal)
        atexit.register(self._save_grid)

        qos_map = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        # Match the Go2 driver's publisher QoS exactly — driver uses
        # QoSProfile(depth=10) which defaults to RELIABLE. A BEST_EFFORT
        # subscriber on a RELIABLE publisher *should* work per RMW rules
        # but doesn't reliably match on all DDS vendors on macOS.
        qos_cloud = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.pub = self.create_publisher(OccupancyGrid, output_topic, qos_map)
        self.sub_cloud = self.create_subscription(
            PointCloud2, input_topic, self._on_cloud, qos_cloud
        )
        # Odom QoS matches the Go2 driver's QoSProfile(depth=10) default.
        qos_odom = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.sub_odom = self.create_subscription(
            Odometry, "/odom", self._on_odom, qos_odom
        )
        self.timer = self.create_timer(1.0 / rate_hz, self._publish)

        self.get_logger().info(
            f"Accumulating {input_topic} into {output_topic} — "
            f"{self.cells}×{self.cells} cells ({self.width_m:.0f}m, "
            f"{self.res}m/cell), z∈[{self.min_z}, {self.max_z}]"
        )
        self._cloud_count = 0
        self._occupied_count = 0
        self._first_cloud_logged = False

    def _load_or_init_grid(self) -> np.ndarray:
        """Reload grid from disk if shape matches; otherwise start fresh."""
        expected_shape = (self.cells, self.cells)
        if self.state_file.exists():
            try:
                loaded = np.load(self.state_file)
                if loaded.shape == expected_shape and loaded.dtype == np.int8:
                    occ = int((loaded == 100).sum())
                    free = int((loaded == 0).sum())
                    self.get_logger().info(
                        f"Reloaded persistent map from {self.state_file}: "
                        f"{occ} occupied, {free} free cells"
                    )
                    return loaded
                self.get_logger().warning(
                    f"Disk map shape={loaded.shape} dtype={loaded.dtype} "
                    f"incompatible with current {expected_shape} int8 — starting fresh"
                )
            except Exception as exc:
                self.get_logger().warning(f"Disk map load failed: {exc} — fresh grid")
        return np.full(expected_shape, -1, dtype=np.int8)

    def _save_grid(self) -> None:
        """Persist grid to disk on shutdown. Idempotent, safe on re-entry."""
        try:
            tmp = self.state_file.with_suffix(self.state_file.suffix + ".tmp")
            np.save(tmp, self.grid)
            os.replace(str(tmp), str(self.state_file))
        except Exception:
            # Shutdown path — don't raise, just swallow.
            pass

    def _on_signal(self, signum: int, _frame=None) -> None:
        """Save on signal then let rclpy handle the shutdown normally."""
        self._save_grid()
        # Request rclpy to exit its spin loop.
        rclpy.try_shutdown()

    def _on_odom(self, msg: Odometry) -> None:
        self._robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _clear_self_radius(self) -> None:
        """Zero out the circular region around the robot back to free (0).

        Called once per cloud after marking occupied cells. Prevents
        Go2's own body and close-range floor returns from accumulating
        as lethal cells under the footprint.
        """
        if self._robot_xy is None:
            return
        rx, ry = self._robot_xy
        cx = (rx - self.origin_x) / self.res
        cy = (ry - self.origin_y) / self.res
        r_cells = self.self_clear_radius / self.res

        lo_x = max(0, int(cx - r_cells))
        hi_x = min(self.cells, int(cx + r_cells) + 1)
        lo_y = max(0, int(cy - r_cells))
        hi_y = min(self.cells, int(cy + r_cells) + 1)
        if lo_x >= hi_x or lo_y >= hi_y:
            return
        # Build a circular mask and mark cells inside as free (0).
        yy, xx = np.ogrid[lo_y:hi_y, lo_x:hi_x]
        mask = (xx - cx) ** 2 + (yy - cy) ** 2 <= r_cells ** 2
        # Index shape (hi_y-lo_y, hi_x-lo_x); assign with boolean mask.
        region = self.grid[lo_y:hi_y, lo_x:hi_x]
        region[mask] = 0
        self.grid[lo_y:hi_y, lo_x:hi_x] = region

    def _on_cloud(self, msg: PointCloud2) -> None:
        if not self._first_cloud_logged:
            self.get_logger().info(
                f"first /point_cloud2 received: frame_id='{msg.header.frame_id}' "
                f"points_approx={msg.width * msg.height}"
            )
            self._first_cloud_logged = True
        if msg.header.frame_id not in ("odom", "map"):
            self.get_logger().warning(
                f"cloud frame_id='{msg.header.frame_id}', expected 'odom' — skipping"
            )
            return
        if self._robot_xy is None:
            # Need robot pose for ray-trace origin; drop this cloud.
            return

        points = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        if points.size == 0:
            return

        # Filter by z for "wall" points (these become occupied).
        z = points["z"]
        mask = (z >= self.min_z) & (z <= self.max_z)
        if not mask.any():
            return
        xs = points["x"][mask]
        ys = points["y"][mask]

        self._raytrace(xs, ys)
        # Clear self-radius AFTER ray-trace so newly-occupied cells
        # under the robot's own reflections get reset to free.
        self._clear_self_radius()
        self._cloud_count += 1
        self._occupied_count = int((self.grid == 100).sum())

    def _raytrace(self, xs: np.ndarray, ys: np.ndarray) -> None:
        """Mark cells along each ray from robot to point as free,
        endpoint as occupied.

        Vectorised: for each of N points we sample the segment at
        resolution `res` and scatter-assign. A point at distance d
        gets ceil(d/res) interior samples + 1 endpoint.

        Performance: for ~5 k filtered points at ~3 m mean range and
        5 cm resolution, that's ~300 k cell writes per cloud. At 2 Hz
        = 600 k/sec — well within numpy's reach on M1.
        """
        rx, ry = self._robot_xy
        # Convert world → grid cell (float, rounded at scatter time).
        rcx = (rx - self.origin_x) / self.res
        rcy = (ry - self.origin_y) / self.res
        pcx = (xs - self.origin_x) / self.res
        pcy = (ys - self.origin_y) / self.res

        # Distance in cells per ray; determines sample count.
        dx = pcx - rcx
        dy = pcy - rcy
        lengths = np.maximum(np.abs(dx), np.abs(dy))  # chebyshev = step count
        # Guard against zero-length rays (point coincident with robot).
        ok = lengths > 0
        if not ok.any():
            return
        dx = dx[ok]
        dy = dy[ok]
        pcx = pcx[ok]
        pcy = pcy[ok]
        lengths = lengths[ok]

        # Use max length so every ray gets the same number of samples;
        # shorter rays will have duplicate cells but the assignment is
        # idempotent, so it's cheap.
        n_steps = int(min(np.max(lengths), 400)) + 1  # cap at grid width
        t = np.linspace(0.0, 1.0, n_steps + 1)  # 0 → 1 inclusive

        # Broadcast: (n_points, n_steps+1) arrays of cell indices
        ray_cx = (rcx + np.outer(dx, t)).astype(np.int32)
        ray_cy = (rcy + np.outer(dy, t)).astype(np.int32)

        # Interior cells (t < 1) → free. Endpoint (last column) → occupied.
        in_bounds = (
            (ray_cx >= 0) & (ray_cx < self.cells)
            & (ray_cy >= 0) & (ray_cy < self.cells)
        )

        # Scatter free (exclude the last column which is the hit cell)
        free_cx = ray_cx[:, :-1][in_bounds[:, :-1]]
        free_cy = ray_cy[:, :-1][in_bounds[:, :-1]]
        self.grid[free_cy, free_cx] = 0

        # Scatter occupied endpoints
        hit_cx = ray_cx[:, -1][in_bounds[:, -1]]
        hit_cy = ray_cy[:, -1][in_bounds[:, -1]]
        self.grid[hit_cy, hit_cx] = 100

    def _filtered_grid(self) -> np.ndarray:
        """Return a copy of self.grid with isolated single-cell occupied
        returns demoted to free (0). Uses a numpy-only 3×3 neighbour
        count — cheaper than full connected-component labeling and fits
        our actual need (killing speckle, not filtering large structures).

        A cell is kept occupied only if ≥ `min_cluster_cells - 1` of its
        8 neighbours are ALSO occupied. With `min_cluster_cells=3` that
        means the cell must have ≥2 occupied neighbours — isolated
        specks and 2-cell pairs are removed; genuine 3+ cell clusters
        (even in a line) survive.
        """
        if self.min_cluster_cells <= 1:
            return self.grid
        occupied = (self.grid == 100).astype(np.uint8)
        # 3×3 neighbour sum including self via shifted-slice addition.
        padded = np.pad(occupied, 1, mode="constant")
        neighbour_sum = np.zeros_like(occupied, dtype=np.int32)
        for dy in range(3):
            for dx in range(3):
                neighbour_sum += padded[dy:dy + occupied.shape[0], dx:dx + occupied.shape[1]]
        # neighbour_sum now includes self (1) + up to 8 neighbours.
        # Keep cell if total ≥ min_cluster_cells (self + ≥ N-1 neighbours).
        keep = (neighbour_sum >= self.min_cluster_cells) & (occupied == 1)
        drop = (occupied == 1) & ~keep
        if not drop.any():
            return self.grid
        out = self.grid.copy()
        out[drop] = 0  # demote noise to free, not unknown
        return out

    def _publish(self) -> None:
        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.res
        msg.info.width = self.cells
        msg.info.height = self.cells
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        published = self._filtered_grid()
        msg.data = published.flatten().tolist()
        self.pub.publish(msg)

        # Periodic stats so we can watch the map fill up without
        # attaching Foxglove.
        if self._cloud_count and self._cloud_count % 20 == 0:
            raw_occ = int((self.grid == 100).sum())
            pub_occ = int((published == 100).sum())
            self.get_logger().info(
                f"clouds={self._cloud_count} "
                f"occ_raw={raw_occ} occ_published={pub_occ} "
                f"noise_dropped={raw_occ - pub_occ} "
                f"coverage={pub_occ / (self.cells * self.cells) * 100:.2f}%"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PersistentMapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
