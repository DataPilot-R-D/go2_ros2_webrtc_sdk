"""Republish Go2 USLAM's onboard localization as the map→odom TF.

Standalone alternative to AMCL: instead of running a 2D particle filter
over /scan + a saved PGM, we let the firmware's 3D LIO solve the
localization problem and just forward the answer into ROS2's TF tree.

USLAM publishes /localization/odom (frame_id=map, child=base_link) — the
same 3D LIO that built the map continues to localize against it. This node
bridges that pose into the standard Nav2 contract by computing
T_map_odom = T_map_base * inv(T_odom_base) and broadcasting it as a TF.

Why this exists: AMCL collapses on the Go2 in narrow doorways because
- a 2D scan slice from a 3D LiDAR aliases between similar corridors
- Quadruped pivot-step drift violates AMCL's diff/omni motion models
The firmware LIO sees the full point cloud and uses IMU, so it doesn't
suffer either problem.

Enable by setting GO2_USE_USLAM_LOCALIZATION=1 in go2-supervisor.py —
the supervisor skips AMCL and spawns this node instead.
"""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


def _quat_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Build a 3x3 rotation matrix from a quaternion (x,y,z,w)."""
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1.0 - s * (qy * qy + qz * qz), s * (qx * qy - qz * qw), s * (qx * qz + qy * qw)],
        [s * (qx * qy + qz * qw), 1.0 - s * (qx * qx + qz * qz), s * (qy * qz - qx * qw)],
        [s * (qx * qz - qy * qw), s * (qy * qz + qx * qw), 1.0 - s * (qx * qx + qy * qy)],
    ])


def _rotation_matrix_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to a quaternion (x,y,z,w). Numerically
    stable form from Shepperd 1978 — picks the largest diagonal term to
    avoid divide-by-near-zero when one axis dominates."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return float(qx), float(qy), float(qz), float(qw)


def _pose_to_matrix(position, orientation) -> np.ndarray:
    """Build a 4x4 transform from geometry_msgs Point + Quaternion."""
    M = np.eye(4)
    M[:3, :3] = _quat_to_rotation_matrix(
        orientation.x, orientation.y, orientation.z, orientation.w
    )
    M[0, 3] = position.x
    M[1, 3] = position.y
    M[2, 3] = position.z
    return M


def _matrix_to_transform(M: np.ndarray) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    t = (float(M[0, 3]), float(M[1, 3]), float(M[2, 3]))
    q = _rotation_matrix_to_quat(M[:3, :3])
    return t, q


class UslamLocalizationNode(Node):

    def __init__(self) -> None:
        super().__init__("uslam_localization")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("broadcast_rate_hz", 20.0)
        # Drop stale localization fixes — USLAM @ ~20 Hz means a >0.5s
        # gap is a stream stall (WebRTC backpressure). Keep the last
        # transform alive instead of broadcasting a stale fix.
        self.declare_parameter("max_loc_age_s", 0.5)
        # The driver's /odom is a relative drift estimate from leg
        # kinematics + IMU. We only need an odom→base_link snapshot at
        # the same moment as the localization fix.
        self.declare_parameter("max_odom_age_s", 0.3)

        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        rate = float(self.get_parameter("broadcast_rate_hz").value)
        self.max_loc_age = float(self.get_parameter("max_loc_age_s").value)
        self.max_odom_age = float(self.get_parameter("max_odom_age_s").value)

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Cache the most recent fix from each stream — they tick
        # asynchronously and we fuse on the broadcast tick.
        self._loc_msg: Odometry | None = None
        self._odom_msg: Odometry | None = None
        self._loc_rx_time: float = 0.0
        self._odom_rx_time: float = 0.0
        self._broadcast_count = 0
        self._last_loc_log_t = 0.0
        self._first_fix_logged = False
        self._silent_loc_warned = False
        self._silent_odom_warned = False
        self._start_t = time.monotonic()

        self.create_subscription(
            Odometry, "/localization/odom", self._on_localization_odom, sensor_qos
        )
        self.create_subscription(
            Odometry, "/odom", self._on_odom, sensor_qos
        )

        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_timer(1.0 / max(rate, 1.0), self._broadcast_tick)

        self.get_logger().info(
            f"USLAM-localization bridge up: "
            f"{self.map_frame}->{self.odom_frame} @ {rate:.1f} Hz "
            f"(waiting for /localization/odom + /odom...)"
        )

    def _on_localization_odom(self, msg: Odometry) -> None:
        self._loc_msg = msg
        self._loc_rx_time = time.monotonic()
        if not self._first_fix_logged:
            self._first_fix_logged = True
            p = msg.pose.pose.position
            self.get_logger().info(
                f"First USLAM fix received: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) "
                f"in frame={msg.header.frame_id!r}"
            )

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_msg = msg
        self._odom_rx_time = time.monotonic()

    def _broadcast_tick(self) -> None:
        # Guard against the shutdown race — timer can fire between
        # destroy_node() and shutdown(), at which point
        # TransformBroadcaster.sendTransform() raises RCLError "context is
        # not valid". Bailing here keeps the driver process exit clean,
        # which matters because the supervisor restarts code=1 processes
        # and a restart loop leaves DDS ghosts (verified 2026-05-12).
        if not rclpy.ok():
            return
        now = time.monotonic()

        # Health logging: wait for first sample on each stream before warning.
        # Check /odom first because its absence means the driver itself isn't
        # publishing — without that, the localization warning would be
        # misleading. Both warnings use the same 10s window so the more
        # actionable diagnostic doesn't get masked by the other.
        if self._odom_msg is None:
            if now - self._start_t > 10.0 and not self._silent_odom_warned:
                self._silent_odom_warned = True
                self.get_logger().warning(
                    "/odom silent after 10s — driver not publishing? Without "
                    "an odom→base_link sample we can't compute map→odom."
                )
            return
        if self._loc_msg is None:
            if now - self._start_t > 10.0 and not self._silent_loc_warned:
                self._silent_loc_warned = True
                self.get_logger().warning(
                    "/localization/odom silent after 10s. "
                    "Is the driver running with GO2_USLAM_LOCALIZATION_KICK=1, "
                    "or has the operator triggered localization/start manually?"
                )
            return

        loc_age = now - self._loc_rx_time
        odom_age = now - self._odom_rx_time

        # Skip when either stream goes stale: a stale fix would freeze
        # the robot's pose in the map and Nav2 would chase ghosts.
        if loc_age > self.max_loc_age:
            if self._broadcast_count > 0 and self._broadcast_count % 60 == 0:
                self.get_logger().warning(
                    f"/localization/odom stale ({loc_age:.2f}s) — TF not broadcast"
                )
            return
        if odom_age > self.max_odom_age:
            return

        # T_map_base from USLAM, T_odom_base from leg kinematics.
        # The driver constructs both — pose+orientation in standard ROS form.
        T_map_base = _pose_to_matrix(
            self._loc_msg.pose.pose.position, self._loc_msg.pose.pose.orientation
        )
        T_odom_base = _pose_to_matrix(
            self._odom_msg.pose.pose.position, self._odom_msg.pose.pose.orientation
        )

        try:
            T_base_odom = np.linalg.inv(T_odom_base)
        except np.linalg.LinAlgError:
            self.get_logger().warning("Singular /odom transform — skipping broadcast")
            return

        T_map_odom = T_map_base @ T_base_odom
        translation, rotation = _matrix_to_transform(T_map_odom)

        tf_msg = TransformStamped()
        # Stamp the broadcast with ROS-now, not the localization message's
        # header.stamp. Driver republishes /localization/odom with the driver
        # node's ingest time, which by the time we read+fuse+broadcast is
        # tens of ms in the past. tf2 rejects "extrapolation into the past"
        # if a consumer (costmap) queries by time slightly ahead. max_loc_age_s
        # already gates freshness, so stamping with now() is safe.
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.odom_frame
        tf_msg.transform.translation.x = translation[0]
        tf_msg.transform.translation.y = translation[1]
        tf_msg.transform.translation.z = translation[2]
        tf_msg.transform.rotation.x = rotation[0]
        tf_msg.transform.rotation.y = rotation[1]
        tf_msg.transform.rotation.z = rotation[2]
        tf_msg.transform.rotation.w = rotation[3]

        self._tf_broadcaster.sendTransform(tf_msg)
        self._broadcast_count += 1

        # Heartbeat: every 5s log current map-frame pose so the operator
        # can sanity-check against the physical robot.
        if now - self._last_loc_log_t > 5.0:
            self._last_loc_log_t = now
            yaw = math.degrees(math.atan2(2.0 * (rotation[3] * rotation[2]),
                                          1.0 - 2.0 * (rotation[2] * rotation[2])))
            p = self._loc_msg.pose.pose.position
            self.get_logger().info(
                f"map->base: ({p.x:.3f}, {p.y:.3f}) "
                f"yaw_odom_correction={yaw:+.1f}° "
                f"loc_age={loc_age*1000:.0f}ms broadcasts={self._broadcast_count}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UslamLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
