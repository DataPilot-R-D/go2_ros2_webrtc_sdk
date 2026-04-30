#!/usr/bin/env python3
"""Custom pure-pursuit controller — drop-in replacement for Nav2 controller_server.

Subscribes to /plan (nav_msgs/Path) emitted by Nav2 planner_server and drives
/cmd_vel directly. Bypasses costmap collision check, lookahead scaling, and the
DWB/RPP heuristics that stall the Go2 in narrow doorways.

State machine mirrors DimOS LocalPlanner:
    idle -> initial_rotation -> path_following -> final_rotation -> arrived

Run as a ROS2 entry point (preferred — packaged via setup.py console_scripts):
    ros2 run go2_robot_sdk custom_controller --ros-args -p speed:=0.35

Wiring (see bottom of file): disable Nav2's controller_server in the
lifecycle_manager_navigation node_names list, leave planner_server +
bt_navigator + behavior_server alive. bt_navigator's FollowPath action will
fail (no controller_server) — that's fine, we listen to /plan directly.

Optional: drive the planner by sending a PoseStamped to /goal_pose (this node
forwards /goal_pose to the planner via NavigateToPose action OR you keep using
your existing Nav2 frontend; planner_server alone publishes /plan when its
ComputePathToPose action is called).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def angle_diff(a: float, b: float) -> float:
    """Shortest signed difference a-b in [-pi, pi]."""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def quat_to_yaw(qz: float, qw: float) -> float:
    return 2.0 * math.atan2(qz, qw)


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float

    def xy(self) -> np.ndarray:
        return np.array([self.x, self.y])


class CustomPurePursuit(Node):
    """Pure-pursuit /plan follower."""

    def __init__(self) -> None:
        super().__init__("custom_pure_pursuit")

        # Params (override via --ros-args -p name:=value)
        # 0.55 → 0.35 — slower for indoor, more time to react to obstacles
        self.declare_parameter("speed", 0.35)            # max forward m/s
        self.declare_parameter("min_linear", 0.2)        # Go2 cmd_vel floor
        self.declare_parameter("min_angular", 0.2)
        self.declare_parameter("k_angular", 0.5)         # P-gain on yaw
        self.declare_parameter("lookahead", 0.5)         # m ahead on path
        self.declare_parameter("xy_tolerance", 0.25)
        self.declare_parameter("yaw_tolerance", 0.2)
        self.declare_parameter("rotate_threshold", math.pi / 2)
        self.declare_parameter("control_hz", 10.0)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        # 0.20 → 0.25 — robot DOES move now (test #24) but bumps furniture
        # not on map. Bigger safety distance + narrower cone catches real
        # obstacles in front without false-stopping at corridor walls.
        self.declare_parameter("safety_min_range", 0.25)
        # Cone narrowed 30 → 15° — wider cone caught side walls in 1m
        # corridors. 15° forward only catches what's directly in robot's path.
        self.declare_parameter("safety_cone_deg", 15.0)
        self.declare_parameter("safety_enable", True)
        self.declare_parameter("stall_timeout_s", 5.0)

        self.speed = self.get_parameter("speed").value
        self.min_linear = self.get_parameter("min_linear").value
        self.min_angular = self.get_parameter("min_angular").value
        self.k_angular = self.get_parameter("k_angular").value
        self.lookahead = self.get_parameter("lookahead").value
        self.xy_tol = self.get_parameter("xy_tolerance").value
        self.yaw_tol = self.get_parameter("yaw_tolerance").value
        self.rotate_threshold = self.get_parameter("rotate_threshold").value
        self.control_hz = self.get_parameter("control_hz").value
        self.global_frame = self.get_parameter("global_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.safety_range = self.get_parameter("safety_min_range").value
        self.safety_cone = math.radians(self.get_parameter("safety_cone_deg").value)
        self.safety_enable = self.get_parameter("safety_enable").value
        self.stall_timeout = self.get_parameter("stall_timeout_s").value

        # TF
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        # State
        self.path: Optional[np.ndarray] = None  # (N,2)
        self.goal_yaw: float = 0.0
        self.state: str = "idle"
        self.last_progress_time: float = 0.0
        self.last_progress_dist: float = float("inf")
        self.scan_blocked: bool = False

        # IO. Nav2 planner_server publishes /plan as VOLATILE+RELIABLE+depth=1
        # (default sensor-data style). TRANSIENT_LOCAL on subscriber side
        # caused QoS mismatch and 0 messages received.
        plan_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Path, "/plan", self.on_plan, plan_qos)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.timer = self.create_timer(1.0 / self.control_hz, self.tick)
        self.get_logger().info(
            f"custom_pure_pursuit up — speed={self.speed} lookahead={self.lookahead}"
        )

    # ---------- callbacks ----------
    def on_plan(self, msg: Path) -> None:
        if not msg.poses:
            self.get_logger().warn("empty /plan received — ignoring")
            return
        pts = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
        last = msg.poses[-1].pose.orientation
        self.path = pts
        self.goal_yaw = quat_to_yaw(last.z, last.w)
        self.state = "initial_rotation"
        self.last_progress_time = self.get_clock().now().nanoseconds / 1e9
        self.last_progress_dist = float("inf")
        self.get_logger().info(f"new /plan: {len(pts)} pts, goal_yaw={self.goal_yaw:.2f}")

    def on_scan(self, msg: LaserScan) -> None:
        if not self.safety_enable:
            self.scan_blocked = False
            return
        # Keep only beams within ±cone of forward (LaserScan: 0 rad = forward
        # for most ROS conventions; pointcloud_to_laserscan inherits robot frame)
        ranges = np.array(msg.ranges, dtype=np.float32)
        n = len(ranges)
        if n == 0:
            return
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        mask = np.abs(angles) < self.safety_cone
        cone = ranges[mask]
        cone = cone[np.isfinite(cone) & (cone > 0.05)]
        if cone.size < 3:
            self.scan_blocked = False
            return
        self.scan_blocked = bool(np.min(cone) < self.safety_range)

    # ---------- TF ----------
    def lookup_pose(self) -> Optional[Pose2D]:
        try:
            tf = self.tf_buf.lookup_transform(
                self.global_frame, self.base_frame, rclpy.time.Time()
            )
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return Pose2D(t.x, t.y, quat_to_yaw(q.z, q.w))

    # ---------- main loop ----------
    def tick(self) -> None:
        if self.state in ("idle", "arrived"):
            return
        if self.path is None or len(self.path) == 0:
            return
        pose = self.lookup_pose()
        if pose is None:
            return  # wait for TF

        if self.scan_blocked and self.state == "path_following":
            self.get_logger().warn("scan blocked — emergency stop")
            self.publish(0.0, 0.0)
            return

        if self.state == "initial_rotation":
            self.do_initial_rotation(pose)
        elif self.state == "path_following":
            self.do_path_following(pose)
        elif self.state == "final_rotation":
            self.do_final_rotation(pose)

    def do_initial_rotation(self, pose: Pose2D) -> None:
        # Aim at the lookahead point (not the path's first stored yaw — Nav2
        # planner often leaves orientation untouched on intermediate poses).
        target = self.lookahead_point(pose.xy())
        desired = math.atan2(target[1] - pose.y, target[0] - pose.x)
        err = angle_diff(desired, pose.yaw)
        if abs(err) < self.yaw_tol:
            self.state = "path_following"
            self.do_path_following(pose)
            return
        self.publish(0.0, self.angular_with_floor(err))

    def do_path_following(self, pose: Pose2D) -> None:
        goal = self.path[-1]
        dist_to_goal = float(np.linalg.norm(goal - pose.xy()))

        # Stall detection
        now = self.get_clock().now().nanoseconds / 1e9
        if dist_to_goal + 0.05 < self.last_progress_dist:
            self.last_progress_dist = dist_to_goal
            self.last_progress_time = now
        elif now - self.last_progress_time > self.stall_timeout:
            self.get_logger().warn(f"stalled {self.stall_timeout}s — aborting")
            self.publish(0.0, 0.0)
            self.state = "idle"
            return

        if dist_to_goal < self.xy_tol:
            self.state = "final_rotation"
            self.do_final_rotation(pose)
            return

        target = self.lookahead_point(pose.xy())
        direction = target - pose.xy()
        desired_yaw = math.atan2(direction[1], direction[0])
        yaw_err = angle_diff(desired_yaw, pose.yaw)

        if abs(yaw_err) > self.rotate_threshold:
            self.publish(0.0, self.angular_with_floor(yaw_err))
            return

        # Slow down as yaw error grows; clamp speed by remaining distance
        scale = max(0.0, 1.0 - abs(yaw_err) / self.rotate_threshold)
        v = min(self.speed * scale, dist_to_goal)
        v = max(v, self.min_linear)
        w = self.k_angular * yaw_err
        w = float(np.clip(w, -self.speed, self.speed))
        # Don't apply min_angular when rolling forward — lets robot drive straight
        self.publish(v, w)

    def do_final_rotation(self, pose: Pose2D) -> None:
        err = angle_diff(self.goal_yaw, pose.yaw)
        if abs(err) < self.yaw_tol:
            self.publish(0.0, 0.0)
            self.state = "arrived"
            self.get_logger().info("arrived")
            return
        self.publish(0.0, self.angular_with_floor(err))

    # ---------- helpers ----------
    def lookahead_point(self, pos: np.ndarray) -> np.ndarray:
        d = np.linalg.norm(self.path - pos, axis=1)
        closest = int(np.argmin(d))
        # Walk forward from closest until cumulative distance >= lookahead
        acc = 0.0
        for i in range(closest, len(self.path) - 1):
            seg = float(np.linalg.norm(self.path[i + 1] - self.path[i]))
            if acc + seg >= self.lookahead:
                t = (self.lookahead - acc) / seg if seg > 1e-6 else 0.0
                return self.path[i] + t * (self.path[i + 1] - self.path[i])
            acc += seg
        return self.path[-1]

    def angular_with_floor(self, err: float) -> float:
        w = self.k_angular * err
        w = float(np.clip(w, -self.speed, self.speed))
        if 0 < abs(w) < self.min_angular:
            w = math.copysign(self.min_angular, w)
        return w

    def publish(self, v: float, w: float) -> None:
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = CustomPurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best-effort stop
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
