# Copyright (c) 2024, RoboVerse community
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import json
import logging
import os
import threading
import asyncio
import time

from aiortc import MediaStreamTrack
from cv_bridge import CvBridge


from scripts_go2.go2_constants import ROBOT_CMD, RTC_TOPIC
from scripts_go2.go2_func import gen_command, gen_mov_command
from scripts_go2.go2_lidar_decoder import update_meshes_for_cloud2
from scripts_go2.go2_math import get_robot_joints
from scripts_go2.go2_camerainfo import load_camera_info
from scripts_go2.webrtc_driver import Go2Connection

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from tf2_ros import TransformBroadcaster, TransformStamped
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from go2_interfaces.msg import Go2State, IMU, LowState
from sensor_msgs.msg import PointCloud2, PointField, JointState, Joy
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Empty, Header, String
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image, CameraInfo


logging.basicConfig(level=logging.WARN)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class RobotBaseNode(Node):

    def __init__(self):
        super().__init__('go2_driver_node')

        self.declare_parameter('robot_ip', os.getenv(
            'ROBOT_IP', os.getenv('GO2_IP')))
        self.declare_parameter('token', os.getenv(
            'ROBOT_TOKEN', os.getenv('GO2_TOKEN', '')))
        self.declare_parameter('conn_type', os.getenv(
            'CONN_TYPE', os.getenv('CONN_TYPE', '')))

        self.robot_ip = self.get_parameter(
            'robot_ip').get_parameter_value().string_value
        self.token = self.get_parameter(
            'token').get_parameter_value().string_value
        self.robot_ip_lst = self.robot_ip.replace(" ", "").split(",")
        self.conn_type = self.get_parameter(
            'conn_type').get_parameter_value().string_value

        self.conn_mode = "single" if len(self.robot_ip_lst) == 1 else "multi"

        self.get_logger().info(f"Received ip list: {self.robot_ip_lst}")
        self.get_logger().info(f"Connection type is {self.conn_type}")

        self.get_logger().info(f"Connection mode is {self.conn_mode}")

        self.conn = {}
        qos_profile = QoSProfile(depth=10)

        self.joint_pub = []
        self.go2_state_pub = []
        self.go2_lidar_pub = []
        self.go2_odometry_pub = []
        self.imu_pub = []
        self.img_pub = []
        self.camera_info_pub = []
        # battery_state is a JSON-encoded String — matches Robot Bridge's
        # generic battery parser (voltage + percentage + charging).
        self.battery_pub = []

        # Nav2 expects /map on TRANSIENT_LOCAL durability so late subscribers
        # still get the most recent map after they start up. Without this
        # the planner stays stuck on "no map received" even though we are
        # publishing.
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        if self.conn_mode == 'single':
            self.joint_pub.append(self.create_publisher(
                JointState, 'joint_states', qos_profile))
            self.go2_state_pub.append(self.create_publisher(
                Go2State, 'go2_states', qos_profile))
            self.go2_lidar_pub.append(self.create_publisher(
                PointCloud2, 'point_cloud2', qos_profile))
            self.go2_odometry_pub.append(
                self.create_publisher(Odometry, 'odom', qos_profile))
            self.imu_pub.append(self.create_publisher(IMU, 'imu', qos_profile))
            self.img_pub.append(self.create_publisher(Image, 'camera/image_raw', qos_profile))
            self.camera_info_pub.append(self.create_publisher(CameraInfo, 'camera/camera_info', qos_profile))
            self.battery_pub.append(self.create_publisher(String, 'battery_state', qos_profile))
            # Publish under /map_uslam first (not /map) to avoid stomping
            # on persistent_map_publisher's /map while we verify USLAM
            # actually feeds data. Once empirically confirmed we switch
            # the topic to /map and remove persistent_map_publisher.
            self.map_pub = self.create_publisher(OccupancyGrid, 'map_uslam', map_qos)

        else:
            for i in range(len(self.robot_ip_lst)):
                self.joint_pub.append(self.create_publisher(
                    JointState, f'robot{i}/joint_states', qos_profile))
                self.go2_state_pub.append(self.create_publisher(
                    Go2State, f'robot{i}/go2_states', qos_profile))
                self.go2_lidar_pub.append(self.create_publisher(
                    PointCloud2, f'robot{i}/point_cloud2', qos_profile))
                self.go2_odometry_pub.append(self.create_publisher(
                    Odometry, f'robot{i}/odom', qos_profile))
                self.imu_pub.append(self.create_publisher(
                    IMU, f'robot{i}/imu', qos_profile))
                self.img_pub.append(self.create_publisher(
                    Image, f'robot{i}/camera/image_raw', qos_profile))
                self.camera_info_pub.append(self.create_publisher(
                    CameraInfo, f'robot{i}/camera/camera_info', qos_profile))
                self.battery_pub.append(self.create_publisher(
                    String, f'robot{i}/battery_state', qos_profile))

        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)

        self.bridge = CvBridge()
        self.camera_info = load_camera_info()

        self.robot_cmd_vel = {}
        self.robot_odom = {}
        self.robot_low_cmd = {}
        self.robot_sport_state = {}
        self.robot_lidar = {}

        self.joy_state = Joy()

        # Subscribe to cmd_vel_out (legacy — expected from twist_mux when
        # installed), cmd_vel_teleop (PAIC2 Robot Bridge) and cmd_vel
        # (Nav2 output). ros2_m1_native on M1 has no twist_mux package,
        # so we mux inside the driver: whichever topic has a fresh value
        # wins, teleop gets priority by being subscribed last.
        _cmd_topics = ('cmd_vel', 'cmd_vel_teleop', 'cmd_vel_out')
        if self.conn_mode == 'single':
            for topic in _cmd_topics:
                self.create_subscription(
                    Twist,
                    topic,
                    lambda msg: self.cmd_vel_cb(msg, "0"),
                    qos_profile)
        else:
            for i in range(len(self.robot_ip_lst)):
                for topic in _cmd_topics:
                    self.create_subscription(
                        Twist,
                        f'robot{str(i)}/{topic}',
                        lambda msg, ri=str(i): self.cmd_vel_cb(msg, ri),
                        qos_profile)

        self.create_subscription(
            Joy,
            'joy',
            self.joy_cb,
            qos_profile)

        # Dedicated mode-change ROS topics so the dashboard (or a curl
        # through rosbridge) can put the dog into BalanceStand / StandUp /
        # StandDown without emulating a gamepad on /joy. Required for
        # teleop: Go2 ignores rt/wirelesscontroller while in standby
        # (mode=0); sending BalanceStand first transitions to gait mode.
        self.create_subscription(
            Empty,
            'stand_up',
            lambda _msg: self._send_sport_cmd("StandUp"),
            qos_profile)
        self.create_subscription(
            Empty,
            'balance_stand',
            lambda _msg: self._send_sport_cmd("BalanceStand"),
            qos_profile)
        self.create_subscription(
            Empty,
            'stand_down',
            lambda _msg: self._send_sport_cmd("StandDown"),
            qos_profile)
        # Full wake-up sequence — use this when the dog is in an idle
        # damp/safety state and plain StandUp / BalanceStand aren't
        # enough. RecoveryStand (1006) forces exit from any abnormal
        # state, then BalanceStand transitions to gait-ready.
        self.create_subscription(
            Empty,
            'recovery_stand',
            lambda _msg: self._send_sport_cmd("RecoveryStand"),
            qos_profile)
        self.create_subscription(
            Empty,
            'wake_up',
            lambda _msg: self._wake_up_sequence(),
            qos_profile)

        # Support for CycloneDDS (EDU version via ethernet)
        if self.conn_type == 'cyclonedds':
            self.create_subscription(
                LowState,
                'lowstate',
                self.publish_joint_state_cyclonedds,
                qos_profile)

            self.create_subscription(
                PoseStamped,
                '/utlidar/robot_pose',
                self.publish_body_poss_cyclonedds,
                qos_profile)

            self.create_subscription(
                PointCloud2,
                '/utlidar/cloud',
                self.publish_lidar_cyclonedds,
                qos_profile)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.timer_lidar = self.create_timer(0.5, self.timer_callback_lidar)

    def timer_callback(self):
        if self.conn_type == 'webrtc':
            try:
                self.publish_odom_webrtc()
                self.publish_odom_topic_webrtc()
                self.publish_robot_state_webrtc()
                self.publish_joint_state_webrtc()
            except Exception as e:
                self.get_logger().debug(f"Publish error: {e}")
            # Keep battery on its own try-block so one failure in the
            # existing publishers doesn't mask the newer code during rollout.
            try:
                self.publish_battery_webrtc()
            except Exception as e:
                self.get_logger().warning(f"Battery publish error: {e}")

    def publish_battery_webrtc(self):
        # Go2 LOW_STATE carries bms_state.soc (0-100) and power_v (volts).
        # Emit as JSON-in-String so Robot Bridge's generic battery parser
        # picks it up without needing a Go2-specific message type.
        for i in range(len(self.battery_pub)):
            low = self.robot_low_cmd.get(str(i))
            if not low:
                continue
            data = low.get("data", {})
            bms = data.get("bms_state", {})
            soc = bms.get("soc")
            voltage = data.get("power_v")
            current_ma = bms.get("current")
            if soc is None and voltage is None:
                continue
            payload = {
                "percentage": float(soc) if soc is not None else 0.0,
                "voltage": float(voltage) if voltage is not None else 0.0,
                "charging": bool(current_ma is not None and current_ma > 0),
            }
            msg = String()
            msg.data = json.dumps(payload)
            self.battery_pub[i].publish(msg)

    def timer_callback_lidar(self):
        if self.conn_type == 'webrtc':
            try:
                self.publish_lidar_webrtc()
            except Exception as e:
                # WARN instead of DEBUG — lidar failures were swallowed
                # silently, freezing /point_cloud2 → /scan → SLAM forever.
                self.get_logger().warning(f"Lidar publish error: {e}")

    def cmd_vel_cb(self, msg, robot_num):
        x = msg.linear.x
        y = msg.linear.y
        z = msg.angular.z

        # Use rt/wirelesscontroller (joystick emulation) for reliable locomotion
        # Coordinate mapping: ROS twist -> Unitree joystick
        #   ly = forward/back (twist.linear.x)
        #   lx = strafe left/right (-twist.linear.y for Unitree convention)
        #   rx = turn (-twist.angular.z for Unitree convention)
        if x != 0.0 or y != 0.0 or z != 0.0:
            self.robot_cmd_vel[robot_num] = json.dumps({
                "type": "msg",
                "topic": "rt/wirelesscontroller",
                "data": {"lx": round(-y, 2), "ly": round(x, 2), "rx": round(-z, 2), "ry": 0}
            })
        else:
            self.robot_cmd_vel[robot_num] = json.dumps({
                "type": "msg",
                "topic": "rt/wirelesscontroller",
                "data": {"lx": 0, "ly": 0, "rx": 0, "ry": 0}
            })


    def joy_cb(self, msg):
        self.joy_state = msg

    def publish_body_poss_cyclonedds(self, msg):
        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.get_clock().now().to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = f"robot0/base_link"
        odom_trans.transform.translation.x = msg.pose.position.x
        odom_trans.transform.translation.y = msg.pose.position.y
        odom_trans.transform.translation.z = msg.pose.position.z + 0.07
        odom_trans.transform.rotation.x = msg.pose.orientation.x
        odom_trans.transform.rotation.y = msg.pose.orientation.y
        odom_trans.transform.rotation.z = msg.pose.orientation.z
        odom_trans.transform.rotation.w = msg.pose.orientation.w
        self.broadcaster.sendTransform(odom_trans)

    def publish_joint_state_cyclonedds(self, msg):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [
            f'robot0/FL_hip_joint', f'robot0/FL_thigh_joint', f'robot0/FL_calf_joint',
            f'robot0/FR_hip_joint', f'robot0/FR_thigh_joint', f'robot0/FR_calf_joint',
            f'robot0/RL_hip_joint', f'robot0/RL_thigh_joint', f'robot0/RL_calf_joint',
            f'robot0/RR_hip_joint', f'robot0/RR_thigh_joint', f'robot0/RR_calf_joint',
        ]
        joint_state.position = [
            msg.motor_state[3].q, msg.motor_state[4].q, msg.motor_state[5].q,
            msg.motor_state[0].q, msg.motor_state[1].q, msg.motor_state[2].q,
            msg.motor_state[9].q, msg.motor_state[10].q, msg.motor_state[11].q,
            msg.motor_state[6].q, msg.motor_state[7].q, msg.motor_state[8].q,
        ]
        self.joint_pub[0].publish(joint_state)

    def publish_lidar_cyclonedds(self, msg):
        msg.header = Header(frame_id="robot0/radar")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.go2_lidar_pub[0].publish(msg)

    def _send_dc(self, robot_num, payload):
        """Send payload over data channel."""
        conn = self.conn[robot_num]
        if not conn.data_channel_opened:
            return
        conn.data_channel.send(payload)

    def joy_cmd(self, robot_num):

        if self.conn_type == 'webrtc':
            if robot_num in self.conn and robot_num in self.robot_cmd_vel and self.robot_cmd_vel[robot_num] is not None:
                self.get_logger().info("Move")
                self._send_dc(robot_num, self.robot_cmd_vel[robot_num])
                self.robot_cmd_vel[robot_num] = None

            if robot_num in self.conn and self.joy_state.buttons and self.joy_state.buttons[1]:
                self.get_logger().info("Stand down")
                stand_down_cmd = gen_command(ROBOT_CMD["StandDown"])
                self._send_dc(robot_num, stand_down_cmd)

            if robot_num in self.conn and self.joy_state.buttons and self.joy_state.buttons[0]:
                self.get_logger().info("Stand up")
                stand_up_cmd = gen_command(ROBOT_CMD["StandUp"])
                self._send_dc(robot_num, stand_up_cmd)
                move_cmd = gen_command(ROBOT_CMD['BalanceStand'])
                self.conn[robot_num].data_channel.send(move_cmd)

    def on_validated(self, robot_num):
        if robot_num in self.conn:
            dc = self.conn[robot_num].data_channel
            self.get_logger().info(
                f"Subscribing to {len(RTC_TOPIC)} topics on DC "
                f"(state={dc.readyState})")
            # Enable video stream from robot
            dc.send(json.dumps({"type": "vid", "topic": "", "data": "on"}))
            for topic in RTC_TOPIC.values():
                dc.send(json.dumps({"type": "subscribe", "topic": topic}))

            # Activate Go2's onboard USLAM so we get the persistent
            # global map on `rt/mapping/grid_map` + `rt/uslam/cloud_map`.
            # Empirical sequence reverse-engineered from z4ziggy/z4rtc
            # and phospho-app/go2_webrtc_connect — the key insight is
            # that the command topic takes a BARE STRING (not a dict)
            # and requires two prerequisite toggles first.
            import time as _t

            # Prereq 1: disable traffic saving. Go2 throttles non-
            # essential topics by default; with it on, USLAM topics
            # never reach the data channel even if USLAM runs.
            dc.send(json.dumps({
                "type": "rtc_inner_req",
                "topic": "",
                "data": {
                    "req_type": "disable_traffic_saving",
                    "instruction": "on",
                },
            }))

            # Three-step USLAM activation dance that preserves teleop:
            #   a) motion_switcher(normal) — USLAM only processes client
            #      commands in `normal` or `mcf`. In `ai`/sport mode
            #      mapping/start is silently dropped.
            #   b) Fire mapping/start + run_mapping_process while the
            #      robot is in `normal`.
            #   c) motion_switcher(ai) — flip back to AI sport mode so
            #      our /cmd_vel_teleop chain regains base control.
            # Empirically USLAM continues publishing its map after step
            # (c) even though base control is no longer its own. That
            # lets us teleop while the onboard SLAM keeps mapping.

            def _motion_switcher(name: str) -> None:
                ms_id = int(_t.time() * 1000) % 2147483647
                dc.send(json.dumps({
                    "type": "req",
                    "topic": "rt/api/motion_switcher/request",
                    "data": {
                        "header": {"identity": {"id": ms_id, "api_id": 1002}},
                        "parameter": json.dumps({"name": name}),
                    },
                }))

            if os.environ.get("GO2_SKIP_USLAM_ACTIVATION") == "1":
                self.get_logger().info(
                    "USLAM activation SKIPPED (GO2_SKIP_USLAM_ACTIVATION=1): "
                    "preserving existing robot state (e.g. Unitree-app-built map + localization)"
                )
                # SAFETY: if we skipped the activation dance (which included a
                # motion_switcher('ai') that keeps the robot in gait mode), the
                # dog may be in damp mode from a prior disconnect and fall over.
                # Wake it up: RecoveryStand → BalanceStand forces stable stance.
                # Disable with GO2_AUTO_WAKE=0.
                # Note: _wake_up_sequence uses threading which breaks when the
                # dc.send requires an asyncio event loop. Schedule on loop here.
                if os.environ.get("GO2_AUTO_WAKE", "1") == "1":
                    import asyncio as _asyncio
                    async def _auto_wake():
                        for cmd_name, delay in (("RecoveryStand", 2.5),
                                                ("BalanceStand", 0.0)):
                            try:
                                self.get_logger().info(
                                    f"GO2_AUTO_WAKE: sending {cmd_name}")
                                dc.send(gen_command(ROBOT_CMD[cmd_name]))
                                if delay:
                                    await _asyncio.sleep(delay)
                            except Exception as exc:
                                self.get_logger().warning(
                                    f"GO2_AUTO_WAKE {cmd_name}: {exc}")
                    _asyncio.get_event_loop().create_task(_auto_wake())
            else:
                _motion_switcher("normal")
                _t.sleep(0.5)  # let Go2 transition before hitting USLAM
                # Bare STRING payload — dicts get silently ignored here.
                for cmd in ("mapping/start", "mapping/run_mapping_process"):
                    dc.send(json.dumps({
                        "type": "msg",
                        "topic": RTC_TOPIC["USLAM_CMD"],
                        "data": cmd,
                    }))
                _t.sleep(0.5)
                _motion_switcher("ai")
                self.get_logger().info(
                    "USLAM activation sequence: normal → mapping/start → "
                    "mapping/run_mapping_process → ai (teleop restored)"
                )

            # PROBE LOCALIZATION + NAVIGATION: comprehensive test of whether
            # localization/* and navigation/set_goal_pose actually work via
            # our WebRTC (as opposed to being app-only like mapping/*).
            # Gated by GO2_PROBE_LOCALIZATION=1. GO2_PROBE_NAV_DELTA_X controls
            # forward micro-goal distance (default 0.3m).
            if os.environ.get("GO2_PROBE_LOCALIZATION") == "1":
                import asyncio
                conn_ref = self.conn[robot_num]
                delta_x = float(os.environ.get("GO2_PROBE_NAV_DELTA_X", "0.3"))

                async def _probe_localization():
                    def _cmd(c):
                        self.get_logger().info(f"LOC PROBE: >>> {c}")
                        dc.send(json.dumps({"type": "msg",
                                            "topic": RTC_TOPIC["USLAM_CMD"], "data": c}))

                    # Wait longer for wake + pose. After reboot, USLAM needs
                    # a kick to publish frontend/odom.
                    await asyncio.sleep(8.0)
                    pose = getattr(self, "_last_frontend_odom_pose", None)
                    if pose is None:
                        self.get_logger().warning(
                            "LOC PROBE: no pose from frontend/odom after 8s — "
                            "trying localization/start to wake USLAM")
                        _cmd("localization/start")
                        await asyncio.sleep(5.0)
                        pose = getattr(self, "_last_frontend_odom_pose", None)
                    if pose is None:
                        pose = getattr(self, "_last_localization_odom_pose", None)
                        if pose:
                            self.get_logger().info(
                                "LOC PROBE: using localization/odom as baseline")
                    if pose is None:
                        self.get_logger().error(
                            "LOC PROBE: STILL no pose — "
                            "using (0,0,0) and continuing probe anyway")
                        pose = (0.0, 0.0, 0.0)
                    x0, y0, yaw0 = pose
                    self.get_logger().info(
                        f"LOC PROBE: baseline pose x={x0:.3f} y={y0:.3f} yaw={yaw0:.3f}")

                    # Step 2-3: enable verbose logging + status query.
                    _cmd("common/enable_logging")
                    await asyncio.sleep(1.5)
                    _cmd("localization/get_status")
                    await asyncio.sleep(3.0)

                    # Step 4: try set_initial_pose_type values 0..3.
                    for n in (0, 1, 2, 3):
                        _cmd(f"localization/set_initial_pose_type/{n}")
                        await asyncio.sleep(3.0)

                    # Step 5: hint at current pose.
                    _cmd(f"localization/set_initial_pose/{x0}/{y0}/{yaw0}")
                    await asyncio.sleep(3.0)

                    # Step 6: start localization.
                    _cmd("localization/start")
                    await asyncio.sleep(5.0)
                    _cmd("localization/get_status")
                    await asyncio.sleep(2.0)

                    # Step 6.5: switch motion mode to 'ai' so sport commands execute.
                    ms_id = int(_t.time() * 1000) % 2147483647
                    self.get_logger().info("LOC PROBE: motion_switcher -> 'ai'")
                    dc.send(json.dumps({
                        "type": "req",
                        "topic": "rt/api/motion_switcher/request",
                        "data": {
                            "header": {"identity": {"id": ms_id, "api_id": 1002}},
                            "parameter": json.dumps({"name": "ai"}),
                        },
                    }))
                    await asyncio.sleep(2.0)
                    # Step 6.7: navigation/start — engage nav engine.
                    _cmd("navigation/start")
                    await asyncio.sleep(2.0)
                    _cmd("navigation/get_status")
                    await asyncio.sleep(1.5)

                    # Step 7: micro navigation goal.
                    target_x = x0 + delta_x
                    target_y = y0
                    target_yaw = yaw0
                    _cmd(f"navigation/set_goal_pose/{target_x}/{target_y}/{target_yaw}")
                    self.get_logger().info(
                        f"LOC PROBE: goal sent. Watching pose for 30s...")
                    # Log pose samples over 30s so we see movement (or lack).
                    for i in range(30):
                        await asyncio.sleep(1.0)
                        p = getattr(self, "_last_frontend_odom_pose", None)
                        loc_p = getattr(self, "_last_localization_odom_pose", None)
                        if p:
                            dx = p[0] - x0
                            dy = p[1] - y0
                            self.get_logger().info(
                                f"LOC PROBE: t={i+1}s pose=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) "
                                f"Δ=({dx:+.3f},{dy:+.3f}) "
                                f"loc_odom={'yes' if loc_p else 'no'}")

                    _cmd("patrol/get_status")
                    await asyncio.sleep(1.5)
                    self.get_logger().info("LOC PROBE: done")

                asyncio.get_event_loop().create_task(_probe_localization())

            # PROBE UPLOAD ROUND-TRIP: upload a local .pcd back to the robot,
            # then download and compare. If Go2 accepts the push, downloads of
            # map.pcd after will match what we sent. Safe because we upload the
            # exact file we downloaded (idempotent overwrite).
            # Gated by GO2_PROBE_UPLOAD_MAP=1. Path: GO2_PROBE_UPLOAD_PATH.
            if os.environ.get("GO2_PROBE_UPLOAD_MAP") == "1":
                import asyncio
                conn_ref = self.conn[robot_num]
                upload_src = os.environ.get(
                    "GO2_PROBE_UPLOAD_PATH", "/tmp/go2-probe/dom_map.pcd")
                target_name = os.environ.get("GO2_PROBE_UPLOAD_TARGET", "map.pcd")

                async def _probe_upload():
                    await asyncio.sleep(5.0)
                    try:
                        with open(upload_src, "rb") as f:
                            data = f.read()
                    except Exception as exc:
                        self.get_logger().error(
                            f"PROBE UPLOAD: open {upload_src} failed: {exc}")
                        return
                    self.get_logger().info(
                        f"PROBE UPLOAD: sending {upload_src} "
                        f"({len(data)} bytes) -> robot as {target_name}")
                    try:
                        n_chunks = await conn_ref.upload_static_file(
                            data, target_name)
                        self.get_logger().info(
                            f"PROBE UPLOAD: all {n_chunks} chunks sent")
                    except Exception as exc:
                        self.get_logger().error(
                            f"PROBE UPLOAD: send failed: {exc}")
                        return
                    # Give robot 3s to commit, then read back.
                    await asyncio.sleep(3.0)
                    self.get_logger().info("PROBE UPLOAD: round-trip download")
                    try:
                        rt = await conn_ref.download_static_file(
                            target_name, timeout=60.0)
                        out = f"/tmp/go2-probe/roundtrip_{target_name}"
                        os.makedirs("/tmp/go2-probe", exist_ok=True)
                        with open(out, "wb") as f:
                            f.write(rt)
                        self.get_logger().info(
                            f"PROBE UPLOAD: round-trip -> {out} ({len(rt)} bytes)")
                    except Exception as exc:
                        self.get_logger().error(
                            f"PROBE UPLOAD: round-trip download failed: {exc}")

                asyncio.get_event_loop().create_task(_probe_upload())

            # PROBE READ-ONLY: download the 3 map files via rtc_inner_req
            # WITHOUT sending any mapping/* commands. Safe on a robot that is
            # actively localized or patrolling from an app-built map.
            # Gated by GO2_PROBE_MAP_READONLY=1.
            if os.environ.get("GO2_PROBE_MAP_READONLY") == "1":
                import asyncio
                conn_ref = self.conn[robot_num]
                async def _probe_readonly():
                    await asyncio.sleep(5.0)
                    os.makedirs("/tmp/go2-probe", exist_ok=True)
                    # Optional: trigger a fresh publish of the served file
                    # (harmless — does not modify state).
                    dc.send(json.dumps({"type": "msg", "topic": RTC_TOPIC["USLAM_CMD"],
                                        "data": "common/get_map_file"}))
                    await asyncio.sleep(1.5)
                    for fp in ("map.pcd", "map.pgm", "map.txt"):
                        try:
                            b = await conn_ref.download_static_file(fp, timeout=60.0)
                            out = f"/tmp/go2-probe/dom_{fp}"
                            with open(out, "wb") as f:
                                f.write(b)
                            self.get_logger().info(
                                f"PROBE RO: {out} ({len(b)} bytes)")
                        except Exception as exc:
                            self.get_logger().error(f"PROBE RO: {fp} failed: {exc}")
                asyncio.get_event_loop().create_task(_probe_readonly())

            # PROBE: one-shot map download. mapping/stop → common/get_map_file
            # → request_static_file. Gated by GO2_PROBE_GET_MAP=1.
            if os.environ.get("GO2_PROBE_GET_MAP") == "1":
                import asyncio
                wait_s = int(os.environ.get("GO2_PROBE_GET_MAP_DELAY", "10"))
                conn_ref = self.conn[robot_num]

                async def _probe_download_map():
                    await asyncio.sleep(wait_s)
                    self.get_logger().info("PROBE: mapping/stop")
                    dc.send(json.dumps({"type": "msg", "topic": RTC_TOPIC["USLAM_CMD"],
                                        "data": "mapping/stop"}))
                    await asyncio.sleep(3.0)
                    dc.send(json.dumps({"type": "msg", "topic": RTC_TOPIC["USLAM_CMD"],
                                        "data": "common/get_map_file"}))
                    await asyncio.sleep(1.5)
                    os.makedirs("/tmp/go2-probe", exist_ok=True)
                    for file_path in ("map.pcd", "map.pgm", "map.txt"):
                        try:
                            b = await conn_ref.download_static_file(file_path, timeout=45.0)
                            out = f"/tmp/go2-probe/{file_path}"
                            with open(out, "wb") as f:
                                f.write(b)
                            self.get_logger().info(f"PROBE: ok {file_path} ({len(b)} bytes)")
                        except Exception as exc:
                            self.get_logger().error(f"PROBE: {file_path} failed: {exc}")

                asyncio.get_event_loop().create_task(_probe_download_map())

            # MULTI-MAP PROBE: use common/set_map_id to force a fresh slot before
            # starting a new mapping session. Also enables USLAM verbose logging
            # to see server_log responses. Gated by GO2_PROBE_MULTIMAP=N where N
            # is the new map_id to switch to.
            if os.environ.get("GO2_PROBE_MULTIMAP"):
                import asyncio
                target_map_id = os.environ.get("GO2_PROBE_MULTIMAP", "1")
                conn_ref = self.conn[robot_num]
                trigger_path = "/tmp/go2-probe/STOP"
                out_dir = "/tmp/go2-probe"

                async def _multimap_probe():
                    os.makedirs(out_dir, exist_ok=True)
                    try:
                        os.remove(trigger_path)
                    except FileNotFoundError:
                        pass

                    def _cmd(c):
                        self.get_logger().info(f"MULTIMAP: send '{c}'")
                        dc.send(json.dumps({"type": "msg",
                                            "topic": RTC_TOPIC["USLAM_CMD"], "data": c}))

                    # Turn on verbose server_log to see what Go2 actually says.
                    _cmd("common/enable_logging")
                    await asyncio.sleep(1.0)
                    # Diagnostic: current state before any changes.
                    _cmd("mapping/get_status")
                    await asyncio.sleep(1.0)
                    _cmd("common/get_map_id")
                    await asyncio.sleep(1.5)
                    # Stop anything in progress cleanly.
                    _cmd("mapping/cancel")
                    await asyncio.sleep(1.5)
                    _cmd("localization/stop")
                    await asyncio.sleep(1.5)
                    # THE CRITICAL STEP: switch to a different map_id slot.
                    _cmd(f"common/set_map_id/{target_map_id}")
                    await asyncio.sleep(2.0)
                    # Confirm switch.
                    _cmd("common/get_map_id")
                    await asyncio.sleep(1.5)
                    _cmd("mapping/get_status")
                    await asyncio.sleep(1.0)
                    # Start mapping in the (hopefully) new slot.
                    _cmd("mapping/start")
                    await asyncio.sleep(2.0)
                    _cmd("mapping/get_status")
                    await asyncio.sleep(1.0)

                    started_at = _t.time()
                    self.get_logger().info(
                        f"MULTIMAP: session on map_id={target_map_id}. "
                        f"Teleop the robot. When done: touch {trigger_path}")
                    while not os.path.exists(trigger_path):
                        await asyncio.sleep(2.0)
                        elapsed = int(_t.time() - started_at)
                        if elapsed and elapsed % 10 == 0:
                            self.get_logger().info(
                                f"MULTIMAP: {elapsed}s elapsed on map_id={target_map_id}")
                    self.get_logger().info("MULTIMAP: STOP → finalizing")
                    _cmd("mapping/stop")
                    await asyncio.sleep(3.0)
                    _cmd("mapping/run_mapping_process")
                    await asyncio.sleep(5.0)
                    _cmd("common/get_map_file")
                    await asyncio.sleep(1.5)
                    for fp in ("map.pcd", "map.pgm", "map.txt"):
                        try:
                            b = await conn_ref.download_static_file(fp, timeout=60.0)
                            out = os.path.join(out_dir, f"multimap{target_map_id}_{fp}")
                            with open(out, "wb") as f:
                                f.write(b)
                            self.get_logger().info(
                                f"MULTIMAP: saved {out} ({len(b)} bytes)")
                        except Exception as exc:
                            self.get_logger().error(f"MULTIMAP: {fp} failed: {exc}")
                    try:
                        os.remove(trigger_path)
                    except FileNotFoundError:
                        pass

                asyncio.get_event_loop().create_task(_multimap_probe())

            # MAPPING SESSION: keep mapping active, download when operator says so.
            # Flow: operator teleops (physical pilot recommended), robot maps continuously,
            # operator creates /tmp/go2-probe/STOP → driver pulls final map files.
            # Gated by GO2_MAPPING_SESSION=1.
            # Optional GO2_MAPPING_RESET_FIRST=1: cycle candidate reset commands
            # before session start (to force fresh map). Watches server_log for
            # "receive client command" echo to detect which candidate works.
            if os.environ.get("GO2_MAPPING_SESSION") == "1":
                import asyncio
                conn_ref = self.conn[robot_num]
                trigger_path = os.environ.get(
                    "GO2_MAPPING_STOP_TRIGGER", "/tmp/go2-probe/STOP")
                out_dir = os.environ.get("GO2_MAPPING_OUT_DIR", "/tmp/go2-probe")

                async def _mapping_session():
                    os.makedirs(out_dir, exist_ok=True)
                    try:
                        os.remove(trigger_path)
                    except FileNotFoundError:
                        pass

                    # CRITICAL: enable_logging unblocks server_log responses.
                    # Without it, mapping/* commands look silent but may be rejected.
                    self.get_logger().info("MAPPING: common/enable_logging")
                    dc.send(json.dumps({"type": "msg", "topic": RTC_TOPIC["USLAM_CMD"],
                                        "data": "common/enable_logging"}))
                    await asyncio.sleep(1.5)
                    dc.send(json.dumps({"type": "msg", "topic": RTC_TOPIC["USLAM_CMD"],
                                        "data": "mapping/get_status"}))
                    await asyncio.sleep(2.0)

                    if os.environ.get("GO2_MAPPING_RESET_FIRST") == "1":
                        self.get_logger().info("MAPPING RESET: stopping any in-progress mapping")
                        dc.send(json.dumps({"type": "msg", "topic": RTC_TOPIC["USLAM_CMD"],
                                            "data": "mapping/stop"}))
                        await asyncio.sleep(2.0)
                        reset_candidates = [
                            "mapping/clear",
                            "mapping/reset",
                            "mapping/new_map",
                            "mapping/delete",
                            "common/delete_map_file",
                            "common/clear_map",
                            "common/reset_map",
                        ]
                        for c in reset_candidates:
                            self.get_logger().info(f"MAPPING RESET: trying '{c}'")
                            dc.send(json.dumps({"type": "msg", "topic": RTC_TOPIC["USLAM_CMD"],
                                                "data": c}))
                            await asyncio.sleep(2.5)
                        self.get_logger().info("MAPPING RESET: re-starting mapping after reset attempts")
                        dc.send(json.dumps({"type": "msg", "topic": RTC_TOPIC["USLAM_CMD"],
                                            "data": "mapping/start"}))
                        await asyncio.sleep(1.5)
                        dc.send(json.dumps({"type": "msg", "topic": RTC_TOPIC["USLAM_CMD"],
                                            "data": "mapping/run_mapping_process"}))
                        await asyncio.sleep(1.5)

                    started_at = _t.time()
                    self.get_logger().info(
                        f"MAPPING: session active. Walk the robot around. "
                        f"When done: `touch {trigger_path}` to finalize.")
                    while True:
                        await asyncio.sleep(2.0)
                        if os.path.exists(trigger_path):
                            break
                        elapsed = int(_t.time() - started_at)
                        if elapsed % 10 == 0:
                            self.get_logger().info(
                                f"MAPPING: {elapsed}s elapsed, still mapping "
                                f"(touch {trigger_path} to stop)")
                    self.get_logger().info("MAPPING: STOP trigger detected → finalizing")
                    dc.send(json.dumps({"type": "msg", "topic": RTC_TOPIC["USLAM_CMD"],
                                        "data": "mapping/stop"}))
                    await asyncio.sleep(3.0)
                    # Re-run post-process so Go2 re-commits the current
                    # accumulated state to the served file (otherwise the
                    # file stays at whatever was committed on first startup).
                    self.get_logger().info("MAPPING: mapping/run_mapping_process (re-commit)")
                    dc.send(json.dumps({"type": "msg", "topic": RTC_TOPIC["USLAM_CMD"],
                                        "data": "mapping/run_mapping_process"}))
                    await asyncio.sleep(5.0)
                    dc.send(json.dumps({"type": "msg", "topic": RTC_TOPIC["USLAM_CMD"],
                                        "data": "common/get_map_file"}))
                    await asyncio.sleep(1.5)
                    for file_path in ("map.pcd", "map.pgm", "map.txt"):
                        try:
                            b = await conn_ref.download_static_file(file_path, timeout=60.0)
                            out = os.path.join(out_dir, f"final_{file_path}")
                            with open(out, "wb") as f:
                                f.write(b)
                            self.get_logger().info(
                                f"MAPPING: saved {out} ({len(b)} bytes)")
                        except Exception as exc:
                            self.get_logger().error(
                                f"MAPPING: final_{file_path} failed: {exc}")
                    try:
                        os.remove(trigger_path)
                    except FileNotFoundError:
                        pass
                    self.get_logger().info(
                        f"MAPPING: session done. Files in {out_dir}/final_*.")

                asyncio.get_event_loop().create_task(_mapping_session())

    async def on_video_frame(self, track: MediaStreamTrack, robot_num):
        logger.info(f"Video frame loop starting for robot {robot_num}")
        frame_count = 0
        # Shared ffmpeg pusher lives on self — Go2 fires on_video_frame
        # multiple times per WebRTC reconnect, and every loop used to
        # spawn/kill its own ffmpeg, ending up in an SIGKILL ping-pong.
        if not hasattr(self, "_rtsp_pusher_state"):
            self._rtsp_pusher_state = {"pusher": None, "disabled": False, "owner": None}

        try:
            while True:
                frame = await track.recv()
                frame_count += 1
                img = frame.to_ndarray(format="bgr24")

                if frame_count <= 3 or frame_count % 100 == 0:
                    logger.info(f"Video frame #{frame_count}: {img.shape}")

                ros_image = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
                ros_image.header.stamp = self.get_clock().now().to_msg()

                camera_info = self.camera_info
                camera_info.header.stamp = ros_image.header.stamp

                if self.conn_mode == 'single':
                    camera_info.header.frame_id = 'front_camera'
                    ros_image.header.frame_id = 'front_camera'
                else:
                    camera_info.header.frame_id = f'robot{str(robot_num)}/front_camera'
                    ros_image.header.frame_id = f'robot{str(robot_num)}/front_camera'

                self.img_pub[robot_num].publish(ros_image)
                self.camera_info_pub[robot_num].publish(camera_info)

                # Optional RTSP push to go2rtc. Go2 Pro has no standard
                # RTSP/WHIP endpoint, so we re-encode the WebRTC video
                # track from aiortc through ffmpeg and PUBLISH it to
                # go2rtc on localhost. Media Gateway + dashboard then
                # pick it up as a regular robot_camera stream.
                rtsp_url = os.environ.get("GO2_RTSP_PUSH_URL", "").strip()
                state = self._rtsp_pusher_state
                if rtsp_url and not state["disabled"]:
                    # Only one loop owns the pusher. Later loops just
                    # skip encoding — Go2 sends the same video track to
                    # every listener anyway, so duplicate encoding is
                    # wasted CPU and fights over the RTSP session.
                    owner = state["owner"]
                    if owner is None or owner == id(track):
                        state["owner"] = id(track)
                        pusher = state["pusher"]
                        if pusher is not None and pusher.poll() is not None:
                            logger.warning(
                                f"RTSP pusher exited (code={pusher.returncode}) — restarting"
                            )
                            pusher = None
                        if pusher is None:
                            pusher = self._start_rtsp_pusher(rtsp_url, img.shape)
                            if pusher is None:
                                state["disabled"] = True
                        state["pusher"] = pusher
                        if pusher is not None:
                            try:
                                pusher.stdin.write(img.tobytes())
                                pusher.stdin.flush()
                            except (BrokenPipeError, ValueError, OSError):
                                logger.warning("RTSP pusher pipe broken — restarting")
                                try:
                                    pusher.kill()
                                    pusher.wait(timeout=1)
                                except Exception:
                                    pass
                                state["pusher"] = None

                await asyncio.sleep(0)
        except Exception as e:
            import traceback
            logger.error(
                f"Video frame loop crashed after {frame_count} frames: "
                f"{type(e).__name__}: {e!r}\n{traceback.format_exc()}"
            )
        finally:
            state = getattr(self, "_rtsp_pusher_state", None)
            if state is not None and state.get("owner") == id(track):
                pusher = state.get("pusher")
                if pusher is not None:
                    try:
                        pusher.stdin.close()
                        pusher.kill()
                    except Exception:
                        pass
                state["pusher"] = None
                # Release owner so another loop (Go2 sends two video
                # tracks, one per encoding) can pick the pusher up.
                # Without this, once the first-claimed loop dies,
                # /point_cloud2 + /scan + camera all stay dark.
                state["owner"] = None

    def _wake_up_sequence(self) -> None:
        # RecoveryStand → wait → BalanceStand. Needed after the dog has
        # been idle long enough to drop into damp / safety mode where
        # standalone StandUp is silently ignored by the sport controller.
        import threading
        def _run():
            import time
            for cmd, delay in (("RecoveryStand", 2.0), ("BalanceStand", 0.0)):
                self._send_sport_cmd(cmd)
                if delay:
                    time.sleep(delay)
        threading.Thread(target=_run, daemon=True).start()

    def _send_sport_cmd(self, cmd_name: str) -> None:
        """Dispatch a ROBOT_CMD API call to every connected robot.

        Used by /stand_up, /balance_stand, /stand_down ROS topics so the
        dashboard can flip the dog out of standby before teleop.
        """
        if cmd_name not in ROBOT_CMD:
            logger.error(f"Unknown sport cmd: {cmd_name}")
            return
        payload = gen_command(ROBOT_CMD[cmd_name])
        for robot_num, conn in self.conn.items():
            try:
                self._send_dc(robot_num, payload)
                logger.info(f"Sent {cmd_name} to robot {robot_num}")
            except Exception as exc:
                logger.warning(f"{cmd_name} dispatch to robot {robot_num} failed: {exc}")

    def _start_rtsp_pusher(self, rtsp_url: str, shape):
        # shape = (h, w, 3) BGR
        import shutil
        import subprocess

        # Kill any leftover ffmpeg processes pushing to the same RTSP
        # target. Driver restart doesn't clean them up if the python
        # process was SIGKILL'd; they linger as zombies, hold a TCP
        # session on go2rtc, and block the fresh pusher.
        try:
            subprocess.run(
                ["pkill", "-9", "-f", f"ffmpeg.*{rtsp_url}"],
                check=False, timeout=2,
            )
        except Exception:
            pass

        h, w = int(shape[0]), int(shape[1])
        fps = int(os.environ.get("GO2_RTSP_PUSH_FPS", "15"))
        ffmpeg_bin = os.environ.get("GO2_FFMPEG_BIN", "").strip() or shutil.which("ffmpeg")
        if not ffmpeg_bin:
            # ros2_m1_native's activate_env.sh overwrites PATH to ROS-only dirs,
            # so a homebrew ffmpeg at /opt/homebrew/bin won't be visible unless
            # GO2_FFMPEG_BIN is explicitly set by the launcher.
            for candidate in ("/opt/homebrew/bin/ffmpeg", "/usr/local/bin/ffmpeg", "/usr/bin/ffmpeg"):
                if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
                    ffmpeg_bin = candidate
                    break
        if not ffmpeg_bin:
            logger.error("ffmpeg not found — set GO2_FFMPEG_BIN or install ffmpeg")
            return None

        cmd = [
            ffmpeg_bin, "-loglevel", "warning",
            "-f", "rawvideo", "-pix_fmt", "bgr24",
            "-s", f"{w}x{h}", "-r", str(fps),
            "-i", "pipe:0",
            "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
            "-g", str(fps * 2), "-b:v", "2M",
            "-f", "rtsp", "-rtsp_transport", "tcp",
            rtsp_url,
        ]
        logger.info(f"Starting RTSP pusher: {' '.join(cmd)}")
        try:
            return subprocess.Popen(cmd, stdin=subprocess.PIPE)
        except FileNotFoundError:
            logger.error(f"ffmpeg not on PATH at {ffmpeg_bin}")
            return None

    def on_data_channel_message(self, _, msg, robot_num):

        if msg.get('topic') == RTC_TOPIC["ULIDAR_ARRAY"]:
            self.robot_lidar[robot_num] = msg

        if msg.get('topic') == RTC_TOPIC['ROBOTODOM']:
            self.robot_odom[robot_num] = msg

        if msg.get('topic') == RTC_TOPIC['LF_SPORT_MOD_STATE']:
            self.robot_sport_state[robot_num] = msg

        if msg.get('topic') == RTC_TOPIC['LOW_STATE']:
            self.robot_low_cmd[robot_num] = msg

        # Handle Go2's onboard USLAM global map. Payload format is
        # assumed to mirror nav_msgs/OccupancyGrid serialized as JSON
        # over the data channel — that's the standard pattern Unitree
        # uses for their ROS1-style bridge. If the format differs we'll
        # see it in the diagnostic log on the first few frames and
        # adapt the decoder.
        topic = msg.get('topic')
        if topic == RTC_TOPIC.get("GRID_MAP"):
            self._on_grid_map(msg)
        elif topic == RTC_TOPIC.get("USLAM_FRONTEND_ODOM"):
            self._inspect_uslam_msg(topic, msg)
            self._cache_pose(msg, "_last_frontend_odom_pose")
        elif topic == RTC_TOPIC.get("USLAM_LOCALIZATION_ODOM"):
            self._inspect_uslam_msg(topic, msg)
            self._cache_pose(msg, "_last_localization_odom_pose")
        elif topic == RTC_TOPIC.get("USLAM_NAVIGATION_GLOBAL_PATH"):
            self._inspect_uslam_msg(topic, msg)
        elif topic in (
            RTC_TOPIC.get("USLAM_CLOUD_MAP"),
            RTC_TOPIC.get("USLAM_SERVER_LOG"),
        ):
            # USLAM status/telemetry channels. server_log in particular
            # is a goldmine when USLAM doesn't start — error messages
            # from the mapping subsystem land there.
            self._inspect_uslam_msg(topic, msg)

    def _cache_pose(self, msg: dict, attr: str) -> None:
        """Extract (x, y, yaw) from a PoseWithCovarianceStamped-like msg."""
        try:
            data = msg.get("data", {})
            pose = data.get("pose", {}).get("pose", data.get("pose"))
            if not isinstance(pose, dict):
                return
            p = pose.get("position", {})
            o = pose.get("orientation", {})
            x = float(p.get("x", 0.0))
            y = float(p.get("y", 0.0))
            # yaw from quaternion (assuming small roll/pitch — typical for floor robot)
            qz = float(o.get("z", 0.0))
            qw = float(o.get("w", 1.0))
            import math
            yaw = 2 * math.atan2(qz, qw)
            setattr(self, attr, (x, y, yaw))
        except Exception:
            pass

    def _inspect_uslam_msg(self, topic: str, msg: dict) -> None:
        if not hasattr(self, "_uslam_inspect_count"):
            self._uslam_inspect_count = {}
        n = self._uslam_inspect_count.get(topic, 0) + 1
        self._uslam_inspect_count[topic] = n
        if n <= 3 or n % 100 == 0:
            data = msg.get("data")
            if isinstance(data, dict):
                preview = {
                    k: (v if not isinstance(v, (list, dict)) else f"{type(v).__name__}(len={len(v)})")
                    for k, v in data.items()
                }
            else:
                preview = f"type={type(data).__name__}"
            self.get_logger().info(
                f"USLAM msg #{n} on {topic}: {preview}"
            )

    def _on_grid_map(self, msg: dict) -> None:
        """Decode Go2's persistent grid map and republish as /map.

        Unitree serializes nav_msgs/OccupancyGrid as JSON with the same
        structure as rosbridge: info.{resolution,width,height,origin},
        data[]. Robot publishes in the onboard SLAM's map frame — we
        republish under our TF tree's "map" frame since we run an
        identity map→odom transform in onboard_slam mode.
        """
        # Always log the first frames so we can verify format empirically.
        self._inspect_uslam_msg(RTC_TOPIC["GRID_MAP"], msg)
        data = msg.get("data")
        if not isinstance(data, dict):
            return
        try:
            info = data.get("info", {})
            grid_data = data.get("data")
            if grid_data is None or "resolution" not in info:
                return
            og = OccupancyGrid()
            og.header.stamp = self.get_clock().now().to_msg()
            # Unitree tags this in its own "map" frame; we normalise.
            og.header.frame_id = "map"
            og.info.resolution = float(info["resolution"])
            og.info.width = int(info["width"])
            og.info.height = int(info["height"])
            origin = info.get("origin", {})
            pos = origin.get("position", {})
            orient = origin.get("orientation", {"w": 1.0})
            og.info.origin.position.x = float(pos.get("x", 0.0))
            og.info.origin.position.y = float(pos.get("y", 0.0))
            og.info.origin.position.z = float(pos.get("z", 0.0))
            og.info.origin.orientation.x = float(orient.get("x", 0.0))
            og.info.origin.orientation.y = float(orient.get("y", 0.0))
            og.info.origin.orientation.z = float(orient.get("z", 0.0))
            og.info.origin.orientation.w = float(orient.get("w", 1.0))
            og.data = [int(v) for v in grid_data]
            self.map_pub.publish(og)
            if not hasattr(self, "_grid_map_published"):
                self.get_logger().info(
                    f"Republished USLAM grid map: {og.info.width}×{og.info.height} "
                    f"@ {og.info.resolution}m/cell, origin="
                    f"({og.info.origin.position.x:.2f},{og.info.origin.position.y:.2f})"
                )
                self._grid_map_published = True
        except (KeyError, ValueError, TypeError) as exc:
            if not hasattr(self, "_grid_map_decode_warned"):
                self.get_logger().warning(
                    f"Failed to decode USLAM grid_map payload ({exc}); "
                    f"first-frame data keys: {list(data.keys())}"
                )
                self._grid_map_decode_warned = True

    def publish_odom_webrtc(self):
        for i in range(len(self.robot_odom)):
            if self.robot_odom[str(i)]:
                odom_trans = TransformStamped()
                odom_trans.header.stamp = self.get_clock().now().to_msg()
                odom_trans.header.frame_id = 'odom'

                if self.conn_mode == 'single':
                    odom_trans.child_frame_id = "base_link"
                else:
                    odom_trans.child_frame_id = f"robot{str(i)}/base_link"

                odom_trans.transform.translation.x = self.robot_odom[str(
                    i)]['data']['pose']['position']['x']
                odom_trans.transform.translation.y = self.robot_odom[str(
                    i)]['data']['pose']['position']['y']
                odom_trans.transform.translation.z = self.robot_odom[str(
                    i)]['data']['pose']['position']['z'] + 0.07
                odom_trans.transform.rotation.x = self.robot_odom[str(
                    i)]['data']['pose']['orientation']['x']
                odom_trans.transform.rotation.y = self.robot_odom[str(
                    i)]['data']['pose']['orientation']['y']
                odom_trans.transform.rotation.z = self.robot_odom[str(
                    i)]['data']['pose']['orientation']['z']
                odom_trans.transform.rotation.w = self.robot_odom[str(
                    i)]['data']['pose']['orientation']['w']
                self.broadcaster.sendTransform(odom_trans)

    def publish_odom_topic_webrtc(self):
        for i in range(len(self.robot_odom)):
            if self.robot_odom[str(i)]:
                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = 'odom'

                if self.conn_mode == 'single':
                    odom_msg.child_frame_id = "base_link"

                else:
                    odom_msg.child_frame_id = f"robot{str(i)}/base_link"

                odom_msg.pose.pose.position.x = self.robot_odom[str(
                    i)]['data']['pose']['position']['x']
                odom_msg.pose.pose.position.y = self.robot_odom[str(
                    i)]['data']['pose']['position']['y']
                odom_msg.pose.pose.position.z = self.robot_odom[str(
                    i)]['data']['pose']['position']['z'] + 0.07
                odom_msg.pose.pose.orientation.x = self.robot_odom[str(
                    i)]['data']['pose']['orientation']['x']
                odom_msg.pose.pose.orientation.y = self.robot_odom[str(
                    i)]['data']['pose']['orientation']['y']
                odom_msg.pose.pose.orientation.z = self.robot_odom[str(
                    i)]['data']['pose']['orientation']['z']
                odom_msg.pose.pose.orientation.w = self.robot_odom[str(
                    i)]['data']['pose']['orientation']['w']
                self.go2_odometry_pub[i].publish(odom_msg)

    def publish_lidar_webrtc(self):
        # Rate-limit the "missing decoded_data" warning so a stream of
        # malformed frames doesn't flood the log every 0.5s tick.
        if not hasattr(self, "_lidar_missing_warn_ts"):
            self._lidar_missing_warn_ts = {}
        for i in range(len(self.robot_lidar)):
            if self.robot_lidar[str(i)]:
                msg = self.robot_lidar[str(i)]
                decoded = msg.get("decoded_data")
                if not decoded:
                    now_ns = self.get_clock().now().nanoseconds
                    last = self._lidar_missing_warn_ts.get(str(i), 0)
                    if now_ns - last > 5_000_000_000:  # 5 s
                        self.get_logger().warning(
                            f"Lidar frame {i} missing decoded_data — "
                            f"skipping publish until next valid frame"
                        )
                        self._lidar_missing_warn_ts[str(i)] = now_ns
                    continue
                points = update_meshes_for_cloud2(
                    decoded["positions"],
                    decoded["uvs"],
                    msg['data']['resolution'],
                    msg['data']['origin'],
                    0
                )
                point_cloud = PointCloud2()
                point_cloud.header = Header(frame_id="odom")
                point_cloud.header.stamp = self.get_clock().now().to_msg()
                fields = [
                    PointField(name='x', offset=0,
                               datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4,
                               datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8,
                               datatype=PointField.FLOAT32, count=1),
                    PointField(name='intensity', offset=12,
                               datatype=PointField.FLOAT32, count=1),
                ]
                point_cloud = point_cloud2.create_cloud(
                    point_cloud.header, fields, points)
                self.go2_lidar_pub[i].publish(point_cloud)

    def publish_joint_state_webrtc(self):

        for i in range(len(self.robot_sport_state)):
            if self.robot_sport_state[str(i)]:
                joint_state = JointState()
                joint_state.header.stamp = self.get_clock().now().to_msg()

                fl_foot_pos_array = [
                    self.robot_sport_state[str(
                        i)]["data"]["foot_position_body"][3],
                    self.robot_sport_state[str(
                        i)]["data"]["foot_position_body"][4],
                    self.robot_sport_state[str(
                        i)]["data"]["foot_position_body"][5]
                ]

                FL_hip_joint, FL_thigh_joint, FL_calf_joint = get_robot_joints(
                    fl_foot_pos_array,
                    0
                )

                fr_foot_pos_array = [
                    self.robot_sport_state[str(
                        i)]["data"]["foot_position_body"][0],
                    self.robot_sport_state[str(
                        i)]["data"]["foot_position_body"][1],
                    self.robot_sport_state[str(
                        i)]["data"]["foot_position_body"][2]
                ]

                FR_hip_joint, FR_thigh_joint, FR_calf_joint = get_robot_joints(
                    fr_foot_pos_array,
                    1
                )

                rl_foot_pos_array = [
                    self.robot_sport_state[str(
                        i)]["data"]["foot_position_body"][9],
                    self.robot_sport_state[str(
                        i)]["data"]["foot_position_body"][10],
                    self.robot_sport_state[str(
                        i)]["data"]["foot_position_body"][11]
                ]

                RL_hip_joint, RL_thigh_joint, RL_calf_joint = get_robot_joints(
                    rl_foot_pos_array,
                    2
                )

                rr_foot_pos_array = [
                    self.robot_sport_state[str(
                        i)]["data"]["foot_position_body"][6],
                    self.robot_sport_state[str(
                        i)]["data"]["foot_position_body"][7],
                    self.robot_sport_state[str(
                        i)]["data"]["foot_position_body"][8]
                ]

                RR_hip_joint, RR_thigh_joint, RR_calf_joint = get_robot_joints(
                    rr_foot_pos_array,
                    3
                )

                if self.conn_mode == 'single':
                    joint_state.name = [
                        'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
                        'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
                        'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
                        'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
                    ]
                else:
                    joint_state.name = [
                        f'robot{str(i)}/FL_hip_joint', f'robot{str(i)}/FL_thigh_joint', f'robot{str(i)}/FL_calf_joint',
                        f'robot{str(i)}/FR_hip_joint', f'robot{str(i)}/FR_thigh_joint', f'robot{str(i)}/FR_calf_joint',
                        f'robot{str(i)}/RL_hip_joint', f'robot{str(i)}/RL_thigh_joint', f'robot{str(i)}/RL_calf_joint',
                        f'robot{str(i)}/RR_hip_joint', f'robot{str(i)}/RR_thigh_joint', f'robot{str(i)}/RR_calf_joint',
                    ]

                joint_state.position = [
                    FL_hip_joint, FL_thigh_joint, FL_calf_joint,
                    FR_hip_joint, FR_thigh_joint, FR_calf_joint,
                    RL_hip_joint, RL_thigh_joint, RL_calf_joint,
                    RR_hip_joint, RR_thigh_joint, RR_calf_joint,
                ]
                self.joint_pub[i].publish(joint_state)

    def publish_robot_state_webrtc(self):
        for i in range(len(self.robot_sport_state)):
            if self.robot_sport_state[str(i)]:
                go2_state = Go2State()
                go2_state.mode = self.robot_sport_state[str(i)]["data"]["mode"]
                go2_state.progress = self.robot_sport_state[str(
                    i)]["data"]["progress"]
                go2_state.gait_type = self.robot_sport_state[str(
                    i)]["data"]["gait_type"]
                go2_state.position = list(
                    map(float, self.robot_sport_state[str(i)]["data"]["position"]))
                go2_state.body_height = float(
                    self.robot_sport_state[str(i)]["data"]["body_height"])
                go2_state.velocity = self.robot_sport_state[str(
                    i)]["data"]["velocity"]
                go2_state.range_obstacle = list(
                    map(float, self.robot_sport_state[str(i)]["data"]["range_obstacle"]))
                go2_state.foot_force = self.robot_sport_state[str(
                    i)]["data"]["foot_force"]
                go2_state.foot_position_body = list(
                    map(float, self.robot_sport_state[str(i)]["data"]["foot_position_body"]))
                go2_state.foot_speed_body = list(
                    map(float, self.robot_sport_state[str(i)]["data"]["foot_speed_body"]))
                self.go2_state_pub[i].publish(go2_state)

                imu = IMU()
                imu.quaternion = list(
                    map(float, self.robot_sport_state[str(i)]["data"]["imu_state"]["quaternion"]))
                imu.accelerometer = list(
                    map(float, self.robot_sport_state[str(i)]["data"]["imu_state"]["accelerometer"]))
                imu.gyroscope = list(
                    map(float, self.robot_sport_state[str(i)]["data"]["imu_state"]["gyroscope"]))
                imu.rpy = list(
                    map(float, self.robot_sport_state[str(i)]["data"]["imu_state"]["rpy"]))
                imu.temperature = self.robot_sport_state[str(
                    i)]["data"]["imu_state"]["temperature"]
                self.imu_pub[i].publish(imu)

    async def connect_robot(self, robot_ip, robot_num, token):
        """Connect a single robot via WebRTC. Must be called before spin starts."""
        conn = Go2Connection(
            robot_ip=robot_ip,
            robot_num=robot_num,
            token=token,
            on_validated=self.on_validated,
            on_message=self.on_data_channel_message,
            on_video_frame=self.on_video_frame,
        )

        self.conn[robot_num] = conn

        if self.conn_type == 'webrtc':
            await conn.connect()

            if conn.data_channel_opened:
                self.get_logger().info(
                    f"Robot {robot_num}: data channel validated and ready!")
                # Subscribe to robot topics. The library handled validation
                # internally, so our on_validated callback was never triggered.
                self.on_validated(robot_num)
            else:
                self.get_logger().warn(
                    f"Robot {robot_num}: data channel validation did not complete")

    async def joy_cmd_loop(self, robot_num):
        """Continuously send joystick commands to a connected robot."""
        while True:
            try:
                self.joy_cmd(robot_num)
            except Exception as e:
                self.get_logger().warn(f"joy_cmd error: {e}")
            await asyncio.sleep(0.1)


async def spin(node: Node):
    cancel = node.create_guard_condition(lambda: None)

    def _spin(node: Node,
              future: asyncio.Future,
              event_loop: asyncio.AbstractEventLoop):
        while not future.cancelled():
            rclpy.spin_once(node, timeout_sec=0.05)
            # Yield the GIL briefly so the asyncio event loop can process
            # aiortc SCTP packets and data channel messages
            time.sleep(0.001)
        if not future.cancelled():
            event_loop.call_soon_threadsafe(future.set_result, None)
    event_loop = asyncio.get_event_loop()
    spin_task = event_loop.create_future()
    spin_thread = threading.Thread(
        target=_spin, args=(node, spin_task, event_loop), daemon=True)
    spin_thread.start()
    try:
        await spin_task
    except asyncio.CancelledError:
        cancel.trigger()
    spin_thread.join()
    node.destroy_guard_condition(cancel)


async def start_node():
    base_node = RobotBaseNode()

    # Phase 1: Connect all robots BEFORE starting the spin thread.
    # The spin thread's rclpy.spin_once() holds the GIL in a tight loop,
    # which starves the asyncio event loop and prevents aiortc from
    # processing SCTP packets needed for data channel validation.
    for i in range(len(base_node.robot_ip_lst)):
        base_node.get_logger().info(
            f"Connecting to robot {i} at {base_node.robot_ip_lst[i]}...")
        await base_node.connect_robot(
            robot_ip=base_node.robot_ip_lst[i],
            robot_num=str(i),
            token=base_node.token,
        )

    # Phase 2: Now start the spin thread and joy_cmd loops.
    base_node.get_logger().info("Phase 2: starting spin thread and joy_cmd loops")
    event_loop = asyncio.get_event_loop()
    spin_task = event_loop.create_task(spin(base_node))
    joy_tasks = [
        event_loop.create_task(base_node.joy_cmd_loop(str(i)))
        for i in range(len(base_node.robot_ip_lst))
    ]

    await asyncio.wait(
        [spin_task, *joy_tasks], return_when=asyncio.FIRST_COMPLETED)


def main():
    rclpy.init()
    asyncio.get_event_loop().run_until_complete(start_node())
    asyncio.get_event_loop().close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
