"""T5.9 (Codex #271 / #274): a lidar frame or a pose received once is published once, with its receive time."""
import ast
from pathlib import Path
import sys

SDK_PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SDK_PKG))
from go2_robot_sdk.received_frames import ReceivedFrames  # noqa: E402


def test_one_received_frame_is_taken_once_however_often_the_timer_ticks():
    frames = ReceivedFrames()
    frames.accept("0", {"topic": "rt/utlidar/voxel_map_compressed"}, now_ns=1_000_000_000)
    taken = [frames.take("0") for _ in range(4)]                 # timer ticks at +0 / +1 / +10 / +100 s
    assert [f is not None for f in taken] == [True, False, False, False]
    assert taken[0].received_ns == 1_000_000_000


def test_each_new_frame_is_taken_once_in_order_and_robots_are_independent():
    frames = ReceivedFrames()
    frames.accept("0", {"n": 1}, 10)
    frames.accept("1", {"n": 9}, 11)
    assert frames.take("0").message == {"n": 1} and frames.take("0") is None
    frames.accept("0", {"n": 2}, 20)
    second = frames.take("0")
    assert second.message == {"n": 2} and second.received_ns == 20 and frames.take("1").message == {"n": 9}
    assert frames.take("2") is None


def test_the_driver_publishes_only_taken_frames_stamped_with_their_receive_time():
    """Source boundary of go2_driver_node (rclpy / WebRTC imports): accept on receive, take on the timer."""
    source = (SDK_PKG / "go2_robot_sdk" / "go2_driver_node.py").read_text()
    tree = ast.parse(source)
    publish = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "publish_lidar_webrtc")
    body = ast.unparse(publish)
    assert "self.lidar_frames.take(str(i))" in body
    assert "Time(nanoseconds=frame.received_ns)" in body and "self.get_clock().now().to_msg()" not in body
    assert "self.lidar_frames.accept(robot_num, msg, self.get_clock().now().nanoseconds)" in source



def test_one_received_pose_gives_one_transform_and_one_odometry_with_its_receive_time():
    """Source boundary: publish_odometry_webrtc takes the pose once and stamps BOTH outputs with its receive time."""
    source = (SDK_PKG / "go2_robot_sdk" / "go2_driver_node.py").read_text()
    tree = ast.parse(source)
    names = {n.name for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}
    assert "publish_odom_webrtc" not in names and "publish_odom_topic_webrtc" not in names     # the cached republishers
    publish = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "publish_odometry_webrtc")
    body = ast.unparse(publish)
    assert body.count("self.odom_frames.take(") == 1 and "self.get_clock().now()" not in body
    assert "odom_trans.header.stamp = stamp" in body and "odom_msg.header.stamp = stamp" in body
    assert "Time(nanoseconds=frame.received_ns)" in body
    assert "self.odom_frames.accept(robot_num, msg, self.get_clock().now().nanoseconds)" in source
    timer = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "timer_callback")
    assert "self.publish_odometry_webrtc()" in ast.unparse(timer)


def test_a_stopped_odometry_publishes_nothing_more():
    frames = ReceivedFrames()
    frames.accept("0", {"data": {"pose": {}}}, 5)
    assert frames.take("0") is not None and all(frames.take("0") is None for _ in range(10))
