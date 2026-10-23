"""Run the real adapter with a single executor and delayed TF transport doubles.

The point transformation, coverage checks and LaserScan publication are production
code. Only ROS transport/time are doubled; these are not live robot evidence.
"""
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest

SDK_PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SDK_PKG))


def stamp(seconds):
    ns = round(seconds * 1e9)
    return NS(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def cloud(seconds=1000.0, points=None):
    if points is None:
        points = [(2 * math.cos(math.radians(d + .5)), 2 * math.sin(math.radians(d + .5)), .2)
                  for d in range(-180, 180)]
    return NS(header=NS(stamp=stamp(seconds), frame_id='odom'), points=points)


def transform(seconds=1000.0):
    return NS(header=NS(stamp=stamp(seconds), frame_id='base_link'), child_frame_id='odom',
              transform=NS(translation=NS(x=0., y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.)))


@pytest.fixture
def adapter(monkeypatch):
    clock = NS(ros=1000.01, monotonic=0.)
    available, calls, published, warnings, timers = {}, [], [], [], []
    overrides = {'output_frame': 'base_link'}

    class Time:
        def __init__(self, nanoseconds=0):
            self.nanoseconds = nanoseconds

        @classmethod
        def from_msg(cls, s):
            return cls(s.sec * 1_000_000_000 + s.nanosec)

        def __sub__(self, other):
            return Time(self.nanoseconds - other.nanoseconds)

    class MissingTransform(Exception):
        pass

    class Buffer:
        def lookup_transform(self, target, source, requested, timeout=None):
            calls.append((target, source, requested.nanoseconds, getattr(timeout, 'nanoseconds', 0)))
            if requested.nanoseconds not in available:
                raise MissingTransform('TF can only arrive in a later executor callback')
            return available[requested.nanoseconds]

    class Node:
        def __init__(self, name):
            self.params = {}

        def declare_parameter(self, name, value):
            self.params[name] = overrides.get(name, value)

        def get_parameter(self, name):
            return NS(value=self.params[name])

        def get_clock(self):
            return NS(now=lambda: Time(round(clock.ros * 1e9)))

        def create_subscription(self, *args):
            return args

        def create_publisher(self, *args):
            return NS(publish=published.append)

        def create_timer(self, period, callback):
            timers.append((period, callback))
            return NS(cancel=lambda: None)

        def get_logger(self):
            return NS(info=lambda _: None, warning=warnings.append)

    mods = {
        'rclpy': NS(time=NS(Time=Time), duration=NS(Duration=lambda seconds: Time(round(seconds * 1e9)))),
        'rclpy.node': NS(Node=Node),
        'sensor_msgs.msg': NS(PointCloud2=object, LaserScan=lambda: NS(header=NS())),
        'sensor_msgs_py': NS(point_cloud2=NS(read_points=lambda msg, **kwargs: msg.points)),
        'tf2_ros': NS(Buffer=Buffer, TransformListener=lambda *args, **kwargs: None,
                      LookupException=MissingTransform, ConnectivityException=MissingTransform,
                      ExtrapolationException=MissingTransform),
    }
    for name, value in mods.items():
        monkeypatch.setitem(sys.modules, name, value)
    path = SDK_PKG / 'go2_robot_sdk/pointcloud_to_scan.py'
    spec = importlib.util.spec_from_file_location('scan_queue_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'monotonic', lambda: clock.monotonic, raising=False)
    node = module.PointCloudToScan()

    def tick(seconds):
        clock.monotonic += seconds
        clock.ros += seconds
        for _, callback in timers:
            callback()

    return NS(node=node, clock=clock, available=available, calls=calls, published=published,
              warnings=warnings, timers=timers, tick=tick, module=module)


def test_immediate_exact_transform_has_no_blocking_wait_and_publishes_once(adapter):
    a = adapter
    a.available[1_000_000_000_000] = transform()
    msg = cloud()
    a.node.cloud_cb(msg)
    a.tick(.02)
    assert len(a.published) == 1
    assert a.published[0].header.stamp is msg.header.stamp
    assert a.published[0].header.frame_id == 'base_link'
    assert len(a.published[0].ranges) == 360
    assert a.published[0].ranges == pytest.approx([2.] * 360)
    assert a.calls and all(call[2] == 1_000_000_000_000 and call[3] == 0 for call in a.calls)


def test_delayed_exact_tf_can_arrive_on_the_same_executor_without_latest_fallback(adapter):
    a = adapter
    a.available[0] = transform(999.)
    msg = cloud()
    a.node.cloud_cb(msg)
    assert not a.published
    a.tick(.02)                                     # still no TF
    a.available[1_000_000_000_000] = transform()
    a.tick(.04)                                     # TF callback has now run
    assert len(a.published) == 1
    assert a.published[0].header.stamp is msg.header.stamp
    assert all(call[2] == 1_000_000_000_000 and call[3] == 0 for call in a.calls)
    a.tick(.02)
    assert len(a.published) == 1                    # the queued cloud is never republished


def test_missing_tf_times_out_on_monotonic_time_even_with_a_frozen_ros_clock(adapter):
    a = adapter
    a.node.cloud_cb(cloud())
    a.clock.monotonic = 1.
    a.tick(0.)
    a.available[1_000_000_000_000] = transform()
    a.tick(0.)
    assert not a.published
    assert any('timeout' in warning for warning in a.warnings)
    a.clock.ros = 1000.11
    a.available[1_000_100_000_000] = transform(1000.1)
    a.node.cloud_cb(cloud(1000.1))
    a.tick(.02)
    assert len(a.published) == 1


def test_a_waiting_cloud_never_gets_a_fresh_stamp_or_a_fresh_age(adapter):
    a = adapter
    a.node.cloud_cb(cloud())
    a.clock.ros = 1000.51                         # source age, independent of queue wait
    a.available[1_000_000_000_000] = transform()
    a.tick(.02)
    assert not a.published
    assert any('age' in warning for warning in a.warnings)


def test_pending_queue_is_bounded_and_drops_the_oldest_without_publishing_it(adapter):
    a = adapter
    a.node.max_pending_clouds = 2
    for t in (1000., 1000.01, 1000.02):
        a.node.cloud_cb(cloud(t))
    for t in (1000., 1000.01, 1000.02):
        a.available[round(t * 1e9)] = transform(t)
    a.tick(.02)
    assert [m.header.stamp.nanosec for m in a.published] == [10_000_000, 20_000_000]
    assert any('overflow' in warning for warning in a.warnings)


def test_zero_stamp_is_rejected_before_tf_can_interpret_it_as_latest(adapter):
    a = adapter
    a.clock.ros = .01
    a.available[0] = transform(0.)
    a.node.cloud_cb(cloud(0.))
    a.tick(.02)
    assert not a.calls and not a.published


def test_duplicate_or_older_clouds_cannot_refresh_the_scan(adapter):
    a = adapter
    a.available[1_000_000_000_000] = transform()
    a.available[999_990_000_000] = transform(999.99)
    a.node.cloud_cb(cloud())
    a.tick(.02)
    a.node.cloud_cb(cloud())
    a.node.cloud_cb(cloud(999.99))
    a.tick(.02)
    assert len(a.published) == 1


@pytest.mark.parametrize('points', [[], [(1., 0., .2)]])
def test_delayed_tf_does_not_weaken_empty_or_sparse_cloud_rejection(adapter, points):
    a = adapter
    a.node.cloud_cb(cloud(points=points))
    a.available[1_000_000_000_000] = transform()
    a.tick(.04)
    assert not a.published
    assert any('cloud not converted' in warning for warning in a.warnings)


def test_each_cloud_at_two_hz_survives_a_delayed_transform(adapter):
    a = adapter
    for i in range(8):
        t = 1000. + i * .5
        a.clock.ros = t + .01
        a.clock.monotonic = i * .5
        a.node.cloud_cb(cloud(t))
        a.available[round(t * 1e9)] = transform(t)
        a.tick(.06)
    assert len(a.published) == 8
    assert len({m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec for m in a.published}) == 8


def test_actual_adapter_transforms_world_points_before_binning_without_inventing_free_rays(adapter):
    a = adapter
    # Robot at (1, 1), heading +y. Invert its pose to get odom -> base_link.
    tf = transform()
    tf.transform.translation = NS(x=-1., y=1., z=0.)
    tf.transform.rotation = NS(x=0., y=0., z=-math.sqrt(.5), w=math.sqrt(.5))
    a.available[1_000_000_000_000] = tf
    body_points = cloud().points
    # One missing bearing remains NaN; the nearest return at +0.5 deg must be placed in body bin 180.
    body_points.pop(200)
    body_points.append((1.2 * math.cos(math.radians(.5)), 1.2 * math.sin(math.radians(.5)), .2))
    world_points = [(1.-y, 1.+x, z) for x,y,z in body_points]
    a.node.cloud_cb(cloud(points=world_points))
    a.tick(.02)
    scan, = a.published
    assert scan.ranges[180] == pytest.approx(1.2)
    assert math.isnan(scan.ranges[200])
    assert not any(math.isinf(r) for r in scan.ranges)
