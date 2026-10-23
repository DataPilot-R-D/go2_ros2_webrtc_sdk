"""T5.9 (Codex #279): /odom's twist is the body-frame velocity derived from received poses, or no Odometry at all."""
import ast
import math
from pathlib import Path
import sys

import pytest

SDK_PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SDK_PKG))
from go2_robot_sdk import odometry_velocity as ov  # noqa: E402

S = 1_000_000_000


def at(t_s, x, y, yaw):
    return ov.PoseSample(int(t_s * S), x, y, yaw)


def test_codex_279_moving_world_plus_y_at_yaw_pi_over_2_is_body_forward():
    v = ov.body_velocity(at(1000.0, 0.0, 0.0, math.pi / 2), at(1001.0, 0.0, 0.4, math.pi / 2))
    assert v.valid and v.vx == pytest.approx(0.4) and v.vy == pytest.approx(0.0, abs=1e-12) and v.wz == pytest.approx(0)


@pytest.mark.parametrize("yaw, dx, dy, vx, vy", [
    (0.0, 0.04, 0.0, 0.4, 0.0),            # facing +x, moving +x: forward
    (0.0, 0.0, 0.04, 0.0, 0.4),            # facing +x, moving +y: left
    (math.pi, -0.04, 0.0, 0.4, 0.0),       # facing -x, moving -x: forward
    (-math.pi / 2, 0.0, -0.04, 0.4, 0.0),  # facing -y, moving -y: forward
    (math.pi / 4, -0.02, 0.02, 0.0, 0.2 * math.sqrt(2)),
])
def test_the_displacement_is_rotated_into_the_body_frame(yaw, dx, dy, vx, vy):
    v = ov.body_velocity(at(10.0, 1.0, 1.0, yaw), at(10.1, 1.0 + dx, 1.0 + dy, yaw))
    assert v.valid and v.vx == pytest.approx(vx, abs=1e-9) and v.vy == pytest.approx(vy, abs=1e-9)


def test_a_turn_across_the_heading_wrap_is_small_and_correctly_signed():
    v = ov.body_velocity(at(10.0, 0.0, 0.0, math.pi - 0.05), at(10.1, 0.0, 0.0, -math.pi + 0.05))
    assert v.valid and v.wz == pytest.approx(1.0)                  # +0.1 rad in 0.1 s, not -62 rad/s
    v = ov.body_velocity(at(10.0, 0.0, 0.0, -math.pi + 0.05), at(10.1, 0.0, 0.0, math.pi - 0.05))
    assert v.wz == pytest.approx(-1.0)


def test_moving_while_turning_uses_the_mid_interval_heading():
    v = ov.body_velocity(at(10.0, 0.0, 0.0, -0.1), at(10.1, 0.04, 0.0, 0.1))
    assert v.vx == pytest.approx(0.4 * math.cos(0.0)) and v.vy == pytest.approx(0.0, abs=1e-12)
    assert v.wz == pytest.approx(2.0)


@pytest.mark.parametrize("previous, current, reason", [
    (None, at(10.0, 0, 0, 0), "first pose"),
    (at(10.0, 0, 0, 0), at(11.5, 0.6, 0, 0), "outside"),             # a gap is not a measurement
    (at(10.0, 0, 0, 0), at(10.0, 0.01, 0, 0), "outside"),            # same receive time
    (at(10.0, 0, 0, 0), at(9.9, 0.01, 0, 0), "outside"),             # out of order
    (at(10.0, 0, 0, 0), at(10.1, math.nan, 0, 0), "non-finite"),
])
def test_no_valid_velocity_is_never_a_measured_standstill(previous, current, reason):
    v = ov.body_velocity(previous, current)
    assert not v.valid and reason in v.reason


def test_the_yaw_comes_from_the_received_quaternion():
    q = {"x": 0.0, "y": 0.0, "z": math.sin(math.pi / 4), "w": math.cos(math.pi / 4)}
    s = ov.sample(5 * S, {"position": {"x": 1.0, "y": 2.0, "z": 0.3}, "orientation": q})
    assert (s.received_ns, s.x, s.y) == (5 * S, 1.0, 2.0) and s.yaw == pytest.approx(math.pi / 2)


def test_the_driver_publishes_the_derived_twist_and_no_odometry_without_one():
    source = (SDK_PKG / "go2_robot_sdk" / "go2_driver_node.py").read_text()
    publish = next(n for n in ast.walk(ast.parse(source))
                   if isinstance(n, ast.FunctionDef) and n.name == "publish_odometry_webrtc")
    body = ast.unparse(publish)
    assert "velocity = body_velocity(self._odom_previous.get(str(i)), current)" in body
    assert body.index("self.broadcaster.sendTransform(odom_trans)") < body.index("if not velocity.valid:") \
        < body.index("odom_msg = Odometry()")
    for line in ("odom_msg.twist.twist.linear.x = velocity.vx", "odom_msg.twist.twist.linear.y = velocity.vy",
                 "odom_msg.twist.twist.angular.z = velocity.wz"):
        assert line in body
    assert "cmd_vel" not in body                       # never an echo of the command
