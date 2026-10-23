"""T5.9 (Codex #279 / #282): /odom's twist is the body-frame SE(2) velocity derived from validated received poses,
or no Odometry at all; a malformed pose publishes neither transform nor odometry."""
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
    return ov.PoseSample(int(round(t_s * S)), x, y, yaw)


def integrate(start, vx, vy, wz, dt, steps=20000):
    """Ground truth: a CONSTANT body twist integrated finely from ``start``."""
    x, y, yaw = start.x, start.y, start.yaw
    h = dt / steps
    for _ in range(steps):
        mid = yaw + wz * h / 2
        x += (math.cos(mid) * vx - math.sin(mid) * vy) * h
        y += (math.sin(mid) * vx + math.cos(mid) * vy) * h
        yaw += wz * h
    return at(start.received_ns / S + dt, x, y, ov.wrap(yaw))


def test_codex_279_moving_world_plus_y_at_yaw_pi_over_2_is_body_forward():
    v = ov.body_velocity(at(1000.0, 0.0, 0.0, math.pi / 2), at(1001.0, 0.0, 0.4, math.pi / 2))
    assert v.valid and v.vx == pytest.approx(0.4) and v.vy == pytest.approx(0.0, abs=1e-12) and v.wz == pytest.approx(0)


@pytest.mark.parametrize("vx, vy, wz, dt", [
    (0.4, 0.0, 1.0, 1.0),          # Codex #282: the chord read 0.38354 here
    (0.4, 0.0, -1.2, 0.5),
    (0.3, 0.1, 0.8, 0.2),
    (0.0, 0.0, 1.2, 0.1),          # turning in place
    (0.5, 0.0, 1e-9, 0.1),         # the series branch
])
@pytest.mark.parametrize("yaw0", [0.0, math.pi / 2, -2.5, math.pi - 0.01])
def test_a_constant_body_twist_is_recovered_exactly(vx, vy, wz, dt, yaw0):
    start = at(50.0, 1.0, -2.0, yaw0)
    v = ov.body_velocity(start, integrate(start, vx, vy, wz, dt))
    assert v.valid
    assert (v.vx, v.vy, v.wz) == (pytest.approx(vx, abs=1e-6), pytest.approx(vy, abs=1e-6), pytest.approx(wz, abs=1e-6))


def test_a_turn_across_the_heading_wrap_is_small_and_correctly_signed():
    v = ov.body_velocity(at(10.0, 0.0, 0.0, math.pi - 0.05), at(10.1, 0.0, 0.0, -math.pi + 0.05))
    assert v.valid and v.wz == pytest.approx(1.0)
    v = ov.body_velocity(at(10.0, 0.0, 0.0, -math.pi + 0.05), at(10.1, 0.0, 0.0, math.pi - 0.05))
    assert v.wz == pytest.approx(-1.0)


@pytest.mark.parametrize("previous, current, reason", [
    (None, at(10.0, 0, 0, 0), "first pose"),
    (at(10.0, 0, 0, 0), at(11.5, 0.6, 0, 0), "outside"),             # a gap is not a measurement
    (at(10.0, 0, 0, 0), at(10.0, 0.01, 0, 0), "outside"),            # same receive time
    (at(10.0, 0, 0, 0), at(9.9, 0.01, 0, 0), "outside"),             # out of order
    (at(10.0, 0, 0, 0), at(10.5, 0.0, 0, 2.0), "turn"),              # ambiguous heading change
])
def test_no_valid_velocity_is_never_a_measured_standstill(previous, current, reason):
    v = ov.body_velocity(previous, current)
    assert not v.valid and reason in v.reason


UNIT = {"x": 0.0, "y": 0.0, "z": math.sin(math.pi / 4), "w": math.cos(math.pi / 4)}


@pytest.mark.parametrize("pose, reason", [
    ({"position": {"x": 1.0, "y": 2.0, "z": 0.3}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 0}}, "not unit"),
    ({"position": {"x": 1.0, "y": 2.0, "z": 0.3}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 2.0}}, "not unit"),
    ({"position": {"x": 1.0, "y": 2.0, "z": 0.3}, "orientation": {**UNIT, "w": math.nan}}, "orientation"),
    ({"position": {"x": 1.0, "y": 2.0, "z": 0.3}, "orientation": {"x": 0, "y": 0, "z": 0}}, "orientation"),
    ({"position": {"x": math.inf, "y": 2.0, "z": 0.3}, "orientation": UNIT}, "position"),
    ({"position": {"x": 1.0, "y": 2.0}, "orientation": UNIT}, "position"),
    ({"position": {"x": True, "y": 2.0, "z": 0.3}, "orientation": UNIT}, "position"),
    ({"orientation": UNIT}, "pose without"),
    (None, "pose without"),
])
def test_a_malformed_pose_is_refused(pose, reason):
    s, why = ov.sample(5 * S, pose)
    assert s is None and reason in why


def test_a_valid_pose_gives_its_yaw():
    s, why = ov.sample(5 * S, {"position": {"x": 1.0, "y": 2.0, "z": 0.3}, "orientation": UNIT})
    assert why == "" and (s.received_ns, s.x, s.y) == (5 * S, 1.0, 2.0) and s.yaw == pytest.approx(math.pi / 2)


def test_the_driver_refuses_a_malformed_pose_before_both_outputs_and_derives_the_twist():
    source = (SDK_PKG / "go2_robot_sdk" / "go2_driver_node.py").read_text()
    publish = next(n for n in ast.walk(ast.parse(source))
                   if isinstance(n, ast.FunctionDef) and n.name == "publish_odometry_webrtc")
    body = ast.unparse(publish)
    refuse = body.index("if current is None:")
    assert refuse < body.index("self.broadcaster.sendTransform(odom_trans)") < body.index("if not velocity.valid:") \
        < body.index("odom_msg = Odometry()")
    assert "k != str(i)" in body[refuse:body.index("continue", refuse)]       # the velocity chain restarts
    assert "velocity = body_velocity(self._odom_previous.get(str(i)), current)" in body
    for line in ("odom_msg.twist.twist.linear.x = velocity.vx", "odom_msg.twist.twist.linear.y = velocity.vy",
                 "odom_msg.twist.twist.angular.z = velocity.wz"):
        assert line in body
    assert "cmd_vel" not in body                       # never an echo of the command
