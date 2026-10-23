"""Go2 data-channel samples (lidar frames, odometry poses), each consumed ONCE with its receive time (T5.9 robot host,
Codex #271 / #274).

The driver used to keep the latest sample and republish it on a timer stamped ``now``: one lidar frame or one pose
received once became "fresh" forever (a cloud, an odom->base_link transform, an Odometry), so no downstream freshness
check - pointcloud_to_scan max_cloud_age_s, the obstacle layer's expected_update_rate, a stamped TF lookup, the
localization gate - could ever see the source stop. Here every accepted sample carries a sequence number and its
receive time on the driver node's ROS clock (the clock the robot host's consumers judge age against); a timer takes a
sample only once and stamps what it publishes with that receive time - a stalled source publishes nothing.

BOUNDARY (Codex #274): the sequence is this driver's LOCAL receive order, not the sensor's acquisition sequence. A
vendor RESEND of old acquisition data arrives as a new sample and is indistinguishable here unless the wire stamp /
sequence contract of the Go2 is verified; unchanged geometry is never treated as stale (a stationary, valid sensor
repeats geometry). No ROS import: tested offline.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Frame:
    message: dict
    sequence: int
    received_ns: int


class ReceivedFrames:
    def __init__(self):
        self._latest: dict[str, Frame] = {}
        self._taken: dict[str, int] = {}
        self._sequence = 0

    def accept(self, robot: str, message: dict, now_ns: int) -> Frame:
        """A frame as it arrives (the data-channel callback): numbered and stamped with its receive time."""
        self._sequence += 1
        frame = Frame(message, self._sequence, int(now_ns))
        self._latest = {**self._latest, robot: frame}
        return frame

    def take(self, robot: str) -> Frame | None:
        """The newest frame not yet published, or None: never the same frame twice."""
        frame = self._latest.get(robot)
        if frame is None or self._taken.get(robot) == frame.sequence:
            return None
        self._taken = {**self._taken, robot: frame.sequence}
        return frame
