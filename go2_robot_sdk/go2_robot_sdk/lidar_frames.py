"""Lidar frames from the Go2 data channel, each consumed ONCE with its receive time (T5.9 robot host, Codex #271).

The driver used to keep the latest decoded frame and republish it on a 0.5 s timer stamped ``now``: one frame
received once became a "fresh" cloud forever, so no downstream freshness check (pointcloud_to_scan max_cloud_age_s,
the obstacle layer's expected_update_rate) could ever see the lidar stop. Here every accepted frame carries a sequence
number and its receive time on the driver node's ROS clock (the same clock pointcloud_to_scan judges age against on
this host); the timer takes a frame only once, and stamps the cloud with that receive time - a stalled source
publishes nothing and the sensor-stall contract stops the controller. No ROS import: tested offline.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Frame:
    message: dict
    sequence: int
    received_ns: int


class LidarFrames:
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
