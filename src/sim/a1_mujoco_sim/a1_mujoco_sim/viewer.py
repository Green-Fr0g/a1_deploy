"""Passive MuJoCo viewer camera behaviour."""
from __future__ import annotations

import math

import numpy as np

from .model import RobotModel


def yaw_degrees(rotation: np.ndarray) -> float:
    """Return the rotation around the world Z axis of a rotation matrix."""
    return math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))


class FollowCamera:
    """Keeps the viewer camera behind and above a tracked body."""

    def __init__(self, body: str = 'trunk', distance: float = 1.6,
                 elevation: float = -18.0) -> None:
        self._body = body
        self._distance = float(distance)
        self._elevation = float(elevation)

    def update(self, viewer, robot: RobotModel) -> None:
        """Move the camera behind the tracked body, matching its heading."""
        pose = robot.body_pose(self._body)
        if pose is None:
            return
        position, rotation = pose
        viewer.cam.lookat[:] = position
        viewer.cam.azimuth = yaw_degrees(rotation)
        viewer.cam.elevation = self._elevation
        viewer.cam.distance = self._distance
