"""ROS-independent geometry and bounded velocity helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    stamp: float = 0.0


def parse_waypoints(raw: str) -> list[tuple[float, float, float]]:
    """Parse every configured point or reject the entire route.

    Dropping an invalid item silently changes the travelled route. Empty
    segments, including a trailing separator, therefore fail before indexing
    or motion can begin.
    """
    if not isinstance(raw, str) or not raw.strip():
        raise ValueError("waypoints must be a nonempty string of coordinate triples")
    points: list[tuple[float, float, float]] = []
    for index, token in enumerate(raw.split(";"), 1):
        values = [part.strip() for part in token.split(",")]
        if len(values) != 3 or not all(values):
            raise ValueError(f"waypoint {index} must contain exactly three coordinates")
        try:
            point = tuple(float(value) for value in values)
        except ValueError as exc:
            raise ValueError(f"waypoint {index} contains a nonnumeric coordinate") from exc
        if not all(math.isfinite(value) for value in point):
            raise ValueError(f"waypoint {index} coordinates must be finite")
        points.append(point)
    return points


def angle_error(target: float, current: float) -> float:
    return math.atan2(math.sin(target - current), math.cos(target - current))


def ground_reached(pose: Pose2D, target: tuple[float, float, float], tolerance_m: float, yaw_tolerance_rad: float = 0.2) -> bool:
    distance = math.hypot(target[0] - pose.x, target[1] - pose.y)
    return distance <= tolerance_m and abs(angle_error(target[2], pose.yaw)) < yaw_tolerance_rad


def ground_command(
    pose: Pose2D,
    target: tuple[float, float, float],
    tolerance_m: float,
    max_speed_mps: float,
    max_yaw_rate_rps: float,
) -> tuple[float, float]:
    dx, dy = target[0] - pose.x, target[1] - pose.y
    distance = math.hypot(dx, dy)
    if distance > tolerance_m:
        heading = math.atan2(dy, dx)
        error = angle_error(heading, pose.yaw)
        linear = min(max_speed_mps, 0.65 * distance) * max(0.0, math.cos(error)) if abs(error) < 0.5 else 0.0
        angular = max(-max_yaw_rate_rps, min(max_yaw_rate_rps, 1.2 * error))
        return linear, angular
    angular = max(-max_yaw_rate_rps, min(max_yaw_rate_rps, angle_error(target[2], pose.yaw)))
    return 0.0, angular


def aerial_reached(pose: Pose2D, target: tuple[float, float, float], tolerance_m: float) -> bool:
    return math.sqrt((target[0] - pose.x) ** 2 + (target[1] - pose.y) ** 2 + (target[2] - pose.z) ** 2) <= tolerance_m


def aerial_command(
    pose: Pose2D,
    target: tuple[float, float, float],
    tolerance_m: float,
    max_speed_mps: float,
) -> tuple[float, float, float]:
    dx, dy, dz = target[0] - pose.x, target[1] - pose.y, target[2] - pose.z
    distance = math.sqrt(dx * dx + dy * dy + dz * dz)
    if distance <= tolerance_m:
        return 0.0, 0.0, 0.0
    speed = min(max_speed_mps, 0.7 * distance)
    norm = max(distance, 1e-6)
    return speed * dx / norm, speed * dy / norm, speed * dz / norm

