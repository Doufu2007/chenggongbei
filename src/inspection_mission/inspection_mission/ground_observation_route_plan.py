"""Pure candidate geometry for observing the current first ground point.

This module performs no I/O and imports no ROS, control or perception code.
The fixed public map/camera hypothesis is not an identified target, a live
free-space certificate, a velocity model or evidence of task completion.
"""
from __future__ import annotations

from dataclasses import dataclass
import math


CANDIDATE_WORLD_SOURCE = (
    "ranmengjia/ICAN@82b041043849a5d86e085390bf0df787366d98cd:"
    "binary_install/share/ican_inspection_sim/worlds/inspection_task1.world"
)
CANDIDATE_WORLD_SHA256 = "595cbb95a395a2ca8923b79a0a5d3523e9198711059307d8b8cde6d47f35c9c1"

# (target center, board size, outside observation). Public fixed geometry only.
_POINTS = {
    "ground_0": ((1.125, -1.350), (.55, .40), (.600, -.450)),
    "ground_1": ((0., -1.350), (.55, .40), (.600, -.500)),
    "ground_2": ((0., -2.700), (.55, .40), (.600, -1.800)),
    "ground_3": ((1.125, -1.925), (.40, .55), (.600, -1.025)),
    "ground_4": ((1.125, -.775), (.40, .55), (.350, 0.)),
}


@dataclass(frozen=True)
class GroundRouteSegment:
    start_xy_m: tuple[float, float]
    end_xy_m: tuple[float, float]
    heading_rad: float
    length_m: float


@dataclass(frozen=True)
class GroundRouteBudgets:
    """Geometric limits plus unvalidated runtime allocations, not a deadline promise."""

    max_path_m: float
    max_origin_displacement_m: float
    max_rotation_rad: float
    global_wall_s: float = 300.
    drive_wall_s: float = 190.
    turn_wall_s: float = 45.
    turn_total_wall_s: float = 100.
    capture_wall_s: float = 10.
    wait_wall_s: float = 5.
    wall_budget_status: str = "provisional_pending_platform_measurement"


@dataclass(frozen=True)
class CandidateGroundProjection:
    """Predicted corners at a hypothetical final pose, without live image input."""

    body_xy_m: tuple[float, float]
    body_yaw_rad: float
    world_corners_xyz_m: tuple[tuple[float, float, float], ...]
    pixel_corners_xy: tuple[tuple[float, float], ...]
    bbox_xyxy_px: tuple[float, float, float, float]
    bbox_extent_px: tuple[float, float]
    all_corners_in_frame: bool
    image_size_wh: tuple[int, int] = (800, 800)
    body_z_m: float = .055
    camera_translation_body_xyz_m: tuple[float, float, float] = (.390, 0., .130)
    intrinsics_fx_fy_cx_cy: tuple[float, float, float, float] = (
        476.7030836014194, 476.7030836014194, 400.5, 400.5
    )
    status: str = "prediction_not_platform_observation"


@dataclass(frozen=True)
class GroundObservationRoutePlan:
    first_id: str
    initial_pose: tuple[float, float, float]
    waypoints_xy_m: tuple[tuple[float, float], ...]
    segments: tuple[GroundRouteSegment, ...]
    target_xy_m: tuple[float, float]
    final_yaw_rad: float
    total_path_m: float
    planned_rotation_rad: float
    target_board_clearance_m: float
    budgets: GroundRouteBudgets
    candidate_projection: CandidateGroundProjection
    candidate_world_source: str = CANDIDATE_WORLD_SOURCE
    candidate_world_sha256: str = CANDIDATE_WORLD_SHA256
    candidate_mapping_status: str = "candidate_unverified"
    status: str = "candidate_plan_not_execution"


def _angle_difference(target, current):
    return math.atan2(math.sin(target-current), math.cos(target-current))


def _initial_pose(values):
    if not isinstance(values, (tuple, list)) or len(values) != 3:
        raise ValueError("initial_pose must contain ground x, y and yaw")
    if any(type(value) not in (int, float) for value in values):
        raise ValueError("initial_pose must be finite numeric ground x, y and yaw")
    try:
        pose = tuple(float(value) for value in values)
    except (ValueError, OverflowError) as exc:
        raise ValueError("invalid initial_pose") from exc
    if (not all(math.isfinite(value) for value in pose)
            or math.hypot(*pose[:2]) > .05+1e-12 or abs(pose[2]) > .05+1e-12):
        raise ValueError("initial_pose is outside the near-origin, near-zero-yaw contract")
    return pose


def _candidate_projection(center, size, body_xy, yaw):
    cx, cy = center
    w, h = size
    # World (x-,y-), (x+,y-), (x+,y+), (x-,y+); board top z=.012.
    corners = ((cx-w/2, cy-h/2, .012), (cx+w/2, cy-h/2, .012),
               (cx+w/2, cy+h/2, .012), (cx-w/2, cy+h/2, .012))
    pixels = []
    cosine, sine = math.cos(yaw), math.sin(yaw)
    for x, y, z in corners:
        dx, dy = x-body_xy[0], y-body_xy[1]
        forward = cosine*dx+sine*dy-.390
        left = -sine*dx+cosine*dy
        up = z-.055-.130
        if forward <= 0:
            raise ValueError("candidate board corner is behind the camera")
        pixels.append((400.5-476.7030836014194*left/forward,
                       400.5-476.7030836014194*up/forward))
    bounds = (min(p[0] for p in pixels), min(p[1] for p in pixels),
              max(p[0] for p in pixels), max(p[1] for p in pixels))
    return CandidateGroundProjection(
        body_xy, yaw, corners, tuple(pixels), bounds,
        (bounds[2]-bounds[0], bounds[3]-bounds[1]),
        all(0 <= x < 800 and 0 <= y < 800 for x, y in pixels),
    )


def plan_ground_observation_route(first_id, initial_pose):
    """Plan only one observed public first ID from actual (x,y,yaw).

    Waypoints are absolute candidate-map XY destinations and exclude the
    initial pose. All total distances/angles use that actual initial pose.
    The permanent-origin cap is relative to the same initial XY, never a
    later waypoint or scan anchor. target_board_clearance_m is the final
    center-to-board-rectangle distance before subtracting any robot radius.
    Runtime must separately validate sensors, local free space and budgets;
    The runtime allocation is 300 s global, 190 s cumulative drive,
    45 s per turn and 100 s cumulative turn. It is provisional and assumes
    no actuator response gain; execution may supply tighter time budgets.
    """
    if not isinstance(first_id, str) or first_id not in _POINTS:
        raise ValueError("one valid public first ground ID is required")
    pose = _initial_pose(initial_pose)
    center, size, observation = _POINTS[first_id]
    waypoints = (observation,) if first_id == "ground_4" else ((.600, 0.), observation)
    position, heading = pose[:2], pose[2]
    segments = []
    rotation = 0.
    for destination in waypoints:
        dx, dy = destination[0]-position[0], destination[1]-position[1]
        next_heading = math.atan2(dy, dx)
        segments.append(GroundRouteSegment(position, destination, next_heading, math.hypot(dx, dy)))
        rotation += abs(_angle_difference(next_heading, heading))
        position, heading = destination, next_heading
    final_yaw = math.atan2(center[1]-observation[1], center[0]-observation[0])
    rotation += abs(_angle_difference(final_yaw, heading))
    path = sum(segment.length_m for segment in segments)
    origin_peak = max(math.dist(pose[:2], waypoint) for waypoint in waypoints)
    gap = math.hypot(max(abs(observation[0]-center[0])-size[0]/2, 0.),
                     max(abs(observation[1]-center[1])-size[1]/2, 0.))
    return GroundObservationRoutePlan(
        first_id, pose, waypoints, tuple(segments), center, final_yaw, path, rotation, gap,
        GroundRouteBudgets(path+.30, origin_peak+.15, rotation+.40),
        _candidate_projection(center, size, observation, final_yaw),
    )
