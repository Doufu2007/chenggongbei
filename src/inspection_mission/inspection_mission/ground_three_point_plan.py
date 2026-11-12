"""Immutable candidate corridor operations, without ROS, I/O or live authority.

Static distances are geometric permission only. Runtime scan, freshness,
tracking, speed and permanent mission budgets remain necessary. Nominal ring
clearance reaches only .650m against the unchanged .641m gate (9mm margin).
"""
from __future__ import annotations

from dataclasses import dataclass
import math


CANDIDATE_WORLD_SOURCE = (
    "ranmengjia/ICAN@82b041043849a5d86e085390bf0df787366d98cd:"
    "binary_install/share/ican_inspection_sim/worlds/inspection_task1.world"
)
CANDIDATE_WORLD_SHA256 = "595cbb95a395a2ca8923b79a0a5d3523e9198711059307d8b8cde6d47f35c9c1"
RING_THRESHOLD_M = .641
LASER_FORWARD_OFFSET_M = .08

# All candidate collision boxes intersecting the actual laser plane z=.135m.
# (name, xmin/ymin/xmax/ymax); source and exact frozen world hash above.
BOXES = (
    ("north_wall/collision", (-.8, .65, 4.25, .70)),
    ("south_wall/collision", (-.8, -3.4, 4.25, -3.35)),
    ("west_wall/collision", (-.825, -3.375, -.775, .675)),
    ("east_wall/collision", (4.225, -3.375, 4.275, .675)),
    ("window_1/bottom", (1.845, -.8275, 2.005, .1725)),
    ("window_1/side_negative", (1.845, -1.0275, 2.005, -.8275)),
    ("window_1/side_positive", (1.845, .1725, 2.005, .3725)),
    ("window_2/bottom", (1.845, -2.8725, 2.005, -1.8725)),
    ("window_2/side_negative", (1.845, -3.0725, 2.005, -2.8725)),
    ("window_2/side_positive", (1.845, -1.8725, 2.005, -1.6725)),
)

# Public candidate center, outside observation, final ground-contact pose.
_POINTS = {
    "ground_0": ((1.125, -1.35), (.6, -.45), (1.115, -1.35), 0.),
    "ground_1": ((0., -1.35), (.6, -.5), (0., -1.35), math.pi),
    "ground_2": ((0., -2.7), (.6, -1.8), (0., -2.69), math.pi),
    "ground_3": ((1.125, -1.925), (.6, -1.025), (1.115, -1.925), -math.pi/2),
    "ground_4": ((1.125, -.775), (.35, -.10), (1.115, -.775), -math.pi/2),
}


@dataclass(frozen=True)
class GroundOperation:
    kind: str
    target_xy_m: tuple[float, float] | None
    anchor_xy_m: tuple[float, float]
    expected_start_yaw_rad: float
    target_yaw_rad: float
    signed_subturn_rad: float
    source_phase: str
    minimum_laser_distance_m: float
    length_m: float = 0.
    settle_required_after: bool = True

    @property
    def heading(self):
        return self.target_yaw_rad

    @property
    def turn_direction(self):
        return "CCW" if self.signed_subturn_rad > 0 else "CW" if self.signed_subturn_rad < 0 else None

    @property
    def rotation_rad(self):
        return abs(self.signed_subturn_rad)


@dataclass(frozen=True)
class GroundLegPlan:
    previous_id: str | None
    target_id: str
    initial_pose: tuple[float, float, float]
    observation_xy_m: tuple[float, float]
    observation_yaw_rad: float
    endpoint_xy_m: tuple[float, float]
    endpoint_yaw_rad: float
    path_m: float
    rotation_rad: float
    to_observation_operations: tuple[GroundOperation, ...]
    entry_operations: tuple[GroundOperation, ...]
    minimum_laser_distance_m: float
    candidate_world_sha256: str = CANDIDATE_WORLD_SHA256
    candidate_mapping_status: str = "candidate_unverified"


def _wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def _pose(values):
    if not isinstance(values, (tuple, list)) or len(values) != 3:
        raise ValueError("actual_pose must contain three finite numeric coordinates")
    if any(type(value) not in (float, int) for value in values):
        raise ValueError("actual_pose must contain three finite numeric coordinates")
    try:
        if not all(math.isfinite(value) for value in values):
            raise ValueError("actual_pose must be finite")
        return tuple(float(value) for value in values)
    except (OverflowError, TypeError):
        raise ValueError("actual_pose must contain three finite numeric coordinates") from None


def _target(target_id):
    if not isinstance(target_id, str) or target_id not in _POINTS:
        raise ValueError("unknown ground target")
    return _POINTS[target_id]


def _point_rect(point, rect):
    return math.hypot(max(rect[0]-point[0], 0., point[0]-rect[2]),
                      max(rect[1]-point[1], 0., point[1]-rect[3]))


def _point_segment(point, start, end):
    dx, dy = end[0]-start[0], end[1]-start[1]
    scale = max(abs(dx), abs(dy))
    if scale == 0.:
        return math.dist(point, start)
    # Scaling avoids overflow for finite, far-away invalid poses.
    ux, uy = dx/scale, dy/scale
    projection = ((point[0]-start[0])/scale*ux+(point[1]-start[1])/scale*uy)/(ux*ux+uy*uy)
    fraction = max(0., min(1., projection))
    return math.hypot(point[0]-start[0]-fraction*dx, point[1]-start[1]-fraction*dy)


def _segment_rect(start, end, rect):
    low, high = 0., 1.
    for coordinate, delta, minimum, maximum in zip(start, (end[0]-start[0], end[1]-start[1]), rect[:2], rect[2:]):
        if abs(delta) < 1e-15:
            if not minimum <= coordinate <= maximum:
                break
        else:
            a, b = sorted(((minimum-coordinate)/delta, (maximum-coordinate)/delta))
            low, high = max(low, a), min(high, b)
            if low > high:
                break
    else:
        return 0.
    corners = ((x, y) for x in (rect[0], rect[2]) for y in (rect[1], rect[3]))
    return min(_point_rect(start, rect), _point_rect(end, rect),
               *(_point_segment(point, start, end) for point in corners))


def _arc_rect(center, start_yaw, end_yaw, rect):
    low, high = sorted((start_yaw, end_yaw))
    candidates = [start_yaw, end_yaw]
    principal = [i*math.pi/2 for i in range(4)]
    principal += [math.atan2(y-center[1], x-center[0]) for x in (rect[0], rect[2]) for y in (rect[1], rect[3])]
    for x in (rect[0], rect[2]):
        value = (x-center[0])/LASER_FORWARD_OFFSET_M
        if -1. <= value <= 1.:
            principal += [math.acos(value), -math.acos(value)]
    for y in (rect[1], rect[3]):
        value = (y-center[1])/LASER_FORWARD_OFFSET_M
        if -1. <= value <= 1.:
            angle = math.asin(value)
            principal += [angle, math.pi-angle]
    for angle in principal:
        for k in range(math.ceil((low-angle)/math.tau), math.floor((high-angle)/math.tau)+1):
            candidates.append(angle+k*math.tau)
    return min(_point_rect((center[0]+LASER_FORWARD_OFFSET_M*math.cos(angle),
                           center[1]+LASER_FORWARD_OFFSET_M*math.sin(angle)), rect) for angle in candidates)


def _turn_clearance(position, heading, delta):
    return min(_arc_rect(position, heading, heading+delta, rect) for _, rect in BOXES)


def _drive_clearance(start, end, heading):
    offset = (LASER_FORWARD_OFFSET_M*math.cos(heading), LASER_FORWARD_OFFSET_M*math.sin(heading))
    a = (start[0]+offset[0], start[1]+offset[1])
    b = (end[0]+offset[0], end[1]+offset[1])
    return min(_segment_rect(a, b, rect) for _, rect in BOXES)


class _Builder:
    def __init__(self, pose):
        self.position, self.heading = pose[:2], _wrap(pose[2])
        self.operations = []
        self.initial_clearance = _turn_clearance(self.position, self.heading, 0.)
        if self.initial_clearance < RING_THRESHOLD_M-1e-12:
            raise ValueError("unsafe candidate initial laser ring")

    def turn(self, target, label):
        target = _wrap(target)
        principal = _wrap(target-self.heading)
        deltas = [principal]
        if abs(principal) > 1e-12:
            deltas.append(principal-math.copysign(math.tau, principal))
        passing = [(abs(delta), delta) for delta in deltas
                   if _turn_clearance(self.position, self.heading, delta) >= RING_THRESHOLD_M-1e-12]
        if not passing:
            raise ValueError("unsafe candidate turn: neither angular arc passes fixed ring")
        _, delta = min(passing)
        if abs(delta) <= 1e-12:
            self.heading = target
            return
        count = math.ceil(abs(delta)/(math.pi/2)-1e-12)
        continuous = self.heading
        for index in range(count):
            step = math.copysign(min(math.pi/2, abs(delta)-index*math.pi/2), delta)
            clearance = _turn_clearance(self.position, continuous, step)
            self.operations.append(GroundOperation("TURN_ONLY", None, self.position,
                _wrap(continuous), _wrap(continuous+step), step, label, clearance))
            continuous += step
        self.heading = target

    def drive(self, target, label):
        distance = math.dist(self.position, target)
        if distance < 1e-12:
            return
        heading = math.atan2(target[1]-self.position[1], target[0]-self.position[0])
        self.turn(heading, label+"_aim")
        clearance = _drive_clearance(self.position, target, self.heading)
        if not math.isfinite(distance) or clearance < RING_THRESHOLD_M-1e-12:
            raise ValueError("unsafe candidate straight line")
        self.operations.append(GroundOperation("DRIVE_TO_XY", target, self.position,
            self.heading, self.heading, 0., label, clearance, distance))
        self.position = target


def _append_entry(builder, target_id):
    _, _, endpoint, endpoint_yaw = _target(target_id)
    if target_id == "ground_0":
        builder.drive((.6, -1.35), "entry_leg_1")
    elif target_id == "ground_2":
        builder.drive((.6, -2.62), "entry_leg_1")
    builder.drive(endpoint, "entry_endpoint")
    builder.turn(endpoint_yaw, "final_entry_alignment")


def plan_ground_entry(target_id, actual_pose) -> tuple[GroundOperation, ...]:
    """Replan entry from a current real pose after current visual authorization.

    This function does not authorize entry or provide recognition evidence.
    The caller must separately check it is still at its observation station.
    """
    _target(target_id)
    builder = _Builder(_pose(actual_pose))
    _append_entry(builder, target_id)
    return tuple(builder.operations)


def plan_ground_leg(previous_id, target_id, actual_pose) -> GroundLegPlan:
    """Plan actual pose -> outside observation -> candidate ground endpoint.

    All point IDs select public geometry only; no class, score or referee
    result is inferred. Both transfer and entry remain subject to live gates.
    """
    center, observation, endpoint, endpoint_yaw = _target(target_id)
    if previous_id is not None:
        _target(previous_id)
    pose = _pose(actual_pose)
    builder = _Builder(pose)
    if previous_id is None:
        if target_id != "ground_4":
            builder.drive((.6, 0.), "first_east_gateway")
        builder.drive(observation, "first_to_observation")
    else:
        builder.drive((.6, pose[1]), "exit_to_spine")
        builder.drive((.6, observation[1]), "spine_to_observation_row")
        builder.drive(observation, "observation_branch")
    observation_yaw = math.atan2(center[1]-observation[1], center[0]-observation[0])
    builder.turn(observation_yaw, "observation_visual_heading")
    boundary = len(builder.operations)
    _append_entry(builder, target_id)
    operations = tuple(builder.operations)
    return GroundLegPlan(previous_id, target_id, pose, observation, observation_yaw,
        endpoint, endpoint_yaw, sum(op.length_m for op in operations),
        sum(op.rotation_rad for op in operations), operations[:boundary], operations[boundary:],
        min([builder.initial_clearance]+[op.minimum_laser_distance_m for op in operations]))
