"""Axis candidate geometry; reverse operations require a new verified executor.

This module is not wired into v26. Static permission cannot replace live scans,
actual direction response, measured braking, recognition or official scoring.
"""
from __future__ import annotations

import math

from .ground_three_point_plan import (
    GroundOperation, GroundLegPlan, _Builder, _drive_clearance,
    _pose, _target, _wrap, RING_THRESHOLD_M,
)


class _AxisBuilder(_Builder):
    def axis_to(self, target, label, *, x_first=True):
        """Move each axis independently, preserving the real current anchor."""
        intermediate = (target[0], self.position[1]) if x_first else (self.position[0], target[1])
        self.drive(intermediate, label+"_axis_1")
        self.drive(target, label+"_axis_2")

    def reverse_axis(self, target, label):
        delta = (target[0]-self.position[0], target[1]-self.position[1])
        if abs(delta[0]) > 1e-12 and abs(delta[1]) > 1e-12:
            raise ValueError("reverse candidate must use one world axis")
        distance = math.hypot(*delta)
        if distance < 1e-12:
            return
        body_yaw = _wrap(math.atan2(delta[1], delta[0])+math.pi)
        self.turn(body_yaw, label+"_reverse_aim")
        clearance = _drive_clearance(self.position, target, self.heading)
        if not math.isfinite(distance) or clearance < RING_THRESHOLD_M-1e-12:
            raise ValueError("unsafe candidate reverse straight line")
        self.operations.append(GroundOperation("DRIVE_REVERSE_TO_XY", target, self.position,
            self.heading, self.heading, 0., label, clearance, distance))
        self.position = target


def _append_axis_entry(builder, target_id):
    _, _, endpoint, endpoint_yaw = _target(target_id)
    if target_id == "ground_2":
        builder.axis_to((0., -1.8), "entry_west", x_first=False)
        builder.reverse_axis(endpoint, "entry_reverse_south")
    elif target_id == "ground_4":
        builder.axis_to(endpoint, "entry_east_then_south", x_first=True)
    else:
        builder.axis_to(endpoint, "entry_south_then_horizontal", x_first=False)
    builder.turn(endpoint_yaw, "final_entry_alignment")


def plan_ground_entry(target_id, actual_pose) -> tuple[GroundOperation, ...]:
    """Geometry only; current visual evidence is enforced by a future executor."""
    _target(target_id)
    builder = _AxisBuilder(_pose(actual_pose))
    _append_axis_entry(builder, target_id)
    return tuple(builder.operations)


def plan_ground_leg(previous_id, target_id, actual_pose) -> GroundLegPlan:
    center, observation, endpoint, endpoint_yaw = _target(target_id)
    if previous_id is not None:
        _target(previous_id)
    pose = _pose(actual_pose)
    builder = _AxisBuilder(pose)
    if previous_id is None:
        builder.axis_to(observation, "first_observation", x_first=True)
    else:
        builder.drive((.6, pose[1]), "exit_to_spine")
        builder.drive((.6, observation[1]), "spine_to_observation_row")
        builder.drive(observation, "observation_branch")
    observation_yaw = math.atan2(center[1]-observation[1], center[0]-observation[0])
    builder.turn(observation_yaw, "observation_visual_heading")
    boundary = len(builder.operations)
    _append_axis_entry(builder, target_id)
    operations = tuple(builder.operations)
    return GroundLegPlan(previous_id, target_id, pose, observation, observation_yaw,
        endpoint, endpoint_yaw, sum(op.length_m for op in operations),
        sum(op.rotation_rad for op in operations), operations[:boundary], operations[boundary:],
        min([builder.initial_clearance]+[op.minimum_laser_distance_m for op in operations]))
