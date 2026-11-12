"""Public static candidate for a ground return; no ROS or live authority."""
from __future__ import annotations

from dataclasses import dataclass

from .ground_three_point_axis_plan import _AxisBuilder
from .ground_three_point_plan import GroundOperation, _pose


@dataclass(frozen=True)
class GroundReturnPlan:
    initial_pose: tuple[float, float, float]
    home_pose: tuple[float, float, float]
    operations: tuple[GroundOperation, ...]
    path_m: float
    rotation_rad: float
    minimum_laser_distance_m: float


def plan_ground_return(actual_pose) -> GroundReturnPlan:
    """Replan from the real terminal pose; original .641m ring may reject it."""
    pose = _pose(actual_pose)
    builder = _AxisBuilder(pose)
    builder.drive((.6, pose[1]), 'return_exit_to_spine')
    builder.drive((.6, -.2), 'return_north_to_turn_anchor')
    builder.reverse_axis((.6, 0.), 'return_reverse_north')
    builder.drive((0., 0.), 'return_west_to_home')
    builder.turn(0., 'return_final_alignment')
    operations = tuple(builder.operations)
    return GroundReturnPlan(pose, (0., 0., 0.), operations,
        sum(op.length_m for op in operations), sum(op.rotation_rad for op in operations),
        min([builder.initial_clearance] + [op.minimum_laser_distance_m for op in operations]))
