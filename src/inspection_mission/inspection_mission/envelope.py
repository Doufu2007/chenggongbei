"""Explicit origin-relative UAV geometry; no robot dimensions are defaulted."""
from __future__ import annotations

from dataclasses import dataclass
import math


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


@dataclass(frozen=True)
class UAVEnvelope:
    """Conservative yaw-level bounds, including clearance and permitted tilt."""
    horizontal_radius_m: float
    below_origin_m: float
    above_origin_m: float
    max_tilt_rad: float


def uav_envelope_from_plan(plan):
    """Absent field retains the legacy sphere; incomplete geometry is an error.

    robot_radius_m encloses the level horizontal body, including every rotor.
    The explicit extents enclose the level body below/above its odometry origin.
    A tilt bound inflates all three extents; it never shrinks the supplied radius.
    """
    if "uav_body_envelope" not in plan:
        return None
    spec = plan["uav_body_envelope"]
    keys = {"below_origin_m", "above_origin_m", "max_tilt_rad"}
    if not isinstance(spec, dict) or set(spec) != keys:
        raise ValueError("uav_body_envelope requires below_origin_m, above_origin_m and max_tilt_rad")
    if (not all(_number(spec[key]) for key in keys)
            or min(spec["below_origin_m"], spec["above_origin_m"]) < 0
            or max(spec["below_origin_m"], spec["above_origin_m"]) <= 0
            or not 0 <= spec["max_tilt_rad"] <= .15):
        raise ValueError("uav_body_envelope requires nonnegative extents and tilt in [0,0.15]")
    radius, clearance = plan["robot_radius_m"], plan["clearance_m"]
    below, above, tilt = (float(spec[key]) for key in
                          ("below_origin_m", "above_origin_m", "max_tilt_rad"))
    sine = math.sin(tilt)
    return UAVEnvelope(radius + max(below, above)*sine + clearance,
                       below + radius*sine + clearance,
                       above + radius*sine + clearance, tilt)
