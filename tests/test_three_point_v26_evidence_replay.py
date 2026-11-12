"""Frozen real public observations reproduce the safety stop, not a motion model."""
import json
import math
from pathlib import Path

import pytest

from inspection_mission.ground_three_point import GroundThreePointGuard
from test_ground_observation_survey import posed_odom

ROOT = Path(__file__).resolve().parents[1]


def test_v26_real_motion_exceeds_cross_track_gate_with_current_first_drive_anchor():
    b = json.loads((ROOT/"runs/analysis/20261112_three_point_v26/bundle.json").read_text(encoding="utf-8"))
    a = b["survey"]["approach"]
    assert len(a["three_point"]["operation_records"]) == 2
    operation = a["three_point"]["operation_records"][1]
    assert operation["operation_index"] == 1 and operation["operation"]["kind"] == "DRIVE_TO_XY"
    origin = tuple(a["drive_origin"])
    assert tuple(operation["actual_start_pose"][:2]) == origin[:2]
    target = operation["operation"]["target_xy_m"]
    bearing = math.atan2(target[1]-origin[1], target[0]-origin[0])
    assert a["drive_yaw_rad"] == pytest.approx(bearing, abs=1e-15)
    sample = next(row for row in b["survey"]["trajectory_10hz"] if row["stamp_s"] == 17.16)
    guard = GroundThreePointGuard(started_wall_s=0.)
    payload = posed_odom(sample["stamp_s"], x=sample["position"][0], yaw=sample["yaw_rad"])
    payload["pose"]["pose"]["position"]["y"] = sample["position"][1]
    guard.update_odom(payload, wall_s=0.)
    guard.drive_origin, guard.drive_yaw = origin, bearing
    guard.phase = "DRIVE"
    assert guard.cross_track() == pytest.approx(.02015264842038747, abs=1e-12)
    assert guard.observed_motion_fault() == "approach_cross_track_limit"
    assert guard.approach_limits.max_cross_track_m == .02


def test_v26_position_differences_are_inconsistent_with_yaw_aligned_translation():
    b = json.loads((ROOT/"runs/analysis/20261112_three_point_v26/bundle.json").read_text(encoding="utf-8"))
    rows = b["survey"]["trajectory_10hz"]
    first = next(row for row in rows if row["stamp_s"] == 14.6)
    last = next(row for row in rows if row["stamp_s"] == 15.24)
    dx, dy = (last["position"][i]-first["position"][i] for i in (0, 1))
    bearing = math.atan2(dy, dx)
    assert -.08 < bearing < -.04
    assert -.27 < first["yaw_rad"] < -.24
    assert abs(bearing-first["yaw_rad"]) > .17
    # A body-aligned unit/gain-only synthetic actuator cannot explain this.
    ideal_dy = dx*math.tan(first["yaw_rad"])
    assert abs(ideal_dy-dy) > .006
