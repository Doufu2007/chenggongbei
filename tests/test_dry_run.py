import math
import subprocess
import sys
from pathlib import Path

import pytest

from inspection_mission.control import Pose2D, parse_waypoints, ground_command, aerial_command, ground_reached
from tools.dry_run import run_dry_run


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("raw", [
    "",
    "   ",
    "0.0,1.0",
    "0.0,1.0,0.0;;1.0,2.0,0.0",
    "0.0,nan,0.0",
    "0.0,1.0,inf",
])
def test_parse_waypoints_rejects_empty_malformed_or_nonfinite_entries(raw):
    with pytest.raises(ValueError, match="waypoint"):
        parse_waypoints(raw)


def test_legacy_waypoint_controller_uses_only_artificial_fixture():
    assert run_dry_run([(0.4, 0.0, 0.0), (0.4, 0.4, 1.57), (0.0, 0.4, 3.14)],
                       [(0.4, 0.0, 1.0), (0.4, 0.4, 1.0)]).completed


def test_dry_run_reaches_home_after_aerial_scan():
    result = run_dry_run(
        [(0.8, 0.0, 0.0), (0.8, 1.0, 1.57), (0.0, 1.0, 3.14), (0.0, 0.0, -1.57)],
        [(0.8, 0.0, 1.05), (0.8, 1.0, 1.05), (0.0, 1.0, 1.05), (0.0, 0.0, 1.05)],
    )
    assert result.completed
    assert result.final_aerial.z <= 0.18


def test_dry_run_uses_explicit_return_waypoints():
    result = run_dry_run(
        [(0.8, 0.0, 0.0), (0.8, 1.0, 1.57), (0.0, 1.0, 3.14)],
        [(0.8, 0.0, 1.05), (0.8, 1.0, 1.05), (0.0, 1.0, 1.05), (0.0, 0.0, 1.05)],
        ground_return_waypoint=(0.0, 0.0, -1.57),
        aerial_return_waypoint=(0.0, 0.0, 0.0),
    )
    assert result.completed
    assert ground_reached(result.final_ground, (0.0, 0.0, -1.57), 0.18)


def test_controller_outputs_are_finite_and_bounded():
    pose = Pose2D()
    ground = ground_command(pose, (2.0, 0.0, 0.0), 0.18, 0.28, 0.55)
    aerial = aerial_command(pose, (0.0, 0.0, 1.05), 0.18, 0.35)
    assert all(math.isfinite(value) for value in ground + aerial)
    assert abs(ground[0]) <= 0.28
    assert abs(ground[1]) <= 0.55
    assert math.sqrt(sum(value * value for value in aerial)) <= 0.35 + 1e-9


def test_ground_controller_turns_before_driving_toward_a_target_behind():
    linear, angular = ground_command(Pose2D(), (-1.0, 0.0, 0.0), 0.18, 0.28, 0.55)
    assert linear == 0.0
    assert abs(angular) > 0.0


def test_dry_run_script_runs_as_standalone_process():
    result = subprocess.run(
        [sys.executable, str(ROOT / "tools" / "dry_run.py")],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert "completed=True" in result.stdout

