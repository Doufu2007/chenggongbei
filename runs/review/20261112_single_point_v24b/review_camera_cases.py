"""Independent camera boundary/source checks against the post-v24b patch."""
from copy import deepcopy
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "src/inspection_mission"), str(ROOT / "tests")]

import pytest
from test_ground_single_point import SingleRun
from test_ground_bounded_approach import image


@pytest.mark.parametrize("wall_age,source_age,expected", [
    (.21698330799834, .125, ""),
    (.5, .5, ""),
    (.500002, .2, "single_point_uav_image_stale"),
    (.2, .500002, "single_point_current_image_clock_inconsistent"),
    (.2, -.12, ""),
    (.2, -.120002, "single_point_current_image_clock_inconsistent")])
def test_camera_wall_and_source_boundaries_are_separate_and_keep_original_capture(wall_age, source_age, expected):
    run = SingleRun().until(lambda item: item.guard.state == "RECOGNIZE_WAIT")
    state = run.guard
    original = deepcopy(state.capture_frames)
    state.last_uav_image = (run.wall-wall_age, state.clock[0]-source_age)
    assert state._current_recognition_condition(run.wall) == expected
    assert state.capture_frames == original
    observation = state.approach_evidence()["single_point"]["camera_liveness"]
    assert observation["last_image_stamp_s"] == state.last_uav_image[1]
    assert observation["camera_max_wall_age_s"] == .5
    assert observation["camera_min_clock_age_s"] == -.12
    assert state.limits.sensor_age_s == .2


@pytest.mark.parametrize("stamp_offset", [0., -.001])
def test_duplicate_or_reversed_source_cannot_keep_frozen_camera_alive(stamp_offset):
    run = SingleRun().until(lambda item: item.guard.state == "RECOGNIZE_WAIT")
    state = run.guard
    original = state.last_uav_image
    run.tick(emit_image=False)
    state.observe_image("/uav/camera/image", image(original[1]+stamp_offset), wall_s=run.wall)
    assert state.last_uav_image == original
    state.last_uav_image = (run.wall-.500002, original[1])
    assert state._current_recognition_condition(run.wall) == "single_point_uav_image_stale"
