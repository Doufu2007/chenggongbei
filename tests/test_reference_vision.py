"""Protect measurement validity of the optional offline benchmark tool."""

import importlib

import numpy as np
import pytest


def _benchmark():
    return importlib.import_module("tools.evaluate_reference_vision")


def test_reference_split_does_not_count_duplicate_bytes_as_held_out():
    # A duplicated reference file must not inflate held-out recognition accuracy.
    assets = [
        {"class_id": "bird", "path": "bird_01.jpg", "sha256": "aaa"},
        {"class_id": "bird", "path": "bird_02.jpg", "sha256": "aaa"},
        {"class_id": "bird", "path": "bird_03.jpg", "sha256": "bbb"},
        {"class_id": "drone", "path": "drone_01.jpg", "sha256": "ccc"},
    ]
    references, held_out, excluded = _benchmark().split_assets(assets)
    assert [item["path"] for item in references] == ["bird_01.jpg", "drone_01.jpg"]
    assert [item["path"] for item in held_out] == ["bird_03.jpg"]
    assert [item["path"] for item in excluded] == ["bird_02.jpg"]


def test_reference_split_rejects_same_bytes_with_conflicting_classes():
    assets = [
        {"class_id": "bird", "path": "bird.jpg", "sha256": "same"},
        {"class_id": "drone", "path": "drone.jpg", "sha256": "same"},
    ]
    with pytest.raises(ValueError, match="conflicting"):
        _benchmark().split_assets(assets)


def test_scoring_marks_wrong_or_multiple_classes_as_errors():
    benchmark = _benchmark()
    detection = {"target_id": "bird", "center_px": [30.0, 40.0], "inlier_count": 20}
    assert benchmark.score_detections([], "bird", [33.0, 44.0])["status"] == "rejected"
    assert benchmark.score_detections([detection], "drone", [33.0, 44.0])["status"] == "misidentified"
    result = benchmark.score_detections([detection], "bird", [33.0, 44.0])
    assert result["status"] == "correct"
    assert result["center_error_px"] == pytest.approx(5.0)
    other = {**detection, "target_id": "drone"}
    assert benchmark.score_detections([detection, other], "bird", [33.0, 44.0])["status"] == "multiple_classes"


def test_projective_ground_truth_uses_surface_center_not_corner_average():
    benchmark = _benchmark()
    image = np.zeros((101, 101, 3), np.uint8)
    corners = np.float32([[0, 0], [400, 0], [600, 400], [0, 200]])
    frame, center = benchmark.project_image(image, corners, (640, 480))
    assert frame.shape == (480, 640, 3)
    assert center == pytest.approx([1200 / 7, 800 / 7], abs=1e-4)


def test_reference_replays_never_enter_generalization_aggregate():
    samples = [
        {"group": "reference_replay", "status": "correct", "inference_ms": 1.0, "center_error_px": 0.0},
        {"group": "held_out_clean", "status": "rejected", "inference_ms": 2.0, "center_error_px": None},
    ]
    summary = _benchmark().summarize(samples)
    assert summary["held_out_generalization"]["count"] == 1
    assert summary["held_out_generalization"]["correct"] == 0
    assert summary["held_out_generalization"]["correct_rate"] == 0.0
