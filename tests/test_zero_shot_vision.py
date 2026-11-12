"""Measurement checks for optional CLIP experiment; no model download in tests."""

import importlib

import cv2
import numpy as np
import pytest


def _experiment():
    return importlib.import_module("tools.evaluate_zero_shot_vision")


@pytest.mark.parametrize("scores,status,label", [
    ([0.35, 0.20], "accepted", "bird"),
    ([0.19, 0.10], "rejected_low_similarity", None),
    ([0.31, 0.30], "rejected_low_margin", None),
])
def test_fixed_rejection_policy_preserves_closed_set_prediction(scores, status, label):
    result = _experiment().classify_cosines(scores, ["bird", "drone"])
    assert result["decision"] == status
    assert result["predicted_class"] == label
    assert result["closed_set_class"] == "bird"


def test_rejection_policy_does_not_silently_accept_invalid_model_output():
    with pytest.raises(ValueError, match="finite"):
        _experiment().classify_cosines([float("nan"), 0.3], ["bird", "drone"])


def test_replaying_opencv_input_gives_clip_rgb_without_swapped_channels(tmp_path):
    # OpenCV stores a blue target as BGR; CLIP must receive the same blue in RGB.
    path = tmp_path / "blue.png"
    path.write_bytes(cv2.imencode(".png", np.full((11, 11, 3), [255, 0, 0], np.uint8))[1].tobytes())
    sample = {"source_path": "blue.png", "transform": {
        "corners_px": [[100, 100], [110, 100], [110, 110], [100, 110]],
        "brightness_alpha": 1.0, "brightness_beta": 0.0}}
    frame = _experiment().replay_frame(sample, tmp_path)
    assert frame.getpixel((105, 105)) == (0, 0, 255)


def test_negative_rejection_is_not_counted_as_target_class_accuracy():
    samples = [
        {"predicted_class": None, "closed_set_class": "bird", "expected_class": None, "inference_ms": 1.0},
        {"predicted_class": "bird", "closed_set_class": "bird", "expected_class": None, "inference_ms": 1.0},
    ]
    result = _experiment()._aggregate(samples)
    assert result["correct"] == 0
    assert result["correct_rejection"] == 1
    assert result["false_accept"] == 1
