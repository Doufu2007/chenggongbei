"""Camera crops keep exact provenance and cannot supply missing target labels."""
from __future__ import annotations

import hashlib
import importlib
import importlib.util
import json

import numpy as np
from PIL import Image
import pytest


def api():
    name = "tools.analyze_platform_camera_frames"
    assert importlib.util.find_spec(name) is not None, "real camera crop analysis tool is missing"
    return importlib.import_module(name)


def fixture(tmp_path):
    image = np.zeros((20, 30, 3), dtype=np.uint8)
    image[:, :, 1] = np.arange(30, dtype=np.uint8)[None, :] * 5
    path = tmp_path / "image.jpg"
    Image.fromarray(image).save(path, "JPEG")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    summary = {"mode": "bounded_ground_motion", "read_only": False,
               "source_evidence": {"sha256": "a" * 64},
               "images": [{"file_name": path.name, "sha256": digest, "topic": "/camera/image",
                           "frame_id": "camera_link", "stamp": 2.9, "width": 30, "height": 20,
                           "source": "unverified_callback", "line": 4}]}
    summary_path = tmp_path / "summary.json"
    summary_path.write_text(json.dumps(summary), encoding="utf-8")
    plan = {"source_callback_sha256": "a" * 64, "review_method": "visual inspection of real source image",
            "images": [{"sha256": digest, "observation": "uniform region; no complete panel observed",
                        "panel_adequacy": "insufficient", "regions": [
                            {"id": "observed_patch", "bbox_xyxy": [3, 4, 18, 16],
                             "observation": "actual pixels only", "panel_adequacy": "insufficient"}]}]}
    return summary_path, plan, np.array(Image.open(path).convert("RGB"))


def test_real_crop_preserves_pixels_hash_and_sensor_provenance_without_target_label(tmp_path):
    path, plan, decoded = fixture(tmp_path)
    records = api().prepare_crops(path, plan, tmp_path / "out")
    assert len(records) == 1
    record = records[0]
    saved = np.array(Image.open(tmp_path / "out" / record["crop_file"]).convert("RGB"))
    assert np.array_equal(saved, decoded[4:16, 3:18])
    assert (record["source"]["topic"], record["source"]["frame_id"], record["source"]["stamp"]) == ("/camera/image", "camera_link", 2.9)
    assert record["source"]["line"] == 4 and record["source"]["source"] == "unverified_callback"
    assert record["crop_sha256"] == hashlib.sha256((tmp_path / "out" / record["crop_file"]).read_bytes()).hexdigest()
    assert record["expected_class"] is None and record["recognized_target"] is None
    assert record["panel_adequacy"] == "insufficient"


@pytest.mark.parametrize("change", ["image_hash", "callback_hash", "image_dimensions", "duplicate_image"])
def test_mismatched_evidence_cannot_silently_become_a_real_crop(tmp_path, change):
    path, plan, _ = fixture(tmp_path)
    summary = json.loads(path.read_text(encoding="utf-8"))
    if change == "image_hash": (tmp_path / "image.jpg").write_bytes(b"changed")
    elif change == "callback_hash": plan["source_callback_sha256"] = "b" * 64
    elif change == "image_dimensions": summary["images"][0]["width"] = 40
    elif change == "duplicate_image": summary["images"].append(dict(summary["images"][0]))
    path.write_text(json.dumps(summary), encoding="utf-8")
    with pytest.raises(ValueError): api().prepare_crops(path, plan, tmp_path / "out")


@pytest.mark.parametrize("bbox", [[-1, 0, 4, 4], [0, 0, 31, 20], [3, 3, 3, 6], [1.5, 0, 4, 5]])
def test_crop_bounds_are_explicit_and_never_clamped_into_different_observation(tmp_path, bbox):
    path, plan, _ = fixture(tmp_path)
    plan["images"][0]["regions"][0]["bbox_xyxy"] = bbox
    with pytest.raises(ValueError): api().prepare_crops(path, plan, tmp_path / "out")


def test_roi_annotations_cannot_supply_class_answers(tmp_path):
    path, plan, _ = fixture(tmp_path)
    plan["images"][0]["regions"][0]["expected_class"] = "landslide"
    with pytest.raises(ValueError): api().prepare_crops(path, plan, tmp_path / "out")
