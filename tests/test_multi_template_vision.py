"""Multiple observed artworks may share one official category identity."""
import json

import cv2
import numpy as np
import pytest
import yaml

from inspection_mission import vision_core
from inspection_mission.validation import validate_configuration
from test_official_adapter import _official_configuration
from test_vision_core import _image_message, _reference_image, _vision_node_boundary


def save_reference(tmp_path, name, image):
    path = tmp_path / (name + ".png")
    path.write_bytes(cv2.imencode(".png", image)[1].tobytes())
    return str(path)


def framed(image):
    result = np.full((600, 800), 225, dtype=np.uint8)
    result[100:400, 100:480] = image
    return result


def match(image, paths):
    return vision_core.detect_targets(image, enable_qr=False, enable_aruco=False,
                                      template_matcher=vision_core.TemplateMatcher(paths))


def test_two_distinct_reference_images_match_the_same_category(tmp_path):
    first, second = _reference_image(41), _reference_image(92)
    paths = {"被困人员": [save_reference(tmp_path, "first", first),
                        save_reference(tmp_path, "second", second)]}
    for artwork in (first, second):
        results = match(framed(artwork), paths)
        assert len(results) == 1
        assert results[0]["target_id"] == "被困人员"
        assert results[0]["center"] == pytest.approx([290 / 800, 250 / 600], abs=0.015)


def test_single_string_reference_is_backward_compatible(tmp_path):
    image = _reference_image(41)
    results = match(framed(image), {"single": save_reference(tmp_path, "single", image)})
    assert len(results) == 1 and results[0]["target_id"] == "single"


@pytest.mark.parametrize("reverse", [False, True])
def test_same_surface_same_category_keeps_one_best_geometry(tmp_path, reverse):
    image = _reference_image(41)
    padded = cv2.copyMakeBorder(image, 0, 0, 0, 120, cv2.BORDER_CONSTANT, value=225)
    paths = [save_reference(tmp_path, "original", image), save_reference(tmp_path, "padded", padded)]
    if reverse:
        paths.reverse()
    results = match(framed(image), {"same-category": paths})
    assert len(results) == 1
    assert results[0]["target_id"] == "same-category"
    isolated = [match(framed(image), {"same-category": path})[0] for path in paths]
    assert results[0]["inlier_count"] == max(item["inlier_count"] for item in isolated)


def test_distinct_surfaces_with_same_category_remain_separate_observations(tmp_path):
    first, second = _reference_image(41), _reference_image(92)
    paths = {"same-category": [save_reference(tmp_path, "first", first),
                              save_reference(tmp_path, "second", second)]}
    image = np.full((600, 1000), 225, np.uint8)
    image[130:430, 40:420] = first
    image[130:430, 580:960] = second
    results = match(image, paths)
    assert len(results) == 2
    assert {item["target_id"] for item in results} == {"same-category"}
    assert sorted(item["center"][0] for item in results) == pytest.approx([0.23, 0.77], abs=0.015)


def test_same_category_duplicates_do_not_hide_competing_other_category(tmp_path):
    image = _reference_image(41)
    original = save_reference(tmp_path, "original", image)
    duplicate = save_reference(tmp_path, "duplicate", image)
    assert match(framed(image), {"category-one": [original, duplicate], "category-two": original}) == []


@pytest.mark.parametrize("value", [[], [""], [None], [1], [["nested.png"]], 17])
def test_invalid_multi_reference_values_fail_explicitly(value):
    with pytest.raises(ValueError, match="reference"):
        vision_core.TemplateMatcher({"category": value})


def test_vision_node_loads_list_without_changing_category_label(tmp_path, monkeypatch):
    first, second = _reference_image(41), _reference_image(92)
    paths = {"被困人员": [save_reference(tmp_path, "first", first),
                        save_reference(tmp_path, "second", second)]}
    node, _ = _vision_node_boundary(monkeypatch, {
        "template_paths_json": json.dumps(paths), "enable_qr": False, "enable_aruco": False})
    node._image_cb(_image_message(framed(second)), "ground")
    assert node.messages[-1]["error"] == ""
    assert [item["target_id"] for item in node.messages[-1]["detections"]] == ["被困人员"]


def config_with_references(tmp_path, references):
    plan_path, profile_path = _official_configuration(tmp_path)
    config = {"inspection_mission": {"ros__parameters": {
        "plan_path": str(plan_path), "interface_profile_path": str(profile_path), "use_sim_time": True}},
        "inspection_vision": {"ros__parameters": {"ground_image_topic": "/ground/image",
            "uav_image_topic": "/uav/image", "use_sim_time": True,
            "template_paths_json": json.dumps(references)}}}
    path = tmp_path / "settings.yaml"
    path.write_text(yaml.safe_dump(config), encoding="utf-8")
    return path


def test_prelaunch_accepts_multiple_absolute_paths_per_category(tmp_path):
    first = save_reference(tmp_path, "first", _reference_image(41))
    second = save_reference(tmp_path, "second", _reference_image(92))
    checked = validate_configuration(config_with_references(tmp_path, {"被困人员": [first, second]}))
    assert json.loads(checked["vision"]["template_paths_json"]) == {"被困人员": [first, second]}


def test_prelaunch_checks_every_path_in_a_category(tmp_path):
    first = save_reference(tmp_path, "first", _reference_image(41))
    with pytest.raises(ValueError, match="absolute"):
        validate_configuration(config_with_references(tmp_path, {"category": [first, "relative.png"]}))
