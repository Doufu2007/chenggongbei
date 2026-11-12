"""Preserve observable geometry without inventing calibration or flying."""
from __future__ import annotations

import copy
import math

import pytest

from inspection_mission.runtime import RuntimeInputs
from test_runtime import configured_profile, odom


def evidence_odom(stamp=5.0, *, child="base_footprint", frame="odom", x=1.0):
    payload = odom(stamp, frame)
    payload["header"]["stamp"] = {"sec": int(stamp), "nanosec": round((stamp % 1) * 1e9)}
    payload["child_frame_id"] = child
    payload["pose"]["pose"]["position"]["x"] = x
    payload["twist"] = {"twist": {"linear": {"x": .1, "y": -.02, "z": .03},
                                     "angular": {"x": .01, "y": .02, "z": .04}},
                         "covariance": [0.0] * 36}
    return payload


def inputs(**kwargs):
    return RuntimeInputs(configured_profile(), {"g1", "g2", "g3"}, **kwargs)


def match(data, stamp, *, now=5.3, child="base_footprint", max_skew=.2, max_age=.5):
    return data.pose_at_exposure("ground", stamp, expected_child_frame=child,
                                 max_skew_s=max_skew, now=now, max_age_s=max_age)


def test_runtime_keeps_original_quaternion_child_header_stamp_and_twist_independent_of_input_mutation():
    data = inputs()
    payload = evidence_odom(5.0)
    payload["header"]["stamp"]["nanosec"] = 123
    q = {"x": math.sin(.01), "y": 0., "z": 0., "w": math.cos(.01)}
    payload["pose"]["pose"]["orientation"] = q
    original = copy.deepcopy(payload)
    data.update_pose("ground", payload, now=5.1)
    full = data.odometry_evidence["ground"]
    assert full.header == original["header"]
    assert full.child_frame_id == "base_footprint"
    assert full.rotation_xyzw == (q["x"], 0., 0., q["w"])
    assert full.stamp_s == 5.000000123
    assert full.twist == original["twist"]
    payload["header"]["frame_id"] = "changed"
    payload["twist"]["twist"]["linear"]["x"] = 200
    matched = match(data, 5.01, now=5.1)
    assert matched.header["frame_id"] == "odom" and matched.twist["twist"]["linear"]["x"] == .1
    assert matched.stamp_s == 5.000000123
    assert data.poses["ground"].yaw == 0  # Original planar controller remains compatible.


def test_geometry_query_requires_confirmed_actual_child_but_legacy_pose_still_works():
    data = inputs()
    data.update_pose("ground", odom(), now=5)
    assert data.poses["ground"] is not None
    with pytest.raises(ValueError, match="child"):
        match(data, 5, now=5)
    with pytest.raises(ValueError, match="child"):
        match(data, 5, now=5, child="")


def test_explicit_child_frame_binding_rejects_wrong_child_and_clears_old_geometry():
    profile = configured_profile()
    profile["channels"]["ground_odom"]["child_frame_id"] = "base_footprint"
    data = RuntimeInputs(profile, {"g1", "g2", "g3"})
    data.update_pose("ground", evidence_odom(), now=5)
    with pytest.raises(ValueError, match="child"):
        data.update_pose("ground", evidence_odom(5.1, child="base_link"), now=5.1)
    assert data.poses["ground"] is None and data.odometry_evidence["ground"] is None
    with pytest.raises(ValueError, match="history"):
        match(data, 5, now=5.1)


def test_exposure_match_uses_only_preceding_original_pose_not_latest_future_pose_or_extrapolation():
    data = inputs()
    data.update_pose("ground", evidence_odom(5., x=1), now=5.)
    data.update_pose("ground", evidence_odom(5.1, x=2), now=5.1)
    selected = match(data, 5.05, now=5.15)
    assert selected.stamp_s == 5.0 and selected.position_m == (1., 2., 0.)
    assert selected.header["stamp"] == {"sec": 5, "nanosec": 0}
    selected.header["frame_id"] = "caller mutation"
    assert match(data, 5.05, now=5.15).header["frame_id"] == "odom"


@pytest.mark.parametrize("stamp,now,skew,age,reason", [
    (4.9, 5.1, .2, 1., "preceding"),
    (5.1, 5.1, .05, 1., "skew"),
    (5.2, 5.1, .3, 1., "future"),
    (5., 6., .2, .5, "stale"),
])
def test_exposure_match_rejects_future_stale_missing_or_excess_skew(stamp, now, skew, age, reason):
    data = inputs()
    data.update_pose("ground", evidence_odom(5.), now=5.)
    with pytest.raises(ValueError, match=reason):
        match(data, stamp, now=now, max_skew=skew, max_age=age)


@pytest.mark.parametrize("limits", [
    {"pose_history_max_samples": 2, "pose_history_span_s": 2.},
    {"pose_history_max_samples": 20, "pose_history_span_s": .15},
])
def test_history_prunes_by_both_count_and_time_span(limits):
    data = inputs(**limits)
    for stamp in (5., 5.1, 5.2):
        data.update_pose("ground", evidence_odom(stamp), now=stamp)
    with pytest.raises(ValueError, match="preceding"):
        match(data, 5.0, now=5.2)
    assert match(data, 5.15, now=5.2).stamp_s == 5.1


@pytest.mark.parametrize("cause", ["quaternion", "stamp_reverse", "same_stamp_changed", "frame_changed", "twist"])
def test_invalid_or_reversed_pose_invalidates_prior_geometry_instead_of_reusing_it(cause):
    data = inputs()
    data.update_pose("ground", evidence_odom(), now=5)
    payload = evidence_odom(5.1)
    if cause == "quaternion":
        payload["pose"]["pose"]["orientation"]["w"] = 0
    elif cause == "stamp_reverse":
        payload = evidence_odom(4.9)
    elif cause == "same_stamp_changed":
        payload = evidence_odom(5., x=999)
    elif cause == "frame_changed":
        payload["child_frame_id"] = "other_body"
    else:
        payload["twist"]["twist"]["angular"]["z"] = float("nan")
    with pytest.raises(ValueError):
        data.update_pose("ground", payload, now=5.1)
    assert data.odometry_evidence["ground"] is None
    with pytest.raises(ValueError, match="history"):
        match(data, 5., now=5.1)


def test_runtime_clock_reverse_clears_both_histories_and_camera_evidence():
    data = inputs()
    data.update_pose("ground", evidence_odom(), now=5)
    data.update_pose("uav", evidence_odom(frame="uav/odom", child="uav/body"), now=5)
    data.update_detections({"source": "ground", "stamp": 5., "frame_id": "camera", "detections": []}, now=5)
    with pytest.raises(ValueError, match="clock.*reversed"):
        data.update_pose("ground", evidence_odom(4.), now=4.)
    assert data.odometry_evidence == {"ground": None, "uav": None}
    assert data.frames == {"ground": None, "uav": None}
    with pytest.raises(ValueError, match="history"):
        match(data, 4., now=4.)


def camera_frame():
    return {"source": "ground", "stamp": 5., "frame_id": "ground/camera_optical",
            "received_stamp": 5.01, "image_width": 640, "image_height": 480,
            "calibration_version": "K:sha256-known", "size_version": "size:v2",
            "coordinate_convention": "opencv_optical", "clock_id": "sim", "epoch_id": "run:1",
            "camera_info_stamp": 4., "position_error_m": .02,
            "detections": [{"target_id": "person", "center": [.5, .5], "confidence": .9,
                            "position_camera_m": [.1, .2, 1.]}]}


def test_detection_retains_explicit_camera_frame_versions_time_and_position_metadata():
    data = inputs()
    payload = camera_frame()
    data.update_detections(payload, now=5.01)
    stored, detection = data.frames["ground"], data.fresh_detections(5.01)[0]
    for key in ("frame_id", "calibration_version", "size_version", "coordinate_convention", "clock_id", "epoch_id", "camera_info_stamp"):
        assert stored[key] == payload[key] and detection[key] == payload[key]
    assert detection["stamp"] == 5. and detection["position_camera_m"] == [.1, .2, 1.]
    payload["detections"][0]["position_camera_m"][2] = 999
    assert data.fresh_detections(5.01)[0]["position_camera_m"][2] == 1.


def test_detection_does_not_invent_missing_camera_frame_or_calibration_metadata():
    data = inputs()
    data.update_detections({"source": "ground", "stamp": 5., "detections": [
        {"target_id": "person", "center": [.5, .5], "confidence": .9}]}, now=5.)
    stored, detection = data.frames["ground"], data.fresh_detections(5.)[0]
    for key in ("frame_id", "calibration_version", "coordinate_convention", "position_error_m"):
        assert key not in stored and key not in detection


@pytest.mark.parametrize("field,value", [
    ("frame_id", ""), ("calibration_version", True), ("size_version", ""),
    ("position_error_m", -.01), ("camera_info_stamp", 6.), ("image_width", True),
])
def test_explicit_invalid_camera_metadata_clears_frame_instead_of_using_old_values(field, value):
    data = inputs()
    payload = camera_frame()
    data.update_detections(payload, now=5.01)
    payload[field] = value
    with pytest.raises(ValueError):
        data.update_detections(payload, now=5.01)
    assert data.frames["ground"] is None


def test_conflicting_item_frame_or_malformed_metric_position_is_rejected():
    data = inputs()
    for changes in ({"frame_id": "other/camera"}, {"position_camera_m": [0., float("inf"), 1.]},
                    {"calibration_version": "different"}):
        data.update_detections(camera_frame(), now=5.01)
        assert data.frames["ground"] is not None
        payload = camera_frame()
        payload["detections"][0].update(changes)
        with pytest.raises(ValueError):
            data.update_detections(payload, now=5.01)
        assert data.frames["ground"] is None


def test_sidecar_and_exposure_return_raw_payloads_cannot_modify_history_or_each_other():
    data = inputs()
    data.update_pose("ground", evidence_odom(), now=5.)
    sidecar = data.odometry_evidence["ground"]
    sidecar.raw_payload["pose"]["pose"]["position"]["x"] = 999
    selected = match(data, 5., now=5.)
    assert selected.raw_payload["pose"]["pose"]["position"]["x"] == 1.
    selected.raw_payload["header"]["frame_id"] = "caller mutation"
    selected.raw_payload["twist"]["twist"]["linear"]["x"] = 999
    repeated = match(data, 5., now=5.)
    assert repeated.raw_payload["header"]["frame_id"] == "odom"
    assert repeated.raw_payload["twist"]["twist"]["linear"]["x"] == .1
    assert sidecar.raw_payload["twist"]["twist"]["linear"]["x"] == .1


@pytest.mark.parametrize("entry_point", ["camera", "query"])
def test_clock_reverse_from_camera_or_query_clears_both_pose_histories_and_frames(entry_point):
    data = inputs()
    data.update_pose("ground", evidence_odom(), now=5.)
    data.update_pose("uav", evidence_odom(frame="uav/odom", child="uav/body"), now=5.)
    for source in ("ground", "uav"):
        data.update_detections({"source": source, "stamp": 5., "detections": []}, now=5.)
    with pytest.raises(ValueError, match="clock.*reversed"):
        if entry_point == "camera":
            data.update_detections({"source": "ground", "stamp": 4., "detections": []}, now=4.)
        else:
            match(data, 4., now=4.)
    assert data.odometry_evidence == {"ground": None, "uav": None}
    assert data.frames == {"ground": None, "uav": None}
    for source, child in (("ground", "base_footprint"), ("uav", "uav/body")):
        with pytest.raises(ValueError, match="history"):
            data.pose_at_exposure(source, 4., expected_child_frame=child,
                                  max_skew_s=.2, now=4., max_age_s=.5)


def test_repeated_identical_odometry_timestamp_does_not_consume_bounded_history():
    data = inputs(pose_history_max_samples=2)
    payload = evidence_odom()
    data.update_pose("ground", payload, now=5.)
    data.update_pose("ground", copy.deepcopy(payload), now=5.01)
    data.update_pose("ground", evidence_odom(5.1), now=5.1)
    assert match(data, 5., now=5.1).stamp_s == 5.


@pytest.mark.parametrize("cause", ["world_frame", "twist_same_stamp", "tilt"])
def test_world_frame_change_changed_same_stamp_twist_and_tilt_clear_geometry_and_camera(cause):
    data = inputs()
    data.update_pose("ground", evidence_odom(), now=5.)
    data.update_detections(camera_frame(), now=5.01)
    payload = evidence_odom(5.1)
    if cause == "world_frame":
        payload["header"]["frame_id"] = "other/world"
    elif cause == "twist_same_stamp":
        payload = evidence_odom()
        payload["twist"]["twist"]["linear"]["x"] = .2
    else:
        payload["pose"]["pose"]["orientation"] = {
            "x": math.sin(.1), "y": 0., "z": 0., "w": math.cos(.1)}
    with pytest.raises(ValueError):
        data.update_pose("ground", payload, now=5.1)
    assert data.poses["ground"] is None and data.odometry_evidence["ground"] is None
    assert data.frames["ground"] is None
    with pytest.raises(ValueError, match="history"):
        match(data, 5., now=5.1)
