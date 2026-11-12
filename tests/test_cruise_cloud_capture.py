"""The v37 read-only collector must retain complete bounded cloud evidence."""
import base64
import hashlib
import importlib
import importlib.util
import math
from array import array
from types import SimpleNamespace

import pytest


def api():
    name = "inspection_mission.cruise_cloud_capture"
    assert importlib.util.find_spec(name) is not None, "complete cruise cloud collector is missing"
    return importlib.import_module(name)


def cloud(data=None, *, stamp=10., **changes):
    sec = int(stamp)
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=round((stamp-sec)*1e9)),
                               frame_id="uav/lidar_link"),
        width=2, height=2,
        fields=[SimpleNamespace(name="x", offset=0, datatype=7, count=1)],
        is_bigendian=False, point_step=4, row_step=12, is_dense=False,
        data=bytearray(range(20))+bytearray(b"TAIL") if data is None else data,
    )
    for name, value in changes.items():
        setattr(message, name, value)
    return message


def pose(stamp, *, z=.85):
    sec = int(stamp)
    message = {
        "header": {"frame_id": "odom", "stamp": {"sec": sec, "nanosec": round((stamp-sec)*1e9)}},
        "child_frame_id": "uav/base_link",
        "pose": {"pose": {"position": {"x": 0., "y": 0., "z": z},
                           "orientation": {"x": 0., "y": 0., "z": 0., "w": 1.}},
                 "covariance": [0.]*36},
        "twist": {"twist": {"linear": {"x": 0., "y": 0., "z": 0.},
                              "angular": {"x": 0., "y": 0., "z": 0.}},
                  "covariance": [0.]*36},
    }
    return {"stamp": stamp, "wall": 2., "xyz": [0., 0., z], "q": [0., 0., 0., 1.],
            "linear": [0., 0., 0.], "angular": [0., 0., 0.], "message": message}


def tf_record(*, parent="uav/base_link", child="uav/lidar_link", source="/tf_static"):
    return {"wall_s": 1.5, "wall_elapsed_s": .5, "source": source,
            "message": {"transforms": [{
                "header": {"frame_id": parent, "stamp": {"sec": 0, "nanosec": 0}},
                "child_frame_id": child,
                "transform": {"translation": {"x": .03, "y": 0., "z": .08},
                              "rotation": {"x": 0., "y": 0., "z": 0., "w": 1.}},
            }]}}


def capture(collector, message=None, *, phase="WAIT", trajectory=None, tf_records=None, wall_s=2.):
    return collector.capture(cloud() if message is None else message, phase=phase, wall_s=wall_s,
                             trajectory=[pose(9.98)] if trajectory is None else trajectory,
                             tf_records=[tf_record()] if tf_records is None else tf_records)


def test_full_payload_layout_header_hash_and_tail_survive_source_mutation():
    collector = api().CruiseCloudCapture()
    message, trajectory, transforms = cloud(), [pose(9.98)], [tf_record()]
    expected = bytes(message.data)
    assert capture(collector, message, trajectory=trajectory, tf_records=transforms) is True
    message.data[-4:] = b"LOST"
    message.header.frame_id = "mutated"
    message.fields[0].name = "mutated"
    trajectory[0]["message"]["pose"]["pose"]["position"]["z"] = 99.
    transforms[0]["message"]["transforms"][0]["transform"]["translation"]["z"] = 99.

    result = collector.snapshot(encode=True)
    frame = result["clouds"][0]
    assert result["frame_count"] == 1 and result["raw_bytes"] == 24
    assert base64.b64decode(frame["data_b64"]) == expected
    assert base64.b64decode(frame["data_b64"])[-4:] == b"TAIL"
    assert frame["data_sha256"] == hashlib.sha256(expected).hexdigest()
    assert frame["header"] == {"frame_id": "uav/lidar_link", "stamp": {"sec": 10, "nanosec": 0}}
    assert frame["arrival_wall_s"] == 2.
    assert (frame["width"], frame["height"], frame["point_step"], frame["row_step"]) == (2, 2, 4, 12)
    assert frame["fields"] == [{"name": "x", "offset": 0, "datatype": 7, "count": 1}]
    assert frame["is_bigendian"] is False and frame["is_dense"] is False
    assert frame["pose_association"]["record"]["message"]["pose"]["pose"]["position"]["z"] == .85
    saved_tf = frame["tf_snapshot"]["records"][0]
    assert saved_tf["source"] == "/tf_static" and saved_tf["wall_s"] == 1.5
    assert saved_tf["message"]["transforms"][0]["transform"]["translation"]["z"] == .08


def test_capture_and_metadata_snapshot_do_not_hash_encode_or_expand_payload(monkeypatch):
    module = api()
    collector = module.CruiseCloudCapture()

    def forbidden(*args, **kwargs):
        raise AssertionError("expensive encoding happened in the callback path")

    monkeypatch.setattr(module.base64, "b64encode", forbidden)
    monkeypatch.setattr(module.hashlib, "sha256", forbidden)
    assert capture(collector) is True
    result = collector.snapshot()
    assert result["encoded"] is False
    assert result["clouds"][0]["data_bytes"] == 24
    assert "data_b64" not in result["clouds"][0] and "data_sha256" not in result["clouds"][0]
    result["clouds"][0]["header"]["frame_id"] = "changed_snapshot"
    assert collector.snapshot()["clouds"][0]["header"]["frame_id"] == "uav/lidar_link"


def test_all_four_slots_accept_once_and_capacity_never_expands():
    collector = api().CruiseCloudCapture()
    for phase in ("WAIT", "ASCEND_CRUISE", "HOLD_CRUISE", "DESCEND_SUPPORT"):
        assert capture(collector, phase=phase) is True
        assert capture(collector, phase=phase) is False
    assert capture(collector, phase="ZERO_RETURN") is False
    result = collector.snapshot()
    assert result["frame_count"] == 4 and result["raw_bytes"] == 96
    assert result["missing_slots"] == []
    assert [frame["phase"] for frame in result["clouds"]] == [
        "WAIT", "ASCEND_CRUISE", "HOLD_CRUISE", "DESCEND_SUPPORT"]
    assert result["counters"]["slot_already_captured"] == 4
    assert result["counters"]["unsupported_phase"] == 1


def test_maximum_sized_frames_are_whole_and_reach_eight_mib_without_truncation():
    collector = api().CruiseCloudCapture()
    data = bytes(2*1024*1024-4)+b"TAIL"
    message = cloud(data, width=1, height=1, point_step=4, row_step=len(data))
    for phase in ("WAIT", "ASCEND_CRUISE", "HOLD_CRUISE", "DESCEND_SUPPORT"):
        assert capture(collector, message, phase=phase) is True
    result = collector.snapshot(encode=True)
    assert result["raw_bytes"] == 8*1024*1024
    assert all(base64.b64decode(frame["data_b64"])[-4:] == b"TAIL" for frame in result["clouds"])


def test_oversized_frame_is_rejected_whole_and_does_not_consume_its_slot():
    collector = api().CruiseCloudCapture()
    data = bytes(2*1024*1024+1)
    assert capture(collector, cloud(data, width=1, height=1, point_step=4, row_step=len(data))) is False
    assert collector.snapshot()["frame_count"] == 0
    assert collector.snapshot()["counters"]["frame_oversize"] == 1
    assert capture(collector) is True


@pytest.mark.parametrize("change", [
    {"data": bytes(23)}, {"data": bytes(25)}, {"data": [0]*24},
    {"width": 0}, {"height": -1}, {"point_step": math.nan}, {"row_step": 7},
    {"fields": []}, {"fields": [SimpleNamespace(name="x", offset=2, datatype=7, count=1)]},
    {"fields": [SimpleNamespace(name="x", offset=0, datatype=99, count=1)]},
    {"fields": [SimpleNamespace(name="x", offset=0, datatype=7, count=0)]},
    {"is_bigendian": 1}, {"is_dense": "false"},
])
def test_invalid_layout_never_retains_partial_payload(change):
    collector = api().CruiseCloudCapture()
    assert capture(collector, cloud(**change)) is False
    result = collector.snapshot(encode=True)
    assert result["frame_count"] == 0 and result["raw_bytes"] == 0
    assert result["counters"]["invalid_layout"] == 1


@pytest.mark.parametrize("sec,nanosec", [(math.nan, 0), (-1, 0), (1., 0), (1, -1), (1, 1000000000), (True, 0)])
def test_invalid_source_stamp_is_rejected(sec, nanosec):
    collector = api().CruiseCloudCapture()
    message = cloud()
    message.header.stamp.sec, message.header.stamp.nanosec = sec, nanosec
    assert capture(collector, message) is False
    assert collector.snapshot()["counters"]["invalid_stamp"] == 1


def test_real_latest_prior_pose_is_chosen_instead_of_nearer_future_pose():
    collector = api().CruiseCloudCapture()
    trajectory = [pose(9.98, z=.83), pose(10.001, z=.99), pose(9.97, z=.82)]
    assert capture(collector, trajectory=trajectory) is True
    association = collector.snapshot()["clouds"][0]["pose_association"]
    assert association["status"] == "valid_prior_pose"
    assert association["odom_stamp_s"] == 9.98 and association["skew_s"] == pytest.approx(.02)
    assert association["record"]["message"]["pose"]["pose"]["position"]["z"] == .83


@pytest.mark.parametrize("trajectory,reason", [([], "no_prior_pose"), ([pose(10.001)], "no_prior_pose"),
                                               ([pose(9.949)], "prior_pose_stale")])
def test_missing_or_stale_pose_is_honest_metadata_and_does_not_block_cloud_capture(trajectory, reason):
    collector = api().CruiseCloudCapture()
    assert capture(collector, trajectory=trajectory) is True
    result = collector.snapshot()
    assert result["clouds"][0]["pose_association"]["reason"] == reason
    assert result["counters"][reason] == 1


def test_pose_skew_boundary_and_raw_stamp_authority_are_preserved():
    collector = api().CruiseCloudCapture()
    raw = pose(10.01)
    raw["stamp"] = 9.99
    assert capture(collector, trajectory=[pose(9.95), raw]) is True
    association = collector.snapshot()["clouds"][0]["pose_association"]
    assert association["odom_stamp_s"] == 9.95
    assert association["skew_s"] == pytest.approx(.05)


def test_missing_lidar_chain_keeps_real_tf_without_inserting_candidate_transform():
    collector = api().CruiseCloudCapture()
    records = [tf_record(child="uav/laser_link")]
    assert capture(collector, tf_records=records) is True
    result = collector.snapshot()
    snapshot = result["clouds"][0]["tf_snapshot"]
    assert snapshot["reason"] == "missing_transform_chain"
    assert snapshot["records"] == records
    assert result["counters"]["missing_transform_chain"] == 1


def test_actual_multihop_lidar_chain_is_recognized_and_source_absence_is_not_invented():
    collector = api().CruiseCloudCapture()
    first = tf_record(child="uav/mount")
    second = tf_record(parent="uav/mount")
    del second["source"]
    assert capture(collector, tf_records=[first, second]) is True
    tf = collector.snapshot()["clouds"][0]["tf_snapshot"]
    assert tf["status"] == "transform_chain_present"
    assert "source" not in tf["records"][1]
    assert tf["source_unavailable"] == 1


def test_array_buffer_and_mapping_messages_keep_the_complete_data():
    collector = api().CruiseCloudCapture()
    original = cloud(array("B", range(24)))
    message = vars(original)
    assert capture(collector, message) is True
    assert base64.b64decode(collector.snapshot(encode=True)["clouds"][0]["data_b64"]) == bytes(range(24))


def test_callback_exceptions_and_uncovered_stages_are_bounded_metadata():
    class Broken:
        @property
        def header(self):
            raise RuntimeError("external callback object failed")

    collector = api().CruiseCloudCapture()
    for _ in range(100):
        assert capture(collector, Broken()) is False
    result = collector.snapshot()
    assert result["frame_count"] == 0 and len(result["errors"]) <= 32
    assert result["errors_dropped"] >= 68
    assert result["counters"]["capture_exception"] == 100
    assert result["missing_slots"] == ["WAIT", "ASCEND_CRUISE", "HOLD_CRUISE", "DESCEND_SUPPORT"]


def test_metadata_overflow_and_bad_arrival_are_reported_without_poisoning_capture():
    collector = api().CruiseCloudCapture()
    assert capture(collector, wall_s=math.nan) is False
    huge = [tf_record(child="frame/"+str(i)) for i in range(200)]
    assert capture(collector, tf_records=huge) is True
    result = collector.snapshot()
    tf = result["clouds"][0]["tf_snapshot"]
    assert len(tf["records"]) <= 128 and tf["records_omitted"] >= 72
    assert result["counters"]["invalid_arrival"] == 1
    assert result["counters"]["tf_snapshot_truncated"] == 1


def test_every_retained_frame_names_the_actual_subscribed_cloud_topic():
    collector = api().CruiseCloudCapture()
    assert capture(collector) is True
    assert collector.snapshot()["clouds"][0]["source"] == "/uav/lidar/points"


def test_static_extrinsic_is_retained_when_dynamic_records_fill_the_metadata_budget():
    collector = api().CruiseCloudCapture()
    static = tf_record()
    dynamic = [tf_record(child="uav/unrelated", source="/tf") for _ in range(128)]
    assert capture(collector, tf_records=[static, *dynamic]) is True
    tf = collector.snapshot()["clouds"][0]["tf_snapshot"]
    assert tf["status"] == "transform_chain_present"
    assert static in tf["records"] and len(tf["records"]) <= 128
    assert tf["records_omitted"] == 1


def test_rejected_tf_record_cannot_leave_a_phantom_lidar_chain():
    collector = api().CruiseCloudCapture()
    record = tf_record()
    invalid = tf_record(child="uav/invalid")["message"]["transforms"][0]
    invalid["transform"]["translation"]["z"] = math.nan
    record["message"]["transforms"].append(invalid)
    assert capture(collector, tf_records=[record]) is True
    tf = collector.snapshot()["clouds"][0]["tf_snapshot"]
    assert tf["records"] == [] and tf["reason"] == "missing_transform_chain"
    assert tf["invalid_records"] == 1


def test_invalid_tf_rotation_never_proves_a_transform_chain():
    collector = api().CruiseCloudCapture()
    record = tf_record()
    record["message"]["transforms"][0]["transform"]["rotation"]["w"] = 0.
    assert capture(collector, tf_records=[record]) is True
    tf = collector.snapshot()["clouds"][0]["tf_snapshot"]
    assert tf["reason"] == "missing_transform_chain" and tf["invalid_records"] == 1


def test_extreme_invalid_arrival_cannot_escape_into_the_control_callback():
    collector = api().CruiseCloudCapture()
    assert capture(collector, wall_s=10**1000) is False
    assert collector.snapshot()["counters"]["invalid_arrival"] == 1


@pytest.mark.parametrize("broken", ["missing_pose", "invalid_rotation", "nan_position"])
def test_malformed_newer_odometry_cannot_replace_a_valid_real_prior_pose(broken):
    collector = api().CruiseCloudCapture()
    invalid = pose(9.99, z=.99)
    if broken == "missing_pose":
        del invalid["message"]["pose"]
    elif broken == "invalid_rotation":
        invalid["message"]["pose"]["pose"]["orientation"]["w"] = 0.
    else:
        invalid["message"]["pose"]["pose"]["position"]["z"] = math.nan
    assert capture(collector, trajectory=[pose(9.98, z=.83), invalid]) is True
    association = collector.snapshot()["clouds"][0]["pose_association"]
    assert association["status"] == "valid_prior_pose" and association["odom_stamp_s"] == 9.98
    assert association["record"]["message"]["pose"]["pose"]["position"]["z"] == .83
