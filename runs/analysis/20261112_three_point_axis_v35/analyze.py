"""Read-only v35 axis evidence audit. Never infers referee acceptance or score."""
from __future__ import annotations

import argparse
import base64
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import zlib

MODE = "ground_three_point_axis"
REVISION = "ground_three_point_axis_v1"
OUTER_REVISION = "ground_three_point_v1"  # inherited additional_events()
SECTION = "three_ground_evidence_gzip_part"
FROZEN_SOURCE_SHA = "e033c119790ad8a32a066cd21cd9fd841e5b967092b8d0ac0355fd5d5f602f76"
MAX_RAW, MAX_PACKED = 32*1024*1024, 8*1024*1024
META = ("mode", "revision", "section", "encoding", "session_id", "bundle",
        "uncompressed_sha256", "compressed_sha256", "uncompressed_bytes",
        "compressed_bytes", "part_count", "read_only", "official_scoring", "partial")
COMPACT = ("point_index", "target_id", "stationary_epoch", "recognition", "attempts",
           "recognition_status", "pose", "path_m", "rotation_rad", "official_acceptance")
POINTS = {
    "ground_0": ((1.125, -1.350), (.55, .40)),
    "ground_1": ((0., -1.350), (.55, .40)),
    "ground_2": ((0., -2.700), (.55, .40)),
    "ground_3": ((1.125, -1.925), (.40, .55)),
    "ground_4": ((1.125, -.775), (.40, .55)),
}
LABELS = {"collapsed_ruins": "坍塌废墟", "landslide": "滑坡塌方", "trapped_people": "被困人员",
          "damaged_vehicle": "受损车辆", "rescue_camp": "救援营地"}
TEMPLATE_SHAS = {
    "200732c46709ed2d6fba5017ce7f56c8cc0908d60eed14b6063b84a9c37cba7f": "collapsed_ruins",
    "76feb761210089a1c3885ce09024ea3da6e991c52fc4bd7cc3f13d7e28b1c13b": "damaged_vehicle",
    "2653c12e906773f388fde39fbee3633ede5531460bb1442096452c41fe10a30d": "landslide",
    "f79dc767cacfd16dc8d3f989ec3ae1743f46c85bafca95223120e19c516cc689": "landslide",
    "781d7c9e505a5a46c9177f584d5aff220560d28a290d1046a6378bcbb38bd014": "rescue_camp",
    "b81c0269dc79776a10edcb54043b3e06a9188d5bbbf4b28ad951e4e998cb864a": "rescue_camp",
    "e31429ccbf79d3898bc24fe5fb5bd8faefbfc6c29d9fc0ed573f78db1fde90b7": "trapped_people",
    "b8aefe526d7d3a3a9045107aae75f0f9cbdcc190c5a61c9d27a9c67646308777": "trapped_people",
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def unique(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, "duplicate JSON key")
        result[key] = value
    return result


def parse_json(raw):
    def invalid(_):
        raise ValueError("nonfinite JSON value")
    def finite_float(value):
        value = float(value)
        require(math.isfinite(value), "nonfinite JSON float")
        return value
    return json.loads(raw, object_pairs_hook=unique, parse_constant=invalid, parse_float=finite_float)


def sha(value):
    return hashlib.sha256(value).hexdigest()


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def vector(value, count):
    return isinstance(value, (list, tuple)) and len(value) == count and all(number(v) for v in value)


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw(quaternion):
    x, y, z, w = quaternion
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def is_sha(value):
    return isinstance(value, str) and re.fullmatch("[0-9a-f]{64}", value) is not None


def decode_bundle(parts):
    require(bool(parts), "missing compressed bundle")
    first = parts[0]
    require(isinstance(first, dict) and all(key in first for key in META), "incomplete bundle metadata")
    require(first["mode"] == MODE and first["revision"] == OUTER_REVISION
            and first["section"] == SECTION and first["encoding"] == "gzip+base64"
            and first["bundle"] == OUTER_REVISION and first["read_only"] is False
            and first["official_scoring"] == "unverified" and first["partial"] is True
            and isinstance(first["session_id"], str) and 1 <= len(first["session_id"]) <= 128,
            "invalid axis bundle identity metadata")
    for key, cap in (("uncompressed_bytes", MAX_RAW), ("compressed_bytes", MAX_PACKED), ("part_count", 512)):
        require(type(first[key]) is int and 1 <= first[key] <= cap, "bundle size or count exceeds bound")
    require(all(is_sha(first[key]) for key in ("uncompressed_sha256", "compressed_sha256")), "invalid SHA256 metadata")
    chunks, duplicates = {}, 0
    for part in parts:
        require(isinstance(part, dict) and all(key in part and part[key] == first[key] for key in META),
                "conflicting bundle metadata")
        index, value = part.get("part_index"), part.get("base64_part")
        require(type(index) is int and 0 <= index < first["part_count"] and isinstance(value, str)
                and 0 < len(value) <= 48000 and re.fullmatch("[A-Za-z0-9+/=]+", value), "invalid bundle part")
        if index in chunks:
            require(chunks[index] == value, "conflicting bundle part")
            duplicates += 1
        chunks[index] = value
    require(len(chunks) == first["part_count"], "missing bundle parts")
    encoded = "".join(chunks[i] for i in range(first["part_count"]))
    require(len(encoded) <= 4*((MAX_PACKED+2)//3), "compressed bundle exceeds bound")
    packed = base64.b64decode(encoded, validate=True)
    require(len(packed) == first["compressed_bytes"], "compressed size mismatch")
    require(sha(packed) == first["compressed_sha256"], "compressed SHA256 mismatch")
    decoder = zlib.decompressobj(16+zlib.MAX_WBITS)
    raw = decoder.decompress(packed, first["uncompressed_bytes"]+1)
    require(decoder.eof and not decoder.unused_data and not decoder.unconsumed_tail
            and len(raw) == first["uncompressed_bytes"], "gzip stream or raw length mismatch")
    require(sha(raw) == first["uncompressed_sha256"], "uncompressed SHA256 mismatch")
    body = parse_json(raw.decode("utf-8"))
    require(isinstance(body, dict) and body.get("schema") == "ground_three_point_evidence_v1"
            and body.get("session_id") == first["session_id"], "body schema/session mismatch")
    require(isinstance(body.get("survey"), dict) and isinstance(body.get("completed_points"), list)
            and isinstance(body.get("capture_image_events"), list) and isinstance(body.get("current_point"), dict),
            "invalid evidence containers")
    approach = body["survey"].get("approach", {})
    require(approach.get("mode") == MODE and approach.get("revision") == REVISION
            and approach.get("three_point", {}).get("planner") == "ground_three_point_axis_plan",
            "missing real nested axis revision/planner")
    require(approach["three_point"].get("session_id") == first["session_id"]
            and body["current_point"].get("session_id") == first["session_id"], "nested session mismatch")
    require(approach["three_point"].get("completed_points") == body["completed_points"], "completed-point copies conflict")
    return body, {**{key: first[key] for key in META}, "duplicate_parts": duplicates}, raw, packed


def load_source(path):
    raw = Path(path).read_bytes()
    require(len(raw) <= 64*1024*1024, "JSONL exceeds bound")
    records = []
    for line_number, line in enumerate(raw.decode("utf-8").splitlines(), 1):
        if not line.strip():
            continue
        row = parse_json(line)
        require(isinstance(row, dict), f"line {line_number}: non-object row")
        event = row.get("payload", row)
        require(isinstance(event, dict), f"line {line_number}: non-object payload")
        if event.get("event") != "probe_snapshot":
            continue
        data = event.get("data", {})
        require(isinstance(data, dict), f"line {line_number}: non-object snapshot")
        if data.get("section") in {SECTION, "three_point_core", "three_point_point"}:
            require(data.get("mode") == MODE, "mixed/legacy three-point mode")
        if data.get("mode") == MODE:
            records.append({"line": line_number, "data": data})
    return raw, records


def bind_core(body, records):
    app = body["survey"]["approach"]
    three = app["three_point"]
    expected = {key: app[key] for key in ("state", "reason", "global_origin", "total_path_m", "accumulated_rotation_rad")}
    expected.update({key: three[key] for key in ("mission_draw", "point_index", "target_id", "frozen_limits")})
    expected.update(completed_count=len(body["completed_points"]), full_mission_complete=False)
    cores = [row for row in records if row["data"].get("section") == "three_point_core"]
    require(bool(cores), "missing independent three_point_core")
    for row in cores:
        data = row["data"]
        require(data.get("session_id") == body["session_id"] and data.get("revision") == OUTER_REVISION,
                "core session/revision mismatch")
        require(all(key in data and data[key] == value for key, value in expected.items()), "core/body conflict")
    compact = [row for row in records if row["data"].get("section") == "three_point_point"]
    seen = set()
    for row in compact:
        data = row["data"]
        require(data.get("session_id") == body["session_id"] and data.get("revision") == OUTER_REVISION,
                "point session/revision mismatch")
        candidates = [point for point in body["completed_points"] if point.get("point_index") == data.get("point_index")]
        require(len(candidates) == 1 and all(key in data and data[key] == candidates[0].get(key) for key in COMPACT),
                "compact point/body conflict")
        seen.add(data["point_index"])
    require(seen == {point["point_index"] for point in body["completed_points"]}, "missing independent completed point")
    return {"core_rows": len(cores), "compact_point_rows": len(compact), "core_matches_body": True}


def jpeg_dimensions(raw):
    require(raw[:2] == b"\xff\xd8" and raw[-2:] == b"\xff\xd9", "invalid JPEG start/end")
    index = 2
    while index < len(raw):
        require(raw[index] == 255, "invalid JPEG segment")
        while index < len(raw) and raw[index] == 255:
            index += 1
        require(index < len(raw), "truncated JPEG marker")
        marker = raw[index]
        index += 1
        if marker in (0xd8, 0xd9, 0x01) or 0xd0 <= marker <= 0xd7:
            continue
        require(index+2 <= len(raw), "truncated JPEG length")
        length = int.from_bytes(raw[index:index+2], "big")
        require(length >= 2 and index+length <= len(raw), "invalid JPEG segment length")
        if marker in (0xc0, 0xc1, 0xc2):
            require(length >= 8, "truncated JPEG dimensions")
            return int.from_bytes(raw[index+5:index+7], "big"), int.from_bytes(raw[index+3:index+5], "big")
        require(marker != 0xda, "JPEG dimensions missing before scan")
        index += length
    raise ValueError("JPEG dimensions not found")


def decode_images(events):
    grouped = {}
    metadata = ("mode", "section", "read_only", "official_scoring", "partial", "topic", "phase",
                "stamp_s", "frame_id", "width", "height", "encoding", "jpeg_sha256", "part_count")
    for event in events:
        require(isinstance(event, (list, tuple)) and len(event) == 2 and event[0] == "probe_snapshot"
                and isinstance(event[1], dict), "invalid embedded image event")
        part = event[1]
        require(all(key in part for key in metadata), "incomplete embedded image metadata")
        require(part["mode"] == MODE and part["section"] == "survey_image_part" and part["read_only"] is False
                and part["official_scoring"] == "unverified" and part["partial"] is True
                and part["topic"] == "/uav/camera/image" and part["frame_id"] == "uav/camera_link"
                and part["encoding"] == "jpeg" and number(part["stamp_s"])
                and is_sha(part["jpeg_sha256"]) and isinstance(part["phase"], str)
                and re.fullmatch(r"point_[0-2]_capture_[0-2]", part["phase"]), "invalid embedded axis image identity")
        for key in ("width", "height"):
            require(type(part[key]) is int and 1 <= part[key] <= 800, "invalid image dimensions")
        require(type(part["part_count"]) is int and 1 <= part["part_count"] <= 17, "invalid image part count")
        index, chunk = part.get("part_index"), part.get("jpeg_base64_part")
        require(type(index) is int and 0 <= index < part["part_count"] and isinstance(chunk, str)
                and 0 < len(chunk) <= 32000 and re.fullmatch("[A-Za-z0-9+/=]+", chunk), "invalid image part")
        identity = (part["phase"], part["stamp_s"])
        group = grouped.setdefault(identity, {"metadata": {key: part[key] for key in metadata}, "parts": {}, "duplicates": 0})
        require(group["metadata"] == {key: part[key] for key in metadata}, "conflicting image metadata")
        if index in group["parts"]:
            require(group["parts"][index] == chunk, "conflicting duplicate image part")
            group["duplicates"] += 1
        group["parts"][index] = chunk
    images = []
    for group in grouped.values():
        meta, chunks = group["metadata"], group["parts"]
        require(len(chunks) == meta["part_count"], "missing image part")
        raw = base64.b64decode("".join(chunks[i] for i in range(meta["part_count"])), validate=True)
        require(len(raw) <= 384*1024 and sha(raw) == meta["jpeg_sha256"], "image size/SHA mismatch")
        require(jpeg_dimensions(raw) == (meta["width"], meta["height"]), "JPEG dimensions differ from metadata")
        images.append({**meta, "jpeg_bytes": len(raw), "duplicate_parts": group["duplicates"], "raw": raw})
    return sorted(images, key=lambda item: item["stamp_s"])


def audit_prior(record):
    pose = record.get("odom")
    valid = (record.get("topic") == "/uav/camera/image" and record.get("frame_id") == "uav/camera_link"
             and record.get("association_error") is None and isinstance(pose, dict)
             and pose.get("source_topic") == "/uav/odom" and pose.get("frame_id") == "odom"
             and pose.get("child_frame_id") == "uav/base_link" and pose.get("association") == "prior_uav_odom"
             and number(pose.get("stamp_s")) and number(record.get("stamp_s")))
    delta = record["stamp_s"]-pose["stamp_s"] if valid else None
    valid = (valid and -.000000001 <= delta <= .05+.000000001
             and number(pose.get("image_minus_odom_s")) and abs(delta-pose["image_minus_odom_s"]) <= 1e-9
             and vector(pose.get("position"), 3) and vector(pose.get("quaternion"), 4)
             and abs(sum(v*v for v in pose["quaternion"])-1) <= .001)
    return {"prior_valid": bool(valid), "image_minus_prior_s": delta}


def audit_match(item, target, epoch, session, images, exposures):
    errors = []
    def check(condition, message):
        if not condition:
            errors.append(message)
    job = item.get("job", {})
    check(job.get("session_id") == session and job.get("frozen_first_ID") == target
          and job.get("stationary_epoch") == epoch, "job context mismatch")
    for key in ("job_seq", "image_stamp_s", "source_sha256", "pose"):
        check(item.get(key) == job.get(key), "result/job " + key + " mismatch")
    check(type(job.get("job_seq")) is int and is_sha(job.get("source_sha256"))
          and is_sha(job.get("source_ros_sha256")), "job sequence/source SHA invalid")
    matches = [image for image in images if image["stamp_s"] == job.get("image_stamp_s")]
    check(len(matches) == 1, "source JPEG missing or ambiguous")
    jpeg_bound = False
    if len(matches) == 1:
        image = matches[0]
        check(image["frame_id"] == job.get("frame_id") and image["topic"] == job.get("topic"), "image/job frame/topic mismatch")
        if job.get("source_ros_encoding") in ("rgb8", "bgr8") and (image["width"], image["height"]) == (800, 800):
            jpeg_bound = image["jpeg_sha256"] == job.get("source_sha256")
            check(jpeg_bound, "worker/source JPEG SHA mismatch")
    records = [record for record in exposures if record.get("topic") == "/uav/camera/image"
               and record.get("stamp_s") == job.get("image_stamp_s")]
    # Capture exposure rows are preserved; identical aliases are harmless.
    check(bool(records), "source exposure record missing")
    prior_valid = all(audit_prior(record)["prior_valid"] and record.get("odom") == job.get("pose") for record in records)
    check(prior_valid, "exposure/job prior mismatch")
    info, K = job.get("camera_info", {}), job.get("K", [])
    kflat = info.get("k", [])
    calibration = (info.get("width") == 800 and info.get("height") == 800 and len(kflat) == 9
                   and K == [kflat[i:i+3] for i in (0, 3, 6)]
                   and info.get("header", {}).get("frame_id") == job.get("frame_id")
                   and all(number(value) and value == 0 for value in info.get("d", [])))
    try:
        stamp = info["header"]["stamp"]
        calibration = calibration and stamp["sec"]+stamp["nanosec"]/1e9 <= job["image_stamp_s"]+1e-9
    except (KeyError, TypeError):
        calibration = False
    check(calibration, "prior CameraInfo/K mismatch")
    check(item.get("class_id") in LABELS and item.get("label") == LABELS.get(item.get("class_id"))
          and TEMPLATE_SHAS.get(item.get("template_sha256")) == item.get("class_id"), "class/label/template mismatch")
    prov = item.get("provenance", {})
    check(prov.get("repository") == "wbxxmz/ros2-car-first"
          and prov.get("commit") == "48b3dac26b6f562ff21dd321db03fd6ffc91b492", "template provenance mismatch")
    center, size = POINTS[target]
    world_center, quad, pixel_quad = item.get("world_center_xy_m"), item.get("world_quad_xy_m"), item.get("source_quad_xy")
    geometry = (vector(world_center, 2) and isinstance(quad, list) and len(quad) == 4
                and all(vector(point, 2) for point in quad) and isinstance(pixel_quad, list) and len(pixel_quad) == 4
                and all(vector(point, 2) and all(0 <= value < 800 for value in point) for point in pixel_quad))
    center_error = math.dist(world_center, center) if geometry else None
    check(geometry and center_error <= .08+1e-9
          and [name for name, (point, _) in POINTS.items() if math.dist(world_center, point) <= .08+1e-9] == [target]
          and all(abs(point[axis]-center[axis]) <= size[axis]/2+.08+1e-9 for point in quad for axis in (0, 1)),
          "candidate board binding failed")
    projection_error = None
    if geometry and calibration and vector(job.get("pose", {}).get("quaternion"), 4):
        pose = job["pose"]
        heading = yaw(pose["quaternion"])
        cosine, sine = math.cos(heading), math.sin(heading)
        fx, fy, cx, cy = kflat[0], kflat[4], kflat[2], kflat[5]
        projected = []
        for u, v in pixel_quad:
            if v <= cy:
                errors.append("source quad above candidate ground horizon")
                break
            depth = fy*.173/(v-cy)
            forward, left = .390+depth, (cx-u)*depth/fx
            projected.append([pose["position"][0]+cosine*forward-sine*left,
                              pose["position"][1]+sine*forward+cosine*left])
        if len(projected) == 4:
            projection_error = max(math.dist(a, b) for a, b in zip(projected, quad))
            check(projection_error <= 2e-5, "independent pixel-to-world quad mismatch")
    return {"job_seq": job.get("job_seq"), "image_stamp_s": job.get("image_stamp_s"),
            "source_sha256": job.get("source_sha256"), "jpeg_byte_binding_verified": jpeg_bound,
            "source_ros_sha_recomputed": False, "prior_binding_verified": prior_valid,
            "center_error_m": center_error, "pixel_world_quad_max_error_m": projection_error,
            "class_id": item.get("class_id"), "valid": not errors, "errors": errors, "worker": item.get("worker")}


def stable_window(trajectory, commands, end_stamp):
    rows = [row for row in trajectory if row["stamp_s"] <= end_stamp+1e-9]
    if not rows:
        return {"observed": False}
    last, kept = rows[-1], []
    for row in reversed(rows):
        if last["stamp_s"]-row["stamp_s"] > 3:
            break
        if (row["linear_speed"] > .01 or row["angular_speed"] > .03
                or math.dist(row["position"], last["position"]) > .005+1e-9
                or abs(sum(a*b for a, b in zip(row["quaternion"], last["quaternion"]))) < math.cos(.02/2)):
            break
        kept.append(row)
    if not kept:
        return {"observed": False}
    first = kept[-1]
    max_anchor_drift = max(math.dist(row["position"], first["position"]) for row in kept)
    anchor_rotation_valid = all(abs(sum(a*b for a, b in zip(row["quaternion"], first["quaternion"])))
                                >= math.cos(.02/2) for row in kept)
    zero = all(not row["linear_x"] and not row["angular_z"] for row in commands
               if first["wall_elapsed_s"] <= row["wall_elapsed_s"] <= last["wall_elapsed_s"])
    return {"observed": True, "sample_count": len(kept), "first_stamp_s": first["stamp_s"],
            "last_stamp_s": last["stamp_s"], "wall_span_s": last["wall_elapsed_s"]-first["wall_elapsed_s"],
            "source_span_s": last["stamp_s"]-first["stamp_s"], "returned_commands_all_zero": zero,
            "max_anchor_drift_m": max_anchor_drift,
            "max_linear_speed_mps": max(row["linear_speed"] for row in kept),
            "max_angular_speed_radps": max(row["angular_speed"] for row in kept),
            "sampled_one_second_stable": min(last["wall_elapsed_s"]-first["wall_elapsed_s"],
                                              last["stamp_s"]-first["stamp_s"]) >= 1.-1e-9 and zero
                                                and max_anchor_drift <= .005+1e-9 and anchor_rotation_valid,
            "interpretation": "10Hz retained samples only; does not reconstruct every high-frequency dwell check"}


def audit_points(body, images):
    survey, app = body["survey"], body["survey"]["approach"]
    current = body["current_point"]
    current_visual = current.get("visual_epoch", current)
    rows = [dict(point, completed=True) for point in body["completed_points"]]
    if not any(point["point_index"] == app["three_point"]["point_index"] for point in rows):
        rows.append({**current_visual, "point_index": app["three_point"]["point_index"],
                     "target_id": app["three_point"]["target_id"], "attempts": current.get("attempts", {}),
                     "recognition_status": current.get("recognition_status", []), "completed": False})
    audits = []
    for point in rows:
        index, target, epoch = point["point_index"], point["target_id"], point.get("stationary_epoch")
        matches = point.get("match_results", [])
        source_images = [image for image in images if image["phase"].startswith(f"point_{index}_")]
        match_audits = [audit_match(match, target, epoch, body["session_id"], source_images, survey["exposure_records"])
                        for match in matches]
        independent = len({match.get("job_seq") for match in matches}) == len(matches)
        independent = independent and len({match.get("image_stamp_s") for match in matches}) == len(matches)
        agreement = (len({match.get("class_id") for match in matches}) <= 1
                     and all(math.dist(a["world_center_xy_m"], b["world_center_xy_m"]) <= .03+1e-9
                             for a in matches for b in matches))
        recognition = point.get("recognition")
        recognition_bound = bool(recognition and len(matches) == 3 and independent and agreement
                                 and all(audit["valid"] for audit in match_audits)
                                 and recognition.get("class_id") == matches[0].get("class_id")
                                 and recognition.get("label") == LABELS.get(recognition.get("class_id"))
                                 and recognition.get("first_id") == target and recognition.get("stationary_epoch") == epoch
                                 and recognition.get("data") == f"发现{recognition.get('label')}，地面巡航任务点确认，将巡航信息发送回指挥中心。")
        attempts = point.get("attempts", {})
        attempt_audits = {}
        for kind, topic in (("recognition", "/inspection/detections"), ("declaration", "/referee/current_target")):
            attempt = attempts.get(kind)
            if attempt is None:
                attempt_audits[kind] = {"attempted": False, "publisher_returned": False}
                continue
            valid = (attempt.get("attempted") is True and attempt.get("topic") == topic
                     and attempt.get("first_id") == target and attempt.get("official_acceptance") is False)
            valid = valid and attempt.get("data") == (recognition.get("data") if kind == "recognition" and recognition else target)
            entry = None
            if kind == "declaration" and vector(attempt.get("position"), 3):
                center, size = POINTS[target]
                entry = all(abs(attempt["position"][axis]-center[axis]) <= size[axis]/2-.02+1e-9 for axis in (0, 1))
                valid = valid and entry and bool(attempts.get("recognition", {}).get("sent"))
            attempt_audits[kind] = {"attempted": True, "publisher_returned": attempt.get("sent") is True,
                                    "metadata_valid": valid, "center_inside_candidate_entry": entry,
                                    "raw": attempt}
        source_audits = []
        for image in source_images:
            records = [row for row in survey["exposure_records"] if row.get("topic") == image["topic"]
                       and row.get("stamp_s") == image["stamp_s"]]
            source_audits.append({"phase": image["phase"], "stamp_s": image["stamp_s"], "jpeg_sha256": image["jpeg_sha256"],
                                  "bytes": image["jpeg_bytes"], "exposure_record_count": len(records),
                                  "priors": [audit_prior(record) for record in records],
                                  "sampled_stationarity": stable_window(survey["trajectory_10hz"], survey["actual_commands"], image["stamp_s"])})
        audits.append({"point_index": index, "target_id": target, "completed_archive": point["completed"],
                       "retained_images": len(source_images), "accepted_matches": len(matches),
                       "independent_match_jobs": independent, "same_class_and_center_agreement": agreement,
                       "three_frame_recognition_bound": recognition_bound, "recognition": recognition,
                       "source_images": source_audits, "matches": match_audits, "attempts": attempt_audits,
                       "recognition_status_raw": point.get("recognition_status", [])})
    return audits


def audit_motion(body, records):
    survey, app = body["survey"], body["survey"]["approach"]
    trajectory, commands = survey["trajectory_10hz"], survey["actual_commands"]
    for row in trajectory:
        require(vector(row.get("position"), 3) and vector(row.get("quaternion"), 4)
                and all(number(row.get(key)) for key in ("stamp_s", "wall_elapsed_s", "yaw_rad", "linear_speed", "angular_speed")),
                "invalid trajectory row")
        require(abs(wrap(yaw(row["quaternion"])-row["yaw_rad"])) <= 1e-9
                and row.get("frame_id") == "odom" and row.get("source_topic") == "/odom"
                and row.get("child_frame_id") == "base_footprint", "trajectory pose/source mismatch")
    require(all(a["stamp_s"] < b["stamp_s"] and a["wall_elapsed_s"] < b["wall_elapsed_s"]
                for a, b in zip(trajectory, trajectory[1:])), "nonmonotonic trajectory")
    for row in commands:
        require(all(number(row.get(key)) for key in ("wall_elapsed_s", "linear_x", "angular_z")), "invalid command row")
    bad_commands = [index for index, row in enumerate(commands) if abs(row["linear_x"]) > .10+1e-9
                    or abs(row["angular_z"]) > .5+1e-9 or (row["linear_x"] and row["angular_z"])
                    or (row["linear_x"] and row.get("state") != "DRIVE")
                    or (row["angular_z"] and row.get("state") != "TURN")]
    logs = []
    log_parts = [row["data"] for row in records if row["data"].get("section") == "motion_log"]
    for part in sorted(log_parts, key=lambda row: row["offset"]):
        logs += part["value"]
    first_active = next((row for row in logs if row["state"] in {"TURN", "DRIVE"}), None)
    origin_stamp = None if first_active is None else first_active.get("timing", {}).get("odom", {}).get("stamp_s")
    retained = trajectory if origin_stamp is None else [row for row in trajectory if row["stamp_s"] >= origin_stamp]
    path = sum(math.dist(a["position"], b["position"]) for a, b in zip(retained, retained[1:]))
    rotation = sum(abs(wrap(a["yaw_rad"]-b["yaw_rad"])) for a, b in zip(retained, retained[1:]))
    operations, ledger_errors = app["three_point"]["operation_records"], []
    last_path, last_rotation = 0., 0.
    for index, operation in enumerate(operations):
        p, r = operation["path_origin_m"], operation["rotation_origin_rad"]
        if p+1e-9 < last_path or r+1e-9 < last_rotation:
            ledger_errors.append(f"operation {index}: cumulative ledger decreased")
        if "path_end_m" in operation:
            if operation["path_end_m"]+1e-9 < p or operation["rotation_end_rad"]+1e-9 < r:
                ledger_errors.append(f"operation {index}: negative increment")
            last_path, last_rotation = operation["path_end_m"], operation["rotation_end_rad"]
        else:
            last_path, last_rotation = p, r
    if last_path > app["total_path_m"]+1e-9 or last_rotation > app["accumulated_rotation_rad"]+1e-9:
        ledger_errors.append("final cumulative ledger below operation end")
    previous = (0., 0.)
    for point in body["completed_points"]:
        if point["path_m"] < previous[0] or point["rotation_rad"] < previous[1]:
            ledger_errors.append("completed point ledger decreased")
        previous = point["path_m"], point["rotation_rad"]
    legs = []
    for leg in app.get("route", {}).get("legs", []):
        gap = math.dist(leg["waypoint_xy_m"], leg["final_position"][:2]) if "final_position" in leg else None
        legs.append({**leg, "recomputed_waypoint_error_m": gap,
                     "reported_error_matches": gap is not None and abs(gap-leg.get("waypoint_error_m", -1)) <= 1e-12,
                     "within_20mm": gap is not None and gap <= .02+1e-9})
    cores = [row["data"] for row in records if row["data"].get("section") == "ground_motion_core"]
    require(len(cores) == 1, "missing/ambiguous motion core")
    motion_core = cores[0]
    require(motion_core["state"] == app["state"] and motion_core["reason"] == app["reason"], "motion/point core conflict")
    full = [row["data"] for row in records if row["data"].get("section") == "ground_motion_full"]
    endpoints = motion_core["command_endpoints"]
    string_counts = {}
    all_attempts = [point.get("attempts", {}) for point in body["completed_points"]]
    if not any(point["point_index"] == app["three_point"]["point_index"] for point in body["completed_points"]):
        all_attempts.append(body["current_point"].get("attempts", {}))
    for kind, topic in (("recognition", "/inspection/detections"), ("declaration", "/referee/current_target")):
        endpoint = endpoints.get(topic, {})
        attempted = sum(bool(attempt.get(kind, {}).get("attempted")) for attempt in all_attempts)
        returned = sum(bool(attempt.get(kind, {}).get("sent")) for attempt in all_attempts)
        string_counts[kind] = {"observed_point_attempts": attempted, "observed_point_returns": returned,
                               "endpoint_publish_attempts": endpoint.get("actual_publish_attempt_count"),
                               "endpoint_publish_returns": endpoint.get("actual_publish_return_count"),
                               "counts_agree": attempted == endpoint.get("actual_publish_attempt_count")
                               and returned == endpoint.get("actual_publish_return_count")}
    return {"trajectory_rows": len(trajectory), "trajectory_dropped": survey["trajectory_dropped"],
            "retained_subset_first_stamp_s": retained[0]["stamp_s"] if retained else None,
            "sampled_path_lower_bound_m": path, "sampled_rotation_lower_bound_rad": rotation,
            "sampled_path_does_not_exceed_ledger": path <= app["total_path_m"]+1e-9,
            "sampled_rotation_does_not_exceed_ledger": rotation <= app["accumulated_rotation_rad"]+1e-9,
            "interpretation": "retained 10Hz samples after first active transition; sampled lower bounds, not exact odom ledger reconstruction",
            "command_rows": len(commands), "commands_dropped": survey["actual_commands_dropped"],
            "returned_nonzero": sum(bool(row["linear_x"] or row["angular_z"]) for row in commands),
            "returned_negative_x": sum(row["linear_x"] < 0 for row in commands), "command_contract_violations": bad_commands,
            "axis_attempt_sequence": full[0].get("axis_publish_attempts") if full else None,
            "axis_attempt_sequence_status": "present_in_full" if full else "not_transmitted_full_snapshot_fell_back_to_sections",
            "publication_failure_count": None if not full else sum(not row["returned"] for row in full[0].get("axis_publish_attempts", [])),
            "string_publications": string_counts, "operations": operations, "ledger_errors": ledger_errors,
            "last_operation_path_m": last_path, "after_last_operation_path_m": app["total_path_m"]-last_path,
            "legs": legs, "motion_log": logs, "callback_timing": motion_core.get("callback_timing"),
            "reported_endpoint_count_nonzero": motion_core.get("published_nonzero_count"),
            "reported_endpoint_count_zero": motion_core.get("published_zero_count")}


def write_csv(path, rows):
    if not rows:
        return
    keys = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows({key: json.dumps(value, ensure_ascii=False) if isinstance(value, (dict, list, tuple)) else value
                         for key, value in row.items()} for row in rows)


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2), encoding="utf-8")


def analyze_file(input_path, output_dir):
    source, output = Path(input_path).resolve(), Path(output_dir).resolve()
    require(source != output and output not in source.parents, "output must not contain input")
    frozen_source = Path(__file__).resolve().with_name("frozen_ground_three_point_axis.py")
    require(sha(frozen_source.read_bytes()) == FROZEN_SOURCE_SHA, "frozen axis source copy SHA mismatch")
    raw, records = load_source(source)
    body, transport, bundle_raw, packed = decode_bundle([row["data"] for row in records if row["data"].get("section") == SECTION])
    binding = bind_core(body, records)
    require(body["survey"].get("mode") == MODE, "survey mode mismatch")
    images = decode_images(body["capture_image_events"])
    point_audits = audit_points(body, images)
    motion = audit_motion(body, records)
    app, points = body["survey"]["approach"], body["completed_points"]
    result = {"schema": "ground_three_point_axis_analysis_v1", "source": str(source), "raw_sha256": sha(raw),
              "raw_bytes": len(raw), "transport_complete": True, "transport": transport, "binding": binding,
              "official_score": None, "official_acceptance": "unverified", "full_mission_complete": False,
              "state": app["state"], "reason": app["reason"], "mission_draw": app["three_point"]["mission_draw"],
              "point_index": app["three_point"]["point_index"], "target_id": app["three_point"]["target_id"],
              "completed_points_observed": len(points), "reported_total_path_m": app["total_path_m"],
              "reported_rotation_rad": app["accumulated_rotation_rad"], "errors": []}
    result.update(point_audits=point_audits, motion=motion,
                  liveness_at_last_check=body["current_point"].get("camera_liveness"),
                  frozen_axis_source_sha256=FROZEN_SOURCE_SHA,
                  frozen_axis_source_copy=str(frozen_source),
                  evidence_limits=["worker class labels retained; SIFT not re-executed by this standard-library auditor",
                                   "JPEG bytes verified; original ROS pixel bytes are not included, so raw ROS SHA cannot be independently recomputed",
                                   "publisher return and local transport counters do not establish referee acceptance",
                                   "third image capture exists, but without a retained result its job/calibration/matcher outcome is unproven"])
    output.mkdir(parents=True, exist_ok=True)
    (output/"bundle.exact.json").write_bytes(bundle_raw)
    (output/"bundle.exact.json.gz").write_bytes(packed)
    write_json(output/"bundle.pretty.json", body)
    image_dir = output/"images"
    image_dir.mkdir(exist_ok=True)
    for image in images:
        (image_dir/(image["phase"]+".jpg")).write_bytes(image["raw"])
    write_csv(output/"trajectory.csv", body["survey"]["trajectory_10hz"])
    write_csv(output/"commands.csv", body["survey"]["actual_commands"])
    write_csv(output/"operations.csv", motion["operations"])
    write_csv(output/"motion_log.csv", motion["motion_log"])
    require(sha(source.read_bytes()) == result["raw_sha256"], "source changed during analysis")
    write_json(output/"summary.json", result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--confirmed-complete", action="store_true", help="run only after parent confirms callback completion")
    args = parser.parse_args()
    require(args.confirmed_complete, "callback completion confirmation required before reading source")
    result = analyze_file(args.input, args.output)
    print(json.dumps({key: result[key] for key in ("transport_complete", "raw_sha256", "state", "reason", "mission_draw",
                     "point_index", "completed_points_observed", "reported_total_path_m", "reported_rotation_rad", "errors")},
                     ensure_ascii=True, allow_nan=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
