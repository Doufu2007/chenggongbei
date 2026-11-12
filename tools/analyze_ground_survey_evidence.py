"""Offline, evidence-only reassembly of active ground observation surveys.

Native JPEG chunks keep their exact decoded bytes. Motion series, image/pose
associations, missing rows and conflicting parts remain explicit. No target,
control causality, official geometry or scoring success is inferred.
"""
from __future__ import annotations

import argparse
import base64
from collections import Counter
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import re
import sys

try:
    from . import analyze_ground_motion_evidence as ground
except ImportError:
    import analyze_ground_motion_evidence as ground


MODE = "ground_observation_survey"
ANALYSIS_KINDS = {MODE: "ground_observation_survey_analysis", "ground_short_drive": "ground_short_drive_analysis",
                  "ground_bounded_approach": "ground_bounded_approach_analysis",
                  "ground_observation_route": "ground_observation_route_analysis",
                  "ground_single_point": "ground_single_point_analysis"}
TRAJECTORY_COLUMNS = ["wall_elapsed_s", "stamp_s", "x", "y", "z", "qx", "qy", "qz", "qw", "yaw_rad", "linear_speed", "angular_speed"]
COMMAND_COLUMNS = ["wall_elapsed_s", "clock_stamp_s", "linear_x", "angular_z", "state"]
SERIES = {"trajectory_10hz": (TRAJECTORY_COLUMNS, 500), "actual_commands": (COMMAND_COLUMNS, 1200)}
APPROACH_SERIES_CAPS = {"trajectory_10hz": 1200, "actual_commands": 5000}
ROUTE_SERIES_CAPS = {"trajectory_10hz": 3200, "actual_commands": 14000}
CAPTURE_MODES = frozenset(("ground_bounded_approach", "ground_observation_route", "ground_single_point"))
COMMON = {"mode", "read_only", "official_scoring", "partial", "section"}
IMAGE_FIELDS = {"topic", "phase", "stamp_s", "frame_id", "width", "height", "encoding", "jpeg_sha256",
                "part_index", "part_count", "jpeg_base64_part"}
SINGLE_POINT_FIELDS = {"session_id", "plan", "in_entry", "stationary_epoch", "epoch_invalid", "recapture_count",
                       "recognition", "match_results", "attempts", "recognition_status", "official_acceptance",
                       "body_fully_inside", "full_mission_complete"}
SINGLE_POINT_OPTIONAL_FIELDS = {"matcher_failures", "camera_liveness"}


def _single_point_detail(data):
    """Validate transport shape only; retain status text and attempts as observations."""
    if not isinstance(data, dict) or SINGLE_POINT_FIELDS - data.keys():
        raise ValueError("single_point evidence requires the complete v1 binding")
    if (not isinstance(data["session_id"], str) or not data["session_id"]
            or data["plan"] is not None and not isinstance(data["plan"], dict)
            or type(data["in_entry"]) is not bool
            or data["epoch_invalid"] is not None and not isinstance(data["epoch_invalid"], str)
            or data["recognition"] is not None and not isinstance(data["recognition"], dict)
            or not isinstance(data["match_results"], list) or not isinstance(data["attempts"], dict)
            or not isinstance(data["recognition_status"], list)):
        raise ValueError("single_point evidence binding or container types are invalid")
    if "matcher_failures" in data and not isinstance(data["matcher_failures"], list):
        raise ValueError("single_point matcher_failures must remain raw observations in a list")
    if data.get("camera_liveness") is not None and not isinstance(data["camera_liveness"], dict):
        raise ValueError("single_point camera_liveness must remain a timing observation or null")
    for field in ("stationary_epoch", "recapture_count"):
        ground._nonnegative_count(data[field], field)
    if data["official_acceptance"] is not False or data["full_mission_complete"] is not False:
        raise ValueError("single_point evidence must not claim official acceptance or full-mission completion")
    return {key: copy.deepcopy(data[key]) for key in SINGLE_POINT_FIELDS | SINGLE_POINT_OPTIONAL_FIELDS if key in data}


def _validate_mode(data, *, expected_mode=MODE):
    if data.get("mode") != expected_mode or data.get("read_only") is not False or data.get("official_scoring") != "unverified":
        raise ValueError("snapshot must identify active " + expected_mode + " with unverified scoring")
    # Pass the expected mode explicitly. Preserve every source mode unchanged.
    ground._validate_active(data, expected_mode=expected_mode)
    if isinstance(data.get("survey"), dict):
        metadata = data["survey"]
        for field in ("trajectory_dropped", "actual_commands_dropped"):
            if field in metadata:
                ground._nonnegative_count(metadata[field], field)
        if "mode" in metadata and metadata["mode"] != expected_mode:
            raise ValueError("observation metadata mode mismatch")
        detail_key = {"ground_short_drive": "drive", "ground_bounded_approach": "approach",
                      "ground_observation_route": "approach", "ground_single_point": "approach"}.get(expected_mode)
        if detail_key is not None:
            if metadata.get("mode") != expected_mode or not isinstance(metadata.get(detail_key), dict):
                raise ValueError(expected_mode + " metadata requires real mode and " + detail_key + " object")
            if metadata[detail_key].get("mode") != expected_mode:
                raise ValueError(detail_key + " metadata mode mismatch")
            if expected_mode == "ground_single_point" and "single_point" in metadata[detail_key]:
                _single_point_detail(metadata[detail_key]["single_point"])
            if expected_mode == "ground_single_point" and "single_point_transport" in metadata[detail_key]:
                reference = metadata[detail_key]["single_point_transport"]
                if (not isinstance(reference, dict) or reference.get("section") != "single_point_core"
                        or not isinstance(reference.get("session_id"), str) or not reference["session_id"]):
                    raise ValueError("single_point_transport must refer to an explicit single_point_core session")


def _native_image_bytes(encoded, metadata):
    raw = base64.b64decode(encoded, validate=True)
    if not 0 < len(raw) <= 384 * 1024 or hashlib.sha256(raw).hexdigest() != metadata["jpeg_sha256"]:
        raise ValueError("native JPEG byte size or SHA256 mismatch")
    if not raw.startswith(b"\xff\xd8\xff") or not raw.endswith(b"\xff\xd9"):
        raise ValueError("native JPEG is incomplete")
    from PIL import Image
    try:
        with Image.open(io.BytesIO(raw)) as image:
            if image.format != "JPEG" or image.size != (metadata["width"], metadata["height"]):
                raise ValueError("native JPEG dimensions do not match metadata")
            image.verify()
        with Image.open(io.BytesIO(raw)) as image:
            image.load()
    except (OSError, SyntaxError, ValueError) as exc:
        raise ValueError("invalid native JPEG: " + str(exc)) from exc
    return raw


def _exposure_validation(image, exposure):
    try:
        if exposure.get("association_error") is not None or exposure["frame_id"] != image["frame_id"]:
            return "invalid_or_unverified_pose_association"
        odom = exposure["odom"]
        body, association = (("base_footprint", "prior_ground_odom") if image["topic"] == "/camera/image"
                             else ("uav/base_link", "prior_uav_odom"))
        skew = image["stamp_s"] - odom["stamp_s"]
        if (odom["frame_id"] != "odom" or odom["child_frame_id"] != body or odom["association"] != association
                or not ground._finite(skew) or not -1e-9 <= skew <= .05 + 1e-9
                or not ground._finite(odom["image_minus_odom_s"]) or abs(skew - odom["image_minus_odom_s"]) > 1e-8
                or len(odom["position"]) != 3 or len(odom["quaternion"]) != 4
                or not all(ground._finite(x) for x in odom["position"] + odom["quaternion"])
                or abs(math.hypot(*odom["quaternion"])-1.) > .001):
            return "invalid_or_unverified_pose_association"
        return "valid_prior_pose_observation"
    except (TypeError, KeyError, ValueError):
        return "invalid_or_unverified_pose_association"


def analyze_file(input_path, output_dir, *, expected_mode=MODE):
    if expected_mode not in ANALYSIS_KINDS:
        raise ValueError("unsupported observation analysis mode")
    source, output = Path(input_path), Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    raw = source.read_bytes() if source.exists() else b""
    result = {"schema_version": 1, "kind": ANALYSIS_KINDS[expected_mode], "status": "pending",
        "mode": None, "read_only": None, "official_scoring": "unverified", "task_completion": "unverified", "score": None,
        "source_evidence": {"path": str(source), "exists": source.exists(), "bytes": len(raw),
                            "sha256": hashlib.sha256(raw).hexdigest() if source.exists() else None},
        "record_count": 0, "event_counts": {}, "missing_events": [], "raw_parts": [], "lifecycle": [],
        "snapshots": {"core_count": 0, "full_count": 0, "assembled": {}, "parts": []},
        "survey": {}, "metadata_observations": [], "survey_fragment_observations": [],
        "survey_fragment_assembly": {}, "series": {}, "image_groups": [], "native_images": [],
        "images": [], "parse_errors": [], "assembly_errors": [], "diagnostic_errors": [],
        "unknown_fields": [], "unknown_sections": [], "unknown_events": [], "conflicts": []}
    if expected_mode == "ground_single_point":
        result.update(single_point={}, single_point_observations=[], missing_single_point_core=True)
    counts, series_groups, image_groups = Counter(), {}, {}
    core = result["snapshots"]["assembled"]
    base_indices = {}

    def unknown(data, known, context, scope):
        fields = {key: value for key, value in data.items() if key not in known}
        if fields: result["unknown_fields"].append({**context, "scope": scope, "fields": fields})

    def conflict(scope, context, **details):
        result["conflicts"].append({**context, "scope": scope, **details})

    def motion_series(data, context):
        unknown(data, COMMON | {"series", "columns", "offset", "total_rows", "rows"}, context, "survey_motion_series")
        name = data.get("series")
        if name not in SERIES:
            result["unknown_sections"].append({**context, "data": data})
            return
        columns, maximum = SERIES[name]
        if expected_mode == "ground_bounded_approach":
            maximum = APPROACH_SERIES_CAPS[name]
        elif expected_mode in {"ground_observation_route", "ground_single_point"}:
            maximum = ROUTE_SERIES_CAPS[name]
        offset, total, rows = data.get("offset"), data.get("total_rows"), data.get("rows")
        if (data.get("columns") != columns or type(offset) is not int or type(total) is not int
                or not 0 <= offset <= total <= maximum or not isinstance(rows, list)
                or len(rows) > 300 or offset + len(rows) > total):
            raise ValueError("invalid motion series columns, offset, total or row count")
        for row in rows:
            if not isinstance(row, list) or len(row) != len(columns):
                raise ValueError("motion series row width mismatch")
            if name == "actual_commands":
                numeric = [row[0], *row[2:4]] + ([] if row[1] is None else [row[1]])
                if not isinstance(row[4], str): raise ValueError("command state must be a string")
            else: numeric = row
            if not all(ground._finite(value) for value in numeric) or row[0] < 0 or (row[1] is not None and row[1] < 0):
                raise ValueError("motion series requires finite numeric values and nonnegative timestamps")
        group = series_groups.setdefault(name, {"columns": columns, "total_rows": total, "rows": {}, "conflicted": False, "parts": []})
        group["parts"].append(context)
        if group["total_rows"] != total:
            group["conflicted"] = True
            conflict("series_total", context, series=name, existing=group["total_rows"], incoming=total)
        for index, row in enumerate(rows, offset):
            if index in group["rows"] and group["rows"][index] != row:
                group["conflicted"] = True
                conflict("series_row", context, series=name, index=index, existing=group["rows"][index], incoming=row)
            else: group["rows"].setdefault(index, row)

    def image_part(data, context):
        unknown(data, COMMON | IMAGE_FIELDS, context, "survey_image_part")
        if IMAGE_FIELDS - data.keys(): raise ValueError("native image fragment is missing required fields")
        index, count, text = data["part_index"], data["part_count"], data["jpeg_base64_part"]
        phases = {"first", "last", "last_associated"}
        if expected_mode in CAPTURE_MODES:
            phases |= {"capture_0", "capture_1", "capture_2"}
        if (data["topic"] not in {"/camera/image", "/uav/camera/image"} or data["phase"] not in phases
                or not isinstance(data["frame_id"], str) or len(data["frame_id"]) > 256
                or not ground._finite(data["stamp_s"]) or data["stamp_s"] < 0
                or type(data["width"]) is not int or not 1 <= data["width"] <= 800
                or type(data["height"]) is not int or not 1 <= data["height"] <= 800 or data["encoding"] != "jpeg"
                or not isinstance(data["jpeg_sha256"], str) or not re.fullmatch(r"[0-9a-f]{64}", data["jpeg_sha256"])
                or type(index) is not int or type(count) is not int or not 0 <= index < count <= 17
                or not isinstance(text, str) or not 0 < len(text) <= 32000):
            raise ValueError("invalid native image fragment metadata")
        base64.b64decode(text, validate=True)
        identity = (data["topic"], data["phase"], data["stamp_s"])
        metadata = {key: data[key] for key in IMAGE_FIELDS - {"part_index", "jpeg_base64_part"}}
        group = image_groups.setdefault(identity, {"metadata": metadata, "parts": {}, "contexts": [], "conflicted": False})
        group["contexts"].append(context)
        if group["metadata"] != metadata:
            group["conflicted"] = True
            conflict("native_image_metadata", context, identity=list(identity), existing=group["metadata"], incoming=metadata)
        if index in group["parts"] and group["parts"][index] != text:
            group["conflicted"] = True
            conflict("native_image_part", context, identity=list(identity), part_index=index,
                     existing=group["parts"][index], incoming=text)
        else: group["parts"].setdefault(index, text)

    def base_snapshot(data, context):
        state = result["snapshots"]
        state["parts"].append({**context, "data": data})
        name = data["section"]
        known = ground.FULL_FIELDS | ground.OPTIONAL_FIELDS | COMMON | {"survey"}
        if name in {"ground_motion_core", "ground_motion_full"}:
            unknown(data, known, context, "probe_snapshot")
            if name == "ground_motion_core" and data["partial"] is not True:
                raise ValueError("survey motion core must remain partial")
            if name == "ground_motion_full" and data["partial"] is not False:
                raise ValueError("survey motion full must have partial=false")
            state["core_count" if name == "ground_motion_core" else "full_count"] += 1
            core.update({key: copy.deepcopy(value) for key, value in data.items() if key in known - {"partial", "section"}})
            if isinstance(data.get("survey"), dict):
                result["metadata_observations"].append({**context, "scope": "full_snapshot", "data": data["survey"]})
                if not result["survey"]: result["survey"] = copy.deepcopy(data["survey"])
            if isinstance(data.get("errors"), list): result["diagnostic_errors"].extend({**context, "data": x} for x in data["errors"])
        elif name == "evidence_errors":
            if not isinstance(data.get("errors"), list): raise ValueError("evidence_errors requires a list")
            result["diagnostic_errors"].extend({**context, "data": x} for x in data["errors"])
        elif name in ground.MAP_SECTIONS:
            if not isinstance(data.get("topic"), str) or "value" not in data: raise ValueError("base map fragment requires topic/value")
            core.setdefault(name, {})[data["topic"]] = data["value"]
        elif name in ground.LIST_SECTIONS | {"tf_edges"}:
            values = data.get("transforms" if name == "tf_edges" else "value")
            offset = data.get("offset")
            if type(offset) is not int or not 0 <= offset <= 100_000 or not isinstance(values, list):
                raise ValueError("invalid base fragment offset/list")
            entries = base_indices.setdefault(name, {})
            for index, value in enumerate(values, offset):
                if index in entries and entries[index] != value: conflict("base_fragment", context, section=name, index=index)
                else: entries.setdefault(index, value)
            core[name] = {str(i): value for i, value in sorted(entries.items())} if name == "tf_edges" else [entries[i] for i in sorted(entries)]
        elif name == "single_point_core" and expected_mode == "ground_single_point":
            unknown(data, COMMON | SINGLE_POINT_FIELDS | SINGLE_POINT_OPTIONAL_FIELDS | {"revision", "state", "reason"}, context, name)
            if data.get("partial") is not True:
                raise ValueError("single_point_core must remain partial")
            detail = _single_point_detail(data)
            result["single_point_observations"].append({**context, "data": copy.deepcopy(data)})
            if result["single_point"] and result["single_point"] != detail:
                conflict(name, context, existing=result["single_point"], incoming=detail)
            result["single_point"] = detail
            result["missing_single_point_core"] = False
        else: result["unknown_sections"].append({**context, "data": data})

    for line_number, line in enumerate(raw.splitlines(), 1):
        if not line.strip(): continue
        try:
            wrapped = json.loads(line.decode("utf-8"), object_pairs_hook=ground._unique_object,
                parse_constant=ground._nonfinite, parse_float=ground._finite_float)
            if not isinstance(wrapped, dict) or not isinstance(wrapped.get("payload"), dict):
                raise ValueError("expected receiver wrapper with payload object")
            payload = wrapped["payload"]
            name, data = payload.get("event"), payload.get("data")
            if not isinstance(name, str) or not isinstance(data, dict): raise ValueError("invalid event/data")
            context = {"line": line_number, "event": name, "payload_timestamp": payload.get("timestamp"),
                       "received_at": wrapped.get("received_at"), "source": wrapped.get("source")}
            result["record_count"] += 1; counts[name] += 1
            unknown(wrapped, {"received_at", "source", "payload"}, context, "wrapper")
            unknown(payload, {"event", "timestamp", "data"}, context, "payload")
            if name == "probe_snapshot":
                result["raw_parts"].append({**context, "data": data})
                _validate_mode(data, expected_mode=expected_mode)
                result["mode"], result["read_only"] = data["mode"], data["read_only"]
                section = data["section"]
                if section.startswith("survey_") and data["partial"] is not True:
                    raise ValueError("survey section must remain partial")
                if section == "survey_metadata":
                    unknown(data, COMMON | {"survey"}, context, section)
                    if not isinstance(data.get("survey"), dict): raise ValueError("survey_metadata requires survey object")
                    result["metadata_observations"].append({**context, "scope": section, "data": data["survey"]})
                    if result["survey"] and result["survey"] != data["survey"]:
                        conflict("survey_metadata", context, existing=result["survey"], incoming=data["survey"])
                    result["survey"] = copy.deepcopy(data["survey"])
                elif section == "survey_motion_series": motion_series(data, context)
                elif section == "survey_image_part": image_part(data, context)
                elif section == "survey_image_errors":
                    if not isinstance(data.get("errors"), list): raise ValueError("survey_image_errors requires errors list")
                    result["diagnostic_errors"].extend({**context, "data": x} for x in data["errors"])
                elif section == "survey":
                    unknown(data, COMMON | {"topic", "value"}, context, section)
                    if (data.get("partial") is not True or not isinstance(data.get("topic"), str)
                            or not data["topic"] or "value" not in data):
                        raise ValueError("fallback survey fragment requires partial topic/value")
                    topic, value = data["topic"], data["value"]
                    if topic == "mode" and value != expected_mode:
                        raise ValueError("fallback survey fragment mode mismatch")
                    result["survey_fragment_observations"].append({**context, "data": copy.deepcopy(data)})
                    assembled = result["survey_fragment_assembly"]
                    if topic in assembled and assembled[topic] != value:
                        conflict("survey_fragment", context, topic=topic,
                                 existing=assembled[topic], incoming=copy.deepcopy(value))
                    else:
                        assembled.setdefault(topic, copy.deepcopy(value))
                else: base_snapshot(data, context)
            elif name == "image_sample":
                raw_image = ground._image_bytes(data); sha = hashlib.sha256(raw_image).hexdigest()
                filename = f"thumbnail_L{line_number:06d}_{sha[:16]}.jpg"
                (output / filename).write_bytes(raw_image)
                result["images"].append({**context, **{k: data[k] for k in ground.EVENT_FIELDS[name] - {"jpeg_base64"}},
                                          "sha256": sha, "bytes": len(raw_image), "file_name": filename})
            elif name in {"launcher_started", "launcher_exit", "probe_started", "probe_finished"}:
                result["lifecycle"].append({**context, "data": data})
                if ("mode" in data and data["mode"] != expected_mode) or ("read_only" in data and data["read_only"] is not False):
                    raise ValueError("observation lifecycle mode or read_only mismatch")
            else: result["unknown_events"].append({**context, "data": data})
        except (ValueError, TypeError, KeyError, OSError, ImportError, RecursionError) as exc:
            result["parse_errors"].append({"line": line_number, "error": str(exc)})

    for name, group in series_groups.items():
        missing = sorted(set(range(group["total_rows"])) - group["rows"].keys())
        result["series"][name] = {"columns": group["columns"], "total_rows": group["total_rows"],
            "received_rows": len(group["rows"]), "missing_rows": missing, "conflicted": group["conflicted"],
            "complete": not missing and not group["conflicted"], "parts": group["parts"],
            "indexed_rows": {str(i): row for i, row in sorted(group["rows"].items())},
            "records": [dict(zip(group["columns"], row)) for i, row in sorted(group["rows"].items())]}
    exposures = result["survey"].get("exposure_records", [])
    for group in image_groups.values():
        metadata = group["metadata"]
        missing = sorted(set(range(metadata["part_count"])) - group["parts"].keys())
        summary = {**metadata, "received_parts": len(group["parts"]), "missing_parts": missing,
                   "conflicted": group["conflicted"], "parts": group["contexts"]}
        result["image_groups"].append(summary)
        if missing or group["conflicted"]: continue
        try:
            native = _native_image_bytes("".join(group["parts"][i] for i in range(metadata["part_count"])), metadata)
            sha = hashlib.sha256(native).hexdigest()
            filename = f"native_L{group['contexts'][0]['line']:06d}_{sha[:16]}.jpg"
            (output / filename).write_bytes(native)
            matches = [item for item in exposures if isinstance(item, dict)
                and item.get("topic") == metadata["topic"] and item.get("phase") == metadata["phase"]
                and item.get("stamp_s") == metadata["stamp_s"]]
            exposure = matches[0] if len(matches) == 1 else None
            result["native_images"].append({**metadata, "sha256": sha, "bytes": len(native), "file_name": filename,
                "parts": group["contexts"], "exposure": exposure,
                "exposure_validation": _exposure_validation(metadata, exposure) if exposure is not None else "no_unique_exposure_evidence"})
        except (ValueError, TypeError, OSError, ImportError) as exc:
            result["assembly_errors"].append({"scope": "native_image", "metadata": metadata, "error": str(exc)})
    result["event_counts"] = dict(counts)
    result["missing_events"] = [name for name in ground.LIFECYCLE_EVENTS if not counts[name]]
    result["missing_series"] = sorted(set(SERIES) - result["series"].keys())
    result["missing_metadata"] = not bool(result["metadata_observations"])
    if expected_mode == "ground_single_point":
        approach = result["survey"].get("approach", {})
        embedded = approach.get("single_point")
        if embedded is not None:
            try:
                detail = _single_point_detail(embedded)
                if result["single_point"] and result["single_point"] != detail:
                    conflict("single_point_metadata", {"source": "survey.approach.single_point"},
                             existing=result["single_point"], incoming=detail)
                elif not result["single_point"]:
                    result["single_point"] = detail
            except ValueError as exc:
                result["assembly_errors"].append({"scope": "single_point_metadata", "error": str(exc)})
        reference = approach.get("single_point_transport")
        if isinstance(reference, dict) and result["single_point"]:
            if reference.get("session_id") != result["single_point"].get("session_id"):
                conflict("single_point_transport", {"source": "survey.approach.single_point_transport"},
                         existing=result["single_point"].get("session_id"), incoming=copy.deepcopy(reference))
    result["expected_capture_phases"] = []
    result["missing_capture_phases"] = []
    if expected_mode in CAPTURE_MODES:
        count = result["survey"].get("approach", {}).get("capture_count", 0)
        if type(count) is not int or not 0 <= count <= 3:
            result["assembly_errors"].append({"scope": "capture_count", "error": "capture_count must be an integer from 0 to 3"})
        else:
            result["expected_capture_phases"] = [f"capture_{index}" for index in range(count)]
            received = {image["phase"] for image in result["native_images"] if image["topic"] == "/uav/camera/image"}
            result["missing_capture_phases"] = [phase for phase in result["expected_capture_phases"] if phase not in received]
    result["missing_snapshot_fields"] = sorted(ground.FULL_FIELDS - core.keys())
    fields = ("state", "reason", "odom_first", "odom_last", "published_nonzero_count", "published_zero_count", "max_observed_displacement_m")
    result["motion"] = {key: core.get(key) for key in fields}
    result["motion"].update(observed_first_last_delta_m=ground._odom_delta(core.get("odom_first"), core.get("odom_last")),
                            pulse_response_causality="unverified", control_delivery="unverified")
    result["command_endpoints"] = core.get("command_endpoints", {})
    if result["parse_errors"] or result["assembly_errors"]: result["status"] = "invalid"
    elif result["record_count"]:
        complete = (result["snapshots"]["full_count"] and not result["missing_events"] and not result["missing_series"]
                    and not result["missing_metadata"] and not result["conflicts"] and not result["diagnostic_errors"]
                    and not result["missing_capture_phases"]
                    and not result.get("missing_single_point_core", False)
                    and all(observation["data"].get(field, 0) == 0
                            for observation in result["metadata_observations"]
                            for field in ("trajectory_dropped", "actual_commands_dropped"))
                    and all(x["complete"] for x in result["series"].values())
                    and all(not x["missing_parts"] and not x["conflicted"] for x in result["image_groups"]))
        result["status"] = "collected" if complete else "partial"
    result["notes"] = [
        "Collected/FINISHED/target_yaw_reached/short_drive_distance_reached indicate diagnostic capture or probe state only, never task completion or scoring.",
        "Actual command rows record publish calls and their times, not actuator acceptance. Trajectory/20Hz command calls alone do not establish response causality.",
        "Candidate geometry and ground_N mapping are observations/hypotheses in the source metadata, not official task or image answers.",
        "Image/pose validation checks the received prior-pose association and skew only; camera extrinsics, projection accuracy and physical alignment remain unverified.",
        "Native JPEGs preserve exactly the decoded source bytes. Thumbnails are separate evidence; missing chunks never create placeholder frames.",
        "Series numeric values were rounded by the source. File order is arrival order, not proof of generation order; raw fragments/conflicts remain available.",
    ]
    if expected_mode == "ground_single_point":
        result["notes"].append("Single-point match results and attempted/sent records are source observations only. Recognition-status String.data stays uninterpreted; no accepted flag, score, complete mission or official settlement is inferred.")
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return result


analyze_evidence = analyze_file


def main(argv=None, *, expected_mode=MODE):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    result = analyze_file(args.input, args.output, expected_mode=expected_mode)
    print(json.dumps({"status": result["status"], "record_count": result["record_count"],
                      "native_images": len(result["native_images"]), "parse_errors": result["parse_errors"],
                      "assembly_errors": result["assembly_errors"], "summary": str(args.output / "summary.json")}, ensure_ascii=True))
    return 2 if result["status"] == "invalid" else 0


if __name__ == "__main__":
    raise SystemExit(main())
