"""Analyze local active ground-probe callbacks without claiming task success.

The v6/v7 probe emits active ``ground_motion_core``/``ground_motion_full``
snapshots and bounded section fragments. This tool preserves their provenance,
including the fact that read_only is false. JPEGs keep their original bytes.
Use: python tools/analyze_ground_motion_evidence.py INPUT.jsonl --output DIR
"""
from __future__ import annotations

import argparse
from collections import Counter
import copy
import hashlib
import json
import math
from pathlib import Path
import sys

try:
    from .analyze_probe_evidence import _finite_float, _image_bytes, _nonfinite, _unique_object, EVENT_FIELDS
except ImportError:  # Direct ``python tools/...py`` invocation.
    from analyze_probe_evidence import _finite_float, _image_bytes, _nonfinite, _unique_object, EVENT_FIELDS


MODE = "bounded_ground_motion"
CORE_FIELDS = {"mode", "read_only", "official_scoring", "startup", "elapsed_s", "state", "reason", "limits",
               "command_endpoints", "sample_counts", "odom_first", "odom_last", "max_observed_displacement_m",
               "published_nonzero_count", "published_zero_count", "errors", "motion_log_dropped",
               "command_endpoint_history_dropped"}
SECTION_FIELDS = {"command_endpoint_history", "odometry", "public_messages", "tf_edges", "motion_log", "image_records"}
FULL_FIELDS = CORE_FIELDS | SECTION_FIELDS
OPTIONAL_FIELDS = {"timing", "graph_refresh_diagnostics", "callback_timing"}  # Older probes may omit these observations.
COMMON_FIELDS = {"mode", "read_only", "official_scoring", "partial", "section"}
COUNT_FIELDS = {"published_nonzero_count", "published_zero_count", "motion_log_dropped", "command_endpoint_history_dropped"}
LIST_SECTIONS = {"command_endpoint_history", "motion_log", "image_records"}
MAP_SECTIONS = {"odometry", "public_messages"}
LIFECYCLE_EVENTS = ("launcher_started", "probe_started", "probe_snapshot", "launcher_exit")


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _nonnegative_count(value, field):
    if type(value) is not int or value < 0:
        raise ValueError(field + " must be a nonnegative integer")


def _validate_active(data, *, expected_mode=MODE):
    if data.get("mode") != expected_mode or data.get("read_only") is not False:
        raise ValueError("snapshot must identify active " + expected_mode + " with read_only=false")
    if data.get("official_scoring") != "unverified":
        raise ValueError("diagnostic official_scoring must remain unverified")
    if type(data.get("partial")) is not bool:
        raise ValueError("snapshot partial flag must be boolean")
    if not isinstance(data.get("section"), str):
        raise ValueError("snapshot section must be a string")
    for name in COUNT_FIELDS & data.keys():
        _nonnegative_count(data[name], name)
    if "sample_counts" in data:
        counts = data["sample_counts"]
        if not isinstance(counts, dict):
            raise ValueError("sample_counts must be an object")
        for value in counts.values():
            _nonnegative_count(value, "sample_counts")
    for name in ("limits", "command_endpoints", "timing", "graph_refresh_diagnostics", "callback_timing"):
        if name in data and not isinstance(data[name], dict):
            raise ValueError(name + " must be an object")
    for name in LIST_SECTIONS | {"errors"}:
        if name in data and not isinstance(data[name], list):
            raise ValueError(name + " must be a list")
    for name in MAP_SECTIONS | {"tf_edges"}:
        if name in data and not isinstance(data[name], dict):
            raise ValueError(name + " must be an object")
    if "command_endpoints" in data:
        for value in data["command_endpoints"].values():
            if not isinstance(value, dict):
                raise ValueError("command endpoint must be an object")
            for key in ("matched_subscriptions", "subscriber_endpoints_omitted"):
                if key in value:
                    _nonnegative_count(value[key], key)
            if "subscriber_endpoints" in value and not isinstance(value["subscriber_endpoints"], list):
                raise ValueError("subscriber_endpoints must be a list")
    for name in ("elapsed_s", "max_observed_displacement_m"):
        if name in data and (not _finite(data[name]) or data[name] < 0):
            raise ValueError(name + " must be nonnegative and finite")
    if "startup" in data and data["startup"] is not None:
        startup = data["startup"]
        if not isinstance(startup, dict) or startup.get("mode") != expected_mode or startup.get("read_only") is not False:
            raise ValueError("startup does not identify the active probe")


def _odom_delta(first, last):
    """Only a frame-compatible positional observation, never pulse causality."""
    try:
        if (first["header"]["frame_id"] != last["header"]["frame_id"]
                or first["child_frame_id"] != last["child_frame_id"]):
            return None
        stamps = []
        points = []
        for message in (first, last):
            stamp = message["header"]["stamp"]
            sec, nano = stamp["sec"], stamp["nanosec"]
            if type(sec) is not int or type(nano) is not int or sec < 0 or not 0 <= nano < 1_000_000_000:
                return None
            stamps.append(sec + nano / 1e9)
            point = tuple(message["pose"]["pose"]["position"][key] for key in ("x", "y", "z"))
            if not all(_finite(value) for value in point):
                return None
            points.append(point)
        return math.dist(*points) if stamps[1] >= stamps[0] else None
    except (KeyError, TypeError, ValueError, OverflowError):
        return None


def analyze_file(input_path, output_dir):
    """Save summary.json and strictly validated original diagnostic JPEGs."""
    source, output = Path(input_path), Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    raw = source.read_bytes() if source.exists() else b""
    result = {
        "schema_version": 1, "kind": "bounded_ground_diagnostic_analysis", "status": "pending",
        "mode": None, "read_only": None, "official_scoring": "unverified", "task_completion": "unverified", "score": None,
        "source_evidence": {"path": str(source), "exists": source.exists(), "bytes": len(raw),
                            "sha256": hashlib.sha256(raw).hexdigest() if source.exists() else None},
        "record_count": 0, "event_counts": {}, "missing_events": [], "missing_snapshot_fields": [],
        "launcher_starts": [], "launcher_exits": [], "probe_starts": [], "probe_finishes": [],
        "snapshots": {"core_count": 0, "full_count": 0, "incomplete_full_count": 0,
                      "partial_count": 0, "assembled": {}, "parts": []},
        "images": [], "diagnostic_errors": [], "parse_errors": [], "unknown_events": [], "unknown_fields": [],
        "unknown_sections": [], "fragment_conflicts": [], "fragment_gaps": {}, "fragment_index_maps": {},
        "command_endpoints": {}, "command_endpoint_observations": [], "timing_observations": [],
    }
    counts = Counter()
    indexed = {}
    assembled = result["snapshots"]["assembled"]

    def unknown(context, scope, data, known):
        fields = {key: value for key, value in data.items() if key not in known}
        if fields:
            result["unknown_fields"].append({**context, "scope": scope, "fields": fields})

    def timing_observation(value, context, scope):
        if isinstance(value, dict):
            result["timing_observations"].append({**context, "scope": scope, "data": value})

    def record_value(name, value, context):
        assembled[name] = copy.deepcopy(value)
        if name == "errors" and isinstance(value, list):
            result["diagnostic_errors"].extend({**context, "data": item} for item in value)
        elif name == "timing":
            timing_observation(value, context, "snapshot")
        elif name == "command_endpoints":
            result["command_endpoint_observations"].append({**context, "scope": "snapshot", "data": value})
        elif name == "command_endpoint_history" and isinstance(value, list):
            result["command_endpoint_observations"].extend({**context, "scope": name, "data": item} for item in value)
        elif name == "motion_log" and isinstance(value, list):
            for item in value:
                if isinstance(item, dict) and "timing" in item:
                    timing_observation(item["timing"], {**context, "wall_elapsed_s": item.get("wall_elapsed_s"),
                                       "state": item.get("state"), "reason": item.get("reason")}, "motion_log_transition")

    def indexed_part(name, offset, values, context):
        if type(offset) is not int or not 0 <= offset <= 100_000 or not isinstance(values, list):
            raise ValueError("invalid section fragment offset or list")
        items = indexed.setdefault(name, {})
        for index, value in enumerate(values, offset):
            if index in items and items[index] != value:
                result["fragment_conflicts"].append({**context, "section": name, "index": index,
                                                     "existing": items[index], "incoming": value})
            elif index not in items:
                items[index] = value
        value = {str(index): item for index, item in sorted(items.items())} if name == "tf_edges" else [items[index] for index in sorted(items)]
        record_value(name, value, context)

    def snapshot(data, context):
        state = result["snapshots"]
        state["parts"].append({**context, "data": data})
        _validate_active(data)
        result["mode"], result["read_only"] = MODE, False
        name = data.get("section")
        if name in ("ground_motion_core", "ground_motion_full"):
            unknown(context, "probe_snapshot", data, FULL_FIELDS | OPTIONAL_FIELDS | COMMON_FIELDS)
            if name == "ground_motion_core":
                if data["partial"] is not True:
                    raise ValueError("ground_motion_core must remain partial")
                state["core_count"] += 1
            else:
                if data["partial"] is not False:
                    raise ValueError("ground_motion_full must have partial=false")
                state["incomplete_full_count" if FULL_FIELDS - data.keys() else "full_count"] += 1
            state["partial_count"] += int(data["partial"])
            for key in (FULL_FIELDS | OPTIONAL_FIELDS) & data.keys():
                record_value(key, data[key], context)
            return
        if data["partial"] is not True:
            raise ValueError("section fragment must remain partial")
        state["partial_count"] += 1
        if name == "evidence_errors":
            unknown(context, "probe_snapshot.fragment", data, COMMON_FIELDS | {"errors"})
            if not isinstance(data.get("errors"), list):
                raise ValueError("evidence_errors fragment requires errors list")
            result["diagnostic_errors"].extend({**context, "data": item} for item in data["errors"])
        elif name in MAP_SECTIONS:
            unknown(context, "probe_snapshot.fragment", data, COMMON_FIELDS | {"topic", "value"})
            if not isinstance(data.get("topic"), str) or "value" not in data:
                raise ValueError("map fragment requires topic and value")
            value = assembled.setdefault(name, {})
            topic = data["topic"]
            if topic in value and value[topic] != data["value"]:
                result["fragment_conflicts"].append({**context, "section": name, "topic": topic,
                                                     "existing": value[topic], "incoming": data["value"]})
            else:
                value[topic] = copy.deepcopy(data["value"])
        elif name in LIST_SECTIONS | {"tf_edges"}:
            key = "transforms" if name == "tf_edges" else "value"
            unknown(context, "probe_snapshot.fragment", data, COMMON_FIELDS | {"offset", key})
            indexed_part(name, data.get("offset"), data.get(key), context)
        else:
            result["unknown_sections"].append({**context, "data": data})

    for line_number, line in enumerate(raw.splitlines(), 1):
        if not line.strip():
            continue
        try:
            wrapped = json.loads(line.decode("utf-8"), object_pairs_hook=_unique_object,
                                 parse_constant=_nonfinite, parse_float=_finite_float)
            if not isinstance(wrapped, dict) or not isinstance(wrapped.get("payload"), dict):
                raise ValueError("expected receiver wrapper with payload object")
            payload = wrapped["payload"]
            if not isinstance(payload.get("event"), str) or not isinstance(payload.get("data"), dict):
                raise ValueError("payload requires event string and data object")
            name, data = payload["event"], payload["data"]
            context = {"line": line_number, "event": name, "payload_timestamp": payload.get("timestamp"),
                       "received_at": wrapped.get("received_at"), "source": wrapped.get("source")}
            result["record_count"] += 1
            counts[name] += 1
            unknown(context, "wrapper", wrapped, {"received_at", "payload", "source"})
            unknown(context, "payload", payload, {"event", "timestamp", "data"})
            if name == "probe_snapshot":
                snapshot(data, context)
                continue
            if name not in EVENT_FIELDS:
                result["unknown_events"].append({**context, "data": data})
                continue
            lifecycle = name in {"launcher_started", "launcher_exit", "probe_started", "probe_finished"}
            known = EVENT_FIELDS[name] | ({"mode", "official_scoring", "node", "limits", "timing"} if lifecycle else set())
            unknown(context, name, data, known)
            destination = {"launcher_started": "launcher_starts", "launcher_exit": "launcher_exits",
                           "probe_started": "probe_starts", "probe_finished": "probe_finishes"}
            if name in destination:
                result[destination[name]].append({**context, "data": data})
                if "read_only" in data and data["read_only"] is not False:
                    raise ValueError("active probe lifecycle cannot be read_only")
                if "mode" in data and data["mode"] != MODE:
                    raise ValueError("active probe lifecycle mode mismatch")
            if name in {"sample_error", "subscription_error"}:
                result["diagnostic_errors"].append({**context, "data": data})
            elif name == "image_sample":
                image = _image_bytes(data)
                digest = hashlib.sha256(image).hexdigest()
                file_name = f"image_L{line_number:06d}_{digest[:16]}.jpg"
                (output / file_name).write_bytes(image)
                metadata = {key: data[key] for key in EVENT_FIELDS["image_sample"] if key != "jpeg_base64"}
                result["images"].append({**context, **metadata, "sha256": digest, "bytes": len(image), "file_name": file_name})
        except (ValueError, TypeError, ImportError, OSError, RecursionError) as exc:
            result["parse_errors"].append({"line": line_number, "error": str(exc)})

    for name, entries in indexed.items():
        result["fragment_index_maps"][name] = {str(index): item for index, item in sorted(entries.items())}
        if entries:
            gaps = sorted(set(range(max(entries) + 1)) - entries.keys())
            if gaps:
                result["fragment_gaps"][name] = gaps
    result["event_counts"] = dict(counts)
    result["missing_events"] = [name for name in LIFECYCLE_EVENTS if not counts[name]]
    result["missing_snapshot_fields"] = sorted(FULL_FIELDS - assembled.keys())
    result["launcher_exit_codes"] = [item["data"]["exit_code"] for item in result["launcher_exits"]
                                      if type(item["data"].get("exit_code")) is int]
    result["command_endpoints"] = assembled.get("command_endpoints", {})
    motion_fields = ("state", "reason", "odom_first", "odom_last", "max_observed_displacement_m",
                     "published_nonzero_count", "published_zero_count")
    result["motion"] = {key: assembled.get(key) for key in motion_fields}
    result["motion"].update(observed_first_last_delta_m=_odom_delta(assembled.get("odom_first"), assembled.get("odom_last")),
                            pulse_response_causality="unverified", control_delivery="unverified")
    result["action_boundaries"] = {"declared_limits": assembled.get("limits"),
        "per_command_sensor_ages": "not_emitted", "final_timing": assembled.get("timing"),
        "timing_scope": "transition_and_final_observations_only" if result["timing_observations"] else "not_emitted",
        "independent_sensor_age_validation": "unverified"}
    if result["parse_errors"]:
        result["status"] = "invalid"
    elif result["record_count"]:
        result["status"] = "collected" if result["snapshots"]["full_count"] and not result["missing_events"] else "partial"
    result["notes"] = [
        "Collected means diagnostic capture only; FINISHED, exit 0, publish counts and matched subscribers are not task or scoring success.",
        "Published messages are attempts, not controller acceptance or actual velocity. Endpoint/QoS values are the observed graph, not a verified official contract.",
        "First/last odometry span the capture, not necessarily the pulse. Their positional difference cannot establish pulse-response causality.",
        "Timing and fresh flags, when emitted, are probe observations at transitions/final capture, not independently verified ages for every command.",
        "Assembly follows callback file order; received order does not establish generation order. Raw snapshot parts, unknowns and conflicts are retained.",
        "Fragments lack a completeness manifest; absence of an internal index gap does not prove the tail or a whole section was received.",
        "Counts from repeated snapshots are not summed. Candidate geometric bounds do not verify current official dimensions.",
        "max_observed_displacement_m is the probe's distance since a pulse origin was established; zero can also mean no pulse occurred.",
    ]
    if result["status"] == "pending":
        result["notes"].append("Input is absent or empty; no motion, sensor or image evidence was invented.")
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return result


analyze_evidence = analyze_file


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        result = analyze_file(args.input, args.output)
    except OSError as exc:
        print(json.dumps({"status": "invalid", "error": str(exc)}, ensure_ascii=True))
        return 2
    print(json.dumps({"status": result["status"], "record_count": result["record_count"],
                      "images": len(result["images"]), "parse_errors": result["parse_errors"],
                      "summary": str(args.output / "summary.json")}, ensure_ascii=True))
    return 2 if result["status"] == "invalid" else 0


if __name__ == "__main__":
    sys.exit(main())
