"""Analyze local callback JSONL without networking or inferring mission success.

JPEG files contain the exact decoded input bytes, never rendered substitutes.
Use: python tools/analyze_probe_evidence.py INPUT.jsonl --output OUTPUT_DIR
"""
from __future__ import annotations

import argparse
import base64
from collections import Counter
import hashlib
import io
import json
import math
from pathlib import Path
import sys


EVENT_FIELDS = {
    "launcher_started": {"read_only"}, "launcher_exit": {"exit_code", "read_only"},
    "probe_started": {"read_only", "duration_s"}, "public_graph": {"topics"},
    "subscription": {"topic", "type", "fields", "publishers"},
    "sample": {"topic", "sample_index", "message"},
    "sample_error": {"topic", "error"}, "subscription_error": {"topic", "error"},
    "heartbeat": {"elapsed_s", "sample_counts"}, "probe_finished": {"read_only", "sample_counts"},
    "image_sample": {"topic", "stamp", "frame_id", "width", "height", "encoding", "jpeg_base64"},
}
SNAPSHOT_FIELDS = {"read_only", "official_scoring", "elapsed_s", "topics", "subscriptions", "samples",
                   "sample_counts", "errors", "public_endpoints"}
MAP_SECTIONS = {"subscriptions", "samples", "sample_counts", "errors", "public_endpoints"}
LIFECYCLE_EVENTS = ("launcher_started", "probe_started", "probe_finished", "probe_snapshot", "launcher_exit")


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key: " + key)
        result[key] = value
    return result


def _nonfinite(_):
    raise ValueError("nonfinite JSON value")


def _finite_float(text):
    value = float(text)
    if not math.isfinite(value):
        raise ValueError("nonfinite JSON float")
    return value


def _mapping(value, section):
    """The metadata fragment converts topic dictionaries into topic/value lists."""
    if isinstance(value, dict):
        return value
    if isinstance(value, list):
        result = {}
        for item in value:
            if (not isinstance(item, dict) or set(item) != {"topic", "value"}
                    or not isinstance(item["topic"], str) or item["topic"] in result):
                raise ValueError("invalid or duplicate topic/value entry in " + section)
            result[item["topic"]] = item["value"]
        return result
    raise ValueError(section + " must be an object or topic/value list")


def _image_bytes(data):
    if not EVENT_FIELDS["image_sample"].issubset(data):
        raise ValueError("image_sample is missing required metadata")
    if (not isinstance(data["topic"], str) or not isinstance(data["frame_id"], str)
            or type(data["stamp"]) not in (int, float) or not math.isfinite(data["stamp"])
            or data["stamp"] < 0 or type(data["width"]) is not int or type(data["height"]) is not int
            or not 1 <= data["width"] <= 480 or not 1 <= data["height"] <= 360
            or data["encoding"] != "jpeg" or not isinstance(data["jpeg_base64"], str)):
        raise ValueError("invalid image_sample metadata")
    try:
        raw = base64.b64decode(data["jpeg_base64"], validate=True)
    except (ValueError, UnicodeError) as exc:
        raise ValueError("invalid JPEG base64") from exc
    if not raw.startswith(b"\xff\xd8\xff") or not raw.endswith(b"\xff\xd9"):
        raise ValueError("incomplete JPEG markers")
    from PIL import Image
    try:
        with Image.open(io.BytesIO(raw)) as image:
            if image.format != "JPEG" or image.size != (data["width"], data["height"]):
                raise ValueError("JPEG format/dimensions do not match metadata")
            image.verify()
        with Image.open(io.BytesIO(raw)) as image:
            image.load()  # Verify the compressed pixel stream, not only marker bytes.
    except (OSError, SyntaxError, ValueError) as exc:
        raise ValueError("invalid JPEG: " + str(exc)) from exc
    return raw


def analyze_file(input_path, output_dir):
    """Write summary.json and validated original JPEGs; return the summary object."""
    source, output = Path(input_path), Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    raw = source.read_bytes() if source.exists() else b""
    result = {
        "schema_version": 1, "status": "pending", "official_scoring": "unverified",
        "source_evidence": {"path": str(source), "exists": source.exists(), "bytes": len(raw),
                            "sha256": hashlib.sha256(raw).hexdigest() if source.exists() else None},
        "record_count": 0, "event_counts": {}, "missing_events": [], "event_presence": {},
        "launcher_starts": [], "launcher_exits": [], "probe_starts": [], "probe_finishes": [],
        "heartbeats": [], "samples": [], "sample_record_counts": {},
        "reported_sample_counts": {}, "sample_count_reports": [],
        "draw_order_payloads": [], "subscriptions": {}, "public_graphs": [],
        "snapshots": {"full_count": 0, "partial_count": 0, "incomplete_count": 0, "assembled": {}, "parts": []},
        "images": [], "diagnostic_errors": [], "parse_errors": [], "unknown_events": [], "unknown_fields": [],
    }
    counts = Counter()

    def unknown(line, scope, data, known):
        fields = {key: value for key, value in data.items() if key not in known}
        if fields:
            result["unknown_fields"].append({"line": line, "scope": scope, "fields": fields})

    def count_report(value, context):
        value = _mapping(value, "sample_counts")
        if any(type(count) is not int or count < 0 for count in value.values()):
            raise ValueError("sample_counts must contain nonnegative integers")
        result["reported_sample_counts"].update(value)
        result["sample_count_reports"].append({**context, "data": value})

    def sample(value, context):
        if not isinstance(value, dict) or not isinstance(value.get("topic"), str) or "message" not in value:
            raise ValueError("sample must contain topic and message")
        entry = {**context, "data": value}
        result["samples"].append(entry)
        if value["topic"] == "/referee/ground_tasks":
            result["draw_order_payloads"].append(entry)

    def section(name, value, context):
        assembled = result["snapshots"]["assembled"]
        if name not in SNAPSHOT_FIELDS:
            result["unknown_fields"].append({"line": context["line"], "scope": "probe_snapshot.section",
                                              "fields": {name: value}})
            return
        if name in MAP_SECTIONS:
            value = _mapping(value, name)
            if name == "samples":
                for item in value.values():
                    sample(item, context)
            elif name == "sample_counts":
                count_report(value, context)
            elif name == "subscriptions":
                result["subscriptions"].update(value)
            elif name == "errors":
                result["diagnostic_errors"].extend({**context, "data": {"topic": topic, "error": error}}
                                                    for topic, error in value.items())
            assembled.setdefault(name, {}).update(value)
        else:
            assembled[name] = value

    def snapshot(data, context):
        state = result["snapshots"]
        state["parts"].append({**context, "data": data})
        if "partial" in data and type(data["partial"]) is not bool:
            raise ValueError("snapshot partial flag must be boolean")
        partial = data.get("partial") is True
        if not partial:
            state["incomplete_count" if SNAPSHOT_FIELDS - data.keys() else "full_count"] += 1
            unknown(context["line"], "probe_snapshot", data, SNAPSHOT_FIELDS | {"partial"})
            for name, value in data.items():
                if name != "partial":
                    section(name, value, context)
            return
        state["partial_count"] += 1
        for key in ("read_only", "official_scoring"):
            if key in data:
                section(key, data[key], context)
        name = data.get("section")
        if not isinstance(name, str):
            raise ValueError("partial snapshot requires a section")
        if name == "metadata":
            unknown(context["line"], "probe_snapshot.metadata", data, SNAPSHOT_FIELDS | {"partial", "section"})
            for key, value in data.items():
                if key not in {"partial", "section"}:
                    section(key, value, context)
        elif name == "samples" and "topic" in data:
            unknown(context["line"], "probe_snapshot.fragment", data,
                    {"read_only", "official_scoring", "partial", "section", "topic", "sample"})
            if not isinstance(data["topic"], str) or "sample" not in data:
                raise ValueError("sample fragment requires topic and sample")
            if not isinstance(data["sample"], dict) or data["sample"].get("topic") != data["topic"]:
                raise ValueError("sample fragment topic does not match sample")
            section(name, {data["topic"]: data["sample"]}, context)
        else:
            unknown(context["line"], "probe_snapshot.fragment", data,
                    {"read_only", "official_scoring", "partial", "section", "topic", "value"})
            if "value" not in data:
                raise ValueError("snapshot section fragment requires value")
            value = data["value"]
            if "topic" in data:
                if not isinstance(data["topic"], str):
                    raise ValueError("snapshot fragment topic must be a string")
                value = {data["topic"]: value}
            section(name, value, context)

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
            unknown(line_number, "wrapper", wrapped, {"received_at", "payload", "source"})
            unknown(line_number, "payload", payload, {"event", "timestamp", "data"})
            if name == "probe_snapshot":
                snapshot(data, context)
                continue
            if name not in EVENT_FIELDS:
                result["unknown_events"].append({**context, "data": data})
                continue
            unknown(line_number, name, data, EVENT_FIELDS[name])
            destination = {"launcher_started": "launcher_starts", "launcher_exit": "launcher_exits",
                           "probe_started": "probe_starts", "probe_finished": "probe_finishes", "heartbeat": "heartbeats"}
            if name in destination:
                result[destination[name]].append({**context, "data": data})
            if name in {"sample_error", "subscription_error"}:
                result["diagnostic_errors"].append({**context, "data": data})
            elif name == "sample":
                sample(data, context)
            elif name == "subscription":
                if not isinstance(data.get("topic"), str):
                    raise ValueError("subscription requires topic")
                result["subscriptions"][data["topic"]] = data
            elif name == "public_graph":
                result["public_graphs"].append({**context, "data": data})
            elif name == "image_sample":
                image = _image_bytes(data)
                digest = hashlib.sha256(image).hexdigest()
                file_name = f"image_L{line_number:06d}_{digest[:16]}.jpg"
                (output / file_name).write_bytes(image)
                metadata = {key: data[key] for key in EVENT_FIELDS["image_sample"] if key != "jpeg_base64"}
                result["images"].append({**context, **metadata, "sha256": digest, "bytes": len(image), "file_name": file_name})
            if name in {"heartbeat", "probe_finished"} and "sample_counts" in data:
                count_report(data["sample_counts"], context)
        except (ValueError, TypeError, ImportError, OSError) as exc:
            result["parse_errors"].append({"line": line_number, "error": str(exc)})

    result["event_counts"] = dict(counts)
    result["sample_record_counts"] = dict(Counter(item["data"]["topic"] for item in result["samples"]))
    result["sample_topics"] = sorted(set(result["sample_record_counts"]) | {item["topic"] for item in result["images"]})
    result["launcher_exit_codes"] = [entry["data"]["exit_code"] for entry in result["launcher_exits"]
                                      if type(entry["data"].get("exit_code")) is int]
    result["startup"] = {"launcher_started": counts["launcher_started"] > 0, "probe_started": counts["probe_started"] > 0}
    result["missing_snapshot_fields"] = sorted(SNAPSHOT_FIELDS - result["snapshots"]["assembled"].keys())
    result["event_presence"] = {name: counts[name] > 0 for name in sorted(set(EVENT_FIELDS) | {"probe_snapshot"})}
    result["missing_events"] = [name for name in LIFECYCLE_EVENTS if counts[name] == 0]
    if result["parse_errors"]:
        result["status"] = "invalid"
    elif result["record_count"]:
        required = {"launcher_started", "probe_started", "probe_snapshot", "launcher_exit"}
        result["status"] = "collected" if all(counts[name] for name in required) and result["snapshots"]["full_count"] else "partial"
    result["notes"] = [
        "Only local diagnostic evidence is analyzed; no mission, coordinates, recognition acceptance or score is inferred.",
        "Missing callbacks do not prove missing ROS callbacks. Received order does not prove generation order.",
        "Reported counts may overlap individual samples and repeated snapshots; they are not added together.",
        "Partial fragments have no completeness manifest and remain partial evidence.",
    ]
    if result["status"] == "pending":
        result["notes"].append("Input is absent or contains no events; waiting for evidence.")
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return result


analyze_evidence = analyze_file


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
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
