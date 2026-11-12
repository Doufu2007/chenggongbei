"""Offline, evidence-bound run ledger. No network, ROS, or account access."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import re
import statistics
import sys
from collections import Counter, defaultdict
from pathlib import Path


SCHEMA_VERSION = 1
ITEM_MAXIMA = {
    "ground_1": 10, "ground_2": 10, "ground_3": 10,
    "window_1": 10, "aerial_1": 15, "aerial_2": 15,
    "window_2": 10, "landing": 10,
}
GROUND_POINT_IDS = frozenset(f"ground_{index}" for index in range(5))
KINDS = {"official_task", "interface_probe", "offline"}
PENDING = {"submitted", "queued", "running"}
SETTLED = {"completed", "success"}
STATUSES = PENDING | SETTLED | {"failed", "timeout", "cancelled", "unknown"}
FIELDS = {
    "schema_version", "record_id", "runId", "kind", "evaluation_mode",
    "condition_id", "variant", "status", "full_mission_completed", "source",
    "script", "draw_order", "scores", "duration_s", "failure_reason", "evidence_paths",
}
_SECRET = re.compile(r"(?:password|passwd|access_token|refresh_token|authorization|cookie|secret|token)\s*[:=]", re.I)


def _keys(value, required, label):
    if not isinstance(value, dict) or set(value) != set(required):
        raise ValueError(f"{label}: expected fields {', '.join(sorted(required))}")


def _string(value, label, *, nullable=False):
    if nullable and value is None:
        return
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{label}: expected nonempty string" + (" or null" if nullable else ""))
    if _SECRET.search(value):
        raise ValueError(f"{label}: credential-like content is forbidden")


def _path(value, label):
    _string(value, label)
    if "://" in value or "?" in value or "\n" in value or "\r" in value:
        raise ValueError(f"{label}: use a local evidence/artifact path, without URL credentials")


def _number(value, label, maximum=None):
    if value is None:
        return
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value < 0
            or (maximum is not None and value > maximum)):
        raise ValueError(f"{label}: expected finite number in [0, {maximum or 'infinity'}] or null")


def validate_record(record):
    """Return a validated copy, preserving every unknown value as null.

    This validates declarations, not the authenticity of manually entered evidence.
    Use --verify-files to check local artifact hashes and evidence availability.
    """
    _keys(record, FIELDS, "record")
    if type(record["schema_version"]) is not int or record["schema_version"] != SCHEMA_VERSION:
        raise ValueError("schema_version: only version 1 is supported")
    for field in ("record_id", "kind", "status"):
        _string(record[field], field)
    for field in ("runId", "condition_id", "variant", "failure_reason"):
        _string(record[field], field, nullable=True)
    if record["kind"] not in KINDS:
        raise ValueError("kind: expected official_task, interface_probe, or offline")
    if record["status"] not in STATUSES:
        raise ValueError("status: unsupported platform status")
    modes = {None} if record["kind"] == "offline" else {"practice", "competition"}
    if not any(record["evaluation_mode"] == mode for mode in modes):
        raise ValueError("evaluation_mode: platform records require practice/competition; offline requires null")
    if record["full_mission_completed"] is not None and type(record["full_mission_completed"]) is not bool:
        raise ValueError("full_mission_completed: expected boolean or null")
    if record["full_mission_completed"] is True and record["kind"] != "official_task":
        raise ValueError("full_mission_completed: only official_task may assert task completion")
    for field in ("source", "script"):
        artifact = record[field]
        _keys(artifact, {"path", "sha256"}, field)
        if artifact["path"] is not None:
            _path(artifact["path"], f"{field}.path")
        digest = artifact["sha256"]
        if digest is not None and (not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest)):
            raise ValueError(f"{field}.sha256: expected lowercase SHA-256 or null")
    if record["draw_order"] is not None:
        if not isinstance(record["draw_order"], list) or not record["draw_order"]:
            raise ValueError("draw_order: expected nonempty list or null")
        for item in record["draw_order"]:
            _string(item, "draw_order item")
    scores = record["scores"]
    score_fields = {"items", "total", "manual"}
    if isinstance(scores, dict) and "ground_points" in scores:
        score_fields.add("ground_points")
    _keys(scores, score_fields, "scores")
    _keys(scores["items"], ITEM_MAXIMA, "scores.items")
    for name, maximum in ITEM_MAXIMA.items():
        _number(scores["items"][name], f"scores.items.{name}", maximum)
    _number(scores["total"], "scores.total", 90)
    _number(scores["manual"], "scores.manual", 30)
    if "ground_points" in scores:
        physical = scores["ground_points"]
        if not isinstance(physical, dict) or not set(physical) <= GROUND_POINT_IDS:
            raise ValueError("scores.ground_points: expected only physical ground_0 through ground_4 IDs")
        for name, value in physical.items():
            _number(value, f"scores.ground_points.{name}", 10)
        # A second view of the same ground score, never an additional score sum.
        if scores["total"] is not None and sum(value for value in physical.values() if value is not None) > scores["total"] + 1e-8:
            raise ValueError("scores.total: less than known physical ground point scores")
    known = [v for v in scores["items"].values() if v is not None]
    if scores["total"] is not None:
        if sum(known) > scores["total"] + 1e-8:
            raise ValueError("scores.total: less than the sum of known item scores")
        if len(known) == len(ITEM_MAXIMA) and not math.isclose(sum(known), scores["total"], abs_tol=1e-8):
            raise ValueError("scores.total: does not match the sum of item scores")
        remaining_maximum = sum(maximum for name, maximum in ITEM_MAXIMA.items()
                                if scores["items"][name] is None)
        if scores["total"] > sum(known) + remaining_maximum + 1e-8:
            raise ValueError("scores.total: exceeds the sum allowed by the partial item scores")
    _number(record["duration_s"], "duration_s")
    if not isinstance(record["evidence_paths"], list):
        raise ValueError("evidence_paths: expected list of local paths")
    for path in record["evidence_paths"]:
        _path(path, "evidence_paths item")
    return copy.deepcopy(record)


def hash_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify_record_files(record, base_dir):
    """Check evidence files and any available artifact/hash pair, without executing files."""
    record = validate_record(record)
    base_dir = Path(base_dir)
    for raw_path in record["evidence_paths"]:
        if not (base_dir / raw_path).is_file():
            raise ValueError("evidence_paths: referenced evidence file is unavailable")
    for name in ("source", "script"):
        artifact = record[name]
        if artifact["path"] is not None:
            path = base_dir / artifact["path"]
            if not path.is_file():
                raise ValueError(f"{name}.path: artifact is unavailable")
            if artifact["sha256"] is not None and hash_file(path) != artifact["sha256"]:
                raise ValueError(f"{name}.sha256: artifact hash mismatch")


def _deduplicate(records):
    seen = {}
    record_ids = {}
    duplicates = 0
    for raw in records:
        record = validate_record(raw)
        key = ("runId", record["runId"]) if record["runId"] else ("record_id", record["record_id"])
        if record["record_id"] in record_ids and record_ids[record["record_id"]] != key:
            raise ValueError("conflicting record_id refers to different runs")
        record_ids[record["record_id"]] = key
        if key in seen:
            prior = {k: v for k, v in seen[key].items() if k != "record_id"}
            current = {k: v for k, v in record.items() if k != "record_id"}
            if prior != current:
                raise ValueError(f"conflicting {key[0]} records; reconcile evidence before analysis")
            duplicates += 1
        else:
            seen[key] = record
    return list(seen.values()), duplicates


def _settled(record):
    return (record["status"] in SETTLED and record["scores"]["total"] is not None
            and record["runId"] is not None and bool(record["evidence_paths"]))


def _completed(record):
    return _settled(record) and record["full_mission_completed"] is True


def _distribution(values, missing=0):
    values = sorted(values)
    return {"n": len(values), "missing": missing,
            "min": values[0] if values else None,
            "median": statistics.median(values) if values else None,
            "p95": values[math.ceil(0.95 * len(values)) - 1] if values else None,
            "max": values[-1] if values else None}


def _failure_reason(record):
    if not record["runId"]:
        return "missing_run_id"
    if record["failure_reason"]:
        return record["failure_reason"]
    if record["scores"]["total"] is None:
        return "official_score_missing"
    if not record["evidence_paths"]:
        return "evidence_missing"
    if record["status"] not in SETTLED:
        return "platform_status_" + record["status"]
    if record["full_mission_completed"] is None:
        return "full_mission_completion_unverified"
    return "full_mission_incomplete"


def summarize_records(records):
    records, duplicates = _deduplicate(records)
    excluded = {kind: sum(r["kind"] == kind for r in records)
                for kind in ("interface_probe", "offline")}
    tasks = [r for r in records if r["kind"] == "official_task"]
    attempts = [r for r in tasks if r["status"] not in PENDING]
    settled = [r for r in attempts if _settled(r)]
    complete = [r for r in attempts if _completed(r)]
    durations = [r["duration_s"] for r in settled if r["duration_s"] is not None]
    manuals = [r["scores"]["manual"] for r in attempts if r["scores"]["manual"] is not None]
    reasons = Counter(_failure_reason(r) for r in attempts if not _completed(r))
    return {
        "schema_version": SCHEMA_VERSION,
        "record_count": len(records), "duplicate_count": duplicates,
        "task_attempts": len(attempts), "pending_tasks": len(tasks) - len(attempts),
        "valid_settled_runs": len(settled), "completed_runs": len(complete),
        "completion_rate": len(complete) / len(attempts) if attempts else None,
        "unknown_completion": sum(r["full_mission_completed"] is None for r in attempts),
        "missing_run_id": sum(not r["runId"] for r in attempts),
        "minimum_valid_total": min((r["scores"]["total"] for r in settled), default=None),
        "valid_duration_s": _distribution(durations, len(settled) - len(durations)),
        "manual_scores": {"known_count": len(manuals), "minimum": min(manuals, default=None),
                          "maximum": max(manuals, default=None)},
        "excluded_counts": excluded,
        "evaluation_modes": dict(sorted(Counter(r["evaluation_mode"] for r in attempts).items())),
        "failure_reasons": dict(sorted(reasons.items())),
        "missing_artifact_hashes": {name: sum(r[name]["sha256"] is None for r in attempts)
                                    for name in ("source", "script")},
        "data_status": "no_task_attempts" if not attempts else (
            "insufficient_attempts" if len(attempts) < 2 else "descriptive_only"),
        "note": "Completion requires explicit full-mission evidence and a valid platform settlement. "
                "Unknown scores remain null; summary does not authenticate manually entered evidence.",
    }


def compare_records(records, baseline, candidate):
    if baseline == candidate:
        raise ValueError("baseline and candidate must be different variants")
    records, duplicates = _deduplicate(records)
    groups = defaultdict(lambda: defaultdict(list))
    missing_condition = 0
    for record in records:
        if (record["kind"] != "official_task" or record["status"] in PENDING
                or record["variant"] not in {baseline, candidate}):
            continue
        if record["condition_id"] is None:
            missing_condition += 1
        else:
            groups[record["condition_id"]][record["variant"]].append(record)
    pairs, unmatched, ambiguous, incompatible = [], [], [], []
    for condition, variants in sorted(groups.items()):
        if any(len(items) > 1 for items in variants.values()):
            ambiguous.append(condition)
            continue
        if baseline not in variants or candidate not in variants:
            unmatched.append(condition)
            continue
        a, b = variants[baseline][0], variants[candidate][0]
        if (a["evaluation_mode"] != b["evaluation_mode"]
                or (a["draw_order"] is not None and b["draw_order"] is not None
                    and a["draw_order"] != b["draw_order"])):
            incompatible.append(condition)
            continue
        score_delta = b["scores"]["total"] - a["scores"]["total"] if _settled(a) and _settled(b) else None
        duration_delta = (b["duration_s"] - a["duration_s"] if _settled(a) and _settled(b)
                          and a["duration_s"] is not None and b["duration_s"] is not None else None)
        pairs.append({"condition_id": condition, "baseline_runId": a["runId"],
                      "candidate_runId": b["runId"], "score_delta": score_delta,
                      "duration_delta_s": duration_delta,
                      "baseline_completed": _completed(a), "candidate_completed": _completed(b)})
    scores = [p["score_delta"] for p in pairs if p["score_delta"] is not None]
    durations = [p["duration_delta_s"] for p in pairs if p["duration_delta_s"] is not None]
    return {"schema_version": SCHEMA_VERSION, "baseline": baseline, "candidate": candidate,
            "paired_conditions": len(pairs), "score_pairs": len(scores),
            "mean_score_delta": statistics.mean(scores) if scores else None,
            "duration_pairs": len(durations),
            "mean_duration_delta_s": statistics.mean(durations) if durations else None,
            "pairs": pairs, "unmatched_conditions": unmatched,
            "ambiguous_conditions": ambiguous, "incompatible_conditions": incompatible,
            "missing_condition_records": missing_condition, "duplicate_count": duplicates,
            "data_status": "insufficient_pairs" if len(pairs) < 2 or len(scores) < 2 else "descriptive_only",
            "note": "Deltas are candidate minus baseline. No significance or superiority claim is inferred. "
                    "Use one condition_id per matched trial; repeated variants are not paired arbitrarily."}


def import_probe(path, *, evidence_path=None):
    """Explicitly import a platform interface probe; never infer a full task from a log."""
    path = Path(path)
    raw = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(raw, dict) or not isinstance(raw.get("runId"), str) or not raw["runId"].strip():
        raise ValueError("probe import requires the original platform runId")
    duration = raw.get("durationMs")
    _number(duration, "durationMs")
    score = raw.get("score")
    _number(score, "scores.total", 90)
    record = {
        "schema_version": SCHEMA_VERSION, "record_id": "probe-" + raw["runId"],
        "runId": raw["runId"], "kind": "interface_probe", "evaluation_mode": "practice",
        "condition_id": None, "variant": "interface_probe", "status": raw.get("status", "unknown"),
        "full_mission_completed": None,
        "source": {"path": None, "sha256": None}, "script": {"path": None, "sha256": None},
        "draw_order": None, "scores": {"items": dict.fromkeys(ITEM_MAXIMA), "total": score, "manual": None},
        "duration_s": duration / 1000 if duration is not None else None,
        "failure_reason": raw.get("failedReason"),
        "evidence_paths": [str(evidence_path if evidence_path is not None else path)],
    }
    return validate_record(record)


def load_records(paths):
    """Load one record or an array per JSON file; reject malformed inputs atomically."""
    files = []
    for raw in paths:
        path = Path(raw)
        if not path.exists():
            raise ValueError("input path does not exist")
        files.extend(sorted(path.rglob("*.json")) if path.is_dir() else [path])
    records = []
    for path in files:
        try:
            raw = json.loads(path.read_text(encoding="utf-8-sig"))
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid JSON at line {exc.lineno}, column {exc.colno}") from exc
        records.extend(raw if isinstance(raw, list) else [raw])
    return records


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("validate", "summary", "compare"):
        command = commands.add_parser(name)
        command.add_argument("paths", nargs="+", help="record JSON file(s) or dedicated ledger directory")
        if name == "validate":
            command.add_argument("--verify-files", action="store_true")
            command.add_argument("--base-dir", type=Path, default=Path.cwd(), help="base for stored paths; default current directory")
        if name == "compare":
            command.add_argument("--baseline", required=True)
            command.add_argument("--candidate", required=True)
    commands.add_parser("hash").add_argument("path", type=Path)
    probe = commands.add_parser("import-probe")
    probe.add_argument("path", type=Path)
    probe.add_argument("--evidence-path", help="stored local path to original evidence")
    probe.add_argument("--output", type=Path, help="new file; existing files are never overwritten")
    args = parser.parse_args(argv)
    try:
        if args.command == "hash":
            result = {"path": str(args.path), "sha256": hash_file(args.path)}
        elif args.command == "import-probe":
            result = import_probe(args.path, evidence_path=args.evidence_path)
            if args.output:
                with args.output.open("x", encoding="utf-8") as handle:
                    json.dump(result, handle, ensure_ascii=False, indent=2, allow_nan=False)
                    handle.write("\n")
        else:
            records = load_records(args.paths)
            if args.command == "summary":
                result = summarize_records(records)
            elif args.command == "compare":
                result = compare_records(records, args.baseline, args.candidate)
            else:
                unique, duplicates = _deduplicate(records)
                if args.verify_files:
                    for record in unique:
                        verify_record_files(record, args.base_dir)
                result = {"schema_version": SCHEMA_VERSION, "valid": True,
                          "record_count": len(unique), "duplicate_count": duplicates,
                          "files_verified": args.verify_files}
        print(json.dumps(result, ensure_ascii=True, indent=2, allow_nan=False))
        return 0
    except OSError as exc:
        # Local OS error text may contain paths or a non-UTF-8 localized message.
        print(f"run_records: local file operation failed (errno={exc.errno})", file=sys.stderr)
        return 2
    except (ValueError, UnicodeError) as exc:
        # Exception messages never include raw input JSON or account payloads.
        print(f"run_records: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
