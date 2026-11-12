"""Read-only signed-Z callback reconstruction; never drives or scores a robot."""
from __future__ import annotations
import argparse
import base64
import bisect
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import statistics

PART = "bounded_vertical_trim_gzip_part"
CORE = "bounded_vertical_trim_core"
META = ("session_id", "encoding", "uncompressed_bytes", "compressed_bytes",
        "uncompressed_sha256", "compressed_sha256", "part_count")


def summary(values):
    values = [x for x in values if x is not None and math.isfinite(x)]
    return {"count": len(values), "min": min(values), "median": statistics.median(values),
            "max": max(values)} if values else {"count": 0, "min": None, "median": None, "max": None}


def dump(path, obj):
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")


def csv_dump(path, rows):
    if not rows: return
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with path.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader(); w.writerows(rows)


def reconstruct(source, out, platform_id=None):
    raw_source = source.read_bytes()
    rows = [json.loads(s) for s in raw_source.decode("utf-8").splitlines() if s.strip()]
    parts = [r["payload"]["data"] for r in rows if r["payload"]["data"].get("section") == PART]
    assert parts, "No complete trim gzip evidence available; do not infer that ROS did not start"
    parts.sort(key=lambda p: p["part_index"])
    meta = {k: parts[0][k] for k in META}
    assert meta["encoding"] == "gzip+base64"
    assert all({k: p[k] for k in META} == meta for p in parts), "conflicting part metadata"
    assert [p["part_index"] for p in parts] == list(range(meta["part_count"])), "missing/duplicate gzip part"
    packed = base64.b64decode("".join(p["base64_part"] for p in parts), validate=True)
    assert len(packed) == meta["compressed_bytes"] and hashlib.sha256(packed).hexdigest() == meta["compressed_sha256"]
    raw = gzip.decompress(packed)
    assert len(raw) == meta["uncompressed_bytes"] and hashlib.sha256(raw).hexdigest() == meta["uncompressed_sha256"]
    body = json.loads(raw)
    assert body["session_id"] == meta["session_id"] and body["schema"] == "bounded_vertical_trim_v1"
    cores = [r["payload"]["data"] for r in rows if r["payload"]["data"].get("section") == CORE]
    assert len(cores) == 1, "must have exactly one matching core"
    core = cores[0]
    assert all(body[k] == v for k, v in core.items() if k in body), "core/recovered disagreement"
    out.mkdir(parents=True, exist_ok=True)
    # Preserve original reconstructed bytes, including full payloads and floats.
    (out/"evidence.json.gz").write_bytes(packed)
    (out/"evidence.json").write_bytes(raw)
    dump(out/"events_without_parts.json", [r for r in rows if r["payload"]["data"].get("section") != PART])
    validation = {"source": str(source), "source_sha256": hashlib.sha256(raw_source).hexdigest(),
        "source_bytes": len(raw_source), "event_count": len(rows), "parts_verified": len(parts),
        "metadata": meta, "compressed_length_and_sha_match": True, "raw_length_and_sha_match": True,
        "shared_core_fields_match": True, "dropped": body["dropped"],
        "sample_counts": {k: len(body[k]) for k in ("trajectory", "commands", "scans", "downscans", "tf_records")},
        "platform_id": platform_id, "callback_source": "unverified_callback",
        "source_authenticated_independently": False}
    dump(out/"validation.json", validation)
    return body, validation


def analyze(body, out, validation):
    traj, cmds = body["trajectory"], body["commands"]
    assert all(a["stamp"] < b["stamp"] for a, b in zip(traj, traj[1:])), "nonadvancing odom"
    assert all(a["wall_elapsed_s"] <= b["wall_elapsed_s"] for a, b in zip(cmds, cmds[1:])), "reversed command wall"
    wall_origin = traj[0]["wall"]-traj[0]["wall_elapsed_s"] if traj else None
    motion = (body["motion_started_wall"]-wall_origin
        if body["motion_started_wall"] is not None and wall_origin is not None else None)
    origin = body["origin"]
    start = max((i for i, x in enumerate(traj) if motion is not None and x["wall_elapsed_s"] <= motion), default=None)
    if start is not None: assert math.dist(traj[start]["xyz"], origin) < 1e-9
    cumulative = 0.
    trajectory_rows = []
    cmd_times = [x["wall_elapsed_s"] for x in cmds]
    trajectory_times = [x["wall_elapsed_s"] for x in traj]
    for i, x in enumerate(traj):
        prev = traj[i-1] if i else None
        if start is not None and i > start: cumulative += math.dist(prev["xyz"], x["xyz"])
        cmd_idx = bisect.bisect_right(cmd_times, x["wall_elapsed_s"])-1
        cmd = cmds[cmd_idx] if cmd_idx >= 0 else {}
        dt_source = x["stamp"]-prev["stamp"] if prev else None
        dt_wall = x["wall_elapsed_s"]-prev["wall_elapsed_s"] if prev else None
        dz = x["xyz"][2]-prev["xyz"][2] if prev else None
        trajectory_rows.append({"index": i, "wall_elapsed_s": x["wall_elapsed_s"], "odom_stamp_s": x["stamp"],
            "x_m": x["xyz"][0], "y_m": x["xyz"][1], "z_m": x["xyz"][2],
            "relative_height_m": x["xyz"][2]-origin[2] if origin else None,
            "vz_feedback_mps": x["linear"][2], "speed_mps": x["speed"], "tilt_rad": x["tilt"],
            "vz_finite_difference_source_mps": dz/dt_source if dt_source else None,
            "vz_finite_difference_wall_mps": dz/dt_wall if dt_wall else None,
            "path_since_origin_m": cumulative, "prior_arrival_cmd_z": cmd.get("linear_z"),
            "prior_arrival_cmd_phase": cmd.get("phase"), "prior_arrival_cmd_state": cmd.get("state")})
    csv_dump(out/"trajectory.csv", trajectory_rows)
    csv_dump(out/"commands.csv", cmds)
    phase_metrics = []
    transitions = body["phase_transitions"]
    for j, t in enumerate(transitions):
        end = transitions[j+1]["wall_elapsed_s"] if j+1 < len(transitions) else (cmd_times[-1] if cmds else t["wall_elapsed_s"])
        start_t = t["wall_elapsed_s"]
        samples = [x for x in trajectory_rows if start_t <= x["wall_elapsed_s"] < end]
        nonzero = [x for x in cmds if x.get("phase") == t["to"] and x["linear_z"]]
        phase_metrics.append({"phase": t["to"], "entry": t, "end_wall_elapsed_s": end,
            "phase_wall_span_s": end-start_t, "samples": len(samples), "actual_nonzero_requests": len(nonzero),
            "request_values": sorted(set(x["linear_z"] for x in nonzero)),
            "first_nonzero_request": nonzero[0] if nonzero else None,
            "last_nonzero_request": nonzero[-1] if nonzero else None,
            "relative_height_m": summary(x["relative_height_m"] for x in samples),
            "vz_feedback_mps": summary(x["vz_feedback_mps"] for x in samples),
            "vz_by_source_stamp_mps": summary(x["vz_finite_difference_source_mps"] for x in samples),
            "vz_by_arrival_wall_mps": summary(x["vz_finite_difference_wall_mps"] for x in samples),
            "controller_reported_phase_samples": body["phase_samples"].get(t["to"])})
    budget_violations = [x for x in cmds if motion is not None and x["linear_z"] and
        x["wall_elapsed_s"] >= motion+body["limits"]["motion_wall_s"]-1e-9]
    low_height_requests = []
    for c in cmds:
        if not c["linear_z"] or not c.get("phase", "").startswith("TRIM_"): continue
        idx = bisect.bisect_right(trajectory_times, c["wall_elapsed_s"])-1
        if idx >= 0 and origin and traj[idx]["xyz"][2]-origin[2] <= .035+1e-9:
            low_height_requests.append({"command": c, "prior_arrival_odom": traj[idx]})
    if start is not None:
        assert abs(cumulative-body["total_path_m"]) < 1e-9, "cumulative path differs from controller"
    result = {"kind": "read_only_trim_response_analysis", "platform_id": validation["platform_id"],
        "state": body["state"], "reason": body["reason"], "official_score": body["official_score"],
        "full_mission_complete": body["full_mission_complete"], "protocol_complete": body["protocol_complete"],
        "protocol_abort_reason": body["protocol_abort_reason"], "support_return_verified": body["support_return_verified"],
        "origin": origin, "motion_start_elapsed_s": motion, "permanent_motion_budget_s": body["limits"]["motion_wall_s"],
        "nonzero_requests_past_permanent_budget": budget_violations,
        "nonzero_trim_requests_with_prior_arrival_height_at_or_below_035": low_height_requests,
        "reconstructed_path_m": cumulative, "reported_path_m": body["total_path_m"],
        "rise_m": summary(x["relative_height_m"] for x in trajectory_rows),
        "max_command_gap_s": max((b-a for a, b in zip(cmd_times, cmd_times[1:])), default=None),
        "max_odom_source_gap_s": max((b["stamp"]-a["stamp"] for a, b in zip(traj, traj[1:])), default=None),
        "max_odom_arrival_gap_s": max((b["wall_elapsed_s"]-a["wall_elapsed_s"] for a, b in zip(traj, traj[1:])), default=None),
        "phase_metrics": phase_metrics, "last_odom": traj[-1] if traj else None,
        "command_counts": {"positive": sum(x["linear_z"] > 0 for x in cmds),
            "negative": sum(x["linear_z"] < 0 for x in cmds), "zero": sum(x["linear_z"] == 0 for x in cmds)},
        "interpretation_limits": ["No official score is expected from this independent identification run.",
            "Commands carry process wall time, not actuator application timestamps; prior-arrival association is descriptive.",
            "Source-stamp finite differences and arrival-wall differences are separate; latter includes delivery jitter.",
            "No linear global response model, hover PID or controlled landing is established by these measurements.",
            "A missing final payload alone cannot establish whether user code started."]}
    dump(out/"metrics.json", result)
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, default=Path("runs/evidence/20261112/callback_trim_v29.jsonl"))
    p.add_argument("--out", type=Path, default=Path(__file__).resolve().parent)
    p.add_argument("--platform-id")
    args = p.parse_args()
    body, validation = reconstruct(args.source, args.out, args.platform_id)
    result = analyze(body, args.out, validation)
    print(json.dumps({"parts": validation["parts_verified"], "counts": validation["sample_counts"],
        "state": result["state"], "reason": result["reason"], "protocol_complete": result["protocol_complete"],
        "official_score": result["official_score"], "command_counts": result["command_counts"]}, ensure_ascii=False))
