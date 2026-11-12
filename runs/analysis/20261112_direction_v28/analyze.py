"""Read-only v28 callback integrity and measured XY-response reconstruction.

No platform calls, ROS imports, control commands, or fabricated observations.
The exact recovered JSON and gzip are preserved before numerical derivation.
"""
from __future__ import annotations

import argparse
import base64
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
import zlib


OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
DEFAULT_SOURCE = ROOT / "runs/evidence/20261112/callback_direction_v28.jsonl"
MODE = "ground_direction_response"
SECTION = "ground_direction_evidence_gzip_part"
CORE = "ground_direction_core"
METADATA = ("mode", "section", "encoding", "session_id", "bundle",
            "uncompressed_sha256", "compressed_sha256", "uncompressed_bytes",
            "compressed_bytes", "part_count")
MAX_RAW, MAX_PACKED = 32 * 1024 * 1024, 8 * 1024 * 1024


def strict_json(raw):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("duplicate JSON key: " + key)
            result[key] = value
        return result
    def invalid(value):
        raise ValueError("nonfinite JSON: " + value)
    def finite_float(value):
        result = float(value)
        if not math.isfinite(result):
            raise ValueError("nonfinite JSON float")
        return result
    return json.loads(raw, object_pairs_hook=unique,
                      parse_constant=invalid, parse_float=finite_float)


def decode_parts(parts):
    if not parts:
        raise ValueError("direction bundle not yet received")
    first = parts[0]
    if any(key not in first for key in METADATA):
        raise ValueError("incomplete bundle metadata")
    if (first["mode"] != MODE or first["section"] != SECTION
            or first["encoding"] != "gzip+base64"
            or first["bundle"] != "ground_direction_response_v1"
            or not isinstance(first["session_id"], str)
            or not 1 <= len(first["session_id"]) <= 128):
        raise ValueError("bundle identity mismatch")
    for key, cap in (("uncompressed_bytes", MAX_RAW),
                     ("compressed_bytes", MAX_PACKED), ("part_count", 512)):
        if type(first[key]) is not int or not 1 <= first[key] <= cap:
            raise ValueError("bundle size/count exceeds bound: " + key)
    for key in ("uncompressed_sha256", "compressed_sha256"):
        if not isinstance(first[key], str) or not re.fullmatch("[0-9a-f]{64}", first[key]):
            raise ValueError("invalid SHA256 metadata")
    chunks, duplicate_parts = {}, 0
    for part in parts:
        if any(part.get(key) != first[key] for key in METADATA):
            raise ValueError("conflicting sessions or bundle metadata")
        index, encoded = part.get("part_index"), part.get("base64_part")
        if (type(index) is not int or not 0 <= index < first["part_count"]
                or not isinstance(encoded, str) or not 0 < len(encoded) <= 48000
                or not re.fullmatch("[A-Za-z0-9+/=]+", encoded)):
            raise ValueError("invalid part index/base64 size")
        if index in chunks:
            if chunks[index] != encoded:
                raise ValueError("conflicting duplicate part")
            duplicate_parts += 1
        chunks[index] = encoded
    if len(chunks) != first["part_count"]:
        raise ValueError(f"bundle incomplete: {len(chunks)}/{first['part_count']} parts")
    encoded = "".join(chunks[i] for i in range(first["part_count"]))
    if len(encoded) > 4 * ((MAX_PACKED + 2) // 3):
        raise ValueError("base64 exceeds bound")
    compressed = base64.b64decode(encoded, validate=True)
    if len(compressed) != first["compressed_bytes"]:
        raise ValueError("compressed length mismatch")
    if hashlib.sha256(compressed).hexdigest() != first["compressed_sha256"]:
        raise ValueError("compressed SHA256 mismatch")
    decoder = zlib.decompressobj(16 + zlib.MAX_WBITS)
    raw = decoder.decompress(compressed, first["uncompressed_bytes"] + 1)
    if (not decoder.eof or decoder.unused_data or decoder.unconsumed_tail
            or len(raw) != first["uncompressed_bytes"]):
        raise ValueError("gzip stream or raw length mismatch")
    if hashlib.sha256(raw).hexdigest() != first["uncompressed_sha256"]:
        raise ValueError("raw SHA256 mismatch")
    body = strict_json(raw.decode("utf-8"))
    if (body.get("schema") != "ground_direction_response_evidence_v1"
            or body.get("mode") != MODE or body.get("session_id") != first["session_id"]):
        raise ValueError("decoded schema/session mismatch")
    for key in ("raw_odometry", "raw_scans", "commands", "state_records",
                "zero_requests", "pulse_records"):
        if not isinstance(body.get(key), list):
            raise ValueError("missing evidence list: " + key)
    return body, compressed, raw, {**{k: first[k] for k in METADATA},
                                  "duplicate_parts": duplicate_parts}


def stamp(payload):
    value = payload["header"]["stamp"]
    sec, nano = value["sec"], value["nanosec"]
    if type(sec) is not int or type(nano) is not int or sec < 0 or not 0 <= nano < 10**9:
        raise ValueError("invalid source timestamp")
    return sec + nano / 1e9


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def odom(payload):
    pose, twist = payload["pose"]["pose"], payload["twist"]["twist"]
    xyz = [pose["position"][k] for k in ("x", "y", "z")]
    q = [pose["orientation"][k] for k in ("x", "y", "z", "w")]
    linear = [twist["linear"][k] for k in ("x", "y", "z")]
    angular = [twist["angular"][k] for k in ("x", "y", "z")]
    if not all(math.isfinite(v) for v in xyz + q + linear + angular):
        raise ValueError("nonfinite odometry")
    norm = math.hypot(*q)
    if norm == 0:
        raise ValueError("zero quaternion")
    x, y, z, w = [v / norm for v in q]
    return {"source_stamp_s": stamp(payload), "xyz_m": xyz, "raw_quaternion": q,
            "quaternion_norm": norm, "yaw_rad": math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)),
            "linear_feedback_mps": linear, "angular_feedback_rps": angular,
            "speed_feedback_mps": math.hypot(*linear),
            "angular_speed_feedback_rps": math.hypot(*angular)}


def response(start, end):
    a, b = odom(start), odom(end)
    dt = b["source_stamp_s"] - a["source_stamp_s"]
    if dt <= 0:
        return {"available": False, "reason": "no distinct increasing source poses"}
    dx, dy, dz = [b["xyz_m"][i] - a["xyz_m"][i] for i in range(3)]
    yaw = a["yaw_rad"]
    bearing = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-12 else None
    return {"available": True, "source_dt_s": dt, "delta_world_m": [dx, dy, dz],
            "mean_world_velocity_mps": [dx/dt, dy/dt, dz/dt],
            "delta_start_body_xy_m": [dx*math.cos(yaw)+dy*math.sin(yaw),
                                      -dx*math.sin(yaw)+dy*math.cos(yaw)],
            "horizontal_displacement_m": math.hypot(dx, dy),
            "start_yaw_rad": yaw, "yaw_delta_rad": wrap(b["yaw_rad"] - yaw),
            "displacement_bearing_rad": bearing,
            "bearing_minus_start_yaw_rad": None if bearing is None else wrap(bearing-yaw),
            "interface_semantics_confirmed": False}


def extrema(values):
    values = list(values)
    return None if not values else {"min": min(values), "median": statistics.median(values),
                                    "max": max(values), "count": len(values)}


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                    encoding="utf-8")


def csv_rows(path, rows):
    if not rows:
        return
    keys = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows({k: json.dumps(v, separators=(",", ":")) if isinstance(v, (dict, list))
                         else v for k, v in row.items()} for row in rows)


def analyze(body):
    trajectory = [{"wall_elapsed_s": r["wall_elapsed_s"], "phase_before_callback": r["phase"],
                   "state_before_callback": r["state"], "pulse_index": r["pulse_index"],
                   **odom(r["payload"])} for r in body["raw_odometry"]]
    source_times = [r["source_stamp_s"] for r in trajectory]
    arrival_times = [r["wall_elapsed_s"] for r in trajectory]
    commands = body["commands"]
    finite_differences = []
    for a, b in zip(trajectory, trajectory[1:]):
        dt = b["source_stamp_s"] - a["source_stamp_s"]
        if dt <= 0:
            continue
        delta = [(b["xyz_m"][i] - a["xyz_m"][i]) / dt for i in range(3)]
        finite_differences.append({"from_source_stamp_s": a["source_stamp_s"],
            "to_source_stamp_s": b["source_stamp_s"], "to_wall_elapsed_s": b["wall_elapsed_s"],
            "delta_source_s": dt, "world_velocity_mps": delta,
            "speed_mps": math.hypot(*delta), "horizontal_speed_mps": math.hypot(*delta[:2]),
            "yaw_velocity_rps": wrap(b["yaw_rad"] - a["yaw_rad"]) / dt})
    pulses = []
    for record in body["pulse_records"]:
        index = record["index"]
        start_raw = record["start"]["raw_odom"]
        start_stamp = stamp(start_raw)
        next_record = next((r for r in body["pulse_records"] if r["index"] == index + 1), None)
        end_stamp = (stamp(record["settled"]["raw_odom"]) if "settled" in record else
                     stamp(next_record["start"]["raw_odom"]) if next_record else
                     source_times[-1] if source_times else start_stamp)
        samples = [r for r in trajectory if start_stamp <= r["source_stamp_s"] <= end_stamp]
        end_raw = (record["settled"]["raw_odom"] if "settled" in record else
                   next_record["start"]["raw_odom"] if next_record else
                   body["raw_odometry"][-1]["payload"])
        pulse_commands = [r for r in commands if r["pulse_index"] == index]
        nonzero = [r for r in pulse_commands if r["sent"] and (r["linear_x"] or r["linear_y"])]
        after_nonzero = [r for r in pulse_commands if r["sent"] and
                         r["linear_x"] == r["linear_y"] == r["angular_z"] == 0 and
                         nonzero and r["wall_elapsed_s"] >= nonzero[-1]["wall_elapsed_s"]]
        measured = response(start_raw, end_raw)
        expected = record["requested_body_xy_mps"]
        expected_bearing = wrap(odom(start_raw)["yaw_rad"] + math.atan2(expected[1], expected[0]))
        request_zeros = []
        for zero in record["zero_requests"]:
            anchor_raw = zero.get("raw_odom")
            if anchor_raw is None:
                request_zeros.append({"zero_request": zero, "available": False})
                continue
            anchor = odom(anchor_raw)
            after = [r for r in samples if r["source_stamp_s"] >= anchor["source_stamp_s"]]
            positions = [anchor["xyz_m"]] + [r["xyz_m"] for r in after
                                              if r["source_stamp_s"] > anchor["source_stamp_s"]]
            path = sum(math.dist(a, b) for a, b in zip(positions, positions[1:]))
            request_zeros.append({"zero_request": zero, "available": bool(after),
                "observed_until_source_stamp_s": after[-1]["source_stamp_s"] if after else None,
                "cumulative_3d_path_after_request_m": path,
                "net_displacement_after_request_m": math.dist(positions[0], positions[-1]),
                "maximum_displacement_after_request_m": max(math.dist(positions[0], p) for p in positions),
                "observed_until_settled": "settled" in record})
        measured_path = sum(math.dist(a["xyz_m"], b["xyz_m"]) for a, b in zip(samples, samples[1:]))
        active_fd = [r for r in finite_differences if nonzero and
                     nonzero[0]["wall_elapsed_s"] <= r["to_wall_elapsed_s"] <= nonzero[-1]["wall_elapsed_s"]]
        pulse = {"index": index, "settled_record_present": "settled" in record,
            "motion_end_reason": record.get("motion_end_reason"),
            "requested_body_xy_mps": expected, "start": odom(start_raw), "end": odom(end_raw),
            "request_count": len(nonzero), "failed_publish_count": sum(not r["sent"] for r in pulse_commands),
            "first_nonzero_command": nonzero[0] if nonzero else None,
            "last_nonzero_command": nonzero[-1] if nonzero else None,
            "first_successful_zero_after_last_nonzero": after_nonzero[0] if after_nonzero else None,
            "reconstructed_response": measured, "reported_response": record.get("response"),
            "reported_response_matches": None if "response" not in record else record["response"] == {
                k: v for k, v in measured.items() if k != "available"},
            "requested_body_interpretation_world_bearing_rad": expected_bearing,
            "measured_bearing_error_from_body_request_rad": None if measured.get("displacement_bearing_rad") is None
                else wrap(measured["displacement_bearing_rad"] - expected_bearing),
            "reconstructed_path_m": measured_path,
            "feedback_speed_mps": extrema(r["speed_feedback_mps"] for r in samples),
            "source_difference_speed_during_command_arrivals_mps": extrema(r["speed_mps"] for r in active_fd),
            "yaw_range_rad": extrema(r["yaw_rad"] for r in samples),
            "maximum_yaw_deviation_from_start_rad": max((abs(wrap(r["yaw_rad"]-odom(start_raw)["yaw_rad"])) for r in samples), default=None),
            "zero_request_observations": request_zeros,
            "sample_count": len(samples), "settled": record.get("settled")}
        pulses.append(pulse)
    start_event = next((r for r in body["state_records"] if r["phase"] == "TURN"), None)
    origin = body["global_origin"]
    start_index = None
    if start_event and origin is not None:
        candidates = [i for i, r in enumerate(trajectory)
                      if r["wall_elapsed_s"] <= start_event["wall_elapsed_s"]
                      and math.dist(r["xyz_m"], origin) <= 1e-12]
        if candidates:
            start_index = candidates[-1]
    measured_path = measured_rotation = None
    if start_index is not None:
        rows = trajectory[start_index:]
        measured_path = sum(math.dist(a["xyz_m"], b["xyz_m"]) for a, b in zip(rows, rows[1:]))
        measured_rotation = sum(abs(wrap(b["yaw_rad"]-a["yaw_rad"])) for a, b in zip(rows, rows[1:]))
    completed = (body["reason"] == "direction_response_observed_official_score_unverified"
                 and len(pulses) == 3 and all(r["settled_record_present"] for r in pulses))
    command_runs = []
    for command in commands:
        key = tuple(command.get(k) for k in ("state", "phase", "pulse_index", "linear_x", "linear_y", "angular_z", "sent"))
        if not command_runs or key != command_runs[-1]["key"]:
            command_runs.append({"key": key, "first": command, "last": command, "count": 1})
        else:
            command_runs[-1]["last"] = command
            command_runs[-1]["count"] += 1
    metrics = {"schema": "ground_direction_response_analysis_v1", "mode": MODE,
        "session_id": body["session_id"], "state": body["state"], "phase": body["phase"], "reason": body["reason"],
        "calibration_three_pulses_observed_complete": completed,
        "not_completed_reason": None if completed else body["reason"],
        "official_score": None, "official_task_success": False, "full_mission_complete": False,
        "interface_semantics_confirmed": False, "limits": body["limits"], "dropped": body["dropped"],
        "resume_count": body["resume_count"], "pulse_count": len(pulses), "pulses": pulses,
        "raw_sample_counts": {k: len(body[k]) for k in ("raw_odometry", "raw_scans", "commands", "state_records", "zero_requests")},
        "first_odom": trajectory[0] if trajectory else None, "last_odom": trajectory[-1] if trajectory else None,
        "source_stamps_strictly_increasing": all(b > a for a, b in zip(source_times, source_times[1:])),
        "arrival_times_nondecreasing": all(b >= a for a, b in zip(arrival_times, arrival_times[1:])),
        "max_source_gap_s": max((b-a for a, b in zip(source_times, source_times[1:])), default=None),
        "max_arrival_gap_s": max((b-a for a, b in zip(arrival_times, arrival_times[1:])), default=None),
        "global_origin": origin, "reconstruction_start_index": start_index,
        "reported_path_m": body["total_path_m"], "reconstructed_path_m": measured_path,
        "path_matches": None if measured_path is None else abs(measured_path-body["total_path_m"]) <= 1e-10,
        "reported_rotation_rad": body["accumulated_rotation_rad"], "reconstructed_rotation_rad": measured_rotation,
        "rotation_matches": None if measured_rotation is None else abs(measured_rotation-body["accumulated_rotation_rad"]) <= 1e-10,
        "maximum_feedback_speed_mps": max((r["speed_feedback_mps"] for r in trajectory), default=None),
        "command_state_runs": command_runs, "sensor_sync": body["sensor_sync"],
        "assumptions": body["assumptions"],
        "timing_limit": "commands use process wall time; finite differences use odom source stamp; actuator application latency is unobserved",
        "verification_scope": "internal callback integrity and own public observations, not independent source authentication or referee score"}
    return metrics, trajectory, finite_differences


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=OUT)
    parser.add_argument("--platform-id", default=None)
    parser.add_argument("--run-id", default=None)
    args = parser.parse_args()
    if not args.input.exists():
        print("WAITING: real callback file does not exist; no analysis artifacts written")
        return 2
    raw_source = args.input.read_bytes()
    if not raw_source:
        print("WAITING: real callback file is empty; no analysis artifacts written")
        return 2
    rows = [strict_json(line) for line in raw_source.decode("utf-8").splitlines() if line.strip()]
    data_rows = [(r, r.get("payload", r).get("data", {})) for r in rows]
    parts = [data for _, data in data_rows if data.get("section") == SECTION]
    cores = [data["data"] for _, data in data_rows if data.get("section") == CORE]
    try:
        body, compressed, raw, metadata = decode_parts(parts)
        if not cores:
            raise ValueError("missing separate direction core")
        for core in cores:
            if not isinstance(core, dict) or any(key not in body or body[key] != value for key, value in core.items()):
                raise ValueError("core differs from recovered bundle")
    except (ValueError, zlib.error, UnicodeError) as exc:
        print("INCOMPLETE_OR_REJECTED:", str(exc))
        return 2
    if args.input.resolve().parent == args.output.resolve():
        raise ValueError("keep source callbacks separate from derived output")
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "evidence.json.gz").write_bytes(compressed)
    (args.output / "evidence.json").write_bytes(raw)
    write_json(args.output / "events_without_parts.json", [r for r, data in data_rows if data.get("section") != SECTION])
    validation = {"source": str(args.input.resolve()), "source_sha256": hashlib.sha256(raw_source).hexdigest(),
        "source_bytes": len(raw_source), "event_count": len(rows), "parts_verified": metadata["part_count"],
        "session_id": body["session_id"], "metadata": metadata, "compressed_length_matches": True,
        "compressed_hash_matches": True, "raw_length_matches": True, "raw_hash_matches": True,
        "core_matches_recovered": True, "core_count": len(cores), "platform_id": args.platform_id,
        "run_id": args.run_id, "callback_sources": sorted(set(r.get("source", "unspecified") for r in rows)),
        "last_received_at": rows[-1].get("received_at"),
        "launcher_exit_codes": [data.get("exit_code") for row, data in data_rows
                                if row.get("payload", row).get("event") == "launcher_exit"],
        "source_authenticated_independently": False}
    write_json(args.output / "validation.json", validation)
    metrics, trajectory, finite_differences = analyze(body)
    metrics.update(platform_id=args.platform_id, run_id=args.run_id,
                   input_sha256=metadata["uncompressed_sha256"])
    write_json(args.output / "metrics.json", metrics)
    csv_rows(args.output / "trajectory.csv", trajectory)
    csv_rows(args.output / "finite_differences.csv", finite_differences)
    csv_rows(args.output / "commands.csv", body["commands"])
    print(json.dumps({k: metrics[k] for k in ("state", "reason", "calibration_three_pulses_observed_complete", "pulse_count", "path_matches", "rotation_matches")}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
