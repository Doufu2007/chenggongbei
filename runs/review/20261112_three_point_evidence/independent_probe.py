"""Read-only production-encoder review; synthetic motion is never a score."""
from __future__ import annotations

import base64
from copy import deepcopy
from datetime import datetime, timezone
import gzip
import hashlib
import json
import math
from pathlib import Path
import random
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT), str(ROOT / "src/inspection_mission"), str(ROOT / "tests")]

from inspection_mission.telemetry import DiagnosticReporter
from test_ground_three_point import ThreeRun
from tools.diagnostic_receiver import validate_event, MAX_BODY_BYTES
from tools.analyze_ground_three_point_evidence import analyze_file, decode_bundle, SECTION


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def wrapped(raw, *, session="review-session", chunk=64, packed=None, declared=None):
    packed = gzip.compress(raw, compresslevel=6, mtime=0) if packed is None else packed
    encoded = base64.b64encode(packed).decode("ascii")
    fragments = [encoded[i:i+chunk] for i in range(0, len(encoded), chunk)]
    return [{"mode": "ground_three_point", "section": SECTION, "encoding": "gzip+base64",
        "session_id": session, "bundle": "ground_three_point_v1",
        "uncompressed_sha256": hashlib.sha256(raw).hexdigest(),
        "compressed_sha256": hashlib.sha256(packed).hexdigest(),
        "uncompressed_bytes": len(raw) if declared is None else declared,
        "compressed_bytes": len(packed), "part_count": len(fragments),
        "part_index": i, "base64_part": item} for i, item in enumerate(fragments)]


BODY = {"schema": "ground_three_point_evidence_v1", "session_id": "review-session",
    "survey": {"task_completion": "unverified"}, "completed_points": [],
    "current_point": {}, "capture_image_events": []}
RAW = json.dumps(BODY, separators=(",", ":"), allow_nan=False).encode()


def rejected(parts):
    try:
        decode_bundle(parts)
    except (ValueError, UnicodeError) as exc:
        return {"rejected": True, "exception": type(exc).__name__, "error": str(exc)}
    return {"rejected": False}


def production_roundtrip(guard, output):
    events = guard.additional_events()  # actual production encoder, not a reimplementation
    receiver_payloads, envelope_sizes = [], []
    for event, data in events:
        envelope = DiagnosticReporter._encode(event, data)
        assert len(envelope) <= MAX_BODY_BYTES
        checked = validate_event(envelope)
        assert checked["event"] == event and checked["data"] == json.loads(json.dumps(data, allow_nan=False))
        receiver_payloads.append(checked)
        envelope_sizes.append(len(envelope))
    parts = [payload["data"] for payload in receiver_payloads if payload["data"].get("section") == SECTION]
    body, transport = decode_bundle(list(reversed(parts)))
    assert body["session_id"] == guard.session_id
    assert body["survey"] == json.loads(json.dumps(guard._observations.survey_evidence(), allow_nan=False))
    assert body["completed_points"] == json.loads(json.dumps(guard.completed_points, allow_nan=False))
    assert len(body["capture_image_events"]) >= 9
    for point in body["completed_points"]:
        assert point["official_acceptance"] is False
    assert body["survey"]["task_completion"] == "unverified"
    assert body["current_point"]["full_mission_complete"] is False
    recovered_raw = gzip.decompress(base64.b64decode("".join(p["base64_part"] for p in parts)))
    assert hashlib.sha256(recovered_raw).hexdigest() == transport["uncompressed_sha256"]
    assert len(recovered_raw) == transport["uncompressed_bytes"]
    path = output / "production_callback.jsonl"
    with path.open("xb") as handle:
        for payload in receiver_payloads:
            handle.write(json.dumps({"source": "unverified_callback", "payload": payload}, ensure_ascii=False,
                allow_nan=False).encode("utf-8") + b"\n")
    original_hash = sha(path)
    summary = analyze_file(path, output / "decoded")
    assert sha(path) == original_hash
    assert summary["transport_complete"] is True
    assert summary["official_score"] is None and summary["full_mission_complete"] is False
    return {"events": len(events), "parts": len(parts), "max_envelope_bytes": max(envelope_sizes),
        "raw_bytes": transport["uncompressed_bytes"], "compressed_bytes": transport["compressed_bytes"],
        "trajectory_rows": len(body["survey"]["trajectory_10hz"]),
        "command_rows": len(body["survey"]["actual_commands"]),
        "capture_fragment_events": len(body["capture_image_events"]), "raw_source_preserved": True,
        "official_score": summary["official_score"], "full_mission_complete": summary["full_mission_complete"],
        "uncompressed_sha256": transport["uncompressed_sha256"]}


def main():
    out = Path(__file__).parent / datetime.now(timezone.utc).strftime("probe_%Y%m%dT%H%M%S%fZ")
    out.mkdir(parents=True, exist_ok=False)
    checked = [ROOT / "tools/analyze_ground_three_point_evidence.py",
        ROOT / "src/inspection_mission/inspection_mission/ground_three_point.py",
        ROOT / "tools/diagnostic_receiver.py", ROOT / "src/inspection_mission/inspection_mission/telemetry.py"]
    result = {"scope": "independent local review; synthetic transport only; no official points",
        "start_hashes": {str(p.relative_to(ROOT)): sha(p) for p in checked}}
    # A single production-guard scenario; this does not rerun the existing nine tests.
    run = ThreeRun()
    for index in range(3):
        run.recognise()
        run.declare()
        if index < 2:
            run.until(lambda r: r.guard.point_index == index+1)
    run.until(lambda r: r.guard.state == "STOP")
    (out / "production_normal").mkdir()
    result["production_normal"] = production_roundtrip(run.guard, out / "production_normal")

    # Stress the actual production encoder at its declared row caps with nine
    # synthetic 800px public-camera frames. No platform sensor data is invented.
    import numpy as np
    rng = random.Random(20261112)
    obs = run.guard._observations
    exemplar = deepcopy(obs.trajectory[0])
    obs.trajectory = []
    for i in range(12500):
        row = deepcopy(exemplar)
        row.update(wall_elapsed_s=i*.1, stamp_s=10.+i*.1,
            position=[rng.uniform(-2., 2.), rng.uniform(-2., 2.), .055],
            quaternion=[0., 0., rng.uniform(-1., 1.), rng.uniform(-1., 1.)],
            yaw_rad=rng.uniform(-math.pi, math.pi), linear_speed=rng.uniform(0., .1),
            angular_speed=rng.uniform(-.5, .5))
        obs.trajectory.append(row)
    exemplar = deepcopy(obs.actual_commands[0]) if obs.actual_commands else {
        "wall_elapsed_s": 0., "clock_stamp_s": 10., "linear_x": 0., "angular_z": 0., "state": "DRIVE"}
    obs.actual_commands = []
    for i in range(30000):
        row = deepcopy(exemplar)
        row.update(wall_elapsed_s=i*.04, clock_stamp_s=10.+i*.04,
            linear_x=rng.uniform(0., .1), angular_z=rng.uniform(-.5, .5))
        obs.actual_commands.append(row)
    native_rng = np.random.default_rng(20261112)
    for frame in run.guard.archived_capture_frames:
        frame["msg"] = SimpleNamespace(width=800, height=800, step=2400, encoding="bgr8",
            data=native_rng.integers(0, 256, (800, 800, 3), dtype=np.uint8).tobytes())
    (out / "production_cap_stress").mkdir()
    result["production_cap_stress"] = production_roundtrip(run.guard, out / "production_cap_stress")

    cases = {}
    parts = wrapped(RAW)
    decoded, meta = decode_bundle(parts + [parts[0]])
    assert decoded == BODY and meta["duplicate_parts"] == 1
    cases["identical_retransmission"] = {"accepted": True, "duplicate_parts": 1}
    cases["missing"] = rejected(parts[:-1])
    cases["conflicting_part"] = rejected(parts + [{**parts[0], "base64_part": "AAAA"}])
    for field, value in [("compressed_bytes", parts[0]["compressed_bytes"]-1),
        ("uncompressed_bytes", len(RAW)-1), ("uncompressed_sha256", "0"*64),
        ("compressed_sha256", "0"*64), ("part_count", 513),
        ("uncompressed_bytes", 32*1024*1024+1), ("compressed_bytes", 8*1024*1024+1)]:
        cases[field+"_"+str(value)] = rejected([{**p, field: value} for p in parts])
    mixed = deepcopy(parts)
    mixed[-1]["session_id"] = "other-session"
    cases["mixed_fragment_session"] = rejected(mixed)
    cases["body_session"] = rejected(wrapped(RAW.replace(b'review-session', b'other-session')))
    for label, raw in [("duplicate_json_key", RAW.replace(b'"survey":', b'"survey":{},"survey":')),
        ("NaN", RAW.replace(b'"survey":{', b'"survey":{"value":NaN,')),
        ("exponent_overflow", RAW.replace(b'"survey":{', b'"survey":{"value":1e309,')),
        ("non_utf8", RAW+b'\xff')]:
        cases[label] = rejected(wrapped(raw))
    packed = gzip.compress(RAW, mtime=0)
    cases["gzip_concatenation"] = rejected(wrapped(RAW, packed=packed+packed))
    cases["gzip_trailing_data"] = rejected(wrapped(RAW, packed=packed+b"tail"))
    cases["bounded_expansion"] = rejected(wrapped(b" "*1000000, declared=100))
    for label in ("bundle.json", "summary.json"):
        folder = out / ("source_collision_"+label.split(".")[0])
        folder.mkdir()
        source = folder / label
        raw = b"".join(json.dumps({"payload": {"event": "probe_snapshot", "data": p}}).encode()+b"\n" for p in parts)
        source.write_bytes(raw)
        try:
            analyze_file(source, folder)
            error = None
        except ValueError as exc:
            error = str(exc)
        cases["source_collision_"+label] = {"rejected": error is not None,
            "source_preserved": source.read_bytes() == raw, "error": error}
    result["adversarial"] = cases
    result["end_hashes"] = {str(p.relative_to(ROOT)): sha(p) for p in checked}
    (out / "results.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({"output": str(out.relative_to(ROOT)), "production_normal": result["production_normal"],
        "production_cap_stress": result["production_cap_stress"], "adversarial": cases}, ensure_ascii=True))


if __name__ == "__main__":
    main()
