"""Independent evidence transport checks; none infer competition scores."""
import base64
import gzip
import hashlib
import json

import pytest

from tools.analyze_ground_three_point_evidence import decode_bundle, analyze_file


def parts_for(body=None, *, chunk=40, raw_override=None):
    body = body or {"schema": "ground_three_point_evidence_v1", "session_id": "run-test",
                    "survey": {"trajectory_10hz": [[1, 2]], "actual_commands": [[3, 4]]},
                    "completed_points": [], "current_point": {}, "capture_image_events": []}
    raw = raw_override or json.dumps(body, ensure_ascii=False, allow_nan=False).encode("utf-8")
    packed = gzip.compress(raw, mtime=0)
    encoded = base64.b64encode(packed).decode("ascii")
    chunks = [encoded[i:i+chunk] for i in range(0, len(encoded), chunk)]
    return [{"mode": "ground_three_point", "section": "three_ground_evidence_gzip_part",
             "encoding": "gzip+base64", "session_id": "run-test", "bundle": "ground_three_point_v1",
             "uncompressed_sha256": hashlib.sha256(raw).hexdigest(),
             "compressed_sha256": hashlib.sha256(packed).hexdigest(),
             "uncompressed_bytes": len(raw), "compressed_bytes": len(packed),
             "part_index": i, "part_count": len(chunks), "base64_part": chunk}
            for i, chunk in enumerate(chunks)], body


def test_out_of_order_transport_recovers_original_rows_and_observations():
    parts, body = parts_for()
    restored, transport = decode_bundle(list(reversed(parts)))
    assert restored == body
    assert transport["duplicate_parts"] == 0
    assert transport["part_count"] == len(parts)


def test_identical_retransmission_is_counted_without_changing_data():
    parts, body = parts_for()
    restored, transport = decode_bundle(parts + [parts[0]])
    assert restored == body
    assert transport["duplicate_parts"] == 1


def test_hash_correct_json_exponent_overflow_is_rejected():
    _, body = parts_for()
    body["survey"]["value"] = "exponent-placeholder"
    raw = json.dumps(body).replace('"exponent-placeholder"', "1e309").encode()
    parts, _ = parts_for(body, raw_override=raw)
    with pytest.raises(ValueError, match="nonfinite"):
        decode_bundle(parts)


@pytest.mark.parametrize("mutation, message", [
    ("missing", "missing"), ("conflict", "conflict"), ("mixed", "metadata"),
    ("hash", "SHA256"), ("session", "session"), ("oversize", "size"),
])
def test_incomplete_or_changed_evidence_is_rejected(mutation, message):
    parts, body = parts_for()
    if mutation == "missing":
        parts.pop()
    elif mutation == "conflict":
        parts.append({**parts[0], "base64_part": "AAAA"})
    elif mutation == "mixed":
        parts[0]["compressed_bytes"] += 1
    elif mutation == "hash":
        for part in parts:
            part["uncompressed_sha256"] = "0" * 64
    elif mutation == "session":
        body["session_id"] = "other-run"
        parts, _ = parts_for(body)
    elif mutation == "oversize":
        for part in parts:
            part["uncompressed_bytes"] = 32 * 1024 * 1024 + 1
    with pytest.raises(ValueError, match=message):
        decode_bundle(parts)


def test_file_analysis_keeps_raw_hash_and_never_infers_score(tmp_path):
    parts, body = parts_for()
    source = tmp_path / "raw.jsonl"
    raw = "".join(json.dumps({"payload": {"event": "probe_snapshot", "data": part}}) + "\n"
                  for part in parts).encode("utf-8")
    source.write_bytes(raw)
    summary = analyze_file(source, tmp_path / "analysis")
    assert summary["transport_complete"] is True
    assert summary["official_score"] is None
    assert summary["full_mission_complete"] is False
    assert summary["raw_sha256"] == hashlib.sha256(raw).hexdigest()
    assert source.read_bytes() == raw
    assert json.loads((tmp_path / "analysis" / "bundle.json").read_text("utf-8")) == body


@pytest.mark.parametrize("mode", ["ground_three_point_camera_recovery", "ground_three_point_recovery_budget",
                                "ground_three_point_return"])
def test_recovery_variants_preserve_actual_envelope_identity_and_bytes(tmp_path, mode):
    parts, body = parts_for()
    for part in parts:
        part["mode"] = mode
    source = tmp_path / "variant.jsonl"
    raw = "".join(json.dumps({"event": "probe_snapshot", "data": part}) + "\n" for part in parts).encode()
    source.write_bytes(raw)
    summary = analyze_file(source, tmp_path / "analysis")
    assert summary["transport_complete"] is True
    assert summary["transport"]["mode"] == mode
    assert summary["raw_sha256"] == hashlib.sha256(raw).hexdigest()
    assert summary["official_score"] is None
    assert source.read_bytes() == raw
    assert json.loads((tmp_path / "analysis" / "bundle.json").read_text("utf-8")) == body


def test_unknown_variant_mode_remains_rejected():
    parts, _ = parts_for()
    for part in parts:
        part["mode"] = "unregistered_variant"
    with pytest.raises(ValueError, match="identity"):
        decode_bundle(parts)


@pytest.mark.parametrize("name", ["bundle.json", "summary.json"])
def test_analysis_refuses_to_overwrite_source_with_an_output(tmp_path, name):
    parts, _ = parts_for()
    source = tmp_path / name
    raw = "".join(json.dumps({"event": "probe_snapshot", "data": part}) + "\n" for part in parts).encode()
    source.write_bytes(raw)
    with pytest.raises(ValueError, match="overwrite source"):
        analyze_file(source, tmp_path)
    assert source.read_bytes() == raw
