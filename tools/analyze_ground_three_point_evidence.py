"""Recover a bounded three-point callback bundle without inferring a score."""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
from pathlib import Path
import re
import zlib


MAX_RAW = 32 * 1024 * 1024
MAX_PACKED = 8 * 1024 * 1024
SECTION = "three_ground_evidence_gzip_part"
MODES = frozenset(("ground_three_point", "ground_three_point_camera_recovery",
                   "ground_three_point_recovery_budget", "ground_three_point_return"))
METADATA = ("mode", "section", "encoding", "session_id", "bundle",
            "uncompressed_sha256", "compressed_sha256", "uncompressed_bytes",
            "compressed_bytes", "part_count")


def _unique(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key")
        result[key] = value
    return result


def _json(raw):
    def invalid(_):
        raise ValueError("nonfinite JSON value")
    def finite_float(value):
        parsed = float(value)
        if not math.isfinite(parsed):
            raise ValueError("nonfinite JSON float")
        return parsed
    return json.loads(raw, object_pairs_hook=_unique, parse_constant=invalid, parse_float=finite_float)


def decode_bundle(parts):
    """Return exact source data and verified byte transport metadata, or reject."""
    if not parts:
        raise ValueError("missing compressed bundle")
    first = parts[0]
    if not isinstance(first, dict) or any(key not in first for key in METADATA):
        raise ValueError("incomplete bundle metadata")
    if (first["mode"] not in MODES or first["section"] != SECTION
            or first["encoding"] != "gzip+base64" or first["bundle"] != "ground_three_point_v1"
            or not isinstance(first["session_id"], str) or not 1 <= len(first["session_id"]) <= 128):
        raise ValueError("invalid bundle identity metadata")
    for key, cap in (("uncompressed_bytes", MAX_RAW), ("compressed_bytes", MAX_PACKED),
                     ("part_count", 512)):
        if type(first[key]) is not int or not 1 <= first[key] <= cap:
            raise ValueError("bundle size or count exceeds bound")
    for key in ("uncompressed_sha256", "compressed_sha256"):
        if not isinstance(first[key], str) or not re.fullmatch("[0-9a-f]{64}", first[key]):
            raise ValueError("invalid SHA256 metadata")
    chunks, duplicates = {}, 0
    for part in parts:
        if not isinstance(part, dict) or any(part.get(k) != first[k] for k in METADATA):
            raise ValueError("conflicting bundle metadata")
        index, encoded = part.get("part_index"), part.get("base64_part")
        if (type(index) is not int or not 0 <= index < first["part_count"]
                or not isinstance(encoded, str) or not 0 < len(encoded) <= 48000
                or not re.fullmatch("[A-Za-z0-9+/=]+", encoded)):
            raise ValueError("invalid bundle part index or size")
        if index in chunks:
            if chunks[index] != encoded:
                raise ValueError("conflicting bundle part")
            duplicates += 1
        chunks[index] = encoded
    if len(chunks) != first["part_count"]:
        raise ValueError("missing bundle parts")
    encoded = "".join(chunks[i] for i in range(first["part_count"]))
    if len(encoded) > 4 * ((MAX_PACKED + 2) // 3):
        raise ValueError("compressed bundle size exceeds bound")
    packed = base64.b64decode(encoded, validate=True)
    if len(packed) != first["compressed_bytes"]:
        raise ValueError("compressed bundle size mismatch")
    if hashlib.sha256(packed).hexdigest() != first["compressed_sha256"]:
        raise ValueError("compressed SHA256 mismatch")
    decoder = zlib.decompressobj(16 + zlib.MAX_WBITS)
    try:
        raw = decoder.decompress(packed, first["uncompressed_bytes"] + 1)
    except zlib.error as exc:
        raise ValueError("invalid gzip bundle") from exc
    if (not decoder.eof or decoder.unused_data or decoder.unconsumed_tail
            or len(raw) != first["uncompressed_bytes"]):
        raise ValueError("uncompressed bundle size or stream mismatch")
    if hashlib.sha256(raw).hexdigest() != first["uncompressed_sha256"]:
        raise ValueError("uncompressed SHA256 mismatch")
    body = _json(raw.decode("utf-8"))
    if (not isinstance(body, dict) or body.get("schema") != "ground_three_point_evidence_v1"
            or body.get("session_id") != first["session_id"]):
        raise ValueError("bundle schema or session mismatch")
    if (not isinstance(body.get("survey"), dict) or not isinstance(body.get("completed_points"), list)
            or not isinstance(body.get("capture_image_events"), list)):
        raise ValueError("invalid evidence containers")
    return body, {**{k: first[k] for k in METADATA}, "duplicate_parts": duplicates}


def analyze_file(input_path, output_dir):
    source, output = Path(input_path), Path(output_dir)
    if source.resolve() in {(output / "bundle.json").resolve(), (output / "summary.json").resolve()}:
        raise ValueError("analysis output would overwrite source")
    raw = source.read_bytes()
    result = {"schema": "ground_three_point_analysis_v1", "source": str(source.resolve()),
              "raw_sha256": hashlib.sha256(raw).hexdigest(), "raw_bytes": len(raw),
              "transport_complete": False, "official_score": None,
              "full_mission_complete": False, "errors": [], "core_observations": []}
    parts = []
    for number, line in enumerate(raw.decode("utf-8").splitlines(), 1):
        try:
            row = _json(line)
            event = row.get("payload", row)
            data = event.get("data", {})
            if event.get("event") != "probe_snapshot" or data.get("mode") not in MODES:
                continue
            if data.get("section") == SECTION:
                parts.append(data)
            elif data.get("section") in {"three_point_core", "three_point_point"}:
                result["core_observations"].append({"line": number, "data": data})
        except (ValueError, TypeError, AttributeError) as exc:
            result["errors"].append({"line": number, "error": str(exc)})
    output.mkdir(parents=True, exist_ok=True)
    try:
        body, transport = decode_bundle(parts)
        (output / "bundle.json").write_text(json.dumps(body, ensure_ascii=False, indent=2), encoding="utf-8")
        result.update(transport_complete=not result["errors"], transport=transport,
                      completed_points_observed=len(body["completed_points"]),
                      interpretation="byte transport only; point history and local state are unverified observations")
    except (ValueError, UnicodeError) as exc:
        result["errors"].append({"scope": "bundle", "error": str(exc)})
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    result = analyze_file(args.input, args.output)
    print(json.dumps({key: result[key] for key in ("transport_complete", "raw_sha256", "errors")}, ensure_ascii=True))
    return 0 if result["transport_complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
