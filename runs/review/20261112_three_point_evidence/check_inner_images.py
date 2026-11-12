"""Recompute preserved JPEG fragment hashes from new local transport fixtures."""
import base64
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).parent / "probe_20261111T224720171738Z"
result = {}
for case in ("production_normal", "production_cap_stress"):
    path = ROOT / case / "decoded/bundle.json"
    before = hashlib.sha256(path.read_bytes()).hexdigest()
    body = json.loads(path.read_text(encoding="utf-8"))
    groups = {}
    for name, data in body["capture_image_events"]:
        assert name == "probe_snapshot" and data["section"] == "survey_image_part"
        assert data["topic"] == "/uav/camera/image"
        key = (data["phase"], data["stamp_s"])
        groups.setdefault(key, []).append(data)
    images = []
    for (phase, stamp), parts in sorted(groups.items()):
        first = parts[0]
        assert {p["part_index"] for p in parts} == set(range(first["part_count"]))
        assert len(parts) == first["part_count"]
        assert all((p["jpeg_sha256"], p["width"], p["height"], p["part_count"]) ==
            (first["jpeg_sha256"], first["width"], first["height"], first["part_count"]) for p in parts)
        blob = base64.b64decode("".join(p["jpeg_base64_part"] for p in sorted(parts, key=lambda p:p["part_index"])), validate=True)
        assert hashlib.sha256(blob).hexdigest() == first["jpeg_sha256"]
        assert blob.startswith(b"\xff\xd8\xff") and blob.endswith(b"\xff\xd9")
        assert len(blob) <= 384*1024
        images.append({"phase": phase, "stamp_s": stamp, "bytes": len(blob),
            "sha256": first["jpeg_sha256"], "width": first["width"], "height": first["height"]})
    assert len(images) == 9
    assert hashlib.sha256(path.read_bytes()).hexdigest() == before
    result[case] = {"frames": len(images), "fragments": len(body["capture_image_events"]),
        "source_preserved": True, "images": images}
target = ROOT / "inner_images.json"
with target.open("x", encoding="utf-8") as handle:
    json.dump(result, handle, indent=2)
print(json.dumps({case: {k:v for k,v in data.items() if k != "images"} for case,data in result.items()}))
