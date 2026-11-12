"""Frozen CLIP image-to-image neighbors for a verified real camera ROI.

No training, threshold selection, class acceptance, ROS, referee or network.
Reference filenames are source annotations, never truth labels for the query.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
from time import perf_counter

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.evaluate_reference_vision import DEFAULT_ASSETS, load_assets, split_assets


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def resolve(path):
    path = Path(path)
    return path if path.is_absolute() else ROOT / path


def normalized(values):
    values = np.asarray(values, dtype=np.float32)
    if values.ndim != 2 or min(values.shape) == 0 or not np.isfinite(values).all():
        raise ValueError("finite nonempty feature matrix required")
    norms = np.linalg.norm(values, axis=1, keepdims=True)
    if not np.isfinite(norms).all() or (norms <= 0).any():
        raise ValueError("zero/invalid feature norm")
    return values / norms


def cosine_neighbors(queries, references, reference_ids):
    queries, references = normalized(queries), normalized(references)
    if queries.shape[1] != references.shape[1] or len(reference_ids) != len(references):
        raise ValueError("feature dimensions/reference IDs disagree")
    if len(set(reference_ids)) != len(reference_ids):
        raise ValueError("reference IDs must be unique")
    matrix = (queries @ references.T).astype(float)
    rankings = []
    for row in matrix:
        indices = sorted(range(len(reference_ids)), key=lambda i: (-row[i], reference_ids[i]))
        rankings.append([{"rank": rank+1, "reference_index": i, "reference_id": reference_ids[i],
                          "cosine": float(row[i])} for rank, i in enumerate(indices)])
    return {"cosine_matrix": matrix.tolist(), "rankings": rankings,
            "class_decision": None, "threshold": None}


def load_query_record(projection_path, query_name):
    projection_path = Path(projection_path)
    projection = json.loads(projection_path.read_text(encoding="utf-8"))
    identities = [("source_image", projection["source_image"])]
    for field in ("source_summary", "candidate_world"):
        if field in projection:
            identities.append((field, projection[field]))
    products = [p for p in projection["products"] if Path(p["path"]).name == query_name]
    if len(products) != 1:
        raise ValueError("exactly one selected query product is required")
    identities.append(("query", products[0]))
    for label, identity in identities:
        if digest(resolve(identity["path"])) != identity["sha256"]:
            raise ValueError(label + " digest mismatch")
    path = resolve(products[0]["path"])
    with Image.open(path) as image:
        image.load()
        size = list(image.size)
    return path, {"projection_path": str(projection_path), "projection_sha256": digest(projection_path),
                  "projection": projection, "mapping_status": projection.get("mapping_status"),
                  "query_size_wh": size, "query_identity": products[0]}


def load_verified_image_encoder(model_root):
    model_root = Path(model_root)
    source = json.loads((model_root / "MODEL_SOURCE.json").read_text(encoding="utf-8"))
    weights = model_root / "weights/ViT-B-32.pt"
    if source.get("status") != "verified" or digest(weights) != source["expected_sha256"]:
        raise ValueError("CLIP weight digest is not verified")
    implementation = json.loads((model_root / "OFFICIAL_SOURCE_MANIFEST.json").read_text(encoding="utf-8"))
    for item in implementation["files"]:
        if digest(model_root / "official_source" / item["path"]) != item["sha256"]:
            raise ValueError("official CLIP implementation digest mismatch")
    sys.path.insert(0, str((model_root / "official_source").resolve()))
    import clip
    import torch
    torch.set_num_threads(2)
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model, preprocess = clip.load(str(weights), device=device, jit=False)
    model.eval()

    def encode(images):
        batch = torch.stack([preprocess(image) for image in images]).to(device)
        with torch.inference_mode():
            values = model.encode_image(batch).float().cpu().numpy()
            if device == "cuda":
                torch.cuda.synchronize()
        return normalized(values)

    return encode, {"weight_source": source, "implementation": implementation,
                    "device": device, "torch": torch.__version__,
                    "device_name": torch.cuda.get_device_name(0) if device == "cuda" else "cpu",
                    "preprocess": repr(preprocess), "feature_math": "image encode -> float32 -> L2 normalize -> float32 dot",
                    "training": False, "text_prompts": None}


def font(size):
    try:
        return ImageFont.truetype("C:/Windows/Fonts/arial.ttf", size)
    except OSError:
        return ImageFont.load_default()


def tile(canvas, image, x, y, lines):
    thumb = ImageOps.contain(image.convert("RGB"), (300, 190))
    canvas.paste(thumb, (x+(310-thumb.width)//2, y+(195-thumb.height)//2))
    draw = ImageDraw.Draw(canvas)
    for i, line in enumerate(lines):
        draw.text((x+8, y+200+i*20), line, fill="white", font=font(16))


def contact_sheets(queries, assets, results, output):
    neighbors = Image.new("RGB", (1240, 1060), (28, 28, 28))
    ImageDraw.Draw(neighbors).text((12, 10), "Frozen CLIP image-feature neighbors / no query class truth or acceptance", fill="white", font=font(22))
    for row, (angle, image) in enumerate(queries):
        y = 50+row*250
        tile(neighbors, image, 0, y, [f"Query CCW {angle} deg", "same source ROI / no truth label"])
        for col, item in enumerate(results["rankings"][row][:3], 1):
            asset = assets[item["reference_index"]]
            with Image.open(asset["absolute_path"]) as reference:
                tile(neighbors, reference, col*310, y,
                     [Path(asset["path"]).name, f"rank {item['rank']} / cosine {item['cosine']:.6f}"])
    neighbors.save(output / "query_reference_neighbors.png")
    gallery = Image.new("RGB", (1240, 810), (28, 28, 28))
    ImageDraw.Draw(gallery).text((12, 10), "All 12 public reference photographs / filename annotations only", fill="white", font=font(22))
    for i, asset in enumerate(assets):
        with Image.open(asset["absolute_path"]) as reference:
            tile(gallery, reference, (i%4)*310, 50+(i//4)*250,
                 [f"{i}: {Path(asset['path']).name}", f"sha256 {asset['sha256'][:16]}"])
    gallery.save(output / "reference_gallery.png")


def analyze(projection_path, output, *, asset_root=DEFAULT_ASSETS,
            model_root=ROOT / "external_resources/openai_CLIP"):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    query_path, query_record = load_query_record(projection_path, "ground4_candidate_rectified_world_axes.png")
    assets, manifest = load_assets(Path(asset_root))
    assets = sorted(assets, key=lambda a: (a["class_id"], a["path"]))
    if len(assets) != 12:
        raise ValueError("this fixed diagnostic expects the 12 verified public photographs")
    references, held_out, excluded = split_assets(assets)
    with Image.open(query_path) as loaded:
        original = loaded.convert("RGB")
    queries = [(angle, original.rotate(angle, expand=True)) for angle in (0, 90, 180, 270)]
    for angle, image in queries:
        image.save(output / f"query_ccw_{angle:03d}.png")
    started = perf_counter()
    encode, model_record = load_verified_image_encoder(model_root)
    model_ms = (perf_counter()-started)*1000
    started = perf_counter()
    q_features = encode([image for _, image in queries])
    r_batches = []
    for index in range(0, len(assets), 4):
        images = []
        for asset in assets[index:index+4]:
            with Image.open(asset["absolute_path"]) as loaded:
                images.append(loaded.convert("RGB"))
        r_batches.append(encode(images))
    r_features = np.vstack(r_batches)
    encode_ms = (perf_counter()-started)*1000
    results = cosine_neighbors(q_features, r_features, [Path(a["path"]).name for a in assets])
    np.savez(output / "normalized_image_features.npz", query=q_features, reference=r_features)
    contact_sheets(queries, assets, results, output)
    query_variants = [{"rotation_ccw_deg": angle, "file": f"query_ccw_{angle:03d}.png",
                       "sha256": digest(output / f"query_ccw_{angle:03d}.png"), "size_wh": list(image.size)}
                      for angle, image in queries]
    report = {"schema": "inspection.clip_image_neighbors.v1", "created_at": datetime.now(timezone.utc).isoformat(),
        "status": "completed", "query": query_record, "query_variants": query_variants,
        "asset_source": {"manifest_path": str(Path(asset_root) / "SOURCE_MANIFEST.json"),
                         "manifest_sha256": digest(Path(asset_root) / "SOURCE_MANIFEST.json"),
                         "repository": manifest["repository"], "commit_sha": manifest["commit_sha"],
                         "license": manifest.get("license"), "official_current_library_verified": False},
        "references_in_column_order": assets, "model": model_record, **results,
        "feature_shape": {"query": list(q_features.shape), "reference": list(r_features.shape)},
        "timing_ms": {"model_verification_and_loading": model_ms, "all_16_image_encodings": encode_ms},
        "ref_held_out_split_feasibility": {"class_photo_counts": dict(Counter(a["class_id"] for a in assets)),
            "existing_reference_paths": [a["path"] for a in references],
            "existing_held_out_paths": [a["path"] for a in held_out],
            "excluded_duplicate_paths": [a["path"] for a in excluded],
            "note": "All 12 images are neighbors in this diagnostic. None is held out for evaluating this query; its class truth is unavailable."},
        "class_ground_truth": None, "accuracy": None, "published_to_referee": False,
        "official_scoring": "unverified", "threshold_selection": False,
        "limitations": ["Neighbor scores are relative similarities, not calibrated class probabilities or an acceptance decision.",
            "All four rotations use the same captured ROI and are not independent samples.",
            "Rectification/upsampling cannot restore source detail; official CLIP resizes and center-crops each input.",
            "The reference filenames supply reference annotations; no query class truth or official texture assignment is read.",
            "This comparison alone cannot identify resolution as the primary failure cause."]}
    products = ["query_reference_neighbors.png", "reference_gallery.png", "normalized_image_features.npz"]
    report["products"] = [{"file": p, "sha256": digest(output / p)} for p in products]
    (output / "image_neighbors.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--projection", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--asset-root", type=Path, default=DEFAULT_ASSETS)
    parser.add_argument("--model-root", type=Path, default=ROOT / "external_resources/openai_CLIP")
    args = parser.parse_args(argv)
    report = analyze(args.projection, args.output, asset_root=args.asset_root, model_root=args.model_root)
    print(json.dumps({"output": str(args.output / "image_neighbors.json"),
        "top3": [[{"reference": i["reference_id"], "cosine": i["cosine"]} for i in r[:3]] for r in report["rankings"]],
        "class_decision": None}, ensure_ascii=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
