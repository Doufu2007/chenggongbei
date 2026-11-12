"""Offline crops of real diagnostic camera frames; no ROS or target claims.

Explicit visually reviewed ROIs select existing pixels, not class answers.
The existing template matcher and optional verified local CLIP retain their
unchanged parameters. Their outputs are exploratory candidates without truth.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import sys
from time import perf_counter

import cv2
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.evaluate_reference_vision import DEFAULT_ASSETS, load_assets, split_assets
from tools.evaluate_zero_shot_vision import PROMPTS, MIN_COSINE, MIN_MARGIN, classify_cosines
from inspection_mission.vision_core import TemplateMatcher, detect_targets


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare_crops(summary_path, plan, output):
    """Verify image bytes and explicit bounds; save lossless observed-pixel crops."""
    summary_path, output = Path(summary_path), Path(output)
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    if plan.get("source_callback_sha256") != summary.get("source_evidence", {}).get("sha256"):
        raise ValueError("ROI plan callback hash does not match the diagnostic summary")
    if not isinstance(plan.get("review_method"), str) or not plan["review_method"].strip():
        raise ValueError("explicit ROI review method is required")
    images = summary.get("images", [])
    hashes = [item["sha256"] for item in images]
    if len(hashes) != len(set(hashes)):
        raise ValueError("image hashes are ambiguous; use separate per-event ROI records")
    plans = plan.get("images", [])
    if len(plans) != len(images) or {item["sha256"] for item in plans} != set(hashes):
        raise ValueError("ROI plan must cover exactly the provided source images")
    by_hash = {item["sha256"]: item for item in plans}
    output.mkdir(parents=True, exist_ok=True)
    records = []
    for source in images:
        image_path = (summary_path.parent / source["file_name"]).resolve()
        if image_path.parent != summary_path.parent.resolve() or digest(image_path) != source["sha256"]:
            raise ValueError("source JPEG path or digest mismatch")
        with Image.open(image_path) as loaded:
            loaded.load()
            pixels = np.array(loaded.convert("RGB"))
        height, width = pixels.shape[:2]
        if (width, height) != (source["width"], source["height"]):
            raise ValueError("source image dimensions do not match diagnostic metadata")
        image_plan = by_hash[source["sha256"]]
        for region in image_plan["regions"]:
            if set(region) != {"id", "bbox_xyxy", "observation", "panel_adequacy"}:
                raise ValueError("ROI annotations must not introduce class/target answers")
            name, bounds = region["id"], region["bbox_xyxy"]
            if not isinstance(name, str) or not re.fullmatch(r"[a-zA-Z0-9_-]{1,64}", name):
                raise ValueError("ROI id must be a safe bounded identifier")
            if (not isinstance(bounds, list) or len(bounds) != 4 or any(type(x) is not int for x in bounds)
                    or not 0 <= bounds[0] < bounds[2] <= width or not 0 <= bounds[1] < bounds[3] <= height):
                raise ValueError("ROI bounds must be integer xyxy within the actual source image")
            if not isinstance(region["observation"], str) or not isinstance(region["panel_adequacy"], str):
                raise ValueError("ROI requires visual observation and panel adequacy annotation")
            crop = pixels[bounds[1]:bounds[3], bounds[0]:bounds[2]].copy()
            filename = f"crop_L{source['line']:06d}_{name}_{source['sha256'][:12]}.png"
            path = output / filename
            Image.fromarray(crop).save(path, format="PNG")
            records.append({"source": dict(source), "source_file": str(image_path), "region_id": name,
                "bbox_xyxy": bounds, "visual_observation": region["observation"],
                "panel_adequacy": region["panel_adequacy"], "crop_file": filename,
                "crop_width": crop.shape[1], "crop_height": crop.shape[0], "crop_sha256": digest(path),
                "crop_rgb_sha256": hashlib.sha256(crop.tobytes()).hexdigest(),
                "expected_class": None, "recognized_target": None})
    return records


def load_local_clip(model_root):
    """Verify already downloaded official artifacts before local inference."""
    model_root = Path(model_root)
    model_record = json.loads((model_root / "MODEL_SOURCE.json").read_text(encoding="utf-8"))
    weights = model_root / "weights/ViT-B-32.pt"
    if model_record.get("status") != "verified" or digest(weights) != model_record["expected_sha256"]:
        raise ValueError("local official CLIP weight digest is not verified")
    implementation = json.loads((model_root / "OFFICIAL_SOURCE_MANIFEST.json").read_text(encoding="utf-8"))
    for record in implementation["files"]:
        if digest(model_root / "official_source" / record["path"]) != record["sha256"]:
            raise ValueError("local CLIP implementation digest mismatch")
    sys.path.insert(0, str((model_root / "official_source").resolve()))
    import clip
    import torch
    torch.set_num_threads(2)
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model, preprocess = clip.load(str(weights), device=device, jit=False)
    model.eval()
    with torch.no_grad():
        text = model.encode_text(clip.tokenize(list(PROMPTS.values())).to(device))
        text = text / text.norm(dim=-1, keepdim=True)

    def classify(crop):
        with torch.no_grad():
            image = model.encode_image(preprocess(Image.fromarray(crop)).unsqueeze(0).to(device))
            image = image / image.norm(dim=-1, keepdim=True)
            values = (image @ text.T).float().cpu().numpy()[0]
            if device == "cuda": torch.cuda.synchronize()
        return classify_cosines(values, list(PROMPTS))

    return classify, {"weight_source": model_record, "implementation": implementation, "device": device,
                      "torch": torch.__version__, "prompts": PROMPTS,
                      "rejection": {"min_cosine": MIN_COSINE, "min_margin": MIN_MARGIN, "calibrated": False}}


def analyze(summary_path, plan_path, output, *, asset_root=DEFAULT_ASSETS, model_root=None):
    output = Path(output)
    summary_path, plan_path = Path(summary_path), Path(plan_path)
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    crops = prepare_crops(summary_path, plan, output)
    assets, manifest = load_assets(Path(asset_root))
    references, _, _ = split_assets(assets)
    matcher = TemplateMatcher({item["class_id"]: item["absolute_path"] for item in references})
    report = {"schema": "inspection.real_camera_crop_diagnostic.v1", "status": "completed",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "summary_path": str(summary_path), "summary_sha256": digest(summary_path),
        "roi_plan_path": str(plan_path), "roi_plan_sha256": digest(plan_path),
        "source_callback_sha256": plan["source_callback_sha256"], "review_method": plan["review_method"],
        "template_source": {"repository": manifest["repository"], "commit_sha": manifest["commit_sha"],
                            "license": manifest.get("license"), "reference_assets": references,
                            "threshold_changes": False, "official_current_library_verified": False},
        "clip_status": "not_requested", "clip": None, "crops": crops, "recognized_targets": [],
        "ground_truth_classes": None, "accuracy": None, "official_scoring": "unverified",
        "notes": ["These are exact crops from received diagnostic JPEGs, not source-resolution camera frames or video.",
                  "ROI adequacy is an explicit visual-review annotation; image statistics are descriptive, not target-presence proof.",
                  "Template detections and CLIP acceptance are exploratory model candidates only. No class truth or official random assignment is known.",
                  "Whole-scene and blank-window controls may trigger false class candidates; no candidate is reported to ROS or a referee.",
                  "No production recognizer, template threshold, fixed CLIP prompt or rejection threshold is changed."]}
    classifier = None
    if model_root is not None:
        try:
            classifier, report["clip"] = load_local_clip(model_root)
            report["clip_status"] = "completed"
        except Exception as exc:
            report["clip_status"] = "failed"
            report["clip_error"] = type(exc).__name__ + ": " + str(exc)
    orb = cv2.ORB_create(nfeatures=2200, fastThreshold=8, edgeThreshold=15)
    for record in crops:
        with Image.open(output / record["crop_file"]) as loaded:
            crop = np.array(loaded.convert("RGB"))
        bgr = cv2.cvtColor(crop, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(crop, cv2.COLOR_RGB2GRAY)
        keypoints = orb.detect(gray, None)
        record["image_statistics"] = {"gray_std": float(gray.std()), "gray_min": int(gray.min()),
            "gray_max": int(gray.max()), "orb_keypoints": len(keypoints),
            "edge_pixels": int(np.count_nonzero(cv2.Canny(gray, 60, 160)))}
        started = perf_counter()
        record["template_candidates"] = detect_targets(bgr, enable_qr=False, enable_aruco=False, template_matcher=matcher)
        record["template_inference_ms"] = (perf_counter()-started)*1000
        record["clip_candidate"] = None
        if classifier is not None:
            started = perf_counter()
            record["clip_candidate"] = classifier(crop)
            record["clip_inference_ms"] = (perf_counter()-started)*1000
        record["interpretation"] = "exploratory_only_no_class_ground_truth"
    (output / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--rois", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--asset-root", type=Path, default=DEFAULT_ASSETS)
    parser.add_argument("--model-root", type=Path)
    args = parser.parse_args(argv)
    report = analyze(args.summary, args.rois, args.output, asset_root=args.asset_root, model_root=args.model_root)
    print(json.dumps({"status": report["status"], "crops": len(report["crops"]), "clip_status": report["clip_status"],
                      "recognized_targets": report["recognized_targets"], "report": str(args.output / "report.json")}, ensure_ascii=True))
    return 2 if report["clip_status"] == "failed" else 0


if __name__ == "__main__":
    raise SystemExit(main())
