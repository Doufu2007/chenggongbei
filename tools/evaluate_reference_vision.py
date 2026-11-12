"""Reproducible offline baseline of the existing matcher, without ROS or Gazebo.

One reference per real class is registered. Other files stay held out. No matcher
thresholds, business defaults or competition task answers are changed here.
"""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import sys
from time import perf_counter

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "inspection_mission"))
from inspection_mission import vision_core

CLASS_LABELS = {
    "collapsed_ruins": "坍塌废墟", "damaged_vehicle": "受损车辆",
    "landslide": "滑坡塌方", "rescue_camp": "救援营地",
    "trapped_people": "被困人员", "window_bird": "飞鸟",
    "window_target": "救援无人机",
}
DEFAULT_ASSETS = ROOT / "external_resources" / "wbxxmz_ros2-car-first"


def split_assets(assets):
    """Sort before selecting one reference per class; exclude repeated bytes."""
    owners = {}
    for item in assets:
        previous = owners.setdefault(item["sha256"], item["class_id"])
        if previous != item["class_id"]:
            raise ValueError("identical reference bytes have conflicting classes")
    references, held_out, excluded = [], [], []
    classes, hashes = set(), set()
    for item in sorted(assets, key=lambda item: (item["class_id"], item["path"])):
        if item["sha256"] in hashes:
            excluded.append(item)
            continue
        hashes.add(item["sha256"])
        if item["class_id"] in classes:
            held_out.append(item)
        else:
            classes.add(item["class_id"])
            references.append(item)
    return references, held_out, excluded


def load_assets(asset_root):
    manifest = json.loads((asset_root / "SOURCE_MANIFEST.json").read_text(encoding="utf-8"))
    assets = []
    for item in manifest["files"]:
        if "error" in item:
            continue
        name = Path(item["path"]).name
        matches = [key for key in CLASS_LABELS if name.startswith(key + "_")]
        if len(matches) != 1:
            raise ValueError(f"unmapped reference image: {name}")
        path = asset_root / item["path"]
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != item["sha256"]:
            raise ValueError(f"reference digest mismatch: {path}")
        assets.append({**item, "class_id": matches[0], "label": CLASS_LABELS[matches[0]],
                       "absolute_path": str(path.resolve())})
    return assets, manifest


def project_image(image, corners, frame_size=(640, 480)):
    height, width = image.shape[:2]
    source = np.float32([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]])
    transform = cv2.getPerspectiveTransform(source, np.float32(corners))
    frame = cv2.warpPerspective(image, transform, frame_size, borderValue=(32, 32, 32))
    source_center = np.float32([[[(width - 1) / 2, (height - 1) / 2]]])
    center = cv2.perspectiveTransform(source_center, transform)[0, 0]
    return frame, center.astype(float).tolist()


def make_scene(image, rng, mode):
    height, width = image.shape[:2]
    target_width = float(rng.uniform(170, 460)) if mode in ("scale", "combined") else 420.0
    scale = min(target_width / width, 320.0 / height)
    shown_w, shown_h = (width - 1) * scale, (height - 1) * scale
    cx, cy = 320.0, 240.0
    corners = np.float32([[cx - shown_w / 2, cy - shown_h / 2],
                          [cx + shown_w / 2, cy - shown_h / 2],
                          [cx + shown_w / 2, cy + shown_h / 2],
                          [cx - shown_w / 2, cy + shown_h / 2]])
    if mode in ("perspective", "combined"):
        corners += rng.uniform(-0.12, 0.12, (4, 2)).astype(np.float32) * [shown_w, shown_h]
    frame, center = project_image(image, corners)
    alpha, beta = 1.0, 0.0
    if mode in ("brightness", "combined"):
        alpha, beta = float(rng.uniform(0.35, 1.35)), float(rng.uniform(0, 35))
        frame = np.clip(frame.astype(np.float32) * alpha + beta, 0, 255).astype(np.uint8)
    parameters = {"mode": mode, "corners_px": corners.astype(float).tolist(),
                  "brightness_alpha": alpha, "brightness_beta": beta,
                  "target_size_before_perspective_px": [shown_w, shown_h]}
    return frame, center, parameters


def score_detections(detections, expected_class, expected_center):
    predicted = sorted({item["target_id"] for item in detections})
    result = {"predicted_classes": predicted, "center_error_px": None}
    if not predicted:
        result["status"] = "correct_rejection" if expected_class is None else "rejected"
    elif expected_class is None:
        result["status"] = "false_accept"
    elif len(predicted) > 1:
        result["status"] = "multiple_classes"
    elif predicted[0] != expected_class:
        result["status"] = "misidentified"
    else:
        result["status"] = "correct"
        accepted = max(detections, key=lambda item: item.get("inlier_count", 0))
        result["center_error_px"] = float(np.linalg.norm(
            np.asarray(accepted["center_px"]) - np.asarray(expected_center)))
    return result


def _aggregate(samples):
    counts = Counter(item["status"] for item in samples)
    timings = [item["inference_ms"] for item in samples]
    errors = [item["center_error_px"] for item in samples if item["center_error_px"] is not None]
    return {"count": len(samples), "correct": counts["correct"],
            "rejected": counts["rejected"], "misidentified": counts["misidentified"],
            "multiple_classes": counts["multiple_classes"],
            "false_accept": counts["false_accept"], "correct_rejection": counts["correct_rejection"],
            "correct_rate": counts["correct"] / len(samples) if samples else None,
            "inference_ms_mean": float(np.mean(timings)) if timings else None,
            "inference_ms_median": float(np.median(timings)) if timings else None,
            "inference_ms_p95": float(np.percentile(timings, 95)) if timings else None,
            "correct_center_error_px_mean": float(np.mean(errors)) if errors else None,
            "correct_center_error_px_max": float(np.max(errors)) if errors else None}


def summarize(samples):
    groups = {group: _aggregate([sample for sample in samples if sample["group"] == group])
              for group in sorted({sample["group"] for sample in samples})}
    groups["held_out_generalization"] = _aggregate(
        [sample for sample in samples if sample["group"].startswith("held_out_")])
    return groups


def evaluate(asset_root, seed=20261111, repeats=3):
    cv2.setNumThreads(1)
    cv2.setRNGSeed(seed)
    rng = np.random.default_rng(seed)
    assets, manifest = load_assets(asset_root)
    references, held_out, excluded = split_assets(assets)
    start = perf_counter()
    matcher = vision_core.TemplateMatcher({item["class_id"]: item["absolute_path"] for item in references})
    initialization_ms = (perf_counter() - start) * 1000
    samples = []

    def measure(frame, expected, center, group, source, parameters):
        cv_seed = seed + len(samples)
        cv2.setRNGSeed(cv_seed)
        start = perf_counter()
        detections = vision_core.detect_targets(
            frame, enable_qr=False, enable_aruco=False, template_matcher=matcher)
        elapsed = (perf_counter() - start) * 1000
        samples.append({"sample_id": len(samples), "group": group, "expected_class": expected,
                        "source_path": source, "expected_center_px": center,
                        "transform": parameters, "opencv_rng_seed": cv_seed,
                        "inference_ms": elapsed, "detections": detections,
                        **score_detections(detections, expected, center)})

    for role, collection in (("reference", references), ("held_out", held_out)):
        for item in collection:
            image = cv2.imdecode(np.frombuffer(Path(item["absolute_path"]).read_bytes(), np.uint8), cv2.IMREAD_COLOR)
            frame, center, parameters = make_scene(image, rng, "identity")
            group = "reference_replay" if role == "reference" else "held_out_clean"
            measure(frame, item["class_id"], center, group, item["path"], parameters)
            for mode in ("perspective", "scale", "brightness", "combined"):
                for repeat in range(repeats):
                    frame, center, parameters = make_scene(image, rng, mode)
                    parameters["repeat"] = repeat
                    measure(frame, item["class_id"], center, role + "_" + mode, item["path"], parameters)
    negatives = [("blank", np.full((480, 640, 3), 128, np.uint8)),
                 ("noise", rng.integers(0, 256, (480, 640, 3), dtype=np.uint8))]
    synthetic = np.full((480, 640, 3), 200, np.uint8)
    for _ in range(60):
        center = tuple(int(v) for v in rng.integers([0, 0], [640, 480]))
        color = tuple(int(v) for v in rng.integers(0, 256, 3))
        cv2.circle(synthetic, center, int(rng.integers(3, 20)), color, -1)
    negatives.append(("unrelated_synthetic", synthetic))
    for name, frame in negatives:
        measure(frame, None, None, "negative_controls", None, {"mode": name})
    return {
        "schema": "inspection.offline_reference_vision.v1", "created_at": datetime.now(timezone.utc).isoformat(),
        "scope": "Offline synthetic photo-on-canvas tests; no official simulator, ROS transport or Gazebo run.",
        "seed": seed, "repeats_per_perturbation": repeats, "frame_size_px": [640, 480],
        "environment": {"python": platform.python_version(), "opencv": cv2.__version__,
                        "numpy": np.__version__, "platform": platform.platform(), "opencv_threads": 1},
        "matcher": {"path": str(Path(vision_core.__file__).relative_to(ROOT)),
                    "sha256": hashlib.sha256(Path(vision_core.__file__).read_bytes()).hexdigest(),
                    "threshold_changes": False, "initialization_ms": initialization_ms},
        "source_repository": manifest["repository"], "source_commit_sha": manifest["commit_sha"],
        "license": manifest.get("license"), "class_labels": CLASS_LABELS,
        "reference_assets": references, "held_out_assets": held_out, "excluded_duplicates": excluded,
        "protocol": {
            "split": "One lexicographically first available file per real class; remaining distinct hashes held out.",
            "reference_replay": "Same photograph as the registered template, resampled onto a plain dark canvas; not generalization.",
            "perturbations": "3 draws each: perspective corner jitter +/-12%; width 170..460px with height cap 320px; brightness alpha .35..1.35 beta 0..35; combined.",
            "centering": "True center is the projective image of the source surface center, not the average of corners.",
            "timing": "perf_counter around detect_targets only; includes normalization and all 7 reference comparisons, excludes image loading/warping.",
            "limitations": "Only 12 third-party images, 5 held-out photos, single target, plain background, no blur/occlusion/video. Perturbations are correlated repeats, not additional independent photos. No tuning or improvement claim."},
        "summary": summarize(samples), "samples": samples,
    }


def write_markdown(report, path):
    replay = report["summary"]["reference_replay"]
    held_out = report["summary"]["held_out_clean"]
    negative = report["summary"]["negative_controls"]
    finding = (f"本次结果：参考图重放 {replay['correct']}/{replay['count']}，"
               f"不同照片保留图 {held_out['correct']}/{held_out['count']}；"
               f"负对照正确拒识 {negative['correct_rejection']}/{negative['count']}，"
               f"误接受 {negative['false_accept']}。")
    if held_out["correct"] == 0:
        finding += "这批结果没有支持跨照片的类别泛化。"
    lines = ["# 七类参考图离线识别基线（2026-11-11）", "",
             finding, "",
             "使用现有 `vision_core.TemplateMatcher`，未修改阈值、业务文件或默认配置。"
             "这是本机离线照片投影测试，尚未运行官方仿真、ROS 图像链路或裁判系统。", "",
             f"固定种子 `{report['seed']}`；7 类各 1 张参考，5 张不同照片保留测试。"
             "训练参考自身及其扰动均单独列出，不计入保留图识别率。", "",
             "| 测试组 | 样本 | 正确 | 拒识 | 错类/多类 | 正确率 | 平均耗时ms | 正确时中心均误差px |",
             "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for group, item in report["summary"].items():
        rate = f"{item['correct_rate']:.1%}" if item["correct_rate"] is not None else "—"
        error = f"{item['correct_center_error_px_mean']:.2f}" if item["correct_center_error_px_mean"] is not None else "—"
        lines.append(f"| {group} | {item['count']} | {item['correct']} | {item['rejected']} | "
                     f"{item['misidentified'] + item['multiple_classes']} | {rate} | "
                     f"{item['inference_ms_mean']:.2f} | {error} |")
    lines += ["", "负对照的正确拒识与误接受数见 JSON，表中正确率只统计目标识别。"
              "`held_out_generalization` 汇总保留照片及其扰动，重复变换相关，不能当作独立样本估计。", "",
              "方法：640×480 纯色画布，透视角点扰动±12%，投影宽度170–460px、高度上限320px，"
              "亮度倍率0.35–1.35、偏置0–35；每种扰动3次。中心真值由源照片中心经同一单应变换投影获得。"
              "计时包含归一化和7类匹配，不含文件加载、投影与模板初始化。", "",
              f"素材：{report['source_repository']} ，固定提交 `{report['source_commit_sha']}`。"
              "12张成功下载的图片及SHA256、参考/保留分组、每帧变换、误识/拒识、中心误差和耗时均记录在同名JSON。"
              "仓库未声明许可，素材并非经核验的当前官方图库。", "",
              "仅5张保留照片，坍塌废墟和受损车辆没有保留图；纯背景、单目标、无运动模糊/遮挡，"
              "不能代表赛场准确率。没有进行阈值调参或算法比较，不主张任何改善收益。"]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset-root", type=Path, default=DEFAULT_ASSETS)
    parser.add_argument("--seed", type=int, default=20261111)
    parser.add_argument("--output", type=Path, default=ROOT / "docs" / "reference_vision_20261111.json")
    args = parser.parse_args()
    report = evaluate(args.asset_root, seed=args.seed)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    write_markdown(report, args.output.with_suffix(".md"))
    print(json.dumps(report["summary"], ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
