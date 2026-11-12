"""Optional fixed-protocol CLIP experiment, separate from mission defaults.

Consumes scenes recorded by evaluate_reference_vision.py. No template training,
hold-out prompt selection, threshold tuning, network access or ROS is performed.
The official model source and verified weights must already be local.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
from time import perf_counter

import cv2
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.evaluate_reference_vision import DEFAULT_ASSETS, project_image

PROMPTS = {
    "collapsed_ruins": "a photo of collapsed buildings and earthquake rubble.",
    "damaged_vehicle": "a photo of a damaged wrecked vehicle.",
    "landslide": "a photo of a landslide with soil and rocks covering a road.",
    "rescue_camp": "a photo of a rescue camp with emergency tents and supplies.",
    "trapped_people": "a photo of people trapped under rubble being rescued.",
    "window_bird": "a photo of a bird flying in the sky.",
    "window_target": "a photo of a rescue drone, a quadcopter flying in the sky.",
}
MIN_COSINE = 0.20
MIN_MARGIN = 0.02


def classify_cosines(scores, class_ids):
    values = np.asarray(scores, dtype=np.float64)
    if (values.ndim != 1 or len(values) != len(class_ids) or len(values) < 2
            or not np.isfinite(values).all()):
        raise ValueError("expected at least two finite class cosine similarities")
    order = np.argsort(-values, kind="stable")
    best, runner_up = float(values[order[0]]), float(values[order[1]])
    margin = best - runner_up
    decision = "accepted"
    if best < MIN_COSINE:
        decision = "rejected_low_similarity"
    elif margin < MIN_MARGIN:
        decision = "rejected_low_margin"
    return {"decision": decision, "predicted_class": class_ids[order[0]] if decision == "accepted" else None,
            "closed_set_class": class_ids[order[0]], "top1_cosine": best, "top2_cosine": runner_up,
            "margin": margin, "cosines": {key: float(value) for key, value in zip(class_ids, values)}}


def replay_frame(sample, asset_root):
    path = asset_root / sample["source_path"]
    image = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"cannot decode reference scene source: {path}")
    parameters = sample["transform"]
    frame, _ = project_image(image, np.float32(parameters["corners_px"]))
    frame = np.clip(frame.astype(np.float32) * parameters["brightness_alpha"]
                    + parameters["brightness_beta"], 0, 255).astype(np.uint8)
    return Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))


def _aggregate(samples):
    count = len(samples)
    positives = [item for item in samples if item["expected_class"] is not None]
    negatives = [item for item in samples if item["expected_class"] is None]
    correct = sum(item["predicted_class"] == item["expected_class"] for item in positives)
    rejected = sum(item["predicted_class"] is None for item in positives)
    closed = sum(item["closed_set_class"] == item["expected_class"] for item in positives)
    false_accept = sum(item["predicted_class"] is not None for item in negatives)
    timings = [item["inference_ms"] for item in samples]
    return {"count": count, "correct": correct, "rejected": rejected,
            "misidentified": len(positives) - correct - rejected,
            "false_accept": false_accept, "correct_rejection": len(negatives) - false_accept,
            "correct_rate": correct / len(positives) if positives else None,
            "closed_set_correct": closed, "closed_set_correct_rate": closed / len(positives) if positives else None,
            "inference_ms_mean": float(np.mean(timings)) if count else None,
            "inference_ms_median": float(np.median(timings)) if count else None,
            "inference_ms_p95": float(np.percentile(timings, 95)) if count else None}


def negative_frames(baseline):
    """Replay the baseline RNG stream to reproduce its exact negative controls."""
    rng = np.random.default_rng(baseline["seed"])
    for sample in baseline["samples"]:
        if sample["source_path"] is None:
            continue
        mode = sample["transform"]["mode"]
        if mode in ("scale", "combined"):
            rng.uniform(170, 460)
        if mode in ("perspective", "combined"):
            rng.uniform(-0.12, 0.12, (4, 2))
        if mode in ("brightness", "combined"):
            rng.uniform(0.35, 1.35)
            rng.uniform(0, 35)
    frames = {"blank": np.full((480, 640, 3), 128, np.uint8),
              "noise": rng.integers(0, 256, (480, 640, 3), dtype=np.uint8)}
    synthetic = np.full((480, 640, 3), 200, np.uint8)
    for _ in range(60):
        center = tuple(int(value) for value in rng.integers([0, 0], [640, 480]))
        color = tuple(int(value) for value in rng.integers(0, 256, 3))
        cv2.circle(synthetic, center, int(rng.integers(3, 20)), color, -1)
    frames["unrelated_synthetic"] = synthetic
    return {key: Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)) for key, frame in frames.items()}


def _write_report(report, output):
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = ["# CLIP七类零样本离线实验（2026-11-11）", "",
             "独立可选实验；未修改任务默认识别器、配置或业务文件。英文提示与拒识阈值在运行前固定，"
             "没有用五张保留图挑选提示或调阈值。", "",
             f"状态：`{report['status']}`。"]
    if report["status"] != "completed":
        lines += ["", f"阻断：`{report.get('error', '模型尚未运行')}`。没有可报告的模型准确率或耗时。"]
    else:
        clean = report["comparison"]["held_out_clean"]["clip"]
        held = report["comparison"]["held_out_generalization"]["clip"]
        negative = report["comparison"].get("negative_controls", {}).get("clip")
        lines += ["", f"结果：5张不同照片保留图 {clean['correct']}/{clean['count']}；"
                  f"保留图及其扰动 {held['correct']}/{held['count']} 正确、{held['rejected']} 拒识、"
                  f"{held['misidentified']} 接受错类。"]
        if negative:
            lines += [f"同基线负对照正确拒识 {negative['correct_rejection']}/{negative['count']}，"
                      f"误接受 {negative['false_accept']}。"]
        lines += ["", "| 同一离线条件 | 样本 | 模板匹配正确 | CLIP正确 | CLIP拒识 | CLIP错类 | CLIP不拒识top1 | 模板平均ms | CLIP平均ms |",
                  "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
        for group, values in report["comparison"].items():
            old, new = values["template_matcher"], values["clip"]
            lines.append(f"| {group} | {new['count']} | {old['correct']} | {new['correct']} | "
                         f"{new['rejected']} | {new['misidentified']} | {new['closed_set_correct']} | "
                         f"{old['inference_ms_mean']:.2f} | {new['inference_ms_mean']:.2f} |")
    lines += ["", "模型：OpenAI官方CLIP ViT-B/32；逐张重放既有JSON的同一640×480画布和变换，"
              "CLIP使用官方224像素预处理。预设接受条件：top1余弦≥0.20且top1-top2≥0.02；"
              "这两个数是未经校准的探索阈值，不是可靠性或概率保证。", "",
              "推理计时包含PIL/CLIP预处理、CPU到GPU传输和图像编码，排除照片读取/场景投影、"
              "模型加载与固定文本编码；CUDA同步后计时。模板匹配在CPU上运行，设备和预处理不同，"
              "耗时仅说明本机配置下的开销。", "",
              "CLIP在此只输出整帧类别，不定位目标，所以没有中心误差结果，也没有完成激光对准能力。"
              "这仍是5张保留照片及其相关扰动的离线实验，没有运行官方仿真、相机链路或裁判系统；"
              "不能视为官方赛场泛化成绩。", "",
              "模型源与MIT许可证保存在external_resources/openai_CLIP/；权重必须通过官方URL标明的SHA256。"
              "全部固定提示、阈值、逐帧余弦分数/拒识决定/耗时和模型来源见同名JSON。"]
    output.with_suffix(".md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(args):
    output = args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    baseline = json.loads(args.baseline.read_text(encoding="utf-8"))
    report = {"schema": "inspection.offline_zero_shot_vision.v1", "status": "started",
              "created_at": datetime.now(timezone.utc).isoformat(),
              "protocol_frozen_at": datetime.now(timezone.utc).isoformat(),
              "prompts": PROMPTS, "rejection": {"min_cosine": MIN_COSINE, "min_margin": MIN_MARGIN,
                                                  "calibrated": False, "hold_out_tuning": False},
              "baseline_path": str(args.baseline),
              "baseline_sha256": hashlib.sha256(args.baseline.read_bytes()).hexdigest(),
              "scope": "Same synthetic scenes, optional whole-frame zero-shot classes; no official platform run.",
              "samples": []}
    _write_report(report, output)
    try:
        protocol_path = output.with_name("zero_shot_vision_protocol_20261111.json")
        if protocol_path.exists():
            frozen = json.loads(protocol_path.read_text(encoding="utf-8"))
            if frozen["prompts"] != report["prompts"] or frozen["rejection"] != report["rejection"]:
                raise RuntimeError("prompts or rejection policy differ from the frozen protocol")
            report["protocol_frozen_at"] = frozen["frozen_at"]
            report["protocol_record_sha256"] = hashlib.sha256(protocol_path.read_bytes()).hexdigest()
        model_record = json.loads((args.model_root / "MODEL_SOURCE.json").read_text(encoding="utf-8"))
        if model_record["status"] != "verified":
            raise RuntimeError("official weight download is not verified: " + str(model_record))
        weights = args.model_root / "weights" / "ViT-B-32.pt"
        with weights.open("rb") as stream:
            digest = hashlib.file_digest(stream, "sha256").hexdigest()
        if digest != model_record["expected_sha256"]:
            raise RuntimeError("official model weight SHA256 mismatch")
        report["model_source"] = model_record
        report["implementation_source"] = json.loads(
            (args.model_root / "OFFICIAL_SOURCE_MANIFEST.json").read_text(encoding="utf-8"))
        for item in report["implementation_source"]["files"]:
            source = args.model_root / "official_source" / item["path"]
            if hashlib.sha256(source.read_bytes()).hexdigest() != item["sha256"]:
                raise RuntimeError("official implementation digest mismatch: " + item["path"])
        for item in baseline["reference_assets"] + baseline["held_out_assets"]:
            source = args.asset_root / item["path"]
            if hashlib.sha256(source.read_bytes()).hexdigest() != item["sha256"]:
                raise RuntimeError("baseline photo digest mismatch: " + item["path"])
        sys.path.insert(0, str((args.model_root / "official_source").resolve()))
        import clip
        import torch
        torch.manual_seed(baseline["seed"])
        torch.set_num_threads(2)
        device = "cuda" if torch.cuda.is_available() else "cpu"
        started = perf_counter()
        model, preprocess = clip.load(str(weights), device=device, jit=False)
        model.eval()
        report["model_loading_ms"] = (perf_counter() - started) * 1000
        report["environment"] = {"torch": torch.__version__, "device": device,
                                  "cuda_device": torch.cuda.get_device_name(0) if device == "cuda" else None,
                                  "torch_threads": torch.get_num_threads()}
        classes = list(PROMPTS)
        controls = negative_frames(baseline)
        with torch.no_grad():
            text_features = model.encode_text(clip.tokenize(list(PROMPTS.values())).to(device))
            text_features = text_features / text_features.norm(dim=-1, keepdim=True)
            # Warm up before measuring; this frame is never scored.
            model.encode_image(preprocess(Image.new("RGB", (640, 480), (128, 128, 128))).unsqueeze(0).to(device))
            if device == "cuda":
                torch.cuda.synchronize()
            for sample in baseline["samples"]:
                frame = (controls[sample["transform"]["mode"]] if sample["source_path"] is None
                         else replay_frame(sample, args.asset_root))
                started = perf_counter()
                tensor = preprocess(frame).unsqueeze(0).to(device)
                features = model.encode_image(tensor)
                features = features / features.norm(dim=-1, keepdim=True)
                cosines = (features @ text_features.T).float().cpu().numpy()[0]
                if device == "cuda":
                    torch.cuda.synchronize()
                elapsed = (perf_counter() - started) * 1000
                record = {"sample_id": sample["sample_id"], "group": sample["group"],
                          "source_path": sample["source_path"], "expected_class": sample["expected_class"],
                          "frame_rgb_sha256": hashlib.sha256(frame.tobytes()).hexdigest(),
                          "inference_ms": elapsed, **classify_cosines(cosines, classes)}
                report["samples"].append(record)
        groups = sorted({sample["group"] for sample in report["samples"]})
        groups.append("held_out_generalization")
        report["comparison"] = {}
        for group in groups:
            selected = [item for item in report["samples"] if
                        (item["group"].startswith("held_out_") if group == "held_out_generalization"
                         else item["group"] == group)]
            report["comparison"][group] = {"template_matcher": baseline["summary"][group], "clip": _aggregate(selected)}
        report["status"] = "completed"
    except Exception as exc:
        report.update(status="failed", error=type(exc).__name__ + ": " + str(exc))
    _write_report(report, output)
    print(json.dumps({"status": report["status"], "error": report.get("error"),
                      "comparison": report.get("comparison")}, ensure_ascii=False, indent=2))
    return 0 if report["status"] == "completed" else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset-root", type=Path, default=DEFAULT_ASSETS)
    parser.add_argument("--baseline", type=Path, default=ROOT / "docs" / "reference_vision_20261111.json")
    parser.add_argument("--model-root", type=Path, default=ROOT / "external_resources" / "openai_CLIP")
    parser.add_argument("--output", type=Path, default=ROOT / "docs" / "zero_shot_vision_20261111.json")
    return run(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
