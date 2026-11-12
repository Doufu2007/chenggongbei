"""Fixed SIFT geometry experiment on two retained real JPEGs. Offline only.

Four inputs only: each full scene and one manual 1:1 rectification.
Every public reference is retained; no semantic class truth is assigned.
"""
from __future__ import annotations

import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
QUAD_V17 = np.float32([[280, 477], [523, 477], [592, 532], [204, 532]])


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_image(path):
    image = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Could not decode {path}")
    return image


def save_image(path, image):
    ok, encoded = cv2.imencode(path.suffix, image)
    if not ok:
        raise ValueError(f"Could not encode {path}")
    path.write_bytes(encoded.tobytes())


def coverage(points, shape):
    h, w = shape[:2]
    p = np.asarray(points, np.float32).reshape(-1, 2)
    if not len(p):
        return {"count": 0, "unique_xy": 0, "hull_area_fraction": 0.0,
                "bbox_width_fraction": 0.0, "bbox_height_fraction": 0.0, "grid_4x4_cells": 0}
    area = float(cv2.contourArea(cv2.convexHull(p))) if len(p) >= 3 else 0.0
    bins = np.floor(p / np.array([w, h]) * 4).clip(0, 3).astype(int)
    return {"count": len(p), "unique_xy": len(np.unique(p.round(3), axis=0)),
            "hull_area_fraction": area / (w * h), "hull_area_px2": area,
            "bbox_xyxy": [float(p[:, 0].min()), float(p[:, 1].min()),
                           float(p[:, 0].max()), float(p[:, 1].max())],
            "bbox_width_fraction": float(np.ptp(p[:, 0]) / w),
            "bbox_height_fraction": float(np.ptp(p[:, 1]) / h),
            "grid_4x4_cells": len(np.unique(bins, axis=0))}


def main():
    OUT.mkdir(exist_ok=True)
    (OUT / "matches").mkdir(exist_ok=True)
    cv2.setNumThreads(1)
    cv2.setRNGSeed(20261112)
    if not hasattr(cv2, "SIFT_create"):
        raise RuntimeError("Installed OpenCV has no SIFT; do not install dependencies.")
    prior_path = ROOT / "runs/analysis/20261112_v16b_rectification_exploratory/report.json"
    prior = json.loads(prior_path.read_text(encoding="utf-8"))
    sources = []
    for name, relative, phase in [
        ("v16b", prior["source_image"]["path"], "last"),
        ("v17", "runs/analysis/20261112_approach_v17/native_L000017_46381698801c0842.jpg", "last_associated"),
    ]:
        path = ROOT / relative
        summary_path = path.parent / "summary.json"
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        meta = next(x for x in summary["native_images"] if x["file_name"] == path.name)
        assert meta["phase"] == phase and digest(path) == meta["sha256"]
        sources.append({"name": name, "path": path, "image": read_image(path),
                        "metadata": meta, "summary_sha256": digest(summary_path)})

    queries = []
    for source in sources:
        name = source["name"]
        raw = source["image"]
        queries.append({"name": name + "_full_scene", "image": raw,
                        "source_name": name, "manual_rectification": False})
        if name == "v16b":
            prior_warp = prior_path.parent / "warp_square.png"
            expected = next(c["sha256"] for c in prior["candidates"] if c["name"] == "warp_square")
            assert digest(prior_warp) == expected
            warp = read_image(prior_warp)
            quad = np.array(prior["manual_annotation"]["quad_xy"], np.float32)
            transform = next(c["homography_source_to_output"] for c in prior["candidates"] if c["name"] == "warp_square")
        else:
            quad = QUAD_V17
            transform = cv2.getPerspectiveTransform(quad, np.float32([[0, 0], [383, 0], [383, 383], [0, 383]]))
            warp = cv2.warpPerspective(raw, transform, (384, 384), flags=cv2.INTER_LINEAR)
            transform = transform.tolist()
        overlay = raw.copy()
        cv2.polylines(overlay, [quad.astype(int)], True, (0, 0, 255), 2)
        save_image(OUT / (name + "_manual_quad.png"), overlay)
        queries.append({"name": name + "_manual_warp_square", "image": warp,
                        "source_name": name, "manual_rectification": True,
                        "manual_quad_xy": quad.tolist(), "homography_source_to_query": transform,
                        "quad_status": "Manual visual boundary annotation, not runtime detection."})

    sift = cv2.SIFT_create()
    bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
    references = []
    for meta in prior["references_in_column_order"]:
        path = Path(meta["absolute_path"])
        assert digest(path) == meta["sha256"]
        image = read_image(path)
        kp, des = sift.detectAndCompute(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), None)
        references.append({"id": path.name, "meta": meta, "image": image, "kp": kp, "des": des})
    for q in queries:
        save_image(OUT / (q["name"] + ".png"), q["image"])
        q["kp"], q["des"] = sift.detectAndCompute(cv2.cvtColor(q["image"], cv2.COLOR_BGR2GRAY), None)

    results = []
    for q in queries:
        cards = []
        for ref in references:
            started = perf_counter()
            pairs = bf.knnMatch(ref["des"], q["des"], k=2) if ref["des"] is not None and q["des"] is not None and len(q["des"]) >= 2 else []
            good = [a for pair in pairs if len(pair) == 2 for a, b in [pair] if a.distance < .75 * b.distance]
            src = np.float32([ref["kp"][m.queryIdx].pt for m in good]).reshape(-1, 2)
            dst = np.float32([q["kp"][m.trainIdx].pt for m in good]).reshape(-1, 2)
            H = mask = None
            if len(good) >= 8:
                cv2.setRNGSeed(20261112)
                H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
            flags = mask.ravel().astype(bool) if mask is not None else np.zeros(len(good), bool)
            inlier_count = int(flags.sum())
            item = {"query": q["name"], "reference": ref["id"], "reference_class_annotation": ref["meta"]["class_id"],
                    "reference_keypoints": len(ref["kp"]), "query_keypoints": len(q["kp"]),
                    "knn_pairs": len(pairs), "good_matches": len(good), "min8_gate_passed": len(good) >= 8,
                    "homography_computed": H is not None, "inliers": inlier_count,
                    "inlier_fraction_of_good": inlier_count / len(good) if good else 0.0,
                    "good_reference_coverage": coverage(src, ref["image"].shape),
                    "good_query_coverage": coverage(dst, q["image"].shape),
                    "inlier_reference_coverage": coverage(src[flags], ref["image"].shape),
                    "inlier_query_coverage": coverage(dst[flags], q["image"].shape),
                    "homography_reference_to_query": H.tolist() if H is not None else None,
                    "projected_reference_quad_xy": None,
                    "matches": [{"reference_xy": src[i].tolist(), "query_xy": dst[i].tolist(),
                                 "distance": float(m.distance), "inlier": bool(flags[i])} for i, m in enumerate(good)]}
            query_display = q["image"].copy()
            if H is not None:
                rh, rw = ref["image"].shape[:2]
                corners = np.float32([[0, 0], [rw-1, 0], [rw-1, rh-1], [0, rh-1]])
                projected = cv2.perspectiveTransform(corners[None], H)[0]
                item["projected_reference_quad_xy"] = projected.tolist()
                finite = bool(np.isfinite(projected).all())
                area = abs(float(cv2.contourArea(projected))) if finite else None
                item["projected_quad_finite"] = finite
                item["projected_quad_convex"] = bool(cv2.isContourConvex(projected)) if finite else False
                item["projected_quad_area_px2"] = area
                item["inlier_query_hull_fraction_of_projected_quad"] = item["inlier_query_coverage"].get("hull_area_px2", 0) / area if area else None
                errors = np.linalg.norm(cv2.perspectiveTransform(src[None], H)[0] - dst, axis=1)
                item["inlier_reprojection_error_px"] = {"median": float(np.median(errors[flags])), "max": float(errors[flags].max())} if inlier_count else None
                if finite and abs(projected).max() < 1e6:
                    cv2.polylines(query_display, [np.rint(projected).astype(int)], True, (0, 255, 255), 2)
            item["matching_ms"] = (perf_counter() - started) * 1000
            shown = cv2.drawMatches(ref["image"], ref["kp"], query_display, q["kp"], good, None,
                                   matchesMask=flags.astype(int).tolist() if H is not None else None,
                                   matchColor=(0, 255, 0), singlePointColor=(128, 128, 128),
                                   flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            item["visualization_lines"] = "RANSAC inliers only" if H is not None else "All ratio-test good matches; no homography"
            title = f"{q['name']} / {ref['id']}: good={len(good)} inlier={inlier_count} H={H is not None}"
            if shown.shape[1] > 1600:
                shown = cv2.resize(shown, None, fx=1600/shown.shape[1], fy=1600/shown.shape[1])
            cv2.putText(shown, title, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 0, 255), 1, cv2.LINE_AA)
            visual = OUT / "matches" / f"{q['name']}__{ref['id']}.jpg"
            save_image(visual, shown)
            item["visualization_path"] = visual.relative_to(OUT).as_posix()
            card = Image.fromarray(cv2.cvtColor(shown, cv2.COLOR_BGR2RGB))
            card.thumbnail((760, 330))
            tile = Image.new("RGB", (780, 360), "white")
            tile.paste(card, (10, 25))
            ImageDraw.Draw(tile).text((10, 6), f"{ref['id']} good={len(good)} inlier={inlier_count}", fill="black")
            cards.append(tile)
            results.append(item)
        sheet = Image.new("RGB", (780*3, 360*4), "#dddddd")
        for i, card in enumerate(cards):
            sheet.paste(card, ((i % 3)*780, (i // 3)*360))
        sheet.save(OUT / (q["name"] + "_all_reference_matches.jpg"))

    report = {"kind": "fixed_sift_geometry_exploratory", "created_at": datetime.now(timezone.utc).isoformat(),
              "opencv_version": cv2.__version__, "protocol": {"SIFT": "cv2.SIFT_create() default parameters",
              "matcher": "BFMatcher(NORM_L2, crossCheck=False), reference descriptors -> query descriptors",
              "knn_k": 2, "ratio_threshold": .75, "minimum_good_matches_for_homography": 8,
              "homography": "cv2.findHomography(reference_xy, query_xy, cv2.RANSAC, 3.0)",
              "ransac_reprojection_threshold_px": 3.0, "seed": 20261112, "threshold_tuning": False,
              "extra_rotations": False, "class_acceptance_rule": None},
              "sources": [{"name": s["name"], "path": s["path"].relative_to(ROOT).as_posix(),
                           "sha256": digest(s["path"]), "summary_sha256": s["summary_sha256"],
                           "metadata": s["metadata"]} for s in sources],
              "queries": [{k: v for k, v in q.items() if k not in ("image", "kp", "des")} |
                          {"path": q["name"] + ".png", "sha256": digest(OUT/(q["name"]+'.png')),
                           "size_wh": list(q["image"].shape[1::-1]), "keypoints": len(q["kp"])} for q in queries],
              "references": [r["meta"] for r in references], "results": results,
              "class_ground_truth": None, "recognized_class": None, "deployed": False,
              "limitations": ["Two retained JPEGs; four correlated inputs, not independent accuracy samples.",
              "Manual board ROI and assumed square geometry are not automatic detection.",
              "Computed homography or min8 gate alone is not recognition; inspect inlier count, uniqueness, coverage, projected shape and residuals.",
              "Third-party reference library is not verified as current official simulator textures.",
              "Resampling creates no new measured detail; no CLIP inference, network access, or platform action."]}
    (OUT / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding="utf-8")
    with (OUT / "scores.csv").open("w", newline="", encoding="utf-8-sig") as f:
        fields = ["query", "reference", "good_matches", "inliers", "inlier_fraction_of_good", "min8_gate_passed", "homography_computed", "reference_inlier_hull_fraction", "query_inlier_hull_fraction", "projected_quad_area_px2"]
        writer = csv.DictWriter(f, fieldnames=fields); writer.writeheader()
        for row in results:
            values = {k: row.get(k) for k in fields}
            values["reference_inlier_hull_fraction"] = row["inlier_reference_coverage"]["hull_area_fraction"]
            values["query_inlier_hull_fraction"] = row["inlier_query_coverage"]["hull_area_fraction"]
            writer.writerow(values)
    print(json.dumps({"queries": len(queries), "references": len(references), "results": len(results),
                      "min8_passed": sum(x["min8_gate_passed"] for x in results),
                      "homographies": sum(x["homography_computed"] for x in results)}, indent=2))
    for q in queries:
        rows = sorted([x for x in results if x["query"] == q["name"]], key=lambda x: (x["inliers"], x["good_matches"]), reverse=True)
        print(q["name"], [(x["reference"], x["good_matches"], x["inliers"]) for x in rows[:3]])


if __name__ == "__main__":
    main()
