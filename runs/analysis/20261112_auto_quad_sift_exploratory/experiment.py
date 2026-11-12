"""Blind, fixed document-scanner quadrilaterals, then fixed SIFT diagnostics.

Only this new exploratory directory is written. Manual corners are loaded after
automatic detection/matching and used solely for development-image evaluation.
"""
from __future__ import annotations

import csv
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
FROZEN = ROOT / "runs/analysis/20261112_sift_geometry_exploratory"
spec = importlib.util.spec_from_file_location("frozen_sift_utilities", FROZEN / "experiment.py")
utilities = importlib.util.module_from_spec(spec)
spec.loader.exec_module(utilities)
digest, read_image, save_image, coverage = utilities.digest, utilities.read_image, utilities.save_image, utilities.coverage

DETECTOR = {"grayscale": True, "gaussian_kernel": [5, 5], "gaussian_sigma": 0,
            "canny_thresholds": [50, 150], "contour_mode": "RETR_LIST",
            "contour_approximation": "CHAIN_APPROX_SIMPLE", "approx_perimeter_fraction": .02,
            "minimum_area_px2_exclusive": 1000, "maximum_area_fraction_exclusive": .60,
            "maximum_minAreaRect_long_short_ratio": 8.0, "convex_four_points_required": True,
            "image_bound_required": True, "HSV_filter": False, "morphology": False}
GEOMETRY_GATE = {"minimum_inliers": 8, "minimum_distinct_xy_each_side": 8,
                 "minimum_hull_area_fraction_each_side": .05,
                 "finite_convex_projection_required": True,
                 "projected_area_fraction_of_query_range": [.10, 2.0],
                 "calibrated": False, "class_acceptance_rule": None}


def order_quad(points):
    p = np.asarray(points, np.float32).reshape(4, 2)
    center = p.mean(axis=0)
    p = p[np.argsort(np.arctan2(p[:, 1]-center[1], p[:, 0]-center[0]))]
    p = np.roll(p, -int(np.argmin(p.sum(axis=1))), axis=0)
    if cv2.contourArea(p, oriented=True) < 0:
        p = p[[0, 3, 2, 1]]
    return p


def detect_quad_candidates(image):
    """Receives pixels only, never source IDs, labels, or manual corners."""
    h, w = image.shape[:2]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    records = []
    accepted = []
    for index, contour in enumerate(contours):
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, .02 * perimeter, True)
        p = approx.reshape(-1, 2)
        area = abs(float(cv2.contourArea(approx)))
        reasons = []
        if len(p) != 4:
            reasons.append("not_four_vertices")
        convex = bool(cv2.isContourConvex(approx))
        if not convex:
            reasons.append("not_convex")
        if area <= 1000:
            reasons.append("area_not_above_1000")
        if area >= .60 * w * h:
            reasons.append("area_not_below_60pct")
        in_bounds = bool(len(p) and ((p[:, 0] >= 0) & (p[:, 0] < w) & (p[:, 1] >= 0) & (p[:, 1] < h)).all())
        if not in_bounds:
            reasons.append("out_of_image_bounds")
        rect_sides = cv2.minAreaRect(approx)[1]
        aspect = max(rect_sides)/min(rect_sides) if min(rect_sides) > 0 else None
        if aspect is None or aspect > 8.0:
            reasons.append("minAreaRect_aspect_above_8_or_degenerate")
        item = {"contour_index": index, "contour_point_count": len(contour), "perimeter_px": perimeter,
                "approx_vertices": len(p), "approx_xy": p.tolist(), "convex": convex,
                "area_px2": area, "area_fraction": area / (w*h), "image_bound": in_bounds,
                "minAreaRect_long_short_ratio": aspect, "rejection_reasons": reasons,
                "candidate_passed": not reasons}
        records.append(item)
        if not reasons:
            item["ordered_quad_xy"] = order_quad(p).tolist()
            accepted.append(item)
    return edges, records, accepted


def match_reference(query_name, image, kp, des, ref, bf):
    pairs = bf.knnMatch(ref["des"], des, k=2) if ref["des"] is not None and des is not None and len(des) >= 2 else []
    good = [a for pair in pairs if len(pair) == 2 for a, b in [pair] if a.distance < .75 * b.distance]
    src = np.float32([ref["kp"][m.queryIdx].pt for m in good]).reshape(-1, 2)
    dst = np.float32([kp[m.trainIdx].pt for m in good]).reshape(-1, 2)
    H = mask = None
    if len(good) >= 8:
        cv2.setRNGSeed(20261112)
        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
    flags = mask.ravel().astype(bool) if mask is not None else np.zeros(len(good), bool)
    rc, qc = coverage(src[flags], ref["image"].shape), coverage(dst[flags], image.shape)
    inliers = int(flags.sum())
    reasons = []
    if inliers < 8:
        reasons.append("fewer_than_8_inliers")
    if rc["unique_xy"] < 8 or qc["unique_xy"] < 8:
        reasons.append("fewer_than_8_distinct_xy_on_either_side")
    if rc["hull_area_fraction"] < .05 or qc["hull_area_fraction"] < .05:
        reasons.append("hull_coverage_below_5pct_on_either_side")
    row = {"query": query_name, "reference": ref["id"], "reference_class_annotation": ref["meta"]["class_id"],
           "reference_keypoints": len(ref["kp"]), "query_keypoints": len(kp), "good_matches": len(good),
           "min8_gate_passed": len(good) >= 8, "homography_computed": H is not None,
           "inliers": inliers, "inlier_fraction_of_good": inliers/len(good) if good else 0,
           "inlier_reference_coverage": rc, "inlier_query_coverage": qc,
           "homography_reference_to_query": H.tolist() if H is not None else None,
           "projected_reference_quad_xy": None,
           "matches": [{"reference_xy": src[i].tolist(), "query_xy": dst[i].tolist(),
                        "distance": float(m.distance), "inlier": bool(flags[i])} for i, m in enumerate(good)]}
    display = image.copy()
    if H is None:
        reasons.append("no_homography")
    else:
        rh, rw = ref["image"].shape[:2]
        projected = cv2.perspectiveTransform(np.float32([[[0, 0], [rw-1, 0], [rw-1, rh-1], [0, rh-1]]]), H)[0]
        row["projected_reference_quad_xy"] = projected.tolist()
        finite = bool(np.isfinite(projected).all())
        convex = finite and bool(cv2.isContourConvex(projected))
        area = abs(float(cv2.contourArea(projected))) if finite else 0
        fraction = area/(image.shape[0]*image.shape[1])
        row.update(projected_quad_finite=finite, projected_quad_convex=convex,
                   projected_quad_area_px2=area, projected_quad_area_fraction=fraction)
        if not finite or not convex:
            reasons.append("projection_nonfinite_or_nonconvex")
        if not .10 <= fraction <= 2.0:
            reasons.append("projection_area_outside_10_to_200pct")
        if inliers:
            err = np.linalg.norm(cv2.perspectiveTransform(src[None], H)[0]-dst, axis=1)[flags]
            row["inlier_reprojection_error_px"] = {"median": float(np.median(err)), "max": float(err.max())}
        if finite and abs(projected).max() < 1e6:
            cv2.polylines(display, [np.rint(projected).astype(int)], True, (0, 255, 255), 2)
    row["exploratory_geometry_gate_passed"] = not reasons
    row["geometry_rejection_reasons"] = reasons
    shown = cv2.drawMatches(ref["image"], ref["kp"], display, kp, good, None,
                           matchesMask=flags.astype(int).tolist() if H is not None else None,
                           matchColor=(0, 255, 0), flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    if shown.shape[1] > 1600:
        shown = cv2.resize(shown, None, fx=1600/shown.shape[1], fy=1600/shown.shape[1])
    cv2.putText(shown, f"{query_name} {ref['id']} good={len(good)} inlier={inliers} geom={not reasons}",
                (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 255), 1, cv2.LINE_AA)
    path = OUT / "matches" / f"{query_name}__{ref['id']}.jpg"
    save_image(path, shown)
    row["visualization_path"] = path.relative_to(OUT).as_posix()
    return row


def evaluate_only_after_detection(auto, manual):
    a, b = np.asarray(auto, np.float32), np.asarray(manual, np.float32)
    intersection, _ = cv2.intersectConvexConvex(a, b)
    union = abs(cv2.contourArea(a)) + abs(cv2.contourArea(b)) - intersection
    options = []
    for points in (a, a[::-1]):
        for shift in range(4):
            distances = np.linalg.norm(np.roll(points, shift, axis=0)-b, axis=1)
            options.append((float(distances.mean()), distances.tolist()))
    mean, distances = min(options)
    return {"IoU": float(intersection/union) if union else 0,
            "minimum_cyclic_or_reversed_mean_corner_error_px": mean,
            "per_corner_error_px_for_minimum_alignment": distances,
            "maximum_corner_error_px": max(distances)}


def main():
    OUT.mkdir(exist_ok=True); (OUT / "matches").mkdir(exist_ok=True)
    cv2.setNumThreads(1); cv2.setRNGSeed(20261112)
    frozen_path = FROZEN / "report.json"
    frozen = json.loads(frozen_path.read_text(encoding="utf-8"))
    images = []
    # This loop passes only image pixels to the detector. No manual corner is loaded here.
    for source in frozen["sources"]:
        path = ROOT / source["path"]
        assert digest(path) == source["sha256"]
        image = read_image(path)
        edges, contours, candidates = detect_quad_candidates(image)
        name = source["name"]
        save_image(OUT / (name + "_canny.png"), edges)
        overlay = image.copy()
        for contour in contours:
            if contour["approx_vertices"] == 4:
                color = (0, 0, 255) if contour["candidate_passed"] else (255, 255, 0)
                points = np.array(contour["approx_xy"], np.int32)
                cv2.polylines(overlay, [points], True, color, 2 if contour["candidate_passed"] else 1)
                if contour["candidate_passed"]:
                    cv2.putText(overlay, str(contour["contour_index"]), tuple(points[0]), cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1)
        save_image(OUT / (name + "_all_quad_overlay.png"), overlay)
        images.append({"name": name, "source": source, "image": image,
                       "contours": contours, "candidates": candidates})
        print(name, "contours", len(contours), "passed_candidates", len(candidates), flush=True)

    total = sum(len(x["candidates"]) for x in images)
    results, queries, references = [], [], []
    if total:
        sift = cv2.SIFT_create()
        bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
        for meta in frozen["references"]:
            path = Path(meta["absolute_path"]); assert digest(path) == meta["sha256"]
            image = read_image(path)
            kp, des = sift.detectAndCompute(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), None)
            references.append({"id": path.name, "meta": meta, "image": image, "kp": kp, "des": des})
        for source in images:
            for candidate in source["candidates"]:
                name = f"{source['name']}_quad_{candidate['contour_index']:04d}"
                quad = np.array(candidate["ordered_quad_xy"], np.float32)
                H = cv2.getPerspectiveTransform(quad, np.float32([[0, 0], [383, 0], [383, 383], [0, 383]]))
                warp = cv2.warpPerspective(source["image"], H, (384, 384), flags=cv2.INTER_LINEAR)
                path = OUT / (name + ".png"); save_image(path, warp)
                kp, des = sift.detectAndCompute(cv2.cvtColor(warp, cv2.COLOR_BGR2GRAY), None)
                queries.append({"name": name, "source_name": source["name"], "contour_index": candidate["contour_index"],
                                "quad_xy": quad.tolist(), "homography_source_to_query": H.tolist(),
                                "path": path.name, "sha256": digest(path), "keypoints": len(kp)})
                for ref in references:
                    results.append(match_reference(name, warp, kp, des, ref, bf))

    # Development evaluation begins here; manual corners have not selected any candidate.
    manual = {q["source_name"]: q["manual_quad_xy"] for q in frozen["queries"] if q["manual_rectification"]}
    evaluations = []
    for source in images:
        for candidate in source["candidates"]:
            evaluations.append({"source_name": source["name"], "contour_index": candidate["contour_index"],
                                **evaluate_only_after_detection(candidate["ordered_quad_xy"], manual[source["name"]])})
    report = {"kind": "blind_fixed_document_scanner_quad_sift_exploratory",
              "created_at": datetime.now(timezone.utc).isoformat(), "opencv_version": cv2.__version__,
              "frozen_input_report_sha256": digest(frozen_path), "detector_protocol": DETECTOR,
              "sift_protocol": frozen["protocol"], "geometry_gate": GEOMETRY_GATE,
              "manual_corners_used_by_detector_or_SIFT": False,
              "manual_evaluation_is_development_only": True,
              "sources": [x["source"] for x in images],
              "detection": [{k: v for k, v in x.items() if k not in ("image", "source")} for x in images],
              "queries": queries, "references": frozen["references"], "results": results,
              "development_manual_quad_evaluations": evaluations,
              "class_ground_truth": None, "recognized_class": None, "deployed": False,
              "status": "completed_fixed_protocol" if total else "stopped_no_quad_candidates",
              "limitations": ["Only two previously inspected development images; no held-out estimate.",
              "All contours/candidates retained; no tuning, candidate choice by class, HSV, morphology or extra rotations.",
              "Geometry gate is an uncalibrated exploratory filter, not validated class acceptance.",
              "Public references are not established as the current official simulator library."]}
    (OUT / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding="utf-8")
    with (OUT / "scores.csv").open("w", newline="", encoding="utf-8-sig") as f:
        fields = ["query", "reference", "good_matches", "inliers", "reference_distinct_xy", "query_distinct_xy", "reference_hull_fraction", "query_hull_fraction", "projected_quad_convex", "projected_quad_area_fraction", "exploratory_geometry_gate_passed", "geometry_rejection_reasons"]
        writer = csv.DictWriter(f, fieldnames=fields); writer.writeheader()
        for row in results:
            values = {k: row.get(k) for k in fields}
            values.update(reference_distinct_xy=row["inlier_reference_coverage"]["unique_xy"], query_distinct_xy=row["inlier_query_coverage"]["unique_xy"],
                          reference_hull_fraction=row["inlier_reference_coverage"]["hull_area_fraction"], query_hull_fraction=row["inlier_query_coverage"]["hull_area_fraction"],
                          geometry_rejection_reasons=";".join(row["geometry_rejection_reasons"]))
            writer.writerow(values)
    print(json.dumps({"sources": len(images), "auto_quad_candidates": total, "sift_pairs": len(results),
                      "geometry_gate_passed": sum(x["exploratory_geometry_gate_passed"] for x in results)}, indent=2))
    print("quad_development_evaluation", evaluations)
    for result in results:
        if result["exploratory_geometry_gate_passed"]:
            print("geometry_pass", result["query"], result["reference"], result["good_matches"], result["inliers"])


if __name__ == "__main__":
    main()
