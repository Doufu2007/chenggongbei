"""Three new CAPTURE inputs, unseen before running the frozen quad detector."""
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
ALGORITHM = ROOT / "runs/analysis/20261112_auto_quad_sift_exploratory/experiment.py"
EXPECTED_ALGORITHM_SHA256 = "2406321bb17d34aca4df93f8a7907f738ae0290d928b276001a9b7d6bfa403af"
spec = importlib.util.spec_from_file_location("frozen_quad", ALGORITHM)
a = importlib.util.module_from_spec(spec); spec.loader.exec_module(a)
assert a.digest(ALGORITHM) == EXPECTED_ALGORITHM_SHA256
a.OUT = OUT  # Destination only; the frozen detector/matcher parameters are untouched.


def main():
    OUT.mkdir(exist_ok=True); (OUT / "matches").mkdir(exist_ok=True)
    cv2.setNumThreads(1); cv2.setRNGSeed(20261112)
    summary_path = ROOT / "runs/analysis/20261112_approach_v18/summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    selected = [x for x in summary["native_images"] if x["phase"] in ("capture_0", "capture_1", "capture_2")]
    assert sorted(x["phase"] for x in selected) == ["capture_0", "capture_1", "capture_2"]
    images = []
    # No manual corner or semantic class is loaded. All three CAPTURE images are run.
    for meta in selected:
        path = summary_path.parent / meta["file_name"]; assert a.digest(path) == meta["sha256"]
        image = a.read_image(path)
        edges, contours, candidates = a.detect_quad_candidates(image)
        name = "v18_" + meta["phase"]
        a.save_image(OUT / (name + "_canny.png"), edges)
        overlay = image.copy()
        for c in contours:
            if c["approx_vertices"] == 4:
                color = (0, 0, 255) if c["candidate_passed"] else (255, 255, 0)
                p = np.array(c["approx_xy"], np.int32)
                cv2.polylines(overlay, [p], True, color, 2 if c["candidate_passed"] else 1)
                if c["candidate_passed"]:
                    cv2.putText(overlay, str(c["contour_index"]), tuple(p[0]), cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1)
        a.save_image(OUT / (name + "_all_quad_overlay.png"), overlay)
        images.append({"name": name, "path": path.relative_to(ROOT).as_posix(), "metadata": meta,
                       "image": image, "contours": contours, "candidates": candidates})
        print(name, "contours", len(contours), "candidates", len(candidates), flush=True)
    frozen = json.loads((a.FROZEN / "report.json").read_text(encoding="utf-8"))
    references, queries, results = [], [], []
    if any(x["candidates"] for x in images):
        sift = cv2.SIFT_create(); bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
        for meta in frozen["references"]:
            path = Path(meta["absolute_path"]); assert a.digest(path) == meta["sha256"]
            image = a.read_image(path)
            kp, des = sift.detectAndCompute(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), None)
            references.append({"id": path.name, "meta": meta, "image": image, "kp": kp, "des": des})
        for source in images:
            for candidate in source["candidates"]:
                name = f"{source['name']}_quad_{candidate['contour_index']:04d}"
                quad = np.array(candidate["ordered_quad_xy"], np.float32)
                H = cv2.getPerspectiveTransform(quad, np.float32([[0, 0], [383, 0], [383, 383], [0, 383]]))
                warp = cv2.warpPerspective(source["image"], H, (384, 384), flags=cv2.INTER_LINEAR)
                path = OUT / (name + ".png"); a.save_image(path, warp)
                kp, des = sift.detectAndCompute(cv2.cvtColor(warp, cv2.COLOR_BGR2GRAY), None)
                queries.append({"name": name, "source_name": source["name"], "contour_index": candidate["contour_index"],
                                "quad_xy": quad.tolist(), "homography_source_to_query": H.tolist(),
                                "path": path.name, "sha256": a.digest(path), "keypoints": len(kp)})
                for ref in references:
                    results.append(a.match_reference(name, warp, kp, des, ref, bf))
    report = {"kind": "frozen_quad_detector_additional_v18_capture_test",
              "created_at": datetime.now(timezone.utc).isoformat(), "algorithm_path": ALGORITHM.relative_to(ROOT).as_posix(),
              "algorithm_sha256": a.digest(ALGORITHM), "parameters_unchanged": True,
              "images_viewed_before_running": False, "source_summary_sha256": a.digest(summary_path),
              "detector_protocol": a.DETECTOR, "sift_protocol": frozen["protocol"], "geometry_gate": a.GEOMETRY_GATE,
              "detection": [{k: v for k, v in x.items() if k != "image"} for x in images],
              "queries": queries, "references": frozen["references"], "results": results,
              "manual_corners_used": False, "manual_quad_evaluations": None, "class_ground_truth": None,
              "recognized_class": None, "deployed": False,
              "status": "completed_fixed_protocol" if queries else "stopped_no_quad_candidates",
              "limitations": ["Three correlated consecutive CAPTURE frames of one scene, not three independent boards.",
              "Additional frames were run before visual inspection; same environment and prior design still prevent broad held-out claims.",
              "No new manual annotation or parameter/combination search."]}
    (OUT / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding="utf-8")
    fields = ["query", "reference", "good_matches", "inliers", "reference_distinct_xy", "query_distinct_xy", "reference_hull_fraction", "query_hull_fraction", "exploratory_geometry_gate_passed", "geometry_rejection_reasons"]
    with (OUT / "scores.csv").open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fields); writer.writeheader()
        for row in results:
            values = {k: row.get(k) for k in fields}
            values.update(reference_distinct_xy=row["inlier_reference_coverage"]["unique_xy"],query_distinct_xy=row["inlier_query_coverage"]["unique_xy"],
                          reference_hull_fraction=row["inlier_reference_coverage"]["hull_area_fraction"],query_hull_fraction=row["inlier_query_coverage"]["hull_area_fraction"],
                          geometry_rejection_reasons=";".join(row["geometry_rejection_reasons"]))
            writer.writerow(values)
    print(json.dumps({"frames": len(images), "candidates": len(queries), "SIFT_pairs": len(results),
                      "geometry_gate_passed": sum(x["exploratory_geometry_gate_passed"] for x in results)}, indent=2))
    assert a.digest(ALGORITHM) == EXPECTED_ALGORITHM_SHA256


if __name__ == "__main__":
    main()
