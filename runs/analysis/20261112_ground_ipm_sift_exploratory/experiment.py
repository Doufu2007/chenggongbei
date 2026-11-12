"""Candidate-calibration full-ground IPM, five fixed images, all references.

No manual board corners, board ID, semantic label, detection, or ROI search is
used to produce the single BEV for each image. Calibration is an assumption.
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
ALGORITHM = ROOT / "runs/analysis/20261112_auto_quad_sift_exploratory/experiment.py"
EXPECTED_ALGORITHM_SHA256 = "2406321bb17d34aca4df93f8a7907f738ae0290d928b276001a9b7d6bfa403af"
spec = importlib.util.spec_from_file_location("frozen_matching", ALGORITHM)
a = importlib.util.module_from_spec(spec); spec.loader.exec_module(a)
assert a.digest(ALGORITHM) == EXPECTED_ALGORITHM_SHA256
a.OUT = OUT

CAMERA_X = .390
CAMERA_Y = 0.0
CAMERA_BODY_Z = .130
BODY_WORLD_Z = .055
PLANE_WORLD_Z = .012
HEIGHT = .173
X_MIN, X_MAX = .45, 3.2
Y_MIN, Y_MAX = -1.6, 1.6
PPM = 300
WIDTH, HEIGHT_PX = 960, 825
CAPTURE0_ANCHORS = {"ground0": [407.2, 471.3], "ground4": [166.3, 513.8], "ground3": [516.8, 452.0]}


def dictionaries(value):
    if isinstance(value, dict):
        yield value
        for x in value.values():
            yield from dictionaries(x)
    elif isinstance(value, list):
        for x in value:
            yield from dictionaries(x)


def camera_info(summary):
    candidates = [x["value"]["first"] for x in dictionaries(summary)
                  if x.get("topic") == "/uav/camera/camera_info" and isinstance(x.get("value"), dict)
                  and isinstance(x["value"].get("first"), dict) and "k" in x["value"]["first"]]
    if not candidates:
        raise ValueError("No measured UAV CameraInfo; do not silently invent intrinsics.")
    for item in candidates:
        assert np.allclose(item["k"], candidates[0]["k"])
    item = candidates[0]
    assert not any(abs(x) > 1e-12 for x in item.get("d", [])), "Nonzero lens distortion unsupported by this fixed prototype"
    assert item["width"] == 800 and item["height"] == 800
    return item


def project_ground(points, ground_to_source):
    return cv2.perspectiveTransform(np.asarray(points, np.float32).reshape(1, -1, 2), ground_to_source)[0]


def main():
    OUT.mkdir(exist_ok=True); (OUT / "matches").mkdir(exist_ok=True)
    cv2.setNumThreads(1); cv2.setRNGSeed(20261112)
    frozen_path = a.FROZEN / "report.json"
    frozen = json.loads(frozen_path.read_text(encoding="utf-8"))
    sources = []
    for source in frozen["sources"]:
        sources.append({"name": source["name"], "path": ROOT/source["path"], "metadata": source["metadata"]})
    v18_summary_path = ROOT / "runs/analysis/20261112_approach_v18/summary.json"
    v18_summary = json.loads(v18_summary_path.read_text(encoding="utf-8"))
    captures = [x for x in v18_summary["native_images"] if x["phase"] in ("capture_0", "capture_1", "capture_2")]
    assert sorted(x["phase"] for x in captures) == ["capture_0", "capture_1", "capture_2"]
    sources.extend({"name": "v18_"+x["phase"], "path": v18_summary_path.parent/x["file_name"], "metadata": x} for x in captures)
    observed_info = camera_info(v18_summary)
    K = np.array(observed_info["k"], np.float64).reshape(3, 3)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    assert abs(fx-476.7030836014194) < 1e-8 and abs(fy-fx) < 1e-8 and cx == 400.5 and cy == 400.5
    assert abs(BODY_WORLD_Z+CAMERA_BODY_Z-PLANE_WORLD_Z-HEIGHT) < 1e-12
    # Candidate optical coordinates: right=-body_left, down=height, forward=X-camera_x.
    M = np.array([[cx, -fx, -cx*CAMERA_X], [cy, 0, fy*HEIGHT-cy*CAMERA_X], [1, 0, -CAMERA_X]], np.float64)
    B = np.array([[0, -PPM, PPM*Y_MAX-.5], [-PPM, 0, PPM*X_MAX-.5], [0, 0, 1]], np.float64)
    H_source_to_bev = B @ np.linalg.inv(M)
    H_bev_to_source = np.linalg.inv(H_source_to_bev)
    rows, cols = np.indices((HEIGHT_PX, WIDTH), dtype=np.float32)
    metric_x = (X_MAX-(rows+.5)/PPM).astype(np.float32)
    metric_y = (Y_MAX-(cols+.5)/PPM).astype(np.float32)
    depth = metric_x-CAMERA_X
    map_u = (cx-fx*metric_y/depth).astype(np.float32)
    map_v = (cy+fy*HEIGHT/depth).astype(np.float32)
    valid = (depth > 0) & (map_u >= 0) & (map_u <= 799) & (map_v >= 0) & (map_v <= 799)
    np.savez_compressed(OUT/"source_mapping.npz", source_u=map_u, source_v=map_v,
                        valid=valid, body_forward_x_m=metric_x, body_left_y_m=metric_y,
                        H_source_to_bev=H_source_to_bev, H_bev_to_source=H_bev_to_source,
                        ground_to_source=M, ground_to_bev=B)
    a.save_image(OUT/"valid_source_mask.png", valid.astype(np.uint8)*255)

    references = []
    sift = cv2.SIFT_create(); bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
    for meta in frozen["references"]:
        path = Path(meta["absolute_path"]); assert a.digest(path) == meta["sha256"]
        image = a.read_image(path)
        kp, des = sift.detectAndCompute(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), None)
        references.append({"id": path.name, "meta": meta, "image": image, "kp": kp, "des": des})
    results = []; query_records = []
    for source in sources:
        path = source["path"]; meta = source["metadata"]
        assert a.digest(path) == meta["sha256"]
        summary_path = path.parent/"summary.json"
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        info = camera_info(summary)
        assert np.allclose(info["k"], K.ravel())
        raw = a.read_image(path)
        image = cv2.remap(raw, map_u, map_v, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
        image[~valid] = 0
        name = source["name"]+"_full_ground_bev"
        bev_path = OUT/(name+".png"); a.save_image(bev_path, image)
        overlay = raw.copy()
        lines = [np.column_stack((np.full(150, x), np.linspace(Y_MIN, Y_MAX, 150))) for x in [.5, 1., 1.5, 2., 2.5, 3.]]
        lines += [np.column_stack((np.linspace(X_MIN, X_MAX, 150), np.full(150, y))) for y in [-1.5, -1., -.5, 0, .5, 1., 1.5]]
        for line in lines:
            p = project_ground(line, M)
            in_frame = (p[:, 0]>=0)&(p[:, 0]<=799)&(p[:, 1]>=0)&(p[:, 1]<=799)
            p = p[in_frame]
            if len(p)>1:
                cv2.polylines(overlay, [np.rint(p).astype(np.int32)], False, (0, 255, 255), 1)
        a.save_image(OUT/(source["name"]+"_source_metric_grid.png"), overlay)
        kp, des = sift.detectAndCompute(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), None)
        query_records.append({"name": name, "source_path": path.relative_to(ROOT).as_posix(), "source_sha256": meta["sha256"],
                              "source_summary_sha256": a.digest(summary_path), "source_metadata": meta,
                              "camera_info": info, "path": bev_path.name, "sha256": a.digest(bev_path),
                              "size_wh": [WIDTH, HEIGHT_PX], "keypoints": len(kp)})
        for ref in references:
            row = a.match_reference(name, image, kp, des, ref, bf)
            row["ROI_geometry_gate_interpretation"] = "Retained ROI-sized diagnostic only; full-scene area thresholds are not a calibrated classification gate."
            if row["homography_computed"]:
                H = np.array(row["homography_reference_to_query"], np.float64)
                rh, rw = ref["image"].shape[:2]
                center = cv2.perspectiveTransform(np.float32([[[(rw-1)/2, (rh-1)/2]]]), H)[0, 0]
                source_center = cv2.perspectiveTransform(center.reshape(1, 1, 2), H_bev_to_source)[0, 0]
                ground_center = cv2.perspectiveTransform(center.reshape(1, 1, 2), np.linalg.inv(B))[0, 0]
                row["projected_template_center_bev_xy"] = center.tolist()
                row["projected_template_center_source_xy"] = source_center.tolist()
                row["projected_template_center_body_forward_left_m"] = ground_center.tolist()
                if row["projected_reference_quad_xy"] is not None:
                    quad = np.array(row["projected_reference_quad_xy"], np.float32)
                    row["projected_template_quad_source_xy"] = cv2.perspectiveTransform(quad[None], H_bev_to_source)[0].tolist()
                    row["inlier_query_hull_fraction_of_projected_quad"] = row["inlier_query_coverage"].get("hull_area_px2", 0)/row["projected_quad_area_px2"] if row.get("projected_quad_area_px2") else None
                if source["name"] == "v18_capture_0":
                    row["all_candidate_anchor_pixel_residuals"] = {key: float(np.linalg.norm(source_center-np.array(value))) for key, value in CAPTURE0_ANCHORS.items()}
            results.append(row)
        print(source["name"], "SIFT_keypoints", len(kp), "completed_all_references", flush=True)
    report = {"kind": "candidate_calibration_full_ground_ipm_sift", "created_at": datetime.now(timezone.utc).isoformat(),
              "candidate_calibration": True, "official_extrinsic_calibration_verified": False,
              "calibration": {"camera_body_xyz_m": [CAMERA_X, CAMERA_Y, CAMERA_BODY_Z],
              "body_world_z_m": BODY_WORLD_Z, "plane_world_z_m": PLANE_WORLD_Z, "camera_height_over_plane_m": HEIGHT,
              "assumed_orientation": "Horizontal body-forward optical axis; optical right=-body left, optical down=-body up; no roll/pitch correction.",
              "K_measured": K.tolist(), "K_source_summary_sha256": a.digest(v18_summary_path), "K_topic": "/uav/camera/camera_info"},
              "IPM": {"body_forward_x_range_m": [X_MIN, X_MAX], "body_left_y_range_m": [Y_MIN, Y_MAX],
              "pixels_per_metre": PPM, "output_size_wh": [WIDTH, HEIGHT_PX],
              "pixel_centers": "X=3.2-(v+0.5)/300; Y=1.6-(u+0.5)/300",
              "source_projection": "u=cx-fx*Y/(X-.390); v=cy+fy*.173/(X-.390)",
              "H_source_to_bev": H_source_to_bev.tolist(), "H_bev_to_source": H_bev_to_source.tolist(),
              "valid_source_fraction": float(valid.mean()), "mapping_path": "source_mapping.npz"},
              "frozen_matcher_sha256": a.digest(ALGORITHM), "sift_protocol": frozen["protocol"],
              "ROI_geometry_gate_diagnostic_only": a.GEOMETRY_GATE, "queries": query_records,
              "references": frozen["references"], "results": results,
              "v18_capture0_candidate_anchor_pixels": CAPTURE0_ANCHORS,
              "anchor_interpretation": "Parent-supplied projections from same candidate calibration and public ground geometry; compare all template centers after matching, never select a class/reference by ID.",
              "manual_quad_used": False, "board_ID_used_for_warp_or_matching": False,
              "class_ground_truth": None, "recognized_class": None, "deployed": False,
              "limitations": ["Candidate extrinsics and horizontal-camera assumption are not official measured calibration.",
              "Full ground BEV has one fixed extent/resolution and no board ROI selection; query area is not comparable to tight-ROI coverage thresholds.",
              "Five correlated images from the same development environment, not held-out accuracy evidence.",
              "Only third-party reference annotations; exact correspondence does not establish the official scoring category.",
              "No production implementation or runtime class/location binding, no network, uploads, downloads or parameter search."]}
    (OUT/"report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding="utf-8")
    fields = ["query", "reference", "good_matches", "inliers", "reference_distinct_xy", "query_distinct_xy", "reference_hull_fraction", "query_hull_fraction", "projected_quad_convex", "projected_quad_area_px2", "inlier_query_hull_fraction_of_projected_quad", "ROI_geometry_gate_passed", "source_center_x", "source_center_y", "body_forward_m", "body_left_m", "anchor_residual_ground0_px", "anchor_residual_ground4_px", "anchor_residual_ground3_px"]
    with (OUT/"scores.csv").open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fields); writer.writeheader()
        for row in results:
            values = {k: row.get(k) for k in fields}
            values.update(reference_distinct_xy=row["inlier_reference_coverage"]["unique_xy"], query_distinct_xy=row["inlier_query_coverage"]["unique_xy"],
                          reference_hull_fraction=row["inlier_reference_coverage"]["hull_area_fraction"], query_hull_fraction=row["inlier_query_coverage"]["hull_area_fraction"],
                          ROI_geometry_gate_passed=row["exploratory_geometry_gate_passed"])
            if "projected_template_center_source_xy" in row:
                values["source_center_x"], values["source_center_y"] = row["projected_template_center_source_xy"]
                values["body_forward_m"], values["body_left_m"] = row["projected_template_center_body_forward_left_m"]
            for key, value in row.get("all_candidate_anchor_pixel_residuals", {}).items():
                values[f"anchor_residual_{key}_px"] = value
            writer.writerow(values)
    print("all_pairs", len(results), flush=True)
    for row in results:
        if row.get("projected_quad_convex") and row["inlier_query_coverage"]["unique_xy"] >= 8:
            print("distributed_convex_diagnostic", row["query"], row["reference"], row["good_matches"], row["inliers"],
                  row["inlier_reference_coverage"]["unique_xy"], row["inlier_query_coverage"]["unique_xy"],
                  row["inlier_reference_coverage"]["hull_area_fraction"], row["inlier_query_coverage"]["hull_area_fraction"],
                  row.get("projected_template_center_source_xy"), row.get("all_candidate_anchor_pixel_residuals"), flush=True)
    assert a.digest(ALGORITHM) == EXPECTED_ALGORITHM_SHA256


if __name__ == "__main__":
    main()
