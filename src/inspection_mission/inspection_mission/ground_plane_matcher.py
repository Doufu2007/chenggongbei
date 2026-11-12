"""Independent candidate-calibration ground-plane template geometry.

This module has no ROS, control, task ID, or referee interface. Class labels are
annotations of explicitly supplied templates, never official acceptance. The
fixed camera extrinsics remain a candidate model, not measured calibration.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import cv2
import numpy as np


PPM = 300
WIDTH, HEIGHT = 960, 825
X_MAX, Y_MAX = 3.2, 1.6
CAMERA_X_M, CANDIDATE_HEIGHT_M = .390, .173
RANSAC_SEED = 20261112


@dataclass(frozen=True)
class TemplateSpec:
    path: str | Path
    class_id: str
    label: str
    provenance: Mapping[str, Any]
    expected_sha256: str | None = None


def _validate_inputs(image_bgr, K, height_m):
    if not isinstance(image_bgr, np.ndarray) or image_bgr.dtype != np.uint8:
        raise ValueError("image must be a uint8 BGR ndarray")
    if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError("image must have three BGR channels")
    if image_bgr.shape[0] == 0 or image_bgr.shape[1] == 0:
        raise ValueError("image must not be empty")
    try:
        matrix = np.asarray(K, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError("K must be a finite 3x3 pinhole matrix") from exc
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("K must be a finite 3x3 pinhole matrix")
    if (matrix[0, 0] <= 0 or matrix[1, 1] <= 0
            or not np.allclose(matrix[2], [0, 0, 1], rtol=0, atol=1e-12)
            or abs(matrix[0, 1]) > 1e-12 or abs(matrix[1, 0]) > 1e-12):
        raise ValueError("K must have positive focal lengths, zero skew, and pinhole last row")
    h, w = image_bgr.shape[:2]
    if not (0 <= matrix[0, 2] < w and 0 <= matrix[1, 2] < h):
        raise ValueError("K principal point must lie inside the input image")
    if (isinstance(height_m, (bool, np.bool_)) or not isinstance(height_m, (int, float, np.number))
            or not math.isfinite(float(height_m))
            or not math.isclose(float(height_m), CANDIDATE_HEIGHT_M, rel_tol=0, abs_tol=1e-12)):
        raise ValueError("height_m must be the fixed positive candidate height 0.173")
    return matrix


def rectify_ground_plane(image_bgr, K, *, height_m=CANDIDATE_HEIGHT_M):
    """One fixed full-ground BEV, without board detection or manual corners.

    X edges [.45,3.2]m forward and Y edges [-1.6,1.6]m left. Pixel centers use
    X=3.2-(v+.5)/300 and Y=1.6-(u+.5)/300. Lens distortion must already be absent
    or rectified by the caller; K alone cannot describe nonzero distortion.
    """
    matrix = _validate_inputs(image_bgr, K, height_m)
    fx, fy, cx, cy = matrix[0, 0], matrix[1, 1], matrix[0, 2], matrix[1, 2]
    ground_to_source = np.array([[cx, -fx, -cx*CAMERA_X_M],
                                [cy, 0, fy*height_m-cy*CAMERA_X_M],
                                [1, 0, -CAMERA_X_M]], np.float64)
    ground_to_bev = np.array([[0, -PPM, PPM*Y_MAX-.5],
                             [-PPM, 0, PPM*X_MAX-.5], [0, 0, 1]], np.float64)
    source_to_bev = ground_to_bev @ np.linalg.inv(ground_to_source)
    rows, cols = np.indices((HEIGHT, WIDTH), dtype=np.float32)
    forward = (X_MAX-(rows+.5)/PPM).astype(np.float32)
    left = (Y_MAX-(cols+.5)/PPM).astype(np.float32)
    depth = forward-CAMERA_X_M
    source_u = (cx-fx*left/depth).astype(np.float32)
    source_v = (cy+fy*height_m/depth).astype(np.float32)
    h, w = image_bgr.shape[:2]
    valid = (depth>0)&(source_u>=0)&(source_u<=w-1)&(source_v>=0)&(source_v<=h-1)
    bev = cv2.remap(image_bgr, source_u, source_v, cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
    bev[~valid] = 0
    return {"bev_bgr": bev, "valid_mask": valid, "source_to_bev": source_to_bev,
            "bev_to_source": np.linalg.inv(source_to_bev), "K": matrix.copy(),
            "height_m": float(height_m), "candidate_calibration": True}


def _support(points, area):
    points = np.asarray(points, np.float32).reshape(-1, 2)
    hull_area = abs(float(cv2.contourArea(cv2.convexHull(points)))) if len(points) >= 3 else 0.0
    return {"distinct_xy": len(np.unique(points.round(3), axis=0)),
            "hull_area_px2": hull_area, "hull_fraction": hull_area/area}


def verify_geometry(template_points, bev_points, inlier_mask, H, template_wh):
    """Verify actual correspondences independently of RANSAC's inlier count.

    Coordinates are SIFT keypoint xy values, with 0.001px rounding solely for
    distinct-location counting. H maps template pixels to the fixed metric BEV.
    """
    reference = np.asarray(template_points, np.float32).reshape(-1, 2)
    query = np.asarray(bev_points, np.float32).reshape(-1, 2)
    mask = np.asarray(inlier_mask).reshape(-1)
    if len(reference) != len(query) or len(mask) != len(reference):
        raise ValueError("correspondence and inlier-mask lengths must agree")
    if not np.isfinite(reference).all() or not np.isfinite(query).all():
        raise ValueError("correspondence coordinates must be finite")
    if not np.isin(mask, [0, 1]).all():
        raise ValueError("inlier mask must contain only booleans or 0/1")
    mask = mask.astype(bool)
    tw, th = template_wh
    if tw <= 1 or th <= 1:
        raise ValueError("template dimensions must exceed one pixel")
    rs = _support(reference[mask], tw*th)
    qs = _support(query[mask], WIDTH*HEIGHT)
    count, good = int(mask.sum()), len(reference)
    ratio = count/good if good else 0.0
    reasons = []
    if rs["distinct_xy"] < 12: reasons.append("template_distinct_xy_below_12")
    if qs["distinct_xy"] < 12: reasons.append("query_distinct_xy_below_12")
    if rs["hull_fraction"] < .15: reasons.append("template_hull_coverage_below_0_15")
    if ratio < .25: reasons.append("inlier_fraction_below_0_25")
    result = {"good_matches": good, "inliers": count, "inlier_fraction": ratio,
              "template_distinct_xy": rs["distinct_xy"], "query_distinct_xy": qs["distinct_xy"],
              "template_hull_fraction": rs["hull_fraction"], "query_hull_area_px2": qs["hull_area_px2"],
              "query_hull_fraction_of_bev": qs["hull_fraction"], "bev_quad_xy": None,
              "plane_area_m2": None, "query_hull_over_projection": None,
              "median_reprojection_error_px": None, "max_reprojection_error_px": None,
              "rejection_reasons": reasons, "geometrically_accepted": False}
    if H is None:
        reasons.append("homography_not_available")
        return result
    H = np.asarray(H, np.float64)
    if H.shape != (3, 3) or not np.isfinite(H).all():
        reasons.append("homography_nonfinite")
        return result
    corners = np.float32([[[0, 0], [tw-1, 0], [tw-1, th-1], [0, th-1]]])
    projected = cv2.perspectiveTransform(corners, H)[0]
    finite = bool(np.isfinite(projected).all())
    area = abs(float(cv2.contourArea(projected))) if finite else 0.0
    convex = finite and bool(cv2.isContourConvex(projected)) and area > 0
    result.update(bev_quad_xy=projected.tolist() if finite else None,
                  projection_finite=finite, projection_convex=convex, plane_area_m2=area/(PPM*PPM),
                  query_hull_over_projection=qs["hull_area_px2"]/area if area else 0.0)
    if not convex: reasons.append("projection_nonconvex_or_degenerate")
    if not .15 <= area/(PPM*PPM) <= .30: reasons.append("plane_area_outside_0_15_to_0_30_m2")
    if result["query_hull_over_projection"] < .10: reasons.append("query_hull_over_projection_below_0_10")
    if count:
        errors = np.linalg.norm(cv2.perspectiveTransform(reference[None], H)[0]-query, axis=1)[mask]
        if np.isfinite(errors).all():
            result["median_reprojection_error_px"] = float(np.median(errors))
            result["max_reprojection_error_px"] = float(errors.max())
        if not np.isfinite(errors).all() or float(np.median(errors)) > 1.5:
            reasons.append("median_reprojection_error_above_1_5_px")
    else:
        reasons.append("no_inliers_for_reprojection_error")
    result["geometrically_accepted"] = not reasons
    return result


def reject_overlapping_class_conflicts(rows):
    """Reject both sides of every cross-class positive-area BEV overlap.

    Same-class reference evidence is retained separately with its own provenance.
    There is no score comparison, suppression winner, class mapping or task ID.
    """
    result = deepcopy(list(rows))
    for row in result:
        row["accepted_candidate"] = bool(row["geometrically_accepted"])
        row["conflicting_template_paths"] = []
    valid = [i for i, row in enumerate(result) if row["geometrically_accepted"]]
    for offset, i in enumerate(valid):
        first = result[i]
        for j in valid[offset+1:]:
            second = result[j]
            if first["class_id"] == second["class_id"]:
                continue
            intersection, _ = cv2.intersectConvexConvex(np.array(first["bev_quad_xy"], np.float32),
                                                       np.array(second["bev_quad_xy"], np.float32))
            if intersection > 1e-6:
                for item, other in ((first, second), (second, first)):
                    item["accepted_candidate"] = False
                    if "overlapping_class_conflict" not in item["rejection_reasons"]:
                        item["rejection_reasons"].append("overlapping_class_conflict")
                    item["conflicting_template_paths"].append(other["template_path"])
    return result


class GroundPlaneMatcher:
    """Read explicit local templates once; emit geometric candidates per frame.

    Descriptors are exactly cv2.SIFT_create() float descriptors. No RootSIFT or
    additional descriptor normalization is applied. Matching is reference->query
    BF L2, Lowe ratio<.75, min8 good, RANSAC3px, with the frozen RNG seed.
    """
    def __init__(self, templates: Sequence[TemplateSpec]):
        if not templates:
            raise ValueError("at least one explicit template is required")
        self._sift = cv2.SIFT_create()
        self._bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
        self._templates = []
        for spec in templates:
            if not isinstance(spec, TemplateSpec):
                raise ValueError("templates must be TemplateSpec objects")
            if not isinstance(spec.class_id, str) or not spec.class_id.strip() or not isinstance(spec.label, str) or not spec.label.strip():
                raise ValueError("template class_id and label must be nonempty annotations")
            if not isinstance(spec.provenance, Mapping):
                raise ValueError("template provenance must be an explicit mapping")
            try:
                provenance = json.loads(json.dumps(dict(spec.provenance), ensure_ascii=False, allow_nan=False))
            except (TypeError, ValueError) as exc:
                raise ValueError("template provenance must contain finite JSON values") from exc
            path = Path(spec.path).resolve()
            raw = path.read_bytes()
            sha = hashlib.sha256(raw).hexdigest()
            if spec.expected_sha256 is not None and sha != spec.expected_sha256:
                raise ValueError(f"template SHA256 mismatch: {path}")
            image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
            if image is None or min(image.shape[:2]) <= 1:
                raise ValueError(f"template is not a readable BGR image: {path}")
            kp, des = self._sift.detectAndCompute(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), None)
            self._templates.append({"path": str(path), "class_id": spec.class_id, "label": spec.label,
                                    "provenance": provenance, "sha256": sha, "image": image, "kp": kp, "des": des})

    def match(self, image_bgr, K, *, height_m=CANDIDATE_HEIGHT_M):
        view = rectify_ground_plane(image_bgr, K, height_m=height_m)
        bev = view["bev_bgr"]
        kp, des = self._sift.detectAndCompute(cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY), None)
        diagnostics = []
        for template in self._templates:
            pairs = self._bf.knnMatch(template["des"], des, k=2) if template["des"] is not None and des is not None and len(des) >= 2 else []
            good = [a for pair in pairs if len(pair) == 2 for a, b in [pair] if a.distance < .75*b.distance]
            source = np.float32([template["kp"][m.queryIdx].pt for m in good]).reshape(-1, 2)
            query = np.float32([kp[m.trainIdx].pt for m in good]).reshape(-1, 2)
            H = mask = None
            if len(good) >= 8:
                cv2.setRNGSeed(RANSAC_SEED)
                H, mask = cv2.findHomography(source, query, cv2.RANSAC, 3.0)
            flags = mask.ravel().astype(bool) if mask is not None else np.zeros(len(good), bool)
            th, tw = template["image"].shape[:2]
            row = verify_geometry(source, query, flags, H, (tw, th))
            if len(good) < 8:
                row["rejection_reasons"].insert(0, "insufficient_good_matches")
            row.update(template_path=template["path"], template_sha256=template["sha256"],
                       class_id=template["class_id"], label=template["label"], provenance=deepcopy(template["provenance"]),
                       template_keypoints=len(template["kp"]), query_keypoints=len(kp),
                       homography_template_to_bev=H.tolist() if H is not None else None,
                       candidate_calibration=True, source_quad_xy=None, body_plane_center_forward_left_m=None,
                       matches=[{"template_xy": source[i].tolist(), "bev_xy": query[i].tolist(),
                                 "distance": float(m.distance), "inlier": bool(flags[i])} for i, m in enumerate(good)])
            if row["bev_quad_xy"] is not None:
                quad = np.array(row["bev_quad_xy"], np.float32)
                source_quad = cv2.perspectiveTransform(quad[None], view["bev_to_source"])[0]
                source_finite = bool(np.isfinite(source_quad).all())
                source_convex = (source_finite and bool(cv2.isContourConvex(source_quad))
                                 and abs(float(cv2.contourArea(source_quad))) > 0)
                row["source_quad_xy"] = source_quad.tolist() if source_finite else None
                row["source_projection_finite"] = source_finite
                row["source_projection_convex"] = source_convex
                plane_quad = np.column_stack((X_MAX-(quad[:,1]+.5)/PPM, Y_MAX-(quad[:,0]+.5)/PPM))
                row["body_plane_quad_forward_left_m"] = plane_quad.tolist()
                depths = plane_quad[:,0]-CAMERA_X_M
                row["quad_camera_depths_m"] = depths.tolist()
                if not bool(np.isfinite(depths).all() and (depths>0).all()):
                    row["rejection_reasons"].append("nonpositive_camera_depth")
                if not source_finite or not source_convex:
                    row["rejection_reasons"].append("source_projection_nonfinite_or_nonconvex")
                row["geometrically_accepted"] = not row["rejection_reasons"]
                center = cv2.perspectiveTransform(np.float32([[[(tw-1)/2, (th-1)/2]]]), np.array(H, np.float64))[0,0]
                if np.isfinite(center).all():
                    row["body_plane_center_forward_left_m"] = [float(X_MAX-(center[1]+.5)/PPM), float(Y_MAX-(center[0]+.5)/PPM)]
                    source_center = cv2.perspectiveTransform(center.reshape(1,1,2), view["bev_to_source"])[0,0]
                    row["source_center_xy"] = source_center.tolist() if np.isfinite(source_center).all() else None
            diagnostics.append(row)
        diagnostics = reject_overlapping_class_conflicts(diagnostics)
        return {"candidates": [deepcopy(x) for x in diagnostics if x["accepted_candidate"]],
                "diagnostics": diagnostics, "candidate_calibration": True, "official_acceptance": False,
                "calibration": {"K": view["K"].tolist(), "height_m": float(height_m),
                                "camera_body_x_m": CAMERA_X_M, "assumed_horizontal_camera": True,
                                "x_range_m": [.45,3.2], "left_y_range_m": [-1.6,1.6], "pixels_per_metre": PPM},
                "bev_bgr": bev, "valid_source_fraction": float(view["valid_mask"].mean()),
                "H_source_to_bev": view["source_to_bev"].tolist(), "H_bev_to_source": view["bev_to_source"].tolist()}
