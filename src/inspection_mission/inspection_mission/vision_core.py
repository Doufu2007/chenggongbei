"""Frame-based OpenCV recognition; reference identities come only from the user.

``center`` is normalized to image width/height, ``bbox`` is xyxy in pixels.
Confidence describes decoder/geometric acceptance, not a calibrated probability.
"""

from __future__ import annotations

from pathlib import Path
from typing import Mapping

import cv2
import numpy as np


def _normalise(image):
    if image is None or image.ndim not in (2, 3) or not image.size:
        raise ValueError("expected a non-empty image")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    gray = np.clip(gray, 0, 255).astype(np.uint8)
    low, high = np.percentile(gray, [2, 98])
    if high > low:
        gray = np.clip((gray.astype(np.float32) - low) * (255.0 / (high - low)), 0, 255).astype(np.uint8)
    return cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)


def _detection(kind, target_id, corners, shape, confidence=1.0, **extra):
    points = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    if not np.isfinite(points).all() or abs(cv2.contourArea(points.astype(np.float32))) < 4:
        return None
    height, width = shape[:2]
    diagonal_a = points[2] - points[0]
    diagonal_b = points[3] - points[1]
    denominator = diagonal_a[0] * diagonal_b[1] - diagonal_a[1] * diagonal_b[0]
    if abs(denominator) < 1e-9:
        return None
    offset = points[1] - points[0]
    parameter = (offset[0] * diagonal_b[1] - offset[1] * diagonal_b[0]) / denominator
    center = points[0] + parameter * diagonal_a
    if not np.isfinite(center).all():
        return None
    return {
        "kind": kind,
        "target_id": str(target_id),
        "center": np.clip(center / [width, height], 0.0, 1.0).tolist(),
        "center_px": center.tolist(),
        "confidence": float(confidence),
        "corners_px": points.tolist(),
        "bbox": [float(points[:, 0].min()), float(points[:, 1].min()),
                 float(points[:, 0].max()), float(points[:, 1].max())],
        **extra,
    }


def validate_aruco_dictionary(name: str) -> str:
    if not isinstance(name, str) or not name.strip():
        raise ValueError("aruco_dictionary must be a non-empty name")
    aruco = getattr(cv2, "aruco", None)
    if aruco is None or not hasattr(aruco, name):
        raise ValueError(f"aruco_dictionary has unknown ArUco dictionary: {name}")
    return name


def normalise_template_paths(templates: Mapping[str, str | list[str]], *, require_absolute=False) -> dict[str, list[str]]:
    """Expand each category's path(s) without inventing a new identity per image."""
    if not isinstance(templates, Mapping):
        raise ValueError("reference mapping must be an object")
    result = {}
    for target_id, configured in templates.items():
        paths = [configured] if isinstance(configured, str) else configured
        if (not isinstance(target_id, str) or not target_id.strip()
                or not isinstance(paths, list) or not paths
                or any(not isinstance(path, str) or not path.strip() for path in paths)):
            raise ValueError("reference mapping requires non-empty target IDs and a path or non-empty list of paths")
        if require_absolute and any(not Path(path).is_absolute() for path in paths):
            raise ValueError("reference image paths must be absolute in the runtime container")
        result[target_id] = list(dict.fromkeys(paths))
    return result


class TemplateMatcher:
    """Cache explicit local reference images and reject unsupported matches.

    At least 12 RANSAC inliers, a 55% inlier ratio, a convex visible quadrilateral
    and spatially distributed reference features are required. Featureless images
    cannot be registered; no model or unknown task answer is downloaded.
    A category may map to one path or a non-empty list of reference paths.
    """

    def __init__(self, templates: Mapping[str, str | list[str]]):
        self._orb = cv2.ORB_create(nfeatures=2200, fastThreshold=8, edgeThreshold=15)
        self._references = []
        references = normalise_template_paths(templates)
        for target_id, paths in references.items():
            for path in paths:
                self._register_reference(target_id, path)

    def _register_reference(self, target_id, path):
        try:
            # imdecode handles local Unicode paths on Windows OpenCV builds.
            raw = np.frombuffer(Path(path).read_bytes(), dtype=np.uint8)
            image = cv2.imdecode(raw, cv2.IMREAD_GRAYSCALE)
        except (OSError, ValueError) as exc:
            raise ValueError(f"reference image unavailable: {path}") from exc
        if image is None:
            raise ValueError(f"reference image cannot be decoded: {path}")
        gray = _normalise(image)
        keypoints, descriptors = self._orb.detectAndCompute(gray, None)
        if descriptors is None or len(keypoints) < 12:
            raise ValueError(f"reference image has insufficient distinct features: {path}")
        self._references.append((target_id, gray.shape, keypoints, descriptors))

    def detect(self, gray) -> list[dict]:
        if not self._references:
            return []
        frame_points, frame_descriptors = self._orb.detectAndCompute(gray, None)
        if frame_descriptors is None or len(frame_points) < 12:
            return []
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        results = []
        frame_supports = []
        for target_id, (height, width), points, descriptors in self._references:
            pairs = matcher.knnMatch(descriptors, frame_descriptors, k=2)
            good = [pair[0] for pair in pairs if len(pair) == 2 and pair[0].distance < 0.72 * pair[1].distance]
            unique = {}
            for match in sorted(good, key=lambda item: item.distance):
                unique.setdefault(match.trainIdx, match)
            good = list(unique.values())
            if len(good) < 12:
                continue
            reference_xy = np.float32([points[m.queryIdx].pt for m in good])
            frame_xy = np.float32([frame_points[m.trainIdx].pt for m in good])
            transform, mask = cv2.findHomography(reference_xy, frame_xy, cv2.RANSAC, 3.5)
            if transform is None or mask is None or not np.isfinite(transform).all():
                continue
            inliers = mask.ravel().astype(bool)
            count = int(inliers.sum())
            ratio = count / len(good)
            if count < 12 or ratio < 0.55:
                continue
            coverage = cv2.contourArea(cv2.convexHull(reference_xy[inliers])) / (width * height)
            if coverage < 0.07:
                continue
            rectangle = np.float32([[[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]]])
            corners = cv2.perspectiveTransform(rectangle, transform)[0]
            if not np.isfinite(corners).all() or not cv2.isContourConvex(corners):
                continue
            area = abs(cv2.contourArea(corners))
            frame_h, frame_w = gray.shape[:2]
            if area < 250 or area > frame_w * frame_h * 0.95:
                continue
            if ((corners < -2).any() or (corners[:, 0] > frame_w + 1).any()
                    or (corners[:, 1] > frame_h + 1).any()):
                continue
            detection = _detection("template", target_id, corners, gray.shape, ratio,
                                   inlier_count=count, match_count=len(good))
            if detection is not None:
                results.append(detection)
                frame_supports.append({match.trainIdx for match, accepted in zip(good, inliers) if accepted})
        # Padding/cropping shifts reference centers without changing the image
        # evidence. Reject competing identities that explain the same observed
        # surface or reuse most of the same RANSAC-inlier frame features.
        ambiguous = set()
        same_surface = set()
        for i, first in enumerate(results):
            for j in range(i + 1, len(results)):
                second = results[j]
                first_polygon = np.asarray(first["corners_px"], dtype=np.float32)
                second_polygon = np.asarray(second["corners_px"], dtype=np.float32)
                intersection_area, _ = cv2.intersectConvexConvex(first_polygon, second_polygon)
                smaller_area = min(abs(cv2.contourArea(first_polygon)), abs(cv2.contourArea(second_polygon)))
                surface_overlap = intersection_area / smaller_area
                shared_features = len(frame_supports[i] & frame_supports[j]) / min(len(frame_supports[i]), len(frame_supports[j]))
                if surface_overlap >= 0.6 or shared_features >= 0.5:
                    if first["target_id"] == second["target_id"]:
                        same_surface.add(frozenset((i, j)))
                    else:
                        ambiguous.update([i, j])
        # First reject every competing category using all raw evidence. Only
        # then collapse same-category duplicates, preserving the strongest
        # RANSAC support. Independent physical surfaces remain independent.
        ranked = sorted((i for i in range(len(results)) if i not in ambiguous),
            key=lambda i: (-results[i]["inlier_count"], -results[i]["confidence"],
                           tuple(value for point in results[i]["corners_px"] for value in point)))
        kept = []
        for index in ranked:
            if not any(frozenset((index, prior)) in same_surface for prior in kept):
                kept.append(index)
        return [results[index] for index in sorted(kept)]


def detect_targets(image, *, enable_qr: bool = True, enable_aruco: bool = True,
                   aruco_dictionary: str = "DICT_4X4_50",
                   template_matcher: TemplateMatcher | None = None) -> list[dict]:
    """Return identities and geometry actually supported by this frame."""
    gray = _normalise(image)
    detections: list[dict] = []
    if enable_qr:
        detector = cv2.QRCodeDetector()
        decoded = []
        if hasattr(detector, "detectAndDecodeMulti"):
            success, texts, points, _ = detector.detectAndDecodeMulti(gray)
            if success and points is not None:
                decoded = [(text, corners) for text, corners in zip(texts, points) if text]
        if not decoded:
            text, points, _ = detector.detectAndDecode(gray)
            if text and points is not None:
                decoded = [(text, points[0])]
        for text, points in decoded:
            detection = _detection("qr", text, points, gray.shape, text=text)
            if detection is not None:
                detections.append(detection)
    if enable_aruco and hasattr(cv2, "aruco"):
        dictionary_name = validate_aruco_dictionary(aruco_dictionary)
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
        if hasattr(cv2.aruco, "ArucoDetector"):
            corners, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(gray)
        elif hasattr(cv2.aruco, "detectMarkers"):
            corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
        else:
            corners, ids = [], None
        if ids is not None:
            for points, marker_id in zip(corners, ids.flatten().tolist()):
                detection = _detection("aruco", f"aruco:{marker_id}", points, gray.shape, id=int(marker_id))
                if detection is not None:
                    detections.append(detection)
    if template_matcher is not None:
        detections.extend(template_matcher.detect(gray))
    return detections

