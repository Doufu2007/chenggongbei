"""Exposure-pose floor IPM followed by the unchanged production ORB gates.

This is a public-scene calibration candidate, not measured platform calibration.
No point identifier, map target center, task answer or control interface enters
the detector. Identity is supplied exclusively by explicit reference images.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import math
from pathlib import Path

import cv2
import numpy as np

from .bounded_vertical_response import _pose
from .ground_plane_matcher import TemplateSpec
from .target_geometry import _rotate
from .vision_core import TemplateMatcher, _normalise


PPM = 600
VIEW_SIZE = 480
FORWARD_RANGE_M = (1.43, 2.23)
LEFT_RANGE_M = (-.4, .4)
CAMERA_OFFSET = (.390, 0., .130)
OPTICAL_Q = (-.5, .5, -.5, .5)
PLANE_Z_M = .012
BOARD_SIZE_M = (.35, .30)
BOARD_AREA_RANGE_M2 = (.08, .13)
DERIVATION_VERSION = 'orb_floor_ipm_v3:long150_then300_if_no_candidate:horizontal_mirror'


class _SmallReferenceMatcher(TemplateMatcher):
    """Only reference preparation differs; detect() and every ORB gate inherit."""
    def __init__(self, templates):
        self._fallback_references = []
        super().__init__(templates)
        self.fallback = TemplateMatcher({})
        self.fallback._references = self._fallback_references

    def _register_reference(self, target_id, path):
        raw = Path(path).read_bytes()
        image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError('unreadable floor reference')
        for longest in (150, 300):
            scale = longest / max(image.shape)
            size = (max(2, round(image.shape[1]*scale)), max(2, round(image.shape[0]*scale)))
            resized = cv2.resize(image, size, interpolation=cv2.INTER_AREA)
            for reflected in (False, True):
                gray = _normalise(cv2.flip(resized, 1) if reflected else resized)
                points, descriptors = self._orb.detectAndCompute(gray, None)
                if descriptors is not None and len(points) >= 12:
                    refs = self._references if longest == 150 else self._fallback_references
                    refs.append((target_id, gray.shape, points, descriptors))


def rectify_floor_plane(image_bgr, calibration, prior, *, image_stamp):
    """Map a fixed geometry-only .8m square into metric, yaw-aligned BEV.

    Bounds describe the public observation view, not a detected board or class.
    The whole camera/body orientation and preceding exposure pose are used.
    Lens distortion must be zero or already rectified by the transport.
    """
    if (not isinstance(image_bgr, np.ndarray) or image_bgr.dtype != np.uint8
            or image_bgr.ndim != 3 or image_bgr.shape[2] != 3 or not image_bgr.size):
        raise ValueError('nonempty uint8 BGR floor image required')
    try:
        K = np.asarray(calibration['k'], np.float64).reshape(3, 3)
        distortion = np.asarray(calibration.get('d', []), np.float64)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('actual pinhole camera calibration required') from exc
    h, w = image_bgr.shape[:2]
    if (not np.isfinite(K).all() or K[0, 0] <= 0 or K[1, 1] <= 0
            or not np.allclose(K[2], [0, 0, 1], rtol=0, atol=1e-12)
            or abs(K[0, 1]) > 1e-12 or abs(K[1, 0]) > 1e-12
            or not 0 <= K[0, 2] < w or not 0 <= K[1, 2] < h
            or not np.isfinite(distortion).all() or np.any(distortion != 0)):
        raise ValueError('finite actual pinhole K with zero distortion required')
    pose = _pose(prior, 'uav/base_link')
    if (not isinstance(image_stamp, (int, float)) or not math.isfinite(image_stamp)
            or not 0 <= image_stamp-pose['stamp'] <= .05+1e-9):
        raise ValueError('bounded preceding exposure pose required')
    q = tuple(x/math.hypot(*pose['q']) for x in pose['q'])
    Rb = np.column_stack([_rotate(q, tuple(v)) for v in np.eye(3)])
    Ro = np.column_stack([_rotate(OPTICAL_Q, tuple(v)) for v in np.eye(3)])
    camera_origin = np.asarray(pose['xyz']) + Rb @ np.asarray(CAMERA_OFFSET)
    Rcw = (Rb @ Ro).T
    yaw = math.atan2(Rb[1, 0], Rb[0, 0])
    forward = np.array([math.cos(yaw), math.sin(yaw), 0.])
    left = np.array([-math.sin(yaw), math.cos(yaw), 0.])
    floor_origin = np.array([pose['xyz'][0], pose['xyz'][1], PLANE_Z_M])
    G = K @ Rcw @ np.column_stack((forward, left, floor_origin-camera_origin))
    B = np.float64([[0, -PPM, PPM*LEFT_RANGE_M[1]-.5],
                    [-PPM, 0, PPM*FORWARD_RANGE_M[1]-.5], [0, 0, 1]])
    if not np.isfinite(G).all() or abs(np.linalg.det(G)) < 1e-9:
        raise ValueError('floor projection is degenerate')
    source_to_bev = B @ np.linalg.inv(G)
    # Every retained metric pixel must project in front of the optical camera.
    rows, cols = np.indices((VIEW_SIZE, VIEW_SIZE), dtype=np.float64)
    metric = np.stack((FORWARD_RANGE_M[1]-(rows+.5)/PPM,
                       LEFT_RANGE_M[1]-(cols+.5)/PPM, np.ones(rows.shape)))
    projected = np.einsum('ij,jhw->ihw', G, metric)
    depth = projected[2]
    with np.errstate(divide='ignore', invalid='ignore'):
        u = (projected[0]/depth).astype(np.float32)
        v = (projected[1]/depth).astype(np.float32)
    valid = (depth > 1e-6) & (u >= 0) & (u <= w-1) & (v >= 0) & (v <= h-1)
    u[~valid] = -1.; v[~valid] = -1.
    bev = cv2.remap(image_bgr, u, v, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return {'bev_bgr': bev, 'bev_to_source': np.linalg.inv(source_to_bev),
            'source_to_bev': source_to_bev, 'valid_fraction': float(valid.mean()),
            'pose_stamp_s': pose['stamp'], 'image_stamp_s': image_stamp}


class FloorPlaneDetector:
    """Cache small explicit references; expose original-camera detections."""
    def __init__(self, templates=None):
        if templates is None:
            from .ground_template_catalog import load_complete_ground_templates
            templates = load_complete_ground_templates()
        templates = list(templates)
        if not templates:
            raise ValueError('explicit floor references required')
        mapping = {}; self.reference_catalog = []
        for spec in templates:
            if (not isinstance(spec, TemplateSpec) or not spec.label.strip()
                    or not spec.class_id.strip()):
                raise ValueError('explicit annotated TemplateSpec required')
            path = Path(spec.path).resolve(); raw = path.read_bytes()
            sha = hashlib.sha256(raw).hexdigest()
            if spec.expected_sha256 is not None and sha != spec.expected_sha256:
                raise ValueError('floor template SHA256 mismatch')
            mapping.setdefault(spec.label, []).append(str(path))
            self.reference_catalog.append({'class_id': spec.class_id, 'label': spec.label,
                'path': str(path), 'sha256': sha, 'provenance': deepcopy(dict(spec.provenance)),
                'derivation_version': DERIVATION_VERSION})
        self._matcher = _SmallReferenceMatcher(mapping)
        if not self._matcher._references:
            raise ValueError('floor references lack spatial image features')
        self.last_diagnostics = []

    def detect(self, image_bgr, calibration, prior, *, image_stamp):
        view = rectify_floor_plane(image_bgr, calibration, prior, image_stamp=image_stamp)
        gray = _normalise(view['bev_bgr'])
        # Each pass considers every class/reference, never stops at a class
        # match. The larger cache is a separate complete detector used only
        # when the complete primary detector returned no candidate.
        found = self._matcher.detect(gray)
        if not found:
            found = self._matcher.fallback.detect(gray)
        h, w = image_bgr.shape[:2]
        output = []; self.last_diagnostics = []
        for detection in found:
            quad = np.asarray(detection['corners_px'], np.float32)
            area = abs(float(cv2.contourArea(quad))) / PPM**2
            edges = np.linalg.norm(np.roll(quad, -1, axis=0)-quad, axis=1)/PPM
            sides = sorted(((edges[0]+edges[2])/2, (edges[1]+edges[3])/2))
            reasons = []
            if not BOARD_AREA_RANGE_M2[0] <= area <= BOARD_AREA_RANGE_M2[1]:
                reasons.append('public_small_board_area_mismatch')
            if any(abs(a-b) > .20*b for a, b in zip(sides, sorted(BOARD_SIZE_M))):
                reasons.append('public_small_board_side_lengths_mismatch')
            source_quad = cv2.perspectiveTransform(quad[None], view['bev_to_source'])[0]
            source_center = cv2.perspectiveTransform(np.float32([[detection['center_px']]]), view['bev_to_source'])[0, 0]
            if (not np.isfinite(source_quad).all() or not np.isfinite(source_center).all()
                    or not cv2.isContourConvex(source_quad)
                    or (source_quad < 0).any() or (source_quad[:, 0] > w-1).any()
                    or (source_quad[:, 1] > h-1).any()):
                reasons.append('source_projection_not_finite_convex_visible')
            row = {'target_id': detection['target_id'], 'inlier_count': detection['inlier_count'],
                'confidence': detection['confidence'], 'plane_area_m2': area,
                'plane_side_lengths_m': sides, 'rejection_reasons': reasons,
                'bev_corners_px': quad.tolist(), 'source_corners_px': source_quad.tolist()}
            self.last_diagnostics.append(row)
            if reasons:
                continue
            item = dict(detection, center_px=source_center.tolist(),
                center=(source_center/[w, h]).tolist(), corners_px=source_quad.tolist(),
                bbox=[float(source_quad[:,0].min()), float(source_quad[:,1].min()),
                      float(source_quad[:,0].max()), float(source_quad[:,1].max())],
                plane_area_m2=area, plane_side_lengths_m=sides, candidate_calibration=True,
                official_acceptance=False, detector_version=DERIVATION_VERSION,
                reference_catalog=[deepcopy(r) for r in self.reference_catalog if r['label']==detection['target_id']])
            output.append(item)
        return output
