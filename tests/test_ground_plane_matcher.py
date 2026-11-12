"""Preimplementation cases; kept outside normal discovery during v19 suite."""
from __future__ import annotations

import hashlib
import importlib.util
from pathlib import Path
import sys

import cv2
import numpy as np
import pytest

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "src/inspection_mission").is_dir())
sys.path.insert(0, str(ROOT / "src/inspection_mission"))


@pytest.fixture
def module():
    spec = importlib.util.find_spec("inspection_mission.ground_plane_matcher")
    if spec is None:
        return None
    from inspection_mission import ground_plane_matcher
    return ground_plane_matcher


def texture(seed=412):
    rng = np.random.default_rng(seed)
    image = np.full((264, 300, 3), 225, np.uint8)
    for i in range(190):
        center = tuple(int(x) for x in rng.integers([8, 8], [292, 256]))
        color = tuple(int(x) for x in rng.integers(0, 200, 3))
        cv2.circle(image, center, int(rng.integers(3, 10)), color, -1, cv2.LINE_AA)
    for i in range(12):
        cv2.putText(image, str(i), (10+(i%4)*72, 42+(i//4)*82), cv2.FONT_HERSHEY_SIMPLEX,
                    .65, (0, 0, 0), 2, cv2.LINE_AA)
    return image


def save_template(tmp_path, image, name="pattern.png"):
    path = tmp_path / name
    ok, encoded = cv2.imencode(".png", image)
    assert ok
    path.write_bytes(encoded.tobytes())
    return path


def synthetic_camera_image(pattern):
    # Independently specified physical rectangle: X=[1.08,1.52],Y=[-.25,.25], area .22m².
    # Template top is the far edge, matching a top-down diagram.
    K = np.array([[1000., 0, 400.], [0, 1000., 300.], [0, 0, 1.]])
    physical = np.float64([[1.52, .25], [1.52, -.25], [1.08, -.25], [1.08, .25]])
    source = np.column_stack((400.-1000.*physical[:, 1]/(physical[:, 0]-.390),
                              300.+173./(physical[:, 0]-.390))).astype(np.float32)
    H = cv2.getPerspectiveTransform(np.float32([[0, 0], [299, 0], [299, 263], [0, 263]]), source)
    image = cv2.warpPerspective(pattern, H, (800, 800), flags=cv2.INTER_LINEAR)
    return image, K, source


def geometry_fixture():
    x, y = np.meshgrid(np.linspace(20, 280, 6), np.linspace(20, 240, 6))
    reference = np.column_stack((x.ravel(), y.ravel())).astype(np.float32)
    H = np.float64([[.5, 0, 300], [0, .5, 500], [0, 0, 1]])
    query = reference*.5 + [300., 500.]
    return reference, query.astype(np.float32), np.ones(36, bool), H


def test_synthetic_known_plane_returns_pixel_quad_metric_position_and_provenance(module, tmp_path):
    assert module is not None, "ground_plane_matcher is not implemented"
    image, K, expected_source = synthetic_camera_image(texture())
    path = save_template(tmp_path, texture())
    provenance = {"source": "synthetic", "official_current_library_verified": False}
    matcher = module.GroundPlaneMatcher([module.TemplateSpec(path, "alpha", "合成纹理", provenance)])
    result = matcher.match(image, K)
    assert len(result["candidates"]) == 1
    item = result["candidates"][0]
    assert item["class_id"] == "alpha" and item["label"] == "合成纹理"
    assert item["template_sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()
    assert item["provenance"] == provenance
    assert item["body_plane_center_forward_left_m"] == pytest.approx([1.30, 0.], abs=.025)
    assert item["plane_area_m2"] == pytest.approx(.22, abs=.02)
    assert np.max(np.linalg.norm(np.array(item["source_quad_xy"])-expected_source, axis=1)) < 5.
    assert result["candidate_calibration"] is True
    assert result["official_acceptance"] is False


def test_valid_distributed_known_homography_passes_all_physical_gates(module):
    r, q, mask, H = geometry_fixture()
    result = module.verify_geometry(r, q, mask, H, (300, 264))
    assert result["geometrically_accepted"] is True
    assert result["rejection_reasons"] == []
    assert result["plane_area_m2"] == pytest.approx(.2184361111111111)


@pytest.mark.parametrize("mutation,reason", [
    ("query_collapse", "query_distinct_xy_below_12"),
    ("template_collapse", "template_distinct_xy_below_12"),
    ("small_template_support", "template_hull_coverage_below_0_15"),
    ("weak_ratio", "inlier_fraction_below_0_25"),
    ("small_area", "plane_area_outside_0_15_to_0_30_m2"),
    ("large_area", "plane_area_outside_0_15_to_0_30_m2"),
    ("bad_reprojection", "median_reprojection_error_above_1_5_px"),
    ("small_query_support", "query_hull_over_projection_below_0_10"),
    ("nonfinite_H", "homography_nonfinite"),
    ("cross_horizon", "projection_nonconvex_or_degenerate"),
])
def test_geometry_rejects_each_independent_failure(module, mutation, reason):
    r, q, mask, H = geometry_fixture()
    if mutation == "query_collapse": q[:] = q[0]
    elif mutation == "template_collapse": r[:] = r[0]
    elif mutation == "small_template_support":
        r = (r-r.mean(axis=0))*.04 + r.mean(axis=0); q = r*.5+[300, 500]
    elif mutation == "weak_ratio":
        r = np.tile(r, (5, 1)); q = np.tile(q, (5, 1)); mask = np.arange(180)<36
    elif mutation == "small_area": H[:2, :2] = np.eye(2)*.2; q = r*.2+[300,500]
    elif mutation == "large_area": H[:2, :2] = np.eye(2)*.9; q = r*.9+[300,500]
    elif mutation == "bad_reprojection": q += 2.
    elif mutation == "small_query_support": q = (q-q.mean(axis=0))*.1 + q.mean(axis=0)
    elif mutation == "nonfinite_H": H[0,0] = np.nan
    elif mutation == "cross_horizon": H = np.float64([[1,0,0],[0,1,0],[.01,0,-1.5]])
    result = module.verify_geometry(r, q, mask, H, (300, 264))
    assert result["geometrically_accepted"] is False
    assert reason in result["rejection_reasons"]


def conflict_row(label, offset=0):
    return {"class_id": label, "template_path": label+".png", "template_sha256": label,
            "label": label, "provenance": {"source": "synthetic"},
            "geometrically_accepted": True, "rejection_reasons": [],
            "bev_quad_xy": [[offset,0],[offset+100,0],[offset+100,100],[offset,100]]}


def test_overlapping_different_classes_are_both_rejected_without_score_selection(module):
    result = module.reject_overlapping_class_conflicts([conflict_row("alpha"), conflict_row("beta", 50)])
    assert all(not x["accepted_candidate"] for x in result)
    assert all("overlapping_class_conflict" in x["rejection_reasons"] for x in result)


def test_different_physical_boards_remain_multiple_candidates(module):
    result = module.reject_overlapping_class_conflicts([conflict_row("alpha"), conflict_row("beta", 150)])
    assert all(x["accepted_candidate"] for x in result)


def test_same_class_overlap_is_retained_as_multiple_reference_evidence(module):
    result = module.reject_overlapping_class_conflicts([conflict_row("alpha"), conflict_row("alpha", 50)])
    assert all(x["accepted_candidate"] for x in result)


@pytest.mark.parametrize("bad", [np.zeros((800,800),np.uint8), np.zeros((8,8,4),np.uint8),
                                  np.zeros((8,8,3),np.float32),np.zeros((0,0,3),np.uint8)])
def test_invalid_image_format_is_rejected(module, bad):
    with pytest.raises(ValueError, match="BGR|uint8|empty"):
        module.rectify_ground_plane(bad,np.eye(3))


@pytest.mark.parametrize("bad", [np.zeros((3,3)), np.eye(2), np.full((3,3),np.nan),
                                  [[-1,0,4],[0,1,4],[0,0,1]], [[1,0,4],[0,1,4],[0,1,1]]])
def test_invalid_intrinsics_are_rejected(module, bad):
    with pytest.raises(ValueError, match="K"):
        module.rectify_ground_plane(np.zeros((8,8,3),np.uint8),bad)


@pytest.mark.parametrize("bad", [0.,-.173,float("nan"),float("inf"),.2])
def test_invalid_or_changed_candidate_height_is_rejected(module, bad):
    with pytest.raises(ValueError, match="height"):
        module.rectify_ground_plane(np.zeros((8,8,3),np.uint8),np.array([[5,0,4],[0,5,4],[0,0,1.]]),height_m=bad)


def test_template_hash_mismatch_is_rejected_at_loading(module, tmp_path):
    path = save_template(tmp_path, texture())
    with pytest.raises(ValueError, match="SHA256"):
        module.GroundPlaneMatcher([module.TemplateSpec(path,"alpha","alpha",{"source":"synthetic"},"0"*64)])


def test_blank_scene_returns_no_candidates_with_all_template_diagnostics(module, tmp_path):
    path = save_template(tmp_path, texture())
    matcher = module.GroundPlaneMatcher([module.TemplateSpec(path,"alpha","alpha",{"source":"synthetic"})])
    result = matcher.match(np.zeros((800,800,3),np.uint8),np.array([[500,0,400],[0,500,400],[0,0,1.]]))
    assert result["candidates"] == []
    assert len(result["diagnostics"]) == 1
    assert "insufficient_good_matches" in result["diagnostics"][0]["rejection_reasons"]


def test_visible_front_fragment_cannot_approve_board_crossing_camera_depth_zero(module, tmp_path):
    # Independent ray inversion renders only physically visible positive-depth rays.
    # Board X=[.30,.74] crosses camera X=.390 although its visible part matches well.
    pattern = texture()
    K = np.array([[300.,0,400.],[0,300.,100.],[0,0,1.]])
    v, u = np.indices((800,800),dtype=np.float64)
    denominator = v-100
    forward = np.full_like(denominator,100.)
    np.divide(51.9,denominator,out=forward,where=denominator>0)
    forward += .390
    left = (400-u)*(forward-.390)/300
    template_u = ((.25-left)/.5*299).astype(np.float32)
    template_v = ((.74-forward)/.44*263).astype(np.float32)
    image = cv2.remap(pattern,template_u,template_v,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
    image[denominator<=0] = 0
    path = save_template(tmp_path,pattern)
    result = module.GroundPlaneMatcher([module.TemplateSpec(path,"synthetic","synthetic",{"source":"synthetic camera-plane stress"})]).match(image,K)
    assert result["candidates"] == []
    row = result["diagnostics"][0]
    assert row["query_distinct_xy"] >= 12, "fixture must exercise strong SIFT support"
    assert "nonpositive_camera_depth" in row["rejection_reasons"]
    assert "source_projection_nonfinite_or_nonconvex" in row["rejection_reasons"]
