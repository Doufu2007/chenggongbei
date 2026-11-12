"""Independent offline review cases; no ROS, web, controls or source mutation."""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "tests"), str(ROOT / "src/inspection_mission")]
from inspection_mission import ground_plane_matcher as gpm
from test_ground_plane_matcher import texture, save_template, synthetic_camera_image, geometry_fixture, conflict_row

cv2.setNumThreads(1)


def test_anisotropic_intrinsics_and_noncentral_principal_point_preserve_metric_axes_and_pixel_centers():
    K = np.array([[850.,0,480.],[0,620.,210.],[0,0,1.]])
    view = gpm.rectify_ground_plane(np.zeros((600,1000,3),np.uint8),K)
    points = np.array([[x,y] for x in [.8,1.3,2.8] for y in [-.2,0,.2]],np.float64)
    # Independent pinhole ray convention: optical right is negative body left.
    original = np.column_stack((480-850*points[:,1]/(points[:,0]-.390),
                                210+620*.173/(points[:,0]-.390)))
    expected_bev = np.column_stack((300*(1.6-points[:,1])-.5,300*(3.2-points[:,0])-.5))
    mapped = cv2.perspectiveTransform(original[None],view['source_to_bev'])[0]
    assert np.max(np.abs(mapped-expected_bev)) < 1e-9
    restored = cv2.perspectiveTransform(expected_bev[None],view['bev_to_source'])[0]
    assert np.max(np.abs(restored-original)) < 1e-9
    assert view['candidate_calibration'] is True


def test_end_to_end_identical_physical_match_with_different_class_annotations_rejects_both(tmp_path):
    pattern=texture();image,K,_=synthetic_camera_image(pattern)
    a=save_template(tmp_path,pattern,'alpha.png');b=save_template(tmp_path,pattern,'beta.png')
    matcher=gpm.GroundPlaneMatcher([gpm.TemplateSpec(a,'alpha','A',{'source':'synthetic'}),
                                    gpm.TemplateSpec(b,'beta','B',{'source':'synthetic'})])
    result=matcher.match(image,K)
    assert result['candidates']==[] and result['official_acceptance'] is False
    assert len(result['diagnostics'])==2
    assert all(r['geometrically_accepted'] and not r['accepted_candidate'] for r in result['diagnostics'])
    assert all('overlapping_class_conflict' in r['rejection_reasons'] for r in result['diagnostics'])


def test_annotations_and_provenance_are_snapshotted_and_never_become_board_id_or_official_acceptance(tmp_path):
    pattern=texture();image,K,_=synthetic_camera_image(pattern);path=save_template(tmp_path,pattern)
    provenance={'source':'synthetic','nested':{'revision':1},'official_current_library_verified':False}
    original=copy.deepcopy(provenance)
    matcher=gpm.GroundPlaneMatcher([gpm.TemplateSpec(path,'alpha','annotation only',provenance)])
    provenance['nested']['revision']=99
    result=matcher.match(image,K);item=result['candidates'][0]
    assert item['provenance']==original and item['label']=='annotation only'
    assert not {'ground_id','board_id','task_id','score'} & item.keys()
    assert result['official_acceptance'] is False
    item['provenance']['nested']['revision']=-1
    assert result['diagnostics'][0]['provenance']==original
    assert matcher.match(image,K)['candidates'][0]['provenance']==original


def test_conflict_chain_checks_every_original_geometric_candidate_and_leaves_input_unchanged():
    rows=[conflict_row('a',0),conflict_row('b',80),conflict_row('c',160)]
    before=copy.deepcopy(rows);result=gpm.reject_overlapping_class_conflicts(rows)
    assert rows==before
    assert all(not r['accepted_candidate'] for r in result)
    assert result[0]['conflicting_template_paths']==['b.png']
    assert result[1]['conflicting_template_paths']==['a.png','c.png']
    assert result[2]['conflicting_template_paths']==['b.png']
    assert all(r['accepted_candidate'] for r in gpm.reject_overlapping_class_conflicts([conflict_row('a'),conflict_row('b',100)]))


@pytest.mark.parametrize('mutation',['nonfinite_points','bad_mask','unequal_lengths'])
def test_invalid_correspondence_inputs_fail_explicitly(mutation):
    r,q,mask,H=geometry_fixture()
    if mutation=='nonfinite_points':q[0,0]=np.inf
    elif mutation=='bad_mask':mask=mask.astype(float);mask[0]=.5
    else:q=q[:-1]
    with pytest.raises(ValueError):gpm.verify_geometry(r,q,mask,H,(300,264))


def test_real_development_report_keeps_all_sources_and_five_frame_rejections_consistent():
    path=ROOT/'runs/analysis/20261112_ground_plane_matcher_dev/development_check/report.json'
    report=json.loads(path.read_text('utf-8'))
    study=json.loads((ROOT/'runs/analysis/20261112_ground_ipm_sift_exploratory/report.json').read_text('utf-8'))
    assert report['module_sha256']==hashlib.sha256((ROOT/report['module_path']).read_bytes()).hexdigest()
    assert report['study_report_sha256']==hashlib.sha256((ROOT/'runs/analysis/20261112_ground_ipm_sift_exploratory/report.json').read_bytes()).hexdigest()
    assert report['accuracy'] is None and report['class_ground_truth'] is None and not report['held_out']
    assert report['official_acceptance'] is False
    refs={str(Path(r['absolute_path']).resolve()):r for r in study['references']}
    assert len(refs)==12 and len(report['frames'])==5
    assert [len(f['result']['candidates']) for f in report['frames']]==[1,1,0,0,0]
    for f in report['frames']:
        source=ROOT/f['source_path']
        assert hashlib.sha256(source.read_bytes()).hexdigest()==f['source_sha256']
        result=f['result']; assert result['candidate_calibration'] and result['official_acceptance'] is False
        assert len(result['diagnostics'])==12
        assert all(not any(k in r for k in ['ground_id','board_id','task_id']) for r in result['diagnostics'])
        K=np.array(result['calibration']['K'])
        for row in result['diagnostics']:
            ref=refs[row['template_path']]
            assert row['template_sha256']==ref['sha256']==hashlib.sha256(Path(row['template_path']).read_bytes()).hexdigest()
            assert row['class_id']==ref['class_id'] and row['provenance']['official_current_library_verified'] is False
            assert row['good_matches']==len(row['matches'])
            inliers=[m for m in row['matches'] if m['inlier']]
            assert row['inliers']==len(inliers)
            if f['name'].startswith('v18'):
                assert not row['accepted_candidate'] and row['rejection_reasons']
                assert 'query_distinct_xy_below_12' in row['rejection_reasons']
            if row['accepted_candidate']:
                assert row['template_distinct_xy']>=12 and row['query_distinct_xy']>=12
                assert row['template_hull_fraction']>=.15 and row['inlier_fraction']>=.25
                assert .15<=row['plane_area_m2']<=.30 and row['query_hull_over_projection']>=.10
                assert row['median_reprojection_error_px']<=1.5 and row['rejection_reasons']==[]
                # Reproject metric outputs with the independent physical formula,
                # instead of calling the module's inverse homography again.
                physical=np.array(row['body_plane_quad_forward_left_m'])
                original=np.column_stack((K[0,2]-K[0,0]*physical[:,1]/(physical[:,0]-.390),
                                          K[1,2]+K[1,1]*.173/(physical[:,0]-.390)))
                assert np.max(np.abs(original-np.array(row['source_quad_xy'])))<1e-3
                x,y=row['body_plane_center_forward_left_m']
                center=[K[0,2]-K[0,0]*y/(x-.390),K[1,2]+K[1,1]*.173/(x-.390)]
                assert np.max(np.abs(np.array(center)-row['source_center_xy']))<1e-3
