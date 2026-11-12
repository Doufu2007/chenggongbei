"""Public geometry synthetic images, never a platform recognition result."""
import importlib
import importlib.util
import math
from pathlib import Path

import cv2
import numpy as np
import pytest

from test_bounded_vertical_response import odom


def api():
    name='inspection_mission.floor_plane_detector'
    assert importlib.util.find_spec(name) is not None, 'floor IPM detector missing'
    return importlib.import_module(name)


def observation():
    prior=odom(30.,x=3.08,y=-2.15,z=.9)
    prior['pose']['pose']['orientation'].update(z=math.sqrt(.5),w=math.sqrt(.5))
    f=400/math.tan(math.radians(40))
    return {'k':[f,0.,400.,0.,f,400.,0.,0.,1.], 'd':[]},prior


def synthetic(image, *, mirrored=False, turns=0, size=(.35,.30)):
    # Independent world-corner projection for pose=(3.08,-2.15,.9,+pi/2).
    # Source top is far edge. No implementation constants generate these data.
    halfw,halfd=size[0]/2,size[1]/2
    world=[(3.08-halfw,-.32+halfd),(3.08+halfw,-.32+halfd),
           (3.08+halfw,-.32-halfd),(3.08-halfw,-.32-halfd)]
    f=400/math.tan(math.radians(40))
    corners=np.float32([[400+f*(x-3.08)/(y+2.15-.390),400+f*1.018/(y+2.15-.390)] for x,y in world])
    transformed=np.rot90(np.fliplr(image) if mirrored else image,turns).copy()
    h,w=transformed.shape[:2]
    H=cv2.getPerspectiveTransform(np.float32([[0,0],[w-1,0],[w-1,h-1],[0,h-1]]),corners)
    return cv2.warpPerspective(transformed,H,(800,800),borderValue=(170,170,170)),corners


@pytest.fixture(scope='module')
def references():
    from inspection_mission.ground_template_catalog import load_complete_ground_templates
    return load_complete_ground_templates()


@pytest.mark.parametrize('index',range(10))
def test_all_ten_public_references_pass_real_orb_ipm_and_source_localization(references,index):
    m=api();spec=references[index]
    detector=m.FloorPlaneDetector(references)
    image=cv2.imdecode(np.frombuffer(Path(spec.path).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    frame,corners=synthetic(image)
    calibration,prior=observation()
    found=detector.detect(frame,calibration,prior,image_stamp=30.)
    assert len(found)==1, (spec.path,detector.last_diagnostics)
    item=found[0]
    assert item['target_id']==spec.label
    assert item['inlier_count']>=12 and item['confidence']>=.55
    assert .08<=item['plane_area_m2']<=.13
    # Center is original exposure pixel, not 480x480 BEV center.
    assert item['center_px']==pytest.approx([400.,400+calibration['k'][0]*1.018/1.44],abs=3.)
    assert item['center']==pytest.approx(np.array(item['center_px'])/800)
    assert item['candidate_calibration'] is True and item['official_acceptance'] is False
    from inspection_mission.full_flight_inspection import localize_floor_pixel
    fix=localize_floor_pixel(item['center_px'],calibration['k'],prior,image_stamp=30.,
        camera_frame='uav/camera_link',session_id='synthetic',target_id=item['target_id'],point_id='a1',now=30.)
    assert fix.position_world_m==pytest.approx((3.08,-.32,.012),abs=.025)


def test_mirrored_rotation_is_explicit_reference_transform_and_wrong_size_rejected(references):
    m=api();detector=m.FloorPlaneDetector(references)
    image=cv2.imdecode(np.frombuffer(Path(references[1].path).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    calibration,prior=observation()
    frame,_=synthetic(image,mirrored=True,turns=1)
    found=detector.detect(frame,calibration,prior,image_stamp=30.)
    assert len(found)==1 and found[0]['target_id']==references[1].label
    bad,_=synthetic(image,size=(.50,.50))
    assert detector.detect(bad,calibration,prior,image_stamp=30.)==[]


def test_missing_stale_future_pose_distortion_or_frame_cannot_create_detections(references):
    m=api();detector=m.FloorPlaneDetector(references)
    calibration,prior=observation();empty=np.full((800,800,3),170,np.uint8)
    assert detector.detect(empty,calibration,prior,image_stamp=30.)==[]
    for stamp in (29.99,30.2):
        with pytest.raises(ValueError):detector.detect(empty,calibration,prior,image_stamp=stamp)
    bad=dict(calibration,d=[.01,0,0,0,0])
    with pytest.raises(ValueError):detector.detect(empty,bad,prior,image_stamp=30.)
    prior['child_frame_id']='other'
    with pytest.raises(ValueError):detector.detect(empty,calibration,prior,image_stamp=30.)


def test_different_annotations_on_identical_surface_are_rejected(tmp_path,references):
    m=api()
    from inspection_mission.ground_plane_matcher import TemplateSpec
    spec=references[1]
    other=TemplateSpec(spec.path,'other','different real annotation',spec.provenance,spec.expected_sha256)
    detector=m.FloorPlaneDetector([spec,other])
    image=cv2.imdecode(np.frombuffer(Path(spec.path).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    frame,_=synthetic(image);calibration,prior=observation()
    assert detector.detect(frame,calibration,prior,image_stamp=30.)==[]


def test_ipm_uses_full_exposure_roll_pitch_yaw_and_actual_intrinsics():
    m=api();roll,pitch,yaw=.025,-.03,1.2
    cr,sr=math.cos(roll/2),math.sin(roll/2)
    cp,sp=math.cos(pitch/2),math.sin(pitch/2)
    cy,sy=math.cos(yaw/2),math.sin(yaw/2)
    q=np.array([sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,
                cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy])
    x,y,z,w=q
    # Independent active quaternion rotation matrix.
    R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    Ropt=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    origin=np.array([1.,-1.,.9]);camera=origin+R@np.array([.390,0,.130])
    fwd=np.array([math.cos(yaw),math.sin(yaw),0])
    left=np.array([-math.sin(yaw),math.cos(yaw),0])
    plane=np.array([1.,-1.,.012])
    metric=np.array([[2.,.15],[2.,-.15],[1.65,-.15],[1.65,.15]])
    world=np.array([plane+f*fwd+l*left for f,l in metric])
    optical=(R@Ropt).T@(world-camera).T
    K=np.array([[510.,0,390.],[0,490.,405.],[0,0,1]])
    projected=(K@optical).T
    source=(projected[:,:2]/projected[:,2:]).astype(np.float32)
    prior=odom(30.,x=1.,y=-1.,z=.9)
    prior['pose']['pose']['orientation'].update(zip(('x','y','z','w'),map(float,q)))
    view=m.rectify_floor_plane(np.zeros((800,800,3),np.uint8),{'k':K.ravel().tolist()},prior,image_stamp=30.)
    rectified=cv2.perspectiveTransform(source[None],view['source_to_bev'])[0]
    expected=np.column_stack((600*(.4-metric[:,1])-.5,600*(2.23-metric[:,0])-.5))
    assert rectified==pytest.approx(expected,abs=1e-3)
