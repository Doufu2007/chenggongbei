"""Independent red regression for a physical projection crossing camera depth=0."""
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'tests'),str(ROOT/'src/inspection_mission')]
from test_ground_plane_matcher import texture,save_template
from inspection_mission.ground_plane_matcher import GroundPlaneMatcher,TemplateSpec


def test_visible_front_part_cannot_approve_a_quad_crossing_the_camera_plane(tmp_path):
    """Render only positive-depth ground rays; near edge of board is behind lens.

    Board rectangle X=[.30,.74], Y=[-.25,.25]m; camera X=.390m.
    This uses independent ray inversion rather than the module's rectification.
    """
    cv2.setNumThreads(1)
    pattern=texture()
    K=np.array([[300.,0,400.],[0,300.,100.],[0,0,1.]])
    v,u=np.indices((800,800),dtype=np.float64)
    den=v-100
    forward=np.full_like(den,100.)
    np.divide(51.9,den,out=forward,where=den>0)
    forward+=.390
    left=(400-u)*(forward-.390)/300
    template_u=((.25-left)/.5*299).astype(np.float32)
    template_v=((.74-forward)/.44*263).astype(np.float32)
    image=cv2.remap(pattern,template_u,template_v,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
    image[den<=0]=0
    path=save_template(tmp_path,pattern)
    result=GroundPlaneMatcher([TemplateSpec(path,'synthetic','synthetic',{'source':'independent horizon stress'})]).match(image,K)
    assert not result['candidates'], [
        {key:row.get(key) for key in ['accepted_candidate','inliers','query_distinct_xy',
                                    'plane_area_m2','source_quad_xy','body_plane_quad_forward_left_m']}
        for row in result['candidates']]
