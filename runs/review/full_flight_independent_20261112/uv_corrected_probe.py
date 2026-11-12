"""Same scale/pose, with orientation-preserving texture homography."""
from pathlib import Path
import sys,json,math
ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'src/inspection_mission'),str(ROOT/'tests')]
import cv2,numpy as np
from inspection_mission import full_flight_inspection as m
from inspection_mission.vision_core import TemplateMatcher
from test_bounded_vertical_response import odom
mapping=m.load_flight_references();matcher=TemplateMatcher(mapping);matcher.labels=list(mapping)
fx=400/math.tan(math.radians(40));K=[fx,0.,400.,0.,fx,400.,0.,0.,1.]
# Swap the near/far edge order from the original mirrored synthetic case.
corners=np.float32([[347.5328607033996,705.2088445939387],[452.4671392966004,705.2088445939387],
                    [464.66879959813537,776.1876456622965],[335.33120040186463,776.1876456622965]])
out=[]
for label,paths in mapping.items():
    if label in m.WINDOW_LABELS.values():continue
    for path in paths:
        im=cv2.imdecode(np.frombuffer(Path(path).read_bytes(),np.uint8),cv2.IMREAD_COLOR);h,w=im.shape[:2]
        H=cv2.getPerspectiveTransform(np.float32([[0,0],[w-1,0],[w-1,h-1],[0,h-1]]),corners)
        canvas=cv2.warpPerspective(im,H,(800,800),borderValue=(170,170,170))
        frame={'width':800,'height':800,'step':2400,'encoding':'bgr8','data':canvas.tobytes(),
            'frame_id':m.CAMERA_FRAME,'stamp':30.,'requested_clock_s':30.}
        p=odom(30.,x=3.08,y=-2.15,z=.9);p['pose']['pose']['orientation'].update(z=math.sqrt(.5),w=math.sqrt(.5))
        result=m._vision_job(frame,{'k':K},p,session_id='synthetic',point_id='a1',kind='aerial',matcher=matcher)
        imagepath=Path(__file__).parent/('uv_corrected_'+Path(path).stem+'.png')
        imagepath.write_bytes(cv2.imencode('.png',canvas)[1].tobytes())
        out.append({'reference':Path(path).name,'label':label,'detections':[{'target_id':d['target_id'],'inliers':d['inlier_count'],'confidence':d['confidence']} for d in result['detections']]})
data={'texture_homography_determinant_sign':'orientation_preserving','corners_px':corners.tolist(),'results':out}
(Path(__file__).parent/'uv_corrected_metrics.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(data,ensure_ascii=False,indent=2))
