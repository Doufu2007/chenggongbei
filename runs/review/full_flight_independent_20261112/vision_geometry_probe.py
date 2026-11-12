"""Synthetic public-reference projections, never a run texture assignment."""
from pathlib import Path
import sys, json, math
ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'src/inspection_mission'),str(ROOT/'tests')]
import cv2
import numpy as np
from pytest import MonkeyPatch
from inspection_mission import full_flight_inspection as m
from inspection_mission.vision_core import TemplateMatcher
from test_bounded_vertical_response import odom
from test_full_flight_inspection import install_ros,feed_node,proof

mapping=m.load_flight_references()
matcher=TemplateMatcher(mapping)
matcher.labels=list(mapping)
fx=400/math.tan(math.radians(40))
K=[fx,0.,400.,0.,fx,400.,0.,0.,1.]
corners=[]
# Public sensor world/body .130 is independent of implementation constant.
# Observer a1=(3.08,-2.15,.9,+pi/2), target plane top=.012.
for x,y in [(3.08-.175,-.32-.15),(3.08+.175,-.32-.15),
            (3.08+.175,-.32+.15),(3.08-.175,-.32+.15)]:
    forward=y-(-2.15+.390)
    opt_x=x-3.08
    opt_y=.9+.130-.012
    corners.append((400+fx*opt_x/forward,400+fx*opt_y/forward))
results=[]
frames=[]
for label,paths in mapping.items():
    if label in m.WINDOW_LABELS.values():continue
    for path in paths:
        image=cv2.imdecode(np.frombuffer(Path(path).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
        h,w=image.shape[:2]
        H=cv2.getPerspectiveTransform(np.float32([[0,0],[w-1,0],[w-1,h-1],[0,h-1]]),np.float32(corners))
        canvas=cv2.warpPerspective(image,H,(800,800),borderValue=(170,170,170))
        raw=canvas.tobytes()
        frame={'width':800,'height':800,'step':2400,'encoding':'bgr8','data':raw,
            'frame_id':m.CAMERA_FRAME,'stamp':30.,'requested_clock_s':30.}
        p=odom(30.,x=3.08,y=-2.15,z=.9)
        p['pose']['pose']['orientation'].update(z=math.sin(math.pi/4),w=math.cos(math.pi/4))
        answer=m._vision_job(frame,{'k':K},p,session_id='synthetic',point_id='a1',kind='aerial',matcher=matcher)
        rows=[{'target_id':d['target_id'],'inlier_count':d['inlier_count'],'center_px':d['center_px'],
            'position_world_m':d['target_fix'].position_world_m} for d in answer['detections']]
        out=Path(__file__).parent/('synthetic_'+Path(path).stem+'.png')
        out.write_bytes(cv2.imencode('.png',canvas)[1].tobytes())
        results.append({'reference':Path(path).name,'label':label,'detected':rows,'synthetic_image':str(out)})
        frames.append((Path(path).name,canvas))
# Exercise the same payload through factory receive -> executor -> poll. The
# fake ROS boundary uses the project's existing message transport fixture;
# all vision code, reference loading, stage association and localization are
# production implementations without replacement.
patch=MonkeyPatch()
install_ros(patch)
wall=[0.]
node=m.create_flight_inspection_node(session_id='synthetic',ground_return_evidence=proof(),wall_clock=lambda:wall[0])
factory=[]
try:
    c=m.FlightMissionController('synthetic',.055);c.start_flight(30.)
    node.guard.controller=c;node.guard.home_z=.055
    c._stage_index=next(i for i,s in enumerate(c._stages) if s.name=='AIR_RECOGNIZE_1')
    c.state='AIR_RECOGNIZE_1'
    for i,(name,canvas) in enumerate(frames):
        wall[0]=i*.30;stamp=30.+wall[0]
        feed_node(node,wall[0],stamp,x=3.08,y=-2.15,z=.9,yaw=math.pi/2)
        header={'frame_id':m.CAMERA_FRAME,'stamp':{'sec':int(stamp),'nanosec':round((stamp-int(stamp))*1e9)}}
        node.receive('/uav/camera/camera_info',{'header':header,'width':800,'height':800,'k':K,'d':[]})
        node.receive('/uav/camera/image',{'header':header,'width':800,'height':800,'step':2400,'encoding':'bgr8','data':canvas.tobytes()})
        assert node.vision_future is not None
        node.vision_future.result(timeout=25)
        node._poll_vision()
        factory.append({'reference':name,'detections_count':len(node.detections),'errors':list(node.errors)})
finally:
    node.destroy_node();patch.undo()
data={'corners_px':corners,'results':results,'real_factory_worker_results':factory}
print(json.dumps(data,ensure_ascii=False,indent=2))
(Path(__file__).parent/'vision_geometry_metrics.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
