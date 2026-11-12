"""Actual factory: real image -> TargetFix -> move -> feedback-gated light.

The plant poses are supplied; this tests protocol/geometry, not dynamics.
"""
from pathlib import Path
import sys,json,math,hashlib
ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'src/inspection_mission'),str(ROOT/'tests')]
import cv2,numpy as np
from pytest import MonkeyPatch
from inspection_mission import full_flight_inspection as m
from inspection_mission.ground_template_catalog import load_complete_ground_templates
from test_full_flight_inspection import install_ros,feed_node,proof
from test_floor_plane_detector import observation,synthetic

patch=MonkeyPatch();_,published,_,_=install_ros(patch);wall=[0.]
node=m.create_flight_inspection_node(session_id='independent',ground_return_evidence=proof(),wall_clock=lambda:wall[0])
try:
    specs=load_complete_ground_templates();spec=specs[0]
    original=cv2.imdecode(np.frombuffer(Path(spec.path).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    bgr,_=synthetic(original);calibration,prior=observation()
    c=node.guard.controller=m.FlightMissionController('independent',.055);c.start_flight(30.)
    node.guard.home_z=.055
    c._stage_index=next(i for i,s in enumerate(c._stages) if s.name=='AIR_RECOGNIZE_1');c.state='AIR_RECOGNIZE_1'
    rows=[]
    for i in range(12):
        wall[0]=i*.30;stamp=30.+wall[0]
        feed_node(node,wall[0],stamp,x=3.08,y=-2.15,z=.9,yaw=math.pi/2)
        header={'frame_id':m.CAMERA_FRAME,'stamp':{'sec':int(stamp),'nanosec':round((stamp-int(stamp))*1e9)}}
        node.receive('/uav/camera/camera_info',dict(calibration,header=header,width=800,height=800))
        node.receive('/uav/camera/image',{'header':header,'width':800,'height':800,'step':2400,'encoding':'bgr8','data':bgr.tobytes()})
        if node.vision_future is not None:node.vision_future.result(timeout=25)
        node.tick();rows.append({'stamp':stamp,'state':c.state,'detected_count':len(node.detections)})
        if c.state=='AIR_MOVE_TO_LIGHT_1':break
    reached_move=c.state=='AIR_MOVE_TO_LIGHT_1'
    fix=c.fixes.get('a1')
    if reached_move:
        target=c._stages[c._stage_index].target
        for j in range(1,45):
            wall[0]+= .1;stamp=round(30.+wall[0],8)
            feed_node(node,wall[0],stamp,x=target[0],y=target[1],z=target[2],yaw=target[3])
            # Independent feedback follows an actual previous ON request.
            last_light=next((p['data'] for t,p in reversed(published) if t=='/uav/laser_fire'),False)
            node.receive('/uav/laser_state',{'data':last_light is True})
            node.tick();rows.append({'stamp':stamp,'state':c.state,'light_feedback':last_light,
                'aim_allowed':bool(c.aim and c.aim.allowed),'aim_reason':c.aim.reason if c.aim else None})
            if c.state=='AIR_NAV_2' or node.guard.stop_wall is not None:break
    data={'full_flight_source_sha256':hashlib.sha256((ROOT/'src/inspection_mission/inspection_mission/full_flight_inspection.py').read_bytes()).hexdigest(),
        'state':c.state,'reason':node.guard.reason,'stop_wall':node.guard.stop_wall,'reached_move':reached_move,
        'genuine_fix_error_m':fix.position_error_m if fix else None,'genuine_fix_stamp_s':fix.stamp_s if fix else None,
        'fix_unchanged_after_light':c.fixes.get('a1')==fix,'recognition_reports':[p['data'] for t,p in published if t=='/inspection/detections'],
        'on_command_count':sum(p['data'] is True for t,p in published if t=='/uav/laser_fire'),
        'rows':rows,'errors':node.errors}
    (Path(__file__).parent/'factory_floor_light_metrics.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in data.items() if k!='rows'},ensure_ascii=False,indent=2))
finally:node.destroy_node();patch.undo()
