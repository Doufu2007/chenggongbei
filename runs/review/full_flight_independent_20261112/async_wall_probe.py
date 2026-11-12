"""Factory asynchronous freshness with true monotonic time and advancing source.

No worker result is awaited while source time is frozen. ROS is the existing
boundary fixture; actual worker, controller, localization and gates run.
"""
from pathlib import Path
import json,sys,math,time,hashlib
ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'src/inspection_mission'),str(ROOT/'tests')]
import cv2,numpy as np
from pytest import MonkeyPatch
from inspection_mission import full_flight_inspection as m
from inspection_mission.ground_template_catalog import load_complete_ground_templates
from test_full_flight_inspection import install_ros,feed_node,proof
from test_floor_plane_detector import observation,synthetic

patch=MonkeyPatch();_,published,_,_=install_ros(patch)
spec=load_complete_ground_templates()[0]
image=cv2.imdecode(np.frombuffer(Path(spec.path).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
bgr,_=synthetic(image);calibration,_=observation();raw=bgr.tobytes()
begin=time.monotonic()
node=m.create_flight_inspection_node(session_id='async-independent',ground_return_evidence=proof(),wall_clock=time.monotonic)
try:
    c=node.guard.controller=m.FlightMissionController('async-independent',.055);c.start_flight(30.)
    node.guard.home_z=.055
    c._stage_index=next(i for i,s in enumerate(c._stages) if s.name=='AIR_RECOGNIZE_1');c.state='AIR_RECOGNIZE_1'
    next_sensor=next_tick=begin;rows=[];worker_completions=[];last_future=None;last_submit=None
    while time.monotonic()-begin<12.:
        wall=time.monotonic();stamp=30.+wall-begin
        if wall>=next_sensor:
            next_sensor=wall+.04
            feed_node(node,wall,stamp,x=3.08,y=-2.15,z=.9,yaw=math.pi/2)
            header={'frame_id':m.CAMERA_FRAME,'stamp':{'sec':int(stamp),'nanosec':round((stamp-int(stamp))*1e9)}}
            node.receive('/uav/camera/camera_info',dict(calibration,header=header,width=800,height=800))
            node.receive('/uav/camera/image',{'header':header,'width':800,'height':800,'step':2400,'encoding':'bgr8','data':raw})
            if node.vision_future is not None and node.vision_future is not last_future:
                last_future=node.vision_future;last_submit=(time.monotonic(),stamp)
        if last_future is not None and last_future.done() and last_submit is not None:
            worker_completions.append({'wall_duration_s':time.monotonic()-last_submit[0],
                'source_age_at_completion_s':node.guard.clock['stamp']-last_submit[1]})
            last_submit=None
        if wall>=next_tick:
            next_tick=wall+.10
            node.tick()
            detection_stamp=max((d['stamp'] for d in node.detections),default=None)
            rows.append({'wall_elapsed_s':time.monotonic()-begin,'source_stamp_s':node.guard.clock['stamp'],
                'state':c.state,'detection_age_s':node.guard.clock['stamp']-detection_stamp if detection_stamp is not None else None,
                'evidence_since':c._evidence_since,'stop_reason':node.guard.reason if node.guard.stop_wall is not None else None})
            if c.state=='AIR_MOVE_TO_LIGHT_1' or node.guard.stop_wall is not None:break
        time.sleep(.002)
    data={'full_flight_source_sha256':hashlib.sha256((ROOT/'src/inspection_mission/inspection_mission/full_flight_inspection.py').read_bytes()).hexdigest(),
        'floor_source_sha256':hashlib.sha256((ROOT/'src/inspection_mission/inspection_mission/floor_plane_detector.py').read_bytes()).hexdigest(),
        'wall_elapsed_s':time.monotonic()-begin,'source_elapsed_s':node.guard.clock['stamp']-30.,'state':c.state,
        'stop_reason':node.guard.reason if node.guard.stop_wall is not None else None,
        'recognition_count':sum(t=='/inspection/detections' for t,p in published),
        'worker_completions':worker_completions,'rows':rows,'errors':node.errors}
    (Path(__file__).parent/'async_wall_metrics.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in data.items() if k!='rows'},ensure_ascii=False,indent=2))
finally:node.destroy_node();patch.undo()
