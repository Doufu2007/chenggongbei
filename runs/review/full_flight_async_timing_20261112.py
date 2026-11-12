"""Real monotonic worker/tick timing with synthetic public-geometry images."""
import sys, json, time, math
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'tests'), str(ROOT/'src/inspection_mission')]
import cv2, numpy as np, pytest
from test_full_flight_inspection import install_ros, feed_node, proof
from test_floor_plane_detector import observation, synthetic
from inspection_mission.ground_template_catalog import load_complete_ground_templates
import inspection_mission.full_flight_inspection as m

spec = load_complete_ground_templates()[0]
image = cv2.imdecode(np.frombuffer(Path(spec.path).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
image, _ = synthetic(image); image_raw = image.tobytes(); calibration, _ = observation()
patch = pytest.MonkeyPatch(); _, published, _, _ = install_ros(patch)
t0 = time.monotonic(); real_wall = lambda: time.monotonic()-t0
node = m.create_flight_inspection_node(session_id='async_timing', ground_return_evidence=proof(), wall_clock=real_wall)
g = node.guard; g.home_z=.055; c = g.controller=m.FlightMissionController('async_timing',.055); c.start_flight(30.)
index = next(i for i,s in enumerate(c._stages) if s.name=='AIR_RECOGNIZE_1')
c._stage_index=index; c.state=c._stages[index].name; c._phase_started=30.
results=[]; last_stamp=None; submitted={}; last_image_wall=-1.; loops=0
while real_wall()<16.:
    wall=real_wall(); source=30.+wall
    feed_node(node,wall,source,x=3.08,y=-2.15,z=.9,yaw=math.pi/2)
    camera=dict(calibration,width=800,height=800,header={'frame_id':m.CAMERA_FRAME,
        'stamp':{'sec':int(source),'nanosec':round((source-int(source))*1e9)}})
    node.receive('/uav/camera/camera_info',camera)
    if wall-last_image_wall>=.033:
        node.receive('/uav/camera/image',dict(width=800,height=800,step=2400,encoding='bgr8',data=image_raw,
            header=camera['header']))
        last_image_wall=wall
    old=node.vision_future
    if old is not None and old not in submitted:submitted[old]=wall
    node.tick();loops+=1
    stamp=node.detections[0]['stamp'] if node.detections else None
    if stamp is not None and stamp!=last_stamp:
        results.append({'wall_s':real_wall(),'source_age_s':30.+real_wall()-stamp,'exposure_stamp_s':stamp,
            'stage':c.state,'evidence_since':c._evidence_since,'inlier_count':node.detections[0].get('inlier_count')})
        last_stamp=stamp
    if c._stage_index!=index or g.stop_wall is not None:break
    time.sleep(.02)
elapsed=real_wall();node.close_watchdog();node.destroy_node();patch.undo()
result={'kind':'real_wall_worker_timing_with_synthetic_public_geometry_images_not_platform',
    'source_clock':'30+time.monotonic_elapsed;not_frozen_while_worker_runs','real_wall_s':elapsed,
    'stage':c.state,'advanced':c._stage_index!=index,'guard_state':g.state,'reason':g.reason,'loops':loops,
    'recognition_publish_count':sum(t=='/inspection/detections' for t,p in published),
    'results':results,'errors':node.errors,'retained_images':len(node.visual_records)}
Path(__file__).with_suffix('.json').write_text(json.dumps(m._safe(result),ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=True))
