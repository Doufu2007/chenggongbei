"""Fixed-frame warm timing only; no platform performance claim."""
from pathlib import Path
import hashlib
import json
import sys
import time

ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'src/inspection_mission'),str(ROOT/'tests')]
import cv2
import numpy as np
from inspection_mission.floor_plane_detector import FloorPlaneDetector
from inspection_mission.ground_template_catalog import load_complete_ground_templates
from test_floor_plane_detector import observation,synthetic

tag=sys.argv[1] if len(sys.argv)>1 else 'current'
specs=load_complete_ground_templates()
image=cv2.imdecode(np.frombuffer(Path(specs[0].path).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
frame,_=synthetic(image);calibration,prior=observation()
start=time.perf_counter();detector=FloorPlaneDetector(specs);initialization=time.perf_counter()-start
timings=[];answers=[]
for _ in range(5):
    start=time.perf_counter()
    found=detector.detect(frame,calibration,prior,image_stamp=30.)
    timings.append(time.perf_counter()-start)
    answers.append([{'label':x['target_id'],'inliers':x['inlier_count']} for x in found])
    assert len(found)==1 and found[0]['target_id']==specs[0].label
result={'source_sha256':hashlib.sha256((ROOT/'src/inspection_mission/inspection_mission/floor_plane_detector.py').read_bytes()).hexdigest(),
    'initialization_wall_s':initialization,'detect_wall_s':timings,'warm_max_wall_s':max(timings[1:]),
    'warm_mean_wall_s':sum(timings[1:])/4,'answers':answers,'tag':tag,
    'limitation':'One public-geometry synthetic frame on this host; not real-time platform timing guarantee.'}
(Path(__file__).parent/('floor_performance_'+tag+'.json')).write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False))
