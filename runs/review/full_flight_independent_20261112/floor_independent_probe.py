"""Non-author floor IPM rotation/mirror and source-localization checks."""
from pathlib import Path
import sys,json,math,hashlib
ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'src/inspection_mission'),str(ROOT/'tests')]
import cv2,numpy as np
from inspection_mission.floor_plane_detector import FloorPlaneDetector
from inspection_mission.ground_template_catalog import load_complete_ground_templates
from inspection_mission.full_flight_inspection import localize_floor_pixel
from test_floor_plane_detector import observation,synthetic
specs=load_complete_ground_templates();detector=FloorPlaneDetector(specs)
calibration,prior=observation();rows=[]
for spec in specs:
    image=cv2.imdecode(np.frombuffer(Path(spec.path).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    for mirrored in (False,True):
        for turns in range(4):
            frame,_=synthetic(image,mirrored=mirrored,turns=turns)
            found=detector.detect(frame,calibration,prior,image_stamp=30.)
            positions=[]
            for d in found:
                fix=localize_floor_pixel(d['center_px'],calibration['k'],prior,image_stamp=30.,camera_frame='uav/camera_link',
                    session_id='independent',target_id=d['target_id'],point_id='a1',now=30.)
                positions.append({'target_id':d['target_id'],'world_xyz':fix.position_world_m,
                    'distance_error_m':math.dist(fix.position_world_m,(3.08,-.32,.012)),'inliers':d['inlier_count']})
            rows.append({'reference':Path(spec.path).name,'expected_label':spec.label,'mirrored':mirrored,'turns':turns,
                'detections':positions,'diagnostics':detector.last_diagnostics if not found else []})
data={'floor_source_sha256':hashlib.sha256((ROOT/'src/inspection_mission/inspection_mission/floor_plane_detector.py').read_bytes()).hexdigest(),
    'case_count':len(rows),'correct_one_count':sum(len(r['detections'])==1 and r['detections'][0]['target_id']==r['expected_label'] for r in rows),
    'wrong_classes':[r for r in rows if any(d['target_id']!=r['expected_label'] for d in r['detections'])],
    'max_localization_error_m':max(d['distance_error_m'] for r in rows for d in r['detections']),
    'rows':rows}
(Path(__file__).parent/'floor_independent_metrics.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in data.items() if k!='rows'},ensure_ascii=False,indent=2))
