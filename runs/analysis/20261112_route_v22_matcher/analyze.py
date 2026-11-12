"""Frozen matcher, actual v22 UAV pixels/K/pose, offline only."""
from __future__ import annotations
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import platform
import sys
from time import perf_counter

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'src/inspection_mission'))
from inspection_mission.ground_plane_matcher import GroundPlaneMatcher,TemplateSpec

MODULE=ROOT/'src/inspection_mission/inspection_mission/ground_plane_matcher.py'
MODULE_SHA='1b9c9b3cd658f5192307161dbbe21954968e9680c98424250d2d0836e1dc710d'
SUMMARY=ROOT/'runs/analysis/20261112_route_v22_independent/summary.json'
LIBRARY=ROOT/'runs/analysis/20261112_ground_ipm_sift_exploratory/report.json'
# Fixed public candidate model geometry, solely compared after pixel matching.
GEOMETRY={'ground_0':{'center':[1.125,-1.350],'size':[.55,.40]},
          'ground_1':{'center':[0.,-1.350],'size':[.55,.40]},
          'ground_2':{'center':[0.,-2.700],'size':[.55,.40]},
          'ground_3':{'center':[1.125,-1.925],'size':[.40,.55]},
          'ground_4':{'center':[1.125,-.775],'size':[.40,.55]}}

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def save(path,image):
    ok,data=cv2.imencode(path.suffix,image)
    if not ok:raise ValueError('encode failed')
    path.write_bytes(data.tobytes())
def rotation(q):
    x,y,z,w=np.asarray(q,np.float64)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def timing(values):
    return {'all_ms':values,'min_ms':min(values),'median_ms':float(np.median(values)),
            'max_ms':max(values),'repetitions':len(values)}

def main():
    assert sha(MODULE)==MODULE_SHA
    summary=json.loads(SUMMARY.read_text(encoding='utf-8'))
    public=summary['snapshots']['assembled']['public_messages']
    task_order=public['/referee/ground_tasks']['last']['data'].split(',')
    info=public['/uav/camera/camera_info']['last']
    assert info['width']==info['height']==800
    assert all(x==0 for x in info['d'])
    K=np.array(info['k'],np.float64).reshape(3,3)
    assert np.allclose(K,np.array(public['/uav/camera/camera_info']['first']['k']).reshape(3,3))
    natives=[x for x in summary['native_images'] if x['topic']=='/uav/camera/image']
    latest=max(natives,key=lambda x:x['stamp_s'])
    associated=max((x for x in natives if x.get('phase')=='last_associated'),key=lambda x:x['stamp_s'])
    selected=[]
    for item in (latest,associated):
        if not any(x['sha256']==item['sha256'] and x['stamp_s']==item['stamp_s'] for x in selected):selected.append(item)
    library=json.loads(LIBRARY.read_text(encoding='utf-8'))
    specs=[TemplateSpec(x['absolute_path'],x['class_id'],x['label'],
                        {'source_url':x['source_url'],'git_blob_sha':x['git_blob_sha'],
                         'official_current_library_verified':False},x['sha256']) for x in library['references']]
    cv2.setNumThreads(1)
    started=perf_counter();matcher=GroundPlaneMatcher(specs);load_ms=(perf_counter()-started)*1000
    frames=[]
    for item in selected:
        path=SUMMARY.parent/item['file_name'];assert sha(path)==item['sha256']
        t=perf_counter();raw=cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv2.IMREAD_COLOR);decode_ms=(perf_counter()-t)*1000
        pose=item.get('exposure',{}).get('odom')
        valid=item.get('exposure_validation')=='valid_prior_pose_observation' and pose is not None
        durations=[];results=[]
        for index in range(1):
            t=perf_counter();result=matcher.match(raw,K,height_m=.173);durations.append((perf_counter()-t)*1000)
            if index==0:
                bev=result.pop('bev_bgr');results=result
            else:
                assert [(x['template_sha256'],x['good_matches'],x['inliers'],x['accepted_candidate']) for x in result['diagnostics']]==[(x['template_sha256'],x['good_matches'],x['inliers'],x['accepted_candidate']) for x in results['diagnostics']]
        overlay=raw.copy();position=np.array(pose['position'],np.float64) if valid else None
        R=rotation(pose['quaternion']) if valid else None
        for row in results['diagnostics']:
            row['world_candidate_center_xyz_m']=None
            if valid and row['body_plane_center_forward_left_m'] is not None:
                forward,left=row['body_plane_center_forward_left_m']
                body=np.array([forward,left,.012-position[2]])
                world=position+R@body
                row['world_candidate_center_xyz_m']=world.tolist()
                row['all_fixed_candidate_distances_m']={key:float(np.linalg.norm(world[:2]-np.array(value['center']))) for key,value in GEOMETRY.items()}
                row['inside_fixed_candidate_rectangles']=[key for key,g in GEOMETRY.items() if bool((abs(world[:2]-np.array(g['center']))<=np.array(g['size'])/2).all())]
                row['current_first_ID_center_distance_m']=row['all_fixed_candidate_distances_m'][task_order[0]]
                row['inside_current_first_ID_rectangle_candidate']=task_order[0] in row['inside_fixed_candidate_rectangles']
            if row['accepted_candidate'] and row['source_quad_xy'] is not None:
                points=np.rint(row['source_quad_xy']).astype(np.int32)
                cv2.polylines(overlay,[points],True,(0,255,0),2)
                cv2.putText(overlay,row['class_id']+' candidate',tuple(points[0]),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,255,0),1)
        # Refresh candidate copies with the post-match world diagnostics; no class/ID selection.
        results['candidates']=[x for x in results['diagnostics'] if x['accepted_candidate']]
        expected=[]
        if valid:
            for key,g in GEOMETRY.items():
                body=R.T@(np.array([*g['center'],.012])-position)
                depth=float(body[0]-.390)
                uv=[float(K[0,2]-K[0,0]*body[1]/depth),float(K[1,2]+K[1,1]*.173/depth)] if depth>0 else None
                expected.append({'ID_candidate':key,'body_forward_left_up_m':body.tolist(),'optical_depth_m':depth,
                                 'predicted_source_center_uv':uv,'in_800px_image':uv is not None and 0<=uv[0]<800 and 0<=uv[1]<800,
                                 'is_current_first_ID':key==task_order[0]})
        prefix='uav_'+str(item['stamp_s']).replace('.','_')
        save(OUT/(prefix+'_bev.png'),bev);save(OUT/(prefix+'_source_overlay.png'),overlay)
        frames.append({'source_path':path.relative_to(ROOT).as_posix(),'source_sha256':sha(path),'source_metadata':item,
                       'selected_as_latest':item['sha256']==latest['sha256'],'selected_as_last_associated':item['sha256']==associated['sha256'],
                       'pose_valid_prior':valid,'actual_camera_info':info,'jpeg_read_decode_ms':decode_ms,
                       'synchronous_match_timing':timing(durations),'single_match_call_only':True,
                       'one_retained_frame_not_new_observations':True,'expected_fixed_geometry_projections':expected,'result':results})
        print('selected',item['file_name'],'stamp',item['stamp_s'],'load_ms',load_ms,'decode_ms',decode_ms,'match_ms',durations,flush=True)
        print('candidates',[(x['class_id'],x['inliers'],x.get('world_candidate_center_xyz_m'),x.get('inside_fixed_candidate_rectangles')) for x in results['candidates']],flush=True)
    report={'kind':'v22_frozen_ground_plane_matcher_offline','created_at':datetime.now(timezone.utc).isoformat(),
            'module_sha256':sha(MODULE),'source_summary_path':SUMMARY.relative_to(ROOT).as_posix(),'source_summary_sha256':sha(SUMMARY),
            'template_manifest_path':LIBRARY.relative_to(ROOT).as_posix(),'template_manifest_sha256':sha(LIBRARY),
            'task_order_from_actual_public_message':task_order,'first_ID':task_order[0],
            'selection_rule':'latest original UAV native and last_associated, deduplicated by SHA/stamp; no first/thumbnail fallback',
            'cold_template_loading_ms':load_ms,'timing_environment':{'python':platform.python_version(),'opencv':cv2.__version__,
             'machine':platform.machine(),'processor':platform.processor(),'opencv_threads':cv2.getNumThreads(),
             'scope':'one process synchronous whole matcher; wall-clock includes IPM/query SIFT/12-template BF/RANSAC/geometry; one call only, not concurrency/ROS scheduling evidence'},
            'fixed_candidate_geometry':GEOMETRY,'geometry_source':'src/inspection_mission/inspection_mission/ground_observation_route_plan.py _POINTS; ID/model association candidate_unverified',
            'world_transform':'Use exposure prior UAV pose quaternion/position; body [forward,left,.012-body_z] rotated to world; never use planned observation pose.',
            'candidate_calibration':True,'class_ground_truth':None,'official_acceptance':False,'published':False,'production_modified':False,
            'frames':frames,'limitations':['v22 completed both drives then stopped on final TURN position drift; no CAPTURE success and not a validated settled observation pose.',
            'A pixel match at another physical candidate rectangle does not support reporting the current first ID.',
            'Extrinsics/ground model and ID-to-model map remain candidate assumptions; no official category or task binding truth.',
            'Timing is local CPU single-thread synchronous evidence, not a safe ROS callback budget or worker concurrency benchmark.']}
    (OUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    fields=['template','class_id','good','inliers','query_distinct','accepted_candidate','world_center','all_distances','inside_rectangles','rejection_reasons']
    with (OUT/'scores.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for frame in frames:
            for row in frame['result']['diagnostics']:
                writer.writerow({'template':Path(row['template_path']).name,'class_id':row['class_id'],'good':row['good_matches'],'inliers':row['inliers'],
                  'query_distinct':row['query_distinct_xy'],'accepted_candidate':row['accepted_candidate'],'world_center':row['world_candidate_center_xyz_m'],
                  'all_distances':row.get('all_fixed_candidate_distances_m'),'inside_rectangles':row.get('inside_fixed_candidate_rectangles'),
                  'rejection_reasons':';'.join(row['rejection_reasons'])})
    assert sha(MODULE)==MODULE_SHA

if __name__=='__main__':main()
