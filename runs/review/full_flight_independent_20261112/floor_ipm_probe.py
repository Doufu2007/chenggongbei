"""Offline model experiment; no ROS, controls, or run-specific class assignment."""
from pathlib import Path
import sys, json, math
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/inspection_mission'))
import cv2
import numpy as np
from inspection_mission.ground_template_catalog import load_complete_ground_templates
from inspection_mission.ground_plane_matcher import verify_geometry

fx=400/math.tan(math.radians(40))
K=np.float64([[fx,0,400],[0,fx,400],[0,0,1]])
# local forward/left plane -> source, camera origin .39 forward, 1.018 above plane
G=K@np.float64([[0,-1,0],[0,0,1.018],[1,0,-.39]])
B=np.float64([[0,-300,479.5],[-300,0,959.5],[0,0,1]])
inv=np.linalg.inv(B@np.linalg.inv(G))
sift=cv2.SIFT_create(); bf=cv2.BFMatcher(cv2.NORM_L2)
templates=[]
for spec in load_complete_ground_templates():
    original=cv2.imdecode(np.frombuffer(Path(spec.path).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    for size in [(105,90),(210,180),(420,360)]:
        ref=cv2.resize(original,size,interpolation=cv2.INTER_AREA)
        for flipped in [False,True]:
            image=cv2.flip(ref,1) if flipped else ref
            kp,des=sift.detectAndCompute(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),None)
            templates.append((spec,size,flipped,kp,des))
results=[]
for spec in load_complete_ground_templates():
    raw=Path(__file__).parent/('synthetic_'+Path(spec.path).stem+'.png')
    canvas=cv2.imdecode(np.frombuffer(raw.read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    bev=cv2.warpPerspective(canvas,np.linalg.inv(inv),(960,825))
    kp,des=sift.detectAndCompute(cv2.cvtColor(bev,cv2.COLOR_BGR2GRAY),None)
    rows=[]
    for ref,size,flipped,rkp,rdes in templates:
        pairs=bf.knnMatch(rdes,des,k=2) if rdes is not None and des is not None and len(des)>1 else []
        good=[a for a,b in pairs if a.distance<.75*b.distance]
        src=np.float32([rkp[m.queryIdx].pt for m in good]).reshape(-1,2)
        dst=np.float32([kp[m.trainIdx].pt for m in good]).reshape(-1,2)
        H=mask=None
        if len(good)>=8:
            cv2.setRNGSeed(20261112);H,mask=cv2.findHomography(src,dst,cv2.RANSAC,3.)
        row=verify_geometry(src,dst,mask.ravel().astype(bool) if mask is not None else np.zeros(len(good),bool),H,size)
        # This experiment explicitly tests a different physical board contract.
        reasons=[r for r in row['rejection_reasons'] if r!='plane_area_outside_0_15_to_0_30_m2']
        if row['plane_area_m2'] is None or not .08<=row['plane_area_m2']<=.13:reasons.append('small_board_area')
        rows.append(dict(reference=Path(ref.path).name,size=size,flipped=flipped,good=row['good_matches'],inliers=row['inliers'],distinct=row['query_distinct_xy'],coverage=row['template_hull_fraction'],area=row['plane_area_m2'],reasons=reasons))
    passed=[r for r in rows if not r['reasons']]
    best=sorted(rows,key=lambda r:r['inliers'],reverse=True)[:4]
    results.append(dict(reference=Path(spec.path).name,passed=passed,best=best))
    print(json.dumps(results[-1],ensure_ascii=False),flush=True)
(Path(__file__).parent/'floor_ipm_probe_metrics.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
