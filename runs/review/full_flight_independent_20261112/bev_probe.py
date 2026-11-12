"""Offline alternatives; no production or frozen matcher changes."""
from pathlib import Path
import sys,json,math
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/inspection_mission'))
import cv2,numpy as np
from inspection_mission.full_flight_inspection import load_flight_references,WINDOW_LABELS
from inspection_mission.vision_core import TemplateMatcher,_normalise

mapping=load_flight_references()
ground={k:v for k,v in mapping.items() if k not in WINDOW_LABELS.values()}
fx=400/math.tan(math.radians(40))
# Patch is 0.8m wide and .8m long, centered 1.44m ahead of the camera;
# this is a geometry-only mapping independent of the template identity.
ppm=600
def project(right,forward):return [400+fx*right/forward,400+fx*1.018/forward]
src=np.float32([project(-.4,1.04),project(.4,1.04),project(.4,1.84),project(-.4,1.84)])
dst=np.float32([[0,480],[480,480],[480,0],[0,0]])
H=cv2.getPerspectiveTransform(src,dst)
refdir=Path(__file__).parent/'derived_reference_experiment';refdir.mkdir(exist_ok=True)
matchers={'original':TemplateMatcher(ground)}
for kind in ('physical_aspect_210x180','physical_aspect_140x120','proportional_300','proportional_150'):
    paths={}
    for label,originals in ground.items():
        paths[label]=[]
        for original in originals:
            im=cv2.imdecode(np.frombuffer(Path(original).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
            if kind.startswith('physical_aspect'):
                size=(210,180) if '210' in kind else (140,120)
            else:
                long=300 if '300' in kind else 150
                scale=long/max(im.shape[:2]);size=(round(im.shape[1]*scale),round(im.shape[0]*scale))
            out=refdir/(kind+'_'+Path(original).stem+'.png')
            out.write_bytes(cv2.imencode('.png',cv2.resize(im,size,interpolation=cv2.INTER_AREA))[1].tobytes())
            paths[label].append(str(out))
    matchers[kind]=TemplateMatcher(paths)
out=[]
for label,originals in ground.items():
    for original in originals:
        name=Path(original).stem
        image=cv2.imdecode(np.frombuffer((Path(__file__).parent/('synthetic_'+name+'.png')).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
        bev=cv2.warpPerspective(image,H,(480,480),borderValue=(0,0,0))
        (Path(__file__).parent/('bev_'+name+'.png')).write_bytes(cv2.imencode('.png',bev)[1].tobytes())
        row={'reference':Path(original).name,'label':label,'matches':{}}
        for kind,matcher in matchers.items():
            variants={}
            for mirror in (False,True):
                for turns in range(4):
                    variant=np.rot90(np.fliplr(bev) if mirror else bev,turns)
                    gray=cv2.cvtColor(variant,cv2.COLOR_BGR2GRAY)
                    found=matcher.detect(_normalise(gray))
                    if found:variants[f'mirror_{mirror}_rot_{turns}']=[{'target_id':r['target_id'],'inliers':r['inlier_count'],'confidence':r['confidence']} for r in found]
            row['matches'][kind]=variants
        out.append(row)
data={'homography_source_to_bev':H.tolist(),'results':out}
(Path(__file__).parent/'bev_metrics.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(data,ensure_ascii=False,indent=2))
