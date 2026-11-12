"""Pure-rotation UV controls for original production ORB, offline only."""
from pathlib import Path
import sys,json,math
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/inspection_mission'))
import cv2
import numpy as np
from inspection_mission.ground_template_catalog import load_complete_ground_templates
from inspection_mission.vision_core import TemplateMatcher,_normalise
mapping={}
specs=load_complete_ground_templates()
for spec in specs:mapping.setdefault(spec.label,[]).append(str(spec.path))
matcher=TemplateMatcher(mapping)
fx=400/math.tan(math.radians(40))
world=np.float64([[3.08-.175,-.32+.15],[3.08+.175,-.32+.15],[3.08+.175,-.32-.15],[3.08-.175,-.32-.15]])
corners=np.column_stack((400+fx*(world[:,0]-3.08)/(world[:,1]+2.15-.39),400+fx*1.018/(world[:,1]+2.15-.39))).astype(np.float32)
rows=[]
for spec in specs:
    image=cv2.imdecode(np.frombuffer(Path(spec.path).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    for angle in range(4):
        rotated=np.rot90(image,angle).copy();h,w=rotated.shape[:2]
        H=cv2.getPerspectiveTransform(np.float32([[0,0],[w-1,0],[w-1,h-1],[0,h-1]]),corners)
        canvas=cv2.warpPerspective(rotated,H,(800,800),borderValue=(170,170,170))
        gray=cv2.cvtColor(canvas,cv2.COLOR_BGR2GRAY)
        raw=matcher.detect(gray);norm=matcher.detect(_normalise(gray))
        rows.append(dict(reference=Path(spec.path).name,rotation=angle*90,raw=[dict(label=r['target_id'],inliers=r['inlier_count']) for r in raw],normalised=[dict(label=r['target_id'],inliers=r['inlier_count']) for r in norm]))
print(json.dumps(rows,ensure_ascii=False,indent=2))
(Path(__file__).parent/'floor_uv_control_metrics.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')
