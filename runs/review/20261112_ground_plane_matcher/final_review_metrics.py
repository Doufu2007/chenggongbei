"""Reproduce final independent physical stress metrics; no production changes."""
from pathlib import Path
import hashlib
import json
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT / 'tests'), str(ROOT / 'src/inspection_mission')]
from test_ground_plane_matcher import texture
from inspection_mission.ground_plane_matcher import GroundPlaneMatcher, TemplateSpec

cv2.setNumThreads(1)
pattern = texture()
K = np.array([[300., 0, 400.], [0, 300., 100.], [0, 0, 1.]])
v, u = np.indices((800, 800), dtype=np.float64)
den = v - 100
forward = np.full_like(den, 100.)
np.divide(51.9, den, out=forward, where=den > 0)
forward += .390
left = (400-u)*(forward-.390)/300
template_u = ((.25-left)/.5*299).astype(np.float32)
template_v = ((.74-forward)/.44*263).astype(np.float32)
image = cv2.remap(pattern, template_u, template_v, cv2.INTER_LINEAR,
                  borderMode=cv2.BORDER_CONSTANT)
image[den <= 0] = 0
assert np.array_equal(pattern, cv2.imdecode(np.frombuffer(
    (OUT/'horizon_template.png').read_bytes(), np.uint8), cv2.IMREAD_COLOR))
result = GroundPlaneMatcher([TemplateSpec(OUT/'horizon_template.png',
    'synthetic', 'synthetic', {'source':'independent horizon stress'})]).match(image, K)
for name, data in [('horizon_source_final.png',image),
                   ('horizon_bev_final.png',result['bev_bgr'])]:
    ok, encoded = cv2.imencode('.png', data)
    assert ok
    (OUT/name).write_bytes(encoded.tobytes())
report_path = ROOT/'runs/analysis/20261112_ground_plane_matcher_dev/development_check_final/report.json'
report = json.loads(report_path.read_text('utf-8'))
paths = ['src/inspection_mission/inspection_mission/ground_plane_matcher.py',
         'tests/test_ground_plane_matcher.py',
         str(report_path.relative_to(ROOT)).replace('\\','/'),
         'runs/review/20261112_ground_plane_matcher/horizon_red.xml',
         'runs/review/20261112_ground_plane_matcher/independent_final_green.xml',
         'runs/review/20261112_ground_plane_matcher/horizon_source_final.png',
         'runs/review/20261112_ground_plane_matcher/horizon_bev_final.png']
keys = ['class_id','inliers','query_distinct_xy','plane_area_m2',
        'quad_camera_depths_m','source_projection_finite','source_projection_convex']
rows = [{'name':f['name'],'count':len(f['result']['candidates']),
         'candidates':[{k:r[k] for k in keys} for r in f['result']['candidates']],
         'diagnostic_count':len(f['result']['diagnostics'])} for f in report['frames']]
record = {
    'source_sha256':{p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in paths},
    'independent_tests':{'tests':9,'failures':0,'errors':0,'duration_s':2.23},
    'horizon_final':{k:v for k,v in result['diagnostics'][0].items() if k != 'matches'},
    'real_frame_check':rows,
    'class_ground_truth':report['class_ground_truth'], 'accuracy':report['accuracy'],
    'held_out':report['held_out'], 'official_acceptance':report['official_acceptance'],
    'evidence_note':'Initial source/BEV PNGs mentioned in draft were absent; final PNGs are regenerated from the independent regression with checked PNG writes. Old red XML and JSON remain unchanged.'}
(OUT/'independent_final_review.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),'utf-8')
print(json.dumps(record,ensure_ascii=False,indent=2))
