"""Same exact transported JPEGs, original priors, stable-K hypothesis; no ROS."""
from pathlib import Path
import hashlib
import json
import sys

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/inspection_mission'))
from inspection_mission.ground_template_catalog import load_complete_ground_templates
from inspection_mission.ground_plane_matcher import GroundPlaneMatcher
from inspection_mission.ground_single_point import bind_ground_candidate


def main():
    source=ROOT/'runs/analysis/20261112_three_point_recovery_v38'
    output=Path(__file__).resolve().parent
    previous=json.loads((source/'visual_replay.json').read_text(encoding='utf-8'))
    K=np.array(previous['K']);specs=load_complete_ground_templates()
    cv2.setNumThreads(1);matcher=GroundPlaneMatcher(specs);frames=[]
    for old in previous['replays']:
        index=old['index'];path=source/f'point_0_capture_{index}.jpg'
        image=cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv2.IMREAD_COLOR)
        result=matcher.match(image,K);bev=result.pop('bev_bgr')
        bound=bind_ground_candidate({'job':{'pose':old['pose']},'matcher':result},'ground_3')
        old_rows={Path(row['template_path']).name:row for row in old['candidates']}
        stable_keys=('good_matches','inliers','query_distinct_xy','template_distinct_xy','rejection_reasons')
        unchanged=all(all(row[key]==old_rows[Path(row['template_path']).name][key] for key in stable_keys)
            for row in result['diagnostics'] if Path(row['template_path']).name in old_rows)
        overlay=image.copy()
        for row in result['candidates']:
            quad=np.rint(row['source_quad_xy']).astype(np.int32)
            cv2.polylines(overlay,[quad],True,(0,255,0),2)
            cv2.putText(overlay,Path(row['template_path']).name,tuple(quad[0]),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,0),1)
        cv2.imwrite(str(output/f'capture_{index}_ten_template_overlay.png'),overlay)
        cv2.imwrite(str(output/f'capture_{index}_bev.png'),bev)
        frames.append({'index':index,'source_image_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
            'pose':old['pose'],'exposure_stamp_s':old['exposure_stamp_s'],'bound':bound,
            'original_eight_diagnostics_unchanged':unchanged,'result':result})
        print(json.dumps({'frame':index,'accepted_templates':[Path(c['template_path']).name for c in result['candidates']],
            'binding_present':bound is not None,'original_eight_unchanged':unchanged}),flush=True)
    report={'scope':previous['scope'],'original_geometry_unchanged':True,'template_count':len(specs),
        'matcher_sha256':hashlib.sha256((ROOT/'src/inspection_mission/inspection_mission/ground_plane_matcher.py').read_bytes()).hexdigest(),
        'templates':[{'name':Path(s.path).name,'class_annotation':s.class_id,'sha256':s.expected_sha256,'provenance':s.provenance} for s in specs],
        'frames':frames}
    (output/'ten_template_replay.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':main()
