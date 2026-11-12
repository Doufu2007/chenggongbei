"""Offline hypothesis replay; first/last K equality is not per-exposure K proof."""
import json
import math
from pathlib import Path
import sys
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/inspection_mission'))
from inspection_mission.ground_single_point import load_ground_templates, bind_ground_candidate
from inspection_mission.ground_plane_matcher import GroundPlaneMatcher
from inspection_mission.ground_observation_route_plan import _POINTS

folder = Path(__file__).resolve().parent
bundle = json.loads((folder/'bundle.json').read_text(encoding='utf-8'))
rows = [json.loads(line)['payload'] for line in (ROOT/'runs/evidence/20261112/callback_three_point_recovery_v38.jsonl').read_text(encoding='utf-8').splitlines()]
info = next(r['data']['value'] for r in rows if r.get('data',{}).get('section') == 'public_messages' and r['data'].get('topic') == '/uav/camera/camera_info')
assert info['first']['k'] == info['last']['k']
K = np.array(info['first']['k']).reshape(3,3)
matcher = GroundPlaneMatcher(load_ground_templates())
replays=[]
for index in range(3):
    record=next(r for r in bundle['survey']['exposure_records'] if r['phase']==f'capture_{index}' and r['topic']=='/uav/camera/image')
    image=cv2.imdecode(np.frombuffer((folder/f'point_0_capture_{index}.jpg').read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    matches=matcher.match(image,K)
    result={'job':{'pose':record['odom']},'matcher':matches}
    bound=bind_ground_candidate(result,'ground_3')
    pose=record['odom']; x,y,_=pose['position']; q=pose['quaternion']
    yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
    diagnostics=[]
    for candidate in matches['diagnostics']:
        center=candidate.get('body_plane_center_forward_left_m')
        world=None if center is None else [x+math.cos(yaw)*center[0]-math.sin(yaw)*center[1],y+math.sin(yaw)*center[0]+math.cos(yaw)*center[1]]
        diagnostics.append({k:v for k,v in candidate.items() if k not in ('matches','provenance','homography_template_to_bev')})
        diagnostics[-1].update(world_center_xy_m=world,target_distance_m=None if world is None else math.dist(world,_POINTS['ground_3'][0]))
    replays.append({'index':index,'exposure_stamp_s':record['stamp_s'],'pose':pose,'bound':bound,'candidates':diagnostics})
output={'scope':'Offline replay from exact transported capture JPEG and original prior pose; K inferred stable from equal first/last CameraInfo, not exact runtime worker reproduction','K':K.tolist(),'replays':replays}
(folder/'visual_replay.json').write_text(json.dumps(output,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps([{'index':r['index'],'bound':r['bound'],'candidates':[{'class':c['class_id'],'accepted':c['accepted_candidate'],'reasons':c['rejection_reasons'],'distance':c['target_distance_m']} for c in r['candidates']]} for r in replays],ensure_ascii=True))
