"""v37 independent narrow review, no live ROS, network or source mutations."""
from pathlib import Path
import hashlib,json,sys
ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'tests'),str(ROOT/'src/inspection_mission')]
import pytest
import test_bounded_cruise_height as height_fx
from test_cruise_cloud_capture import cloud

result={'schema':'v37_independent_probe_v1','production_modified':False,'uploaded':False,'checks':[]}
manifest=json.loads((ROOT/'runs/review/cruise_height_v37_20261112/manifest.json').read_text(encoding='utf-8'))
rows=manifest['files']+manifest['frozen_parents']
for row in rows:
    actual=hashlib.sha256((ROOT/row['path']).read_bytes()).hexdigest()
    assert actual==row['sha256'],row['path']
    if 'expected_sha256' in row:assert actual==row['expected_sha256']
result['checks'].append({'name':'all_new_files_and_six_frozen_parents','passed':True,'hash_count':len(rows)})

for kind in ('cloud_after_stop','delayed_previous_phase_cloud'):
    with pytest.MonkeyPatch.context() as mp:
        node,callbacks,wall,cpu=height_fx.make(mp)
        try:
            g=node.guard
            assert g.phase=='HOLD_CRUISE'
            before={'state':g.state,'phase':g.phase,'phase_started_wall':g.phase_started_wall,
                    'phase_entry_odom_stamp':g.phase_samples[g.phase]['entry_odom_stamp']}
            if kind=='cloud_after_stop':
                wall[0]=1.31;node.stop('independent_review_terminal_capture')
                wall[0]=1.32;source=11.32
            else:
                wall[0]=1.31;source=11.22
            callbacks['/uav/lidar/points'](cloud(stamp=source))
            snapshot=node.cloud_capture.snapshot()
            result['checks'].append({'name':kind,'found':snapshot['frame_count']>0,
                'before':before,'state_when_callback_invoked':g.state,
                'source_stamp_s':source,'arrival_wall_s':wall[0],
                'captured_clouds':snapshot['clouds'],'missing_slots':snapshot['missing_slots']})
        finally:node.close_watchdog()
out=ROOT/'runs/review/cruise_height_v37_20261112/independent_probe_results.json'
out.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False,indent=2))
