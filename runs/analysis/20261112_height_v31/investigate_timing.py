"""Read-only timing associations; no attribution to unobserved execution stages."""
import hashlib
import json
from pathlib import Path
import zipfile

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[2]
b=json.loads((OUT/'evidence.json').read_text(encoding='utf8'))
fault=b['phase_transitions'][-1]['wall_elapsed_s']
all_rows=[];series={}
for name in ('trajectory','commands','scans','downscans','tf_records'):
    rows=b[name]
    def stamp(row):
        if name=='trajectory':return row['stamp']
        if name=='commands':return row.get('odom_stamp')
        msg=row['message'];header=msg.get('header') or msg.get('transforms',[{}])[0].get('header')
        if header is None:return None
        s=header['stamp'];return s['sec']+s['nanosec']/1e9
    gaps=[{'gap_s':z['wall_elapsed_s']-a['wall_elapsed_s'],
        'before_wall_s':a['wall_elapsed_s'],'after_wall_s':z['wall_elapsed_s'],
        'before_source_stamp':stamp(a),'after_source_stamp':stamp(z)} for a,z in zip(rows,rows[1:])]
    series[name]={'count':len(rows),'largest_gap':max(gaps,key=lambda x:x['gap_s']),
        'near_fault_entries':[{'wall_s':r['wall_elapsed_s'],'source_stamp':stamp(r)} for r in rows if 2.20<=r['wall_elapsed_s']<=2.46]}
    all_rows.extend((r['wall_elapsed_s'],name) for r in rows)
all_rows.sort()
gaps=sorted([{'gap_s':z[0]-a[0],'before':a,'after':z} for a,z in zip(all_rows,all_rows[1:])],key=lambda x:x['gap_s'],reverse=True)
old=json.loads((ROOT/'runs/analysis/20261112_approach_v17/summary.json').read_text(encoding='utf8'))['snapshots']['assembled']['callback_timing']
oldselected={k:{m:old['spans'][k][m] for m in ('wall_s','cpu_s','entry_gap_wall_s')} for k in ('receive:/scan','receive:/odom','receive:/uav/odom','receive:/tf','spin_once','tick')}
source_identity={}
with zipfile.ZipFile(ROOT/'dist/inspection_height_v31_src.zip') as z:
    for suffix in ('bounded_vertical_response.py','bounded_height_hold.py','bounded_ground_probe.py','telemetry.py'):
        key='src/inspection_mission/inspection_mission/'+suffix
        raw=z.read(key)
        source_identity[key]={'sha256':hashlib.sha256(raw).hexdigest(),'archive_matches_current_source':raw==(ROOT/key).read_bytes()}
r={'schema':'height_v31_readonly_timing_investigation_v1','fault_wall_s':fault,
    'source_identity':source_identity,'series':series,'largest_gaps_between_all_retained_entries':gaps[:8],
    'unretained_input_entries':['/clock','/odom'],'v31_does_not_include_callback_timing':not any(k in b for k in ('callback_timing','graph_refresh_diagnostics')),
    'old_v17_timing_is_a_different_run_and_adapter':True,'old_v17_selected_spans':oldselected,
    'limits':['entry timestamps are not callback exit or duration measurements',
        'combined retained-record gap does not prove no unretained callback ran',
        'no source publication wall timestamp, DDS readiness, executor scheduling or host preemption measurements',
        'old v17 timing and local pure-guard cost cannot be substituted for v31 measurements']}
(OUT/'timing_investigation.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps({'largest_combined_gap':gaps[0],
    'largest_series_gaps':{k:v['largest_gap']['gap_s'] for k,v in series.items()},
    'old_v17_max_wall_s':{k:v['wall_s']['max'] for k,v in oldselected.items()},
    'source_archive_matches':all(x['archive_matches_current_source'] for x in source_identity.values())},ensure_ascii=False,indent=2))
