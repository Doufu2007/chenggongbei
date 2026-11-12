"""Freeze final local-review artifacts; no network or control actions."""
from pathlib import Path
import hashlib
import json
import py_compile

ROOT=Path(__file__).resolve().parents[3]
MODULES=[
    'src/inspection_mission/inspection_mission/full_inspection_runtime.py',
    'src/inspection_mission/inspection_mission/ground_match_result_recorder.py',
    'src/inspection_mission/inspection_mission/ground_template_catalog.py',
    'src/inspection_mission/inspection_mission/ground_three_point_return.py',
    'src/inspection_mission/inspection_mission/floor_plane_detector.py',
    'tests/test_ground_match_result_recorder.py',
    'tests/test_full_inspection_runtime.py',
    'tests/test_ground_template_catalog.py',
    'tests/test_floor_plane_detector.py',
    'runs/analysis/20261112_v38_visual_root_cause/replay_ten_templates.py',
    'runs/analysis/20261112_v38_visual_root_cause/independent_integration_review.py',
]
for relative in MODULES:py_compile.compile(str(ROOT/relative),doraise=True)
HERE=Path(__file__).parent
(HERE/'py_compile_final.txt').write_text('PASS: '+str(len(MODULES))+' named Python sources compiled.\n',encoding='utf-8')
ARTIFACTS=MODULES+[
    'docs/v38_visual_root_cause_20261112.md',
    'docs/full_inspection_integration_independent_review_20261112.md',
    'docs/floor_plane_detector_20261112.md',
    'runs/analysis/20261112_v38_visual_root_cause/ten_template_replay.json',
    'runs/analysis/20261112_v38_visual_root_cause/root_cause_summary.json',
    'runs/analysis/20261112_v38_visual_root_cause/integration_initial_review.txt',
    'runs/analysis/20261112_v38_visual_root_cause/integration_recorder_green.txt',
    'runs/analysis/20261112_v38_visual_root_cause/integration_latest_heartbeat_review.txt',
    'runs/analysis/20261112_v38_visual_root_cause/py_compile_final.txt',
    'runs/review/full_flight_independent_20261112/floor_detector_red.txt',
    'runs/review/full_flight_independent_20261112/floor_detector_first_green.txt',
    'runs/review/full_flight_independent_20261112/floor_detector_final_green.txt',
    'runs/review/full_flight_independent_20261112/floor_detector_single150_green.txt',
    'runs/review/full_flight_independent_20261112/floor_detector_fast150_fallback300_green.txt',
    'runs/review/full_flight_independent_20261112/floor_performance_probe.py',
    'runs/review/full_flight_independent_20261112/floor_performance_before.json',
    'runs/review/full_flight_independent_20261112/floor_performance_single150.json',
    'runs/review/full_flight_independent_20261112/floor_performance_fast150_fallback300.json',
]
records=[]
for relative in ARTIFACTS:
    path=ROOT/relative;raw=path.read_bytes()
    records.append(dict(path=relative,size_bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest()))
manifest=dict(schema='local_review.v1',date='2026-11-12',artifacts=records,
    checks={'integration_latest':{'passed':19,'duration_s':1.76},
            'floor_author_tests':{'passed':14,'duration_s':16.21},
            'py_compile':{'passed':len(MODULES)}},
    visual_replay={'real_jpegs':3,'accepted':3,'original_24_diagnostics_identical':True,
                   'threshold_changes':False},
    limitations=['Three related static real JPEG exposures; stable-K assumption, not per-exposure K.',
                 'Floor detector tests are public-geometry synthetic inputs, not platform validation.',
                 'Recorder authored by height agent; parent root performed non-author review.',
                 'Flight-control full independent review is outside this integration review.',
                 'Public candidate camera extrinsics and small-board dimensions are not measured calibration.'])
(HERE/'final_review_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({'compiled':len(MODULES),'artifacts':len(records),
                  'manifest':str(HERE/'final_review_manifest.json')},ensure_ascii=False))
