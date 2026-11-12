"""Validate strict new-schema decoder and distrust of forged success flags."""
import copy
import importlib.util
import json
from pathlib import Path
from independent_probe import Transport,ROOT

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('height_analysis',ROOT/'runs/analysis/20261112_height_v31/analyze.py')
api=importlib.util.module_from_spec(spec);spec.loader.exec_module(api)

f=Transport();z=.055
try:
    for i in range(600):
        cmd=next((v[2] for t,v in reversed(f.rows) if t=='/uav/cmd_vel'),0.)
        vz=2*(cmd-.02695) if cmd else -.0539
        z=max(.055,z+vz*.05)
        if z==.055:vz=0.
        f.feed(i*.05,z,vz)
        if f.node.guard.state=='FINISHED':break
    events=[{'payload':{'event':event,'data':d}} for event,d in f.node.guard.diagnostic_events()]
    source=HERE/'synthetic_callback.jsonl'
    source.write_text(''.join(json.dumps(x)+'\n' for x in events),encoding='utf8')
    body,validation=api.reconstruct(source,HERE/'synthetic_analysis','synthetic_transport')
    positive=api.analyze(body,HERE/'synthetic_analysis',validation)
    assert positive['independent_full_local_protocol_observed'],positive
    assert positive['independent_hold_3s_source_and_arrival']
    assert not positive['command_measurement_or_formula_errors']
    assert not positive['nonzero_at_or_below_phase_low_cutoff']
    # Keep claimed success untouched, corrupt all actual HOLD observations.
    forged=copy.deepcopy(body)
    lo,hi=positive['hold_interval']
    for r in forged['trajectory']:
        if lo-.05<=r['wall_elapsed_s']<=hi:r['linear'][2]=.011
    # Path remains unchanged, as only measured speed changes.
    bad_dir=HERE/'forged_flags_analysis';bad_dir.mkdir(exist_ok=True)
    bad=api.analyze(forged,bad_dir,validation)
    assert bad['reported']['hold_verified'] and bad['reported']['protocol_complete']
    assert not bad['independent_hold_3s_source_and_arrival']
    assert not bad['independent_full_local_protocol_observed']
    checks={'normal_actual_factory_raw_roundtrip':True,'forged_success_flags_do_not_pass_actual_dwell':True}
    # Wrong mode and omitted fragment are rejected before a positive inference.
    for case,alter in [('wrong_section',lambda es:[e['payload']['data'].update(section='old_vertical_section') for e in es]),
                       ('truncated_fragment',lambda es:es.pop(next(i for i,e in enumerate(es) if e['payload']['data'].get('section')==api.PART)))]:
        es=copy.deepcopy(events);alter(es)
        case_source=HERE/(case+'.jsonl');case_source.write_text(''.join(json.dumps(x)+'\n' for x in es),encoding='utf8')
        try:api.reconstruct(case_source,HERE/case,case)
        except AssertionError:checks[case+'_rejected']=True
        else:raise AssertionError(case+' accepted')
    api.dump(HERE/'analyzer_validation.json',checks)
    print(json.dumps(checks,indent=2))
finally:f.close()
