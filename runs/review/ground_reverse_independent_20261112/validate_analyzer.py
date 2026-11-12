"""Synthetic mode-adaptation check; output stays outside real v30 evidence."""
import importlib.util
import json
from pathlib import Path
from independent_probe import Transport, ROOT

spec=importlib.util.spec_from_file_location("reverse_analysis",ROOT/"runs/analysis/20261112_reverse_v30/analyze.py")
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
f=Transport().first()
try:
    events=json.loads(json.dumps(f.node.guard.additional_events()))
    parts=[p for e,p in events if p.get("section")==module.api.SECTION]
    core=next(p["data"] for e,p in events if p.get("section")==module.api.CORE)
    body,packed,raw,metadata=module.api.decode_parts(parts)
    assert body["mode"]=="ground_reverse_response"
    assert all(body[k]==v for k,v in core.items())
    metrics,traj,fd=module.api.analyze(body)
    assert metrics["mode"]=="ground_reverse_response" and metrics["pulses"]
    parts[0]["mode"]="ground_direction_response"
    try:
        module.api.decode_parts(parts)
        raise AssertionError("direction mode accepted in reverse analyzer")
    except ValueError:pass
    print("Synthetic reverse bundle/core reconstruction and wrong-mode refusal passed; not platform evidence")
finally:f.close()
