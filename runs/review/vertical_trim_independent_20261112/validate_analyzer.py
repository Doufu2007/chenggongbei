"""Synthetic decoder roundtrip and corruption refusal; no platform evidence."""
import importlib.util
import json
from pathlib import Path
from independent_probe import Transport, ROOT

spec = importlib.util.spec_from_file_location("trim_analysis", ROOT/"runs/analysis/20261112_trim_v29/analyze.py")
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
out = Path(__file__).with_name("synthetic_analyzer_check")
out.mkdir(exist_ok=True)
f = Transport().negative()
try:
    g = f.node.guard
    rows = [{"source": "synthetic_review_fixture_not_platform", "payload": {"event": event, "data": payload}}
        for event, payload in g.diagnostic_events()]
    raw = out/"synthetic_callback.jsonl"
    raw.write_text("".join(json.dumps(row)+"\n" for row in rows), encoding="utf-8")
    body, validation = module.reconstruct(raw, out/"valid", "synthetic-not-platform")
    metrics = module.analyze(body, out/"valid", validation)
    assert metrics["command_counts"]["negative"] > 0
    assert not metrics["nonzero_requests_past_permanent_budget"]
    assert not metrics["nonzero_trim_requests_with_prior_arrival_height_at_or_below_035"]
    original = rows[-1]["payload"]["data"]["compressed_sha256"]
    rows[-1]["payload"]["data"]["compressed_sha256"] = "0"*64
    bad = out/"corrupt_callback.jsonl"
    bad.write_text("".join(json.dumps(row)+"\n" for row in rows), encoding="utf-8")
    try:
        module.reconstruct(bad, out/"corrupt_refused")
        raise RuntimeError("corrupt payload was accepted")
    except AssertionError: pass
    rows[-1]["payload"]["data"]["compressed_sha256"] = original
    rows = rows[:-1]
    missing = out/"missing_callback.jsonl"
    missing.write_text("".join(json.dumps(row)+"\n" for row in rows), encoding="utf-8")
    try:
        module.reconstruct(missing, out/"missing_refused")
        raise RuntimeError("missing payload was accepted")
    except AssertionError: pass
    print("Synthetic roundtrip, corrupted hash refusal and missing payload refusal passed; not platform evidence")
finally: f.close()
