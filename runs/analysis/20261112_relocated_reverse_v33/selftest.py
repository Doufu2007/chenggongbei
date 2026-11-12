"""In-memory synthetic decoder self-check only; no platform files/results."""
import base64
from copy import deepcopy
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path

spec=importlib.util.spec_from_file_location("v33_readonly_analysis",Path(__file__).with_name("analyze.py"))
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)


def fixture():
    core={"mode":a.MODE,"session_id":"synthetic_decoder_selfcheck_not_platform",
          "turn_records":[],"pulse_records":[],"relocated_contract":{"revision":a.BUNDLE}}
    body={**core,"schema":a.SCHEMA,"raw_odometry":[],"raw_scans":[],"commands":[],"state_records":[],"zero_requests":[]}
    raw=json.dumps(body,separators=(",",":"),allow_nan=False).encode()
    packed=gzip.compress(raw,mtime=0);encoded=base64.b64encode(packed).decode()
    mid=(len(encoded)//8)*4
    chunks=[encoded[:mid],encoded[mid:]]
    meta={"mode":a.MODE,"session_id":core["session_id"],"section":a.SECTION,"bundle":a.BUNDLE,
          "encoding":"gzip+base64","uncompressed_bytes":len(raw),"compressed_bytes":len(packed),
          "uncompressed_sha256":hashlib.sha256(raw).hexdigest(),"compressed_sha256":hashlib.sha256(packed).hexdigest(),"part_count":2}
    parts=[{**meta,"part_index":i,"base64_part":chunk} for i,chunk in enumerate(chunks)]
    return body,parts,[core]


def rejects(change):
    body,parts,cores=fixture()
    change(body,parts,cores)
    try:a.decode_parts(parts,cores)
    except (ValueError,KeyError):return
    raise AssertionError("corrupt synthetic decoder case unexpectedly accepted")


body,parts,cores=fixture()
actual,packed,raw,meta=a.decode_parts(list(reversed(parts)),cores)
assert actual==body and meta["part_count"]==2
actual,_,_,meta=a.decode_parts(parts+[deepcopy(parts[0])],cores)
assert actual==body and meta["duplicate_parts"]==1
rejects(lambda b,p,c:p.pop())
rejects(lambda b,p,c:p[0].update(base64_part="AAAA"))
rejects(lambda b,p,c:p[1].update(session_id="other_session"))
rejects(lambda b,p,c:[q.update(section="ground_direction_evidence_gzip_part") for q in p])
rejects(lambda b,p,c:[q.update(bundle="ground_direction_response_v1") for q in p])
rejects(lambda b,p,c:c[0].update(unexpected_extra_key=1))
rejects(lambda b,p,c:c[0].pop("turn_records"))
rejects(lambda b,p,c:c.clear())
rejects(lambda b,p,c:[q.update(compressed_sha256="0"*64) for q in p])
rejects(lambda b,p,c:[q.update(uncompressed_bytes=q["uncompressed_bytes"]+1) for q in p])
rejects(lambda b,p,c:p.append({**p[0],"base64_part":"AAAA"}))
for invalid in ('{"x":1,"x":2}','{"x":NaN}','{"x":1e999}'):
    try:a.strict_json(invalid)
    except ValueError:pass
    else:raise AssertionError("non-strict JSON accepted")
print("16 in-memory decoder integrity checks passed; no platform results created")
