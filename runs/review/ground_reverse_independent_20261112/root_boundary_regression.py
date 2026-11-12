"""Assert the independent delayed-DDS counterexample sends only zero."""
from pathlib import Path
import json
import runpy

api = runpy.run_path(str(Path(__file__).with_name("independent_probe.py")))
result = api["delayed_dds_cross_pulse_deadline"]()
print(json.dumps(result, ensure_ascii=True))
assert result["wall_after_query"] > result["deadline"]
assert result["outputs"] and all(row == (0., 0., 0.) for row in result["outputs"]), "expired reverse request was actually published"
assert result["decision"]["linear_x"] == 0.
assert result["phase"] == "PULSE_SETTLE"
