"""Register measured direction practice without inventing a task score."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
IDENTITY = "7724ce5c-3e6b-469e-a368-2e6aa7025044"
snapshot = ROOT / "runs/evidence/20261112/direction_v28_list_after_analysis.json"
official = next(r for r in json.loads(snapshot.read_text(encoding="utf-8"))["data"]["list"] if r["id"] == IDENTITY)
assert official["status"] == "failed" and official["score"] is None
record = json.loads((ROOT / "runs/records/vertical_v27_997eb6e5.json").read_text(encoding="utf-8"))
record.update(record_id=IDENTITY, runId=official["runId"], variant="ground-direction-v28",
    status=official["status"], duration_s=float(official["durationMs"])/1000.,
    failure_reason="Platform did not extract a score. This isolated experiment intentionally omitted scoring messages. Three measured XY response pulses and their stationary stops completed; full inspection was not attempted.")
artifacts = json.loads((ROOT / "runs/evidence/20261112/direction_v28_artifacts.json").read_text(encoding="utf-8"))["artifacts"]
for field, artifact in zip(("source", "script"), artifacts):
    record[field] = {"path": artifact["path"], "sha256": artifact["sha256"]}
record["evidence_paths"] = [
    "runs/evidence/20261112/direction_v28_submit.json",
    "runs/evidence/20261112/direction_v28_artifacts.json",
    "runs/evidence/20261112/direction_v28_list_after_analysis.json",
    "runs/evidence/20261112/callback_direction_v28.jsonl",
    "runs/analysis/20261112_direction_v28/validation.json",
    "runs/analysis/20261112_direction_v28/metrics.json",
    "runs/analysis/20261112_direction_v28/evidence.json",
]
import sys
sys.path.insert(0, str(ROOT / "tools"))
from run_records import verify_record_files
verify_record_files(record, ROOT)
destination = ROOT / "runs/records/direction_v28_7724ce5c.json"
with destination.open("x", encoding="utf-8") as handle:
    json.dump(record, handle, indent=2, ensure_ascii=False)
print(json.dumps({"id": IDENTITY, "status": record["status"], "score": record["scores"]["total"],
    "duration_s": record["duration_s"], "files_verified": True}))
