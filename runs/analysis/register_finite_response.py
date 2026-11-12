"""Register a settled, non-scoring practice and verify its local evidence."""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from run_records import verify_record_files

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("version")
parser.add_argument("identity")
parser.add_argument("--snapshot", required=True)
parser.add_argument("--analysis", required=True)
parser.add_argument("--raw", required=True)
parser.add_argument("--reason", required=True)
args = parser.parse_args()
snapshot = ROOT / args.snapshot
official = next(row for row in json.loads(snapshot.read_text(encoding="utf-8"))["data"]["list"] if row["id"] == args.identity)
if official["status"] != "failed" or official["score"] is not None:
    raise ValueError("Only settled non-scoring diagnostic practices use this helper")
record = json.loads((ROOT / "runs/records/vertical_v27_997eb6e5.json").read_text(encoding="utf-8"))
record.update(record_id=args.identity, runId=official["runId"], variant=args.version,
    status=official["status"], duration_s=float(official["durationMs"])/1000., failure_reason=args.reason)
manifest_path = "runs/evidence/20261112/"+args.version+"_artifacts.json"
artifacts = json.loads((ROOT / manifest_path).read_text(encoding="utf-8"))["artifacts"]
for field, artifact in zip(("source", "script"), artifacts):
    record[field] = {"path": artifact["path"], "sha256": artifact["sha256"]}
record["evidence_paths"] = ["runs/evidence/20261112/"+args.version+"_submit.json", manifest_path,
    args.snapshot, args.raw, args.analysis+"/validation.json", args.analysis+"/metrics.json", args.analysis+"/evidence.json"]
verify_record_files(record, ROOT)
destination = ROOT / ("runs/records/"+args.version+"_"+args.identity[:8]+".json")
with destination.open("x", encoding="utf-8") as handle:
    json.dump(record, handle, ensure_ascii=False, indent=2)
print(json.dumps({"id": args.identity, "status": record["status"], "score": None,
    "duration_s": record["duration_s"], "files_verified": True}))
