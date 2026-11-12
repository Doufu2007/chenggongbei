"""Independently recompute public callback geometry, without ROS or network."""
from pathlib import Path
import hashlib
import json
import math

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
raw = ROOT / "runs/evidence/20261112/callback_three_point_v26.jsonl"
summary = json.loads((OUT / "decoded/summary.json").read_text(encoding="utf-8"))
bundle = json.loads((OUT / "decoded/bundle.json").read_text(encoding="utf-8"))
survey = bundle["survey"]
approach = survey["approach"]
origin = approach["drive_origin"]
bearing = approach["drive_yaw_rad"]


def cross(row):
    dx, dy = (row["position"][i] - origin[i] for i in (0, 1))
    return -dx * math.sin(bearing) + dy * math.cos(bearing)


rows = survey["trajectory_10hz"]
commands = survey["actual_commands"]
first_over = next(row for row in rows if cross(row) > .02)
before = rows[rows.index(first_over)-1]
stop = next(command for command in commands if command["state"] == "STOP")
last_nonzero = next(command for command in reversed(commands) if command["linear_x"] or command["angular_z"])
operation = approach["three_point"]["operation_records"][1]
target = operation["operation"]["target_xy_m"]
listing = json.loads((ROOT / "runs/evidence/20261112/three_point_v26_list_0704.json").read_text(encoding="utf-8-sig"))
official = next(row for row in listing["data"]["list"] if row["id"] == "0425d992-ddd4-4b91-9d21-b3d6a394728e")
events = [json.loads(line)["payload"] for line in raw.read_text(encoding="utf-8").splitlines()]

assert summary["transport_complete"] and summary["errors"] == []
assert official["status"] == "failed" and official["score"] is None
assert tuple(operation["actual_start_pose"][:2]) == tuple(origin[:2])
assert abs(math.atan2(target[1]-origin[1], target[0]-origin[0]) - bearing) < 1e-15
assert cross(before) < .02 < cross(first_over)
assert last_nonzero["wall_elapsed_s"] < first_over["wall_elapsed_s"] < stop["wall_elapsed_s"]
assert all(command["linear_x"] == command["angular_z"] == 0. for command in commands if command["wall_elapsed_s"] >= stop["wall_elapsed_s"])
assert abs(approach["cross_track_m"] - cross(rows[-1])) < 1e-8
pre_stop = [row for row in rows if 16.34 <= row["stamp_s"] <= first_over["stamp_s"]]
assert all(cross(second) > cross(first) for first, second in zip(pre_stop, pre_stop[1:]))

metrics = {
    "source": "本队公开回调；平台状态来自本队提交列表缓存",
    "platform": {key: official[key] for key in ("id", "status", "score", "runId", "durationMs", "submissionNo", "updatedAt")},
    "callback_events": len(events), "callback_bytes": raw.stat().st_size,
    "raw_sha256": hashlib.sha256(raw.read_bytes()).hexdigest(),
    "transport_complete": summary["transport_complete"],
    "launcher_exit_code": next(event["data"]["exit_code"] for event in events if event["event"] == "launcher_exit"),
    "trajectory_count": len(rows), "command_count": len(commands),
    "trajectory_dropped": survey["trajectory_dropped"], "commands_dropped": survey["actual_commands_dropped"],
    "completed_points_observed": len(bundle["completed_points"]),
    "capture_count": approach["capture_count"], "capture_image_events": len(bundle["capture_image_events"]),
    "draw": approach["three_point"]["mission_draw"], "reason": approach["reason"],
    "gate_m": .02, "snapshot_cross_track_m": approach["cross_track_m"],
    "last_trajectory_cross_track_m": cross(rows[-1]),
    "previous_sample": {"stamp_s": before["stamp_s"], "wall_elapsed_s": before["wall_elapsed_s"], "cross_track_m": cross(before)},
    "first_sample_over_gate": {"stamp_s": first_over["stamp_s"], "wall_elapsed_s": first_over["wall_elapsed_s"], "cross_track_m": cross(first_over)},
    "last_nonzero_command": last_nonzero, "first_zero_stop_command": stop,
    "cross_track_trajectory": [{"stamp_s": row["stamp_s"], "wall_elapsed_s": row["wall_elapsed_s"], "cross_track_m": cross(row)} for row in rows if 13.4 <= row["stamp_s"] <= 17.36],
    "limits": "10Hz抽样不能确定样本间精确越线时刻；字节完整不等于官方接受；不推断本局未公开的底层驱动原因。",
}
(OUT / "independent_metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")
print(json.dumps({key: metrics[key] for key in ("transport_complete", "reason", "snapshot_cross_track_m", "completed_points_observed")}, ensure_ascii=False))
