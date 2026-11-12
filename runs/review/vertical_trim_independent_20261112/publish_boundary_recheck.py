"""Focused independent verification of the post-ground publish boundary fix."""
import hashlib
import json
from pathlib import Path
from independent_probe import Transport, actual_full_protocol, ROOT


def delayed_case(fault):
    f = Transport().negative()
    try:
        f.now[0] = 3.16
        g = f.node.guard
        phase_deadline = g.phase_started_wall+.3
        original = f.node.publisher_by_topic["/cmd_vel"].publish
        def ground(msg):
            original(msg)
            if fault == "ground_delay":
                f.now[0] = 3.51
            elif fault == "dds_query_delay":
                f.now[0] = 3.17
                def late_query():
                    f.now[0] = 3.51
                    return 1
                f.node.publisher_by_topic["/uav/cmd_vel"].get_subscription_count = late_query
            elif fault == "dds_query_error":
                def failed_query(): raise RuntimeError("independent slow-boundary query failure")
                f.node.publisher_by_topic["/uav/cmd_vel"].get_subscription_count = failed_query
            elif fault == "dds_lost":
                f.node.publisher_by_topic["/uav/cmd_vel"].get_subscription_count = lambda: 0
        f.node.publisher_by_topic["/cmd_vel"].publish = ground
        before = len(f.rows)
        decision = f.node.tick()
        requests = [v for t, v in f.rows[before:] if t == "/uav/cmd_vel"]
        assert requests and all(v == (0.,)*6 for v in requests), (fault, requests)
        assert g.commands[-1]["linear_z"] == 0.
        assert decision.linear_z == 0.
        result = {"fault": fault, "phase_deadline": phase_deadline,
            "after_ground_or_dds_wall": f.now[0], "uav_outputs": requests,
            "state": g.state, "phase": g.phase, "reason": g.reason,
            "last_command": g.commands[-1]}
        # Resume source delivery: an interrupted trim must never re-arm.
        for i in range(72, 81): f.feed(round(i*.05, 9), .155)
        assert all(v == (0.,)*6 for t, v in f.rows[before:] if t == "/uav/cmd_vel")
        return result
    finally: f.close()


results = {"targeted_boundaries": [delayed_case(x) for x in
    ("ground_delay", "dds_query_delay", "dds_query_error", "dds_lost")],
    "normal_actual_path": actual_full_protocol(),
    "sha256": {p.as_posix(): hashlib.sha256((ROOT/p).read_bytes()).hexdigest()
        for p in (Path("src/inspection_mission/inspection_mission/bounded_vertical_trim.py"),
                  Path("scripts/run_bounded_vertical_trim.sh"))}}
Path(__file__).with_name("publish_boundary_recheck_metrics.json").write_text(
    json.dumps(results, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
print(json.dumps(results, ensure_ascii=False, indent=2))
