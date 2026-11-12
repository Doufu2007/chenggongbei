"""Reproduce whether a successful slow ground publish can age a UAV request."""
import json
from pathlib import Path
from independent_probe import Transport

f = Transport().negative()
try:
    g = f.node.guard
    f.now[0] = 3.16
    original = f.node.publisher_by_topic["/cmd_vel"].publish
    def delayed(msg):
        original(msg)
        f.now[0] = 3.51
    f.node.publisher_by_topic["/cmd_vel"].publish = delayed
    before = len(f.rows)
    decision = f.node.tick()
    result = {"phase": g.phase, "phase_started_wall": g.phase_started_wall,
              "phase_end_wall": g.phase_started_wall+.30,
              "before_ground_publish_wall": 3.16, "after_ground_publish_wall": f.now[0],
              "uav_outputs_after_delay": [v for t, v in f.rows[before:] if t == "/uav/cmd_vel"],
              "recorded_command": g.commands[-1], "state_after_tick": g.state,
              "returned_decision": {"linear_z": decision.linear_z, "state": decision.state, "reason": decision.reason}}
    Path(__file__).with_name("delayed_ground_metrics.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
finally:
    f.close()
