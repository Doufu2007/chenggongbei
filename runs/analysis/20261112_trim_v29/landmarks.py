"""Read-only platform plateaus, request deadlines and support-height dwell audit."""
from pathlib import Path
import json
import math
import statistics

OUT = Path(__file__).resolve().parent
b = json.loads((OUT/"evidence.json").read_text(encoding="utf-8"))
tr, cmds, origin = b["trajectory"], b["commands"], b["origin"]
rows = []
for phase, duration in (("ASCEND_TO_010", None), ("TRIM_POS_025", 1.),
                        ("TRIM_POS_030", 1.), ("TRIM_NEG_010", .3)):
    cc = [x for x in cmds if x["phase"] == phase and x["linear_z"]]
    if not cc: continue
    first, last = cc[0]["wall_elapsed_s"], cc[-1]["wall_elapsed_s"]
    # Use an explicit descriptive interior window, excluding initial response
    # and phase-gate zero gaps. No claim of exact command application timing.
    margin = .06 if phase == "TRIM_NEG_010" else .15
    samples = [x for x in tr if first+margin <= x["wall_elapsed_s"] <= last-.025]
    transition = next(x for x in b["phase_transitions"] if x["to"] == phase)
    deadline = transition["wall_elapsed_s"]+duration if duration else None
    rows.append({"phase": phase, "command": cc[0]["linear_z"], "sample_count": len(samples),
        "first_request_wall_s": first, "last_request_wall_s": last,
        "phase_deadline_s": deadline, "nonzero_after_phase_deadline":
            [x for x in cc if deadline is not None and x["wall_elapsed_s"] >= deadline],
        "interior_start_s": first+margin, "interior_end_s": last-.025,
        "vz_median_mps": statistics.median(x["linear"][2] for x in samples) if samples else None,
        "vz_range_mps": [min(x["linear"][2] for x in samples), max(x["linear"][2] for x in samples)] if samples else None,
        "first_odom": samples[0] if samples else None, "last_odom": samples[-1] if samples else None})
finished = next(x for x in cmds if x["state"] == "FINISHED")
support = [x for x in tr if finished["wall_elapsed_s"]-1. <= x["wall_elapsed_s"] <= finished["wall_elapsed_s"]]
assert support
support_metrics = {"requested_finish_wall_s": finished["wall_elapsed_s"], "sample_count": len(support),
    "source_span_s": support[-1]["stamp"]-support[0]["stamp"],
    "arrival_span_s": support[-1]["wall_elapsed_s"]-support[0]["wall_elapsed_s"],
    "max_height_error_m": max(abs(x["xyz"][2]-origin[2]) for x in support),
    "max_speed_mps": max(x["speed"] for x in support),
    "max_angular_speed_rps": max(x["angular_speed"] for x in support),
    "max_position_distance_from_window_first_m": max(math.dist(x["xyz"], support[0]["xyz"]) for x in support),
    "interpretation": "samples in the last 1s requested window; discrete first-to-last source/arrival spans shown explicitly, not a contact sensor"}
pre_finish = [x for x in tr if x["wall_elapsed_s"] <= finished["wall_elapsed_s"]]
last_invalid = max((i for i, x in enumerate(pre_finish) if abs(x["xyz"][2]-origin[2]) > .005
    or x["speed"] > .005 or x["angular_speed"] > .01), default=-1)
continuous = pre_finish[last_invalid+1:]
support_metrics["continuous_recent_valid_block"] = {"first_wall_s": continuous[0]["wall_elapsed_s"],
    "last_wall_s": continuous[-1]["wall_elapsed_s"],
    "wall_span_s": continuous[-1]["wall_elapsed_s"]-continuous[0]["wall_elapsed_s"],
    "source_span_s": continuous[-1]["stamp"]-continuous[0]["stamp"],
    "position_radius_from_block_first_m": max(math.dist(x["xyz"], continuous[0]["xyz"]) for x in continuous),
    "samples": len(continuous)} if continuous else None
peak = max(tr, key=lambda x:x["xyz"][2])
zero_return = next(x for x in b["phase_transitions"] if x["to"] == "ZERO_RETURN")
within_1mm = next(x for x in tr if x["wall_elapsed_s"] > zero_return["wall_elapsed_s"] and abs(x["xyz"][2]-origin[2]) <= .001)
metrics = {"interior_plateaus": rows, "support_window": support_metrics,
    "peak": peak, "zero_return_entry": zero_return, "first_return_within_1mm": within_1mm,
    "motion_start_to_finished_wall_s": finished["wall_elapsed_s"]-
        (b["motion_started_wall"]-(tr[0]["wall"]-tr[0]["wall_elapsed_s"]))}
(OUT/"landmarks.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
print(json.dumps({"plateaus": [{k:v for k,v in r.items() if k not in ("first_odom", "last_odom")} for r in rows],
    "support_window": support_metrics, "motion_s": metrics["motion_start_to_finished_wall_s"]}, ensure_ascii=False, indent=2))

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
    ts = [x["wall_elapsed_s"] for x in tr]
    axes[0].plot(ts, [x["xyz"][2]-origin[2] for x in tr], label="Measured relative height")
    axes[0].axhline(.1, color="#777777", ls="--", label="0.10m ascent target")
    axes[0].axhline(.15, color="#a94113", ls=":", label="0.15m observed stop boundary")
    axes[0].set_ylabel("Relative height (m)")
    axes[1].step([x["wall_elapsed_s"] for x in cmds], [x["linear_z"] for x in cmds], where="post", label="Actual published Z request")
    axes[1].plot(ts, [x["linear"][2] for x in tr], label="Measured odom vz")
    axes[1].set_xlabel("Process wall time (s)"); axes[1].set_ylabel("Vertical velocity (m/s)")
    for ax in axes:
        ax.legend(loc="upper right", fontsize=8); ax.grid(alpha=.2)
        for t in b["phase_transitions"][1:]: ax.axvline(t["wall_elapsed_s"], color="#777777", lw=.6, alpha=.4)
    axes[0].set_title("v29 finite identification: positive near-zero requests bracket ascent/descent")
    fig.tight_layout(); fig.savefig(OUT/"height_and_requests.png", dpi=180); fig.savefig(OUT/"height_and_requests.svg")
    plt.close(fig)
except ImportError: pass
