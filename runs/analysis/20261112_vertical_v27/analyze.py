"""Read-only reconstruction and numerical audit of the bounded vertical v27 callback."""
from pathlib import Path
import bisect
import csv
import hashlib
import json
import math
import statistics

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
body = json.loads((OUT / "evidence.json").read_text("utf-8"))
trajectory, commands = body["trajectory"], body["commands"]
wall_origin = trajectory[0]["wall"] - trajectory[0]["wall_elapsed_s"]
motion_elapsed = body["motion_started_wall"] - wall_origin
observe_elapsed = body["observe_started_wall"] - wall_origin
origin = body["origin"]
first_positive_index = next(i for i, x in enumerate(commands) if x["linear_z"] > 0)
last_positive_index = max(i for i, x in enumerate(commands) if x["linear_z"] > 0)
first_positive, last_positive = commands[first_positive_index], commands[last_positive_index]
first_zero = commands[last_positive_index + 1]
peak_index = max(range(len(trajectory)), key=lambda i: trajectory[i]["xyz"][2])
peak = trajectory[peak_index]
start_index = max(i for i, x in enumerate(trajectory) if x["wall_elapsed_s"] <= motion_elapsed)
assert math.dist(trajectory[start_index]["xyz"], origin) < 1e-12


def landmark(x):
    return {k: x[k] for k in ("wall_elapsed_s", "stamp", "xyz", "linear", "speed", "tilt")}


running_path = 0.
running_vertical = 0.
path_after_peak = 0.
path_before_peak = 0.
threshold_sample = None
trajectory_rows = []
command_elapsed = [x["wall_elapsed_s"] for x in commands]
for i, sample in enumerate(trajectory):
    if i > start_index:
        delta = math.dist(trajectory[i - 1]["xyz"], sample["xyz"])
        running_path += delta
        running_vertical += abs(sample["xyz"][2] - trajectory[i - 1]["xyz"][2])
        if i <= peak_index:
            path_before_peak += delta
        else:
            path_after_peak += delta
        if threshold_sample is None and running_path >= .3 - 1e-9:
            threshold_sample = {**landmark(sample), "path_m": running_path}
    command_index = bisect.bisect_right(command_elapsed, sample["wall_elapsed_s"]) - 1
    prior_command = commands[command_index] if command_index >= 0 else None
    row = {"index": i, "arrival_wall_elapsed_s": sample["wall_elapsed_s"],
           "odom_stamp_s": sample["stamp"], "x_m": sample["xyz"][0],
           "y_m": sample["xyz"][1], "z_m": sample["xyz"][2],
           "vx_feedback_mps": sample["linear"][0], "vy_feedback_mps": sample["linear"][1],
           "vz_feedback_mps": sample["linear"][2], "speed_mps": sample["speed"],
           "tilt_rad": sample["tilt"], "path_since_origin_m": running_path,
           "prior_arrival_cmd_z_mps": prior_command["linear_z"] if prior_command else "",
           "prior_arrival_cmd_state": prior_command["state"] if prior_command else ""}
    if i:
        previous = trajectory[i - 1]
        row["finite_difference_vz_by_odom_stamp_mps"] = (sample["xyz"][2] - previous["xyz"][2]) / (sample["stamp"] - previous["stamp"])
    else:
        row["finite_difference_vz_by_odom_stamp_mps"] = ""
    trajectory_rows.append(row)

with (OUT / "trajectory.csv").open("w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=list(trajectory_rows[0]))
    writer.writeheader()
    writer.writerows(trajectory_rows)
with (OUT / "commands.csv").open("w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=list(commands[0]))
    writer.writeheader()
    writer.writerows(commands)

state_runs = []
last = None
for i, command in enumerate(commands):
    key = command["state"], command["linear_z"]
    if key != last:
        state_runs.append({"index": i, **command})
        last = key

ascent_plateau = [x for x in trajectory if first_positive["wall_elapsed_s"] + .3 <= x["wall_elapsed_s"] <= first_zero["wall_elapsed_s"] - .1]
descent_plateau = [x for x in trajectory if first_zero["wall_elapsed_s"] + .1 <= x["wall_elapsed_s"] <= first_zero["wall_elapsed_s"] + 2.]
after_zero = []
arrival_times = [x["wall_elapsed_s"] for x in trajectory]
for offset in (0., .02, .05, .1, .2, .5, 1., 2., 3., 4.):
    index = bisect.bisect_left(arrival_times, first_zero["wall_elapsed_s"] + offset)
    if index < len(trajectory):
        sample = trajectory[index]
        after_zero.append({"requested_after_zero_s": offset,
                           "actual_after_zero_s": sample["wall_elapsed_s"] - first_zero["wall_elapsed_s"],
                           **landmark(sample)})

touchdown = next(x for x in trajectory[peak_index + 1:] if x["xyz"][2] <= origin[2] + .001)
reconstructed_rise = peak["xyz"][2] - origin[2]
reconstructed_drift = max(math.dist(x["xyz"][:2], origin[:2]) for x in trajectory[start_index:])
assert math.isclose(running_path, body["total_path_m"], rel_tol=0., abs_tol=1e-12)
assert math.isclose(reconstructed_rise, body["max_rise_m"], rel_tol=0., abs_tol=1e-12)
assert math.isclose(reconstructed_drift, body["max_horizontal_drift_m"], rel_tol=0., abs_tol=1e-12)
metrics = {
    "mode": "read_only_v27_response_analysis", "platform_id": "997eb6e5-bdda-4abc-88c2-ecb252319b4a",
    "input_sha256": hashlib.sha256((OUT / "evidence.json").read_bytes()).hexdigest(),
    "state": body["state"], "reason": body["reason"], "target_reached": body["target_reached"],
    "official_score": body["official_score"], "full_mission_complete": body["full_mission_complete"],
    "origin": origin, "motion_start_elapsed_s": motion_elapsed, "observe_start_elapsed_s": observe_elapsed,
    "command_counts": {"positive_z": sum(x["linear_z"] > 0 for x in commands), "zero": sum(x["linear_z"] == 0 for x in commands)},
    "first_positive_command": first_positive, "last_positive_command": last_positive,
    "first_zero_after_ascent": first_zero, "command_state_runs": state_runs,
    "max_command_gap_s": max(commands[i]["wall_elapsed_s"] - commands[i - 1]["wall_elapsed_s"] for i in range(1, len(commands))),
    "peak": landmark(peak), "first_sample": landmark(trajectory[0]), "last_sample": landmark(trajectory[-1]),
    "return_within_1mm_of_origin": landmark(touchdown),
    "time_from_first_zero_to_return_within_1mm_s": touchdown["wall_elapsed_s"] - first_zero["wall_elapsed_s"],
    "reconstructed_max_rise_m": reconstructed_rise,
    "reconstructed_max_horizontal_drift_m": reconstructed_drift,
    "reconstructed_total_path_m": running_path,
    "reconstructed_vertical_path_m": running_vertical,
    "path_from_origin_to_peak_m": path_before_peak,
    "path_after_peak_m": path_after_peak,
    "first_total_path_threshold_crossing": threshold_sample,
    "ascent_plateau_vz_median_mps": statistics.median(x["linear"][2] for x in ascent_plateau),
    "ascent_plateau_vz_range_mps": [min(x["linear"][2] for x in ascent_plateau), max(x["linear"][2] for x in ascent_plateau)],
    "descent_plateau_vz_median_mps": statistics.median(x["linear"][2] for x in descent_plateau),
    "descent_plateau_vz_range_mps": [min(x["linear"][2] for x in descent_plateau), max(x["linear"][2] for x in descent_plateau)],
    "maximum_feedback_speed_mps": max(x["speed"] for x in trajectory),
    "maximum_tilt_rad": max(x["tilt"] for x in trajectory),
    "max_odom_stamp_gap_s": max(trajectory[i]["stamp"] - trajectory[i - 1]["stamp"] for i in range(1, len(trajectory))),
    "max_odom_arrival_gap_s": max(trajectory[i]["wall_elapsed_s"] - trajectory[i - 1]["wall_elapsed_s"] for i in range(1, len(trajectory))),
    "samples_after_zero": after_zero,
    "timing_limit": "command timing is process wall arrival; odom has independent simulation stamp; exact actuator application latency is unobserved",
    "verification_scope": "callback file internal integrity and own public observation consistency; not independent cryptographic source authentication or official scoring",
}
(OUT / "metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps({k: v for k, v in metrics.items() if k not in ("samples_after_zero", "command_state_runs")}, ensure_ascii=False, indent=2))

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (height_ax, velocity_ax) = plt.subplots(2, 1, sharex=True, figsize=(9., 6.))
    elapsed = [x["wall_elapsed_s"] for x in trajectory]
    height_ax.plot(elapsed, [x["xyz"][2] for x in trajectory], color="#205493", label="Measured odom z")
    height_ax.axhline(origin[2] + .15, color="#7c4d00", linestyle="--", label="Rise target")
    height_ax.axhline(origin[2], color="#666666", linestyle=":", label="Initial support height")
    height_ax.set_ylabel("Height (m)")
    height_ax.legend(loc="upper right")
    velocity_ax.step(command_elapsed, [x["linear_z"] for x in commands], where="post", color="#205493", label="Requested z velocity")
    velocity_ax.plot(elapsed, [x["linear"][2] for x in trajectory], color="#a94113", label="Odom z velocity")
    velocity_ax.axhline(0., color="#666666", linewidth=.7)
    velocity_ax.set_ylabel("Vertical velocity (m/s)")
    velocity_ax.set_xlabel("Process wall time since experiment start (s)")
    velocity_ax.legend(loc="upper right")
    for ax in (height_ax, velocity_ax):
        ax.axvline(first_zero["wall_elapsed_s"], color="#444444", linestyle="--", linewidth=.8)
        ax.axvline(threshold_sample["wall_elapsed_s"], color="#a94113", linestyle=":", linewidth=.8)
        ax.grid(axis="y", alpha=.2)
    height_ax.set_title("v27: ascent followed by descent during repeated zero commands")
    fig.tight_layout()
    fig.savefig(OUT / "height_and_command.png", dpi=180)
    fig.savefig(OUT / "height_and_command.svg")
    plt.close(fig)
except ImportError:
    (OUT / "plot_unavailable.txt").write_text("matplotlib unavailable\n", encoding="utf-8")
