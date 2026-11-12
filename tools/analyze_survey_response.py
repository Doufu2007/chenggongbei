"""Summarize received survey commands and odometry without causal attribution.

Wall time matches local observations. Simulated header intervals compute pose
rates. Command /clock is separately retained because its 10 Hz phase and sensor
delivery mean a wall-time pair is not an actuator response measurement.
"""
from __future__ import annotations

import argparse
from bisect import bisect_right
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def stats(values):
    values = list(values)
    return None if not values else {"n": len(values), "min": min(values),
        "median": statistics.median(values), "max": max(values)}


def summarize(summary):
    commands = summary["series"]["actual_commands"]["records"]
    trajectory = summary["series"]["trajectory_10hz"]["records"]
    for name, rows in (("commands", commands), ("trajectory", trajectory)):
        if not rows or any(not isinstance(row["wall_elapsed_s"], (int, float))
                or not math.isfinite(row["wall_elapsed_s"]) for row in rows):
            raise ValueError(name + " needs finite wall time")
        if any(b["wall_elapsed_s"] < a["wall_elapsed_s"] for a, b in zip(rows, rows[1:])):
            raise ValueError(name + " wall time reversed")
    command_times = [row["wall_elapsed_s"] for row in commands]
    target = summary["survey"]["target_yaw"]
    windows = []
    start = 0
    for end in range(1, len(commands) + 1):
        if end < len(commands) and commands[end]["state"] == commands[start]["state"]:
            continue
        rows = commands[start:end]
        begin = rows[0]["wall_elapsed_s"]
        finish = commands[end]["wall_elapsed_s"] if end < len(commands) else max(
            commands[-1]["wall_elapsed_s"], trajectory[-1]["wall_elapsed_s"])
        poses = [row for row in trajectory if begin <= row["wall_elapsed_s"] < finish]
        changes = [wrap(b["yaw_rad"] - a["yaw_rad"]) for a, b in zip(poses, poses[1:])]
        window = {"state": rows[0]["state"], "start_wall_s": begin, "end_wall_s": finish,
            "duration_wall_s": finish - begin, "command_rows": len(rows),
            "command_abs_rad_s": stats(abs(row["angular_z"]) for row in rows),
            "nonzero_rows": sum(bool(row["linear_x"] or row["angular_z"]) for row in rows),
            "pose_rows": len(poses), "observed_signed_delta_rad": sum(changes),
            "observed_absolute_delta_rad": sum(abs(value) for value in changes)}
        if poses:
            window.update(first_pose=poses[0], last_pose=poses[-1],
                first_pose_after_window_start_s=poses[0]["wall_elapsed_s"] - begin,
                window_end_after_last_pose_s=finish - poses[-1]["wall_elapsed_s"],
                pose_angular_speed_abs_rad_s=stats(row["angular_speed"] for row in poses),
                last_error_rad=wrap(target - poses[-1]["yaw_rad"]))
        windows.append(window)
        start = end

    intervals = []
    for left, right in zip(trajectory, trajectory[1:]):
        begin, finish = left["wall_elapsed_s"], right["wall_elapsed_s"]
        dt_wall, dt_sim = finish - begin, right["stamp_s"] - left["stamp_s"]
        index = bisect_right(command_times, finish) - 1
        if index < 0:
            continue
        command = commands[index]
        first_index = max(0, bisect_right(command_times, begin) - 1)
        interval_commands = commands[first_index:index + 1]
        states = sorted({row["state"] for row in interval_commands})
        angle = wrap(right["yaw_rad"] - left["yaw_rad"])
        age = finish - command["wall_elapsed_s"]
        retained = (states == ["TURN"] and dt_sim > 0 and 0 < dt_wall <= .35
                    and 0 <= age <= .1)
        intervals.append({"start_wall_s": begin, "end_wall_s": finish,
            "start_stamp_s": left["stamp_s"], "end_stamp_s": right["stamp_s"],
            "dt_wall_s": dt_wall, "dt_sim_s": dt_sim, "delta_yaw_rad": angle,
            "pose_rate_sim_rad_s": None if dt_sim <= 0 else angle / dt_sim,
            "pose_rate_wall_rad_s": None if dt_wall <= 0 else angle / dt_wall,
            "right_odom_angular_speed_abs_rad_s": right["angular_speed"],
            "right_error_rad": wrap(target - right["yaw_rad"]),
            "prior_command_index": index, "prior_command": command,
            "right_pose_wall_minus_command_wall_s": age,
            "right_pose_stamp_minus_command_clock_s": None if command["clock_stamp_s"] is None
                else right["stamp_s"] - command["clock_stamp_s"],
            "overlapping_command_states": states,
            "overlapping_command_abs_min": min(abs(row["angular_z"]) for row in interval_commands),
            "overlapping_command_abs_max": max(abs(row["angular_z"]) for row in interval_commands),
            "retained_turn_interval": retained})
    bins = defaultdict(list)
    for row in intervals:
        if not row["retained_turn_interval"]:
            continue
        magnitude = abs(row["prior_command"]["angular_z"])
        label = ("[.45,.5]" if magnitude >= .45 else "[.35,.45)" if magnitude >= .35
                 else "[.30,.35)" if magnitude >= .30 else "[.25,.30)" if magnitude >= .25 else "[0,.25)")
        bins[label].append(row)
    summaries = {}
    for label, rows in bins.items():
        summaries[label] = {"n": len(rows), "wall_start_s": rows[0]["start_wall_s"],
            "wall_end_s": rows[-1]["end_wall_s"],
            "command_abs_rad_s": stats(abs(row["prior_command"]["angular_z"]) for row in rows),
            "pose_rate_sim_abs_rad_s": stats(abs(row["pose_rate_sim_rad_s"]) for row in rows),
            "odom_twist_angular_abs_rad_s": stats(row["right_odom_angular_speed_abs_rad_s"] for row in rows),
            "wall_match_age_s": stats(row["right_pose_wall_minus_command_wall_s"] for row in rows),
            "pose_stamp_minus_command_clock_s": stats(row["right_pose_stamp_minus_command_clock_s"]
                for row in rows if row["right_pose_stamp_minus_command_clock_s"] is not None),
            "observed_delta_rad": sum(row["delta_yaw_rad"] for row in rows),
            "observed_sim_duration_s": sum(row["dt_sim_s"] for row in rows)}
    turning = [row for row in intervals if row["retained_turn_interval"]]
    last_turn_end = turning[-1]["end_wall_s"] if turning else None
    last_three = [] if last_turn_end is None else [
        row for row in turning if row["end_wall_s"] >= last_turn_end - 3.]
    final_error = abs(wrap(target - trajectory[-1]["yaw_rad"]))
    rate = statistics.median(abs(row["pose_rate_sim_rad_s"]) for row in last_three) if last_three else None
    return {"kind": "survey_observed_response_only", "causal_gain": "unverified",
        "score": None, "task_completion": "unverified", "windows": windows,
        "intervals": intervals, "command_bins": summaries,
        "end_error_rad": final_error, "last_three_turn_seconds": {
            "selection_basis": "last three local wall seconds of retained TURN intervals",
            "n": len(last_three), "median_abs_pose_rate_sim_rad_s": rate,
            "median_abs_pose_rate_wall_rad_s": statistics.median(
                abs(row["pose_rate_wall_rad_s"]) for row in last_three) if last_three else None,
            "wall_match_age_s": stats(row["right_pose_wall_minus_command_wall_s"] for row in last_three),
            "pose_stamp_minus_command_clock_s": stats(row["right_pose_stamp_minus_command_clock_s"]
                for row in last_three if row["right_pose_stamp_minus_command_clock_s"] is not None)},
        "constant_observed_rate_scenario": {"assumed_rate_rad_s": rate,
            "rate_denominator": "odometry header simulated seconds",
            "additional_sim_seconds_to_error_tolerance_025": max(0., final_error - .025) / rate if rate else None,
            "angle_over_extra_20_sim_seconds_rad": 20. * rate if rate is not None else None,
            "interpretation": "Arithmetic scenario in simulated seconds only; holds late observed rate constant. Not a wall-time forecast or actuator model."},
        "notes": ["Latest prior command uses local wall receipt order; no future command is paired.",
            "Pose rate uses consecutive received odometry headers, not command /clock time.",
            "Retained TURN intervals exclude state transitions, gaps over .35s and command age over .1s.",
            "All interval offsets remain in the output. Timing, queued feedback, transients and changing command values prevent causal gain claims.",
            "Twist angular_speed is a nonnegative norm; signed direction is available only from pose differences."]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("summary", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    raw = args.summary.read_bytes()
    result = summarize(json.loads(raw))
    result["source_summary"] = {"path": str(args.summary), "sha256": hashlib.sha256(raw).hexdigest()}
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "response.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"windows": len(result["windows"]), "intervals": len(result["intervals"]),
        "output": str(args.output / "response.json")}, ensure_ascii=False))


if __name__ == "__main__":
    main()
