"""Plot received survey evidence; the chart is not a platform screenshot."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("response", type=Path)
    args = parser.parse_args()
    result = json.loads(args.response.read_text(encoding="utf-8"))
    source = Path(result["source_summary"]["path"])
    raw = source.read_bytes()
    if hashlib.sha256(raw).hexdigest() != result["source_summary"]["sha256"]:
        raise ValueError("Source summary changed since response analysis")
    summary = json.loads(raw)
    commands = summary["series"]["actual_commands"]["records"]
    poses = summary["series"]["trajectory_10hz"]["records"]
    intervals = [row for row in result["intervals"] if row["retained_turn_interval"]]
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, constrained_layout=True)
    axes[0].step([row["wall_elapsed_s"] for row in commands],
                 [abs(row["angular_z"]) for row in commands], where="post", color="#264653")
    axes[0].set(ylabel="Requested |yaw rate|\n(rad/s)", ylim=(-.02, .55))
    axes[1].plot([row["wall_elapsed_s"] for row in poses],
                 [row["angular_speed"] for row in poses], color="#a8a29e", lw=1,
                 label="Received odometry twist norm")
    axes[1].scatter([row["end_wall_s"] for row in intervals],
                   [abs(row["pose_rate_sim_rad_s"]) for row in intervals], s=7,
                   color="#2a6f97", label="Pose difference / header seconds (TURN only)")
    axes[1].set(ylabel="Observed |yaw rate|\n(rad/s)", ylim=(-.005, .15))
    axes[1].legend(loc="upper right", fontsize=8)
    axes[2].plot([row["wall_elapsed_s"] for row in poses],
                 [math.degrees(row["yaw_rad"]) for row in poses], color="#2a6f97", label="Received yaw")
    axes[2].axhline(math.degrees(summary["survey"]["target_yaw"]), ls="--", color="#9c6644",
                       label="Candidate observation direction (unverified mapping)")
    axes[2].set(ylabel="Yaw (degrees)", xlabel="Local receipt wall time (seconds)")
    axes[2].legend(loc="lower left", fontsize=8)
    for ax in axes:
        for window in result["windows"]:
            if window["state"] == "PAUSE":
                ax.axvspan(window["start_wall_s"], window["end_wall_s"], color="#f4d35e", alpha=.25)
        ax.grid(axis="y", alpha=.2)
        ax.spines[["top", "right"]].set_visible(False)
    fig.suptitle("v12 received commands and odometry: descriptive association only", fontsize=13)
    fig.supxlabel("Yellow = PAUSE. Command age and header/clock offsets remain in response.json.\n"
                  "Publish calls do not establish actuator acceptance or causal gain. No official score.", fontsize=9)
    out = args.response.parent / "observed_response.png"
    fig.savefig(out, dpi=160)
    fig.savefig(out.with_suffix(".svg"))
    plt.close(fig)
    print(out)


if __name__ == "__main__":
    main()
