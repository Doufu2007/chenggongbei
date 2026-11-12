"""Plot measured v28 displacement only, using recovered real callback metrics."""
from pathlib import Path
import json
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
metrics = json.loads((OUT / "metrics.json").read_text("utf-8"))
fig, axes = plt.subplots(1, 3, figsize=(10.5, 3.9))
for ax, pulse in zip(axes, metrics["pulses"]):
    response = pulse["reconstructed_response"]
    dx, dy = [x * 1000 for x in response["delta_world_m"][:2]]
    length = math.hypot(dx, dy)
    expected = pulse["requested_body_interpretation_world_bearing_rad"]
    bx, by = length * math.cos(expected), length * math.sin(expected)
    ax.annotate("", xy=(dx, dy), xytext=(0, 0),
                arrowprops={"arrowstyle": "->", "color": "#205493", "lw": 2})
    ax.plot([0, dx], [0, dy], color="#205493", label="Measured displacement")
    ax.plot([0, bx], [0, by], "--", color="#a94113", label="Body request direction")
    ax.plot(0, 0, "o", color="#444444", ms=3)
    request = pulse["requested_body_xy_mps"]
    ax.set_title(f"Pulse {pulse['index'] + 1}: XY request {request}\n"
                 f"Direction error {math.degrees(pulse['measured_bearing_error_from_body_request_rad']):.2f} deg")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-1, 12)
    ax.set_ylim(-8, 8)
    ax.grid(alpha=.2)
    ax.set_xlabel("World displacement X (mm)")
axes[0].set_ylabel("World displacement Y (mm)")
axes[1].legend(loc="upper center", bbox_to_anchor=(.5, -.23), ncol=2, fontsize=8)
fig.suptitle("v28: measured short responses at yaw around -0.576 rad", fontsize=11)
fig.text(.5, .01, "Dashed direction is scaled to measured length; no actuator model or full-task score is inferred.",
         ha="center", fontsize=8)
fig.tight_layout(rect=(0, .07, 1, .92))
fig.savefig(OUT / "response_vectors.png", dpi=180)
fig.savefig(OUT / "response_vectors.svg")
plt.close(fig)
