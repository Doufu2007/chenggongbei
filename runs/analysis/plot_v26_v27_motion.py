"""Plot two actual diagnostic runs; these are not full-mission results."""
from pathlib import Path
import json
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "runs/analysis/20261112_motion_evidence"
OUT.mkdir(exist_ok=True)
font = Path("C:/Windows/Fonts/msyh.ttc")
if font.exists():
    plt.rcParams["font.family"] = FontProperties(fname=str(font)).get_name()
plt.rcParams.update({"axes.unicode_minus": False, "font.size": 10,
                     "axes.spines.top": False, "axes.spines.right": False})
ground = json.loads((ROOT / "runs/analysis/20261112_three_point_v26/bundle.json").read_text(encoding="utf-8"))
flight = json.loads((ROOT / "runs/analysis/20261112_vertical_v27/evidence.json").read_text(encoding="utf-8"))
approach = ground["survey"]["approach"]
origin, yaw = approach["drive_origin"], approach["drive_yaw_rad"]
trajectory = ground["survey"]["trajectory_10hz"]
fig, axes = plt.subplots(1, 2, figsize=(11.8, 4.6), layout="constrained")
ax = axes[0]
x = [(r["position"][0]-origin[0])*100 for r in trajectory]
y = [(r["position"][1]-origin[1])*100 for r in trajectory]
for cross in (-2., 2.):
    ax.plot([cross*-math.sin(yaw), 11*math.cos(yaw)+cross*-math.sin(yaw)],
            [cross*math.cos(yaw), 11*math.sin(yaw)+cross*math.cos(yaw)],
            color="0.7", linewidth=1, linestyle=":")
ax.plot([0, 11*math.cos(yaw)], [0, 11*math.sin(yaw)],
        color="#6f6f6f", linestyle="--", label="规划方向")
ax.plot(x, y, color="#256494", linewidth=2, label="平台实际轨迹")
ax.scatter(x[-1], y[-1], c="#b35040", s=35, zorder=3)
ax.annotate("横偏超限停车", (x[-1], y[-1]), xytext=(4, 16), textcoords="offset points", color="#984535")
ax.set(aspect="equal", xlabel="相对转后起点 X（厘米）", ylabel="Y（厘米）",
       title="v26：首次斜向直行，未完成地面点")
ax.legend(loc="lower left", frameon=False)
ax.grid(alpha=.15)
ax = axes[1]
t = [r["wall_elapsed_s"] for r in flight["trajectory"]]
z = [(r["xyz"][2]-flight["origin"][2])*100 for r in flight["trajectory"]]
positive = [r["wall_elapsed_s"] for r in flight["commands"] if r["linear_z"] > 0]
ax.axvspan(min(positive), max(positive), color="#c6dfd0", alpha=.7, label="正 Z 指令时段")
ax.axhline(15, color="0.55", linestyle="--", linewidth=1, label="15 厘米目标")
ax.plot(t, z, color="#256494", linewidth=2, label="实际相对高度")
ax.annotate("归零后回落", (t[z.index(max(z))], max(z)), xytext=(28, -22),
            textcoords="offset points", color="#984535")
ax.set(xlabel="程序墙钟（秒）", ylabel="相对高度（厘米）",
       title="v27：真实上升成功，尚不能悬停", ylim=(-.5, 17))
ax.legend(frameon=False, loc="upper left", fontsize=9)
ax.grid(alpha=.15)
fig.suptitle("官方平台真实诊断数据｜两局均非完整巡检成绩", fontsize=13)
fig.savefig(OUT / "v26_v27_motion.png", dpi=170)
fig.savefig(OUT / "v26_v27_motion.pdf")
print(str(OUT / "v26_v27_motion.png"))
