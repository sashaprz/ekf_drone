"""
Generate project_hero.png: a single shareable summary graphic for the whole
project's arc - complementary filter -> progressively better EKFs -> quaternion
MEKF -> GPS-fused position tracking. Not meant to replace the per-section README
charts; this is the "post about it" image.
"""

import matplotlib.pyplot as plt
import numpy as np
from test_gps_tracking import run

color_comp = "#1B9E77"
color_angle = "#2C7BB6"
color_raw = "#D95F02"
color_quat = "#7570B3"
color_gps = "#2C7BB6"
color_nogps = "#D95F02"
ink = "#3A3A3A"
muted = "#6B6B6B"
grid_color = "#DDDDDD"

def style_axis(ax):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_color(grid_color)
    ax.spines["bottom"].set_color(grid_color)
    ax.tick_params(colors=ink)
    ax.yaxis.grid(True, color=grid_color, linewidth=0.8)
    ax.set_axisbelow(True)

fig = plt.figure(figsize=(13, 6.8))
gs = fig.add_gridspec(1, 2, left=0.07, right=0.97, top=0.80, bottom=0.13, wspace=0.28)

#---- title block ----
fig.text(0.07, 0.92, "drone sim — 17x steadier, 9x more accurate with GPS", fontsize=22, fontweight="bold", color=ink)

#---- panel A: attitude robustness to gyro bias (10x-bias scenario, avg RMS across roll/pitch/yaw) ----
ax1 = fig.add_subplot(gs[0, 0])
filters = ["Complementary", "Angle-based\nEKF", "Raw-vector\n+bias EKF", "Quaternion\nMEKF"]
colors_a = [color_comp, color_angle, color_raw, color_quat]
rms_10x_bias = {
    "Complementary": [9.603, 5.029, 2.534],
    "Angle-based": [0.681, 0.521, 0.268],
    "Raw-vector + bias": [0.449, 0.308, 0.229],
    "Quaternion MEKF": [0.452, 0.308, 0.231],
}
avg_rms = [np.mean(v) for v in rms_10x_bias.values()]

x = np.arange(len(filters))
bars = ax1.bar(x, avg_rms, width=0.6, color=colors_a)
for b, v in zip(bars, avg_rms):
    ax1.text(b.get_x() + b.get_width()/2, v + max(avg_rms)*0.03, f"{v:.2f}°",
              ha="center", va="bottom", fontsize=10, color=ink)

ax1.set_xticks(x)
ax1.set_xticklabels(filters, color=ink, fontsize=9.5)
ax1.set_ylabel("Avg. attitude RMS error (°)", color=ink)
ax1.set_title("Robustness to 10x gyro bias", fontsize=13, color=ink, pad=12)
ax1.set_ylim(0, max(avg_rms) * 1.25)
style_axis(ax1)

#---- panel B: GPS-fused position tracking vs. pure dead reckoning ----
ax2 = fig.add_subplot(gs[0, 1])
_, _, log_with = run(gps_enabled=True, accel_mag=1.0, bias_error=0.05)
_, _, log_without = run(gps_enabled=False, accel_mag=1.0, bias_error=0.05)

t = [row[0] for row in log_with]
pos_true = [row[2] for row in log_with]
pos_gps = [row[1] for row in log_with]
pos_nogps = [row[1] for row in log_without]

ax2.plot(t, pos_true, color=ink, linewidth=2, linestyle="--", label="True position")
ax2.plot(t, pos_nogps, color=color_nogps, linewidth=2, label="Dead reckoning only")
ax2.plot(t, pos_gps, color=color_gps, linewidth=2, label="With GPS fusion")
ax2.set_xlabel("Time (s)", color=ink)
ax2.set_ylabel("North position (m)", color=ink)
ax2.set_title("Position tracking with GPS fusion", fontsize=13, color=ink, pad=12)
style_axis(ax2)
ax2.legend(frameon=False, loc="upper left", fontsize=9.5)

fig.savefig("project_hero.png", dpi=150, facecolor="white")
print("wrote project_hero.png")
