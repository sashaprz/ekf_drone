"""Generate gps_tracking_trajectory.png and gps_tracking_rms.png from test_gps_tracking.py's results for the README."""

import matplotlib.pyplot as plt
import numpy as np
from test_gps_tracking import run

color_gps = "#2C7BB6"
color_nogps = "#D95F02"
ink = "#3A3A3A"
grid_color = "#DDDDDD"

def style_axis(ax):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_color(grid_color)
    ax.spines["bottom"].set_color(grid_color)
    ax.tick_params(colors=ink)
    ax.yaxis.grid(True, color=grid_color, linewidth=0.8)
    ax.set_axisbelow(True)

#---- trajectory plot: gentle accel + accel bias - the clearest illustration of drift vs. correction ----
_, _, log_with = run(gps_enabled=True, accel_mag=1.0, bias_error=0.05)
_, _, log_without = run(gps_enabled=False, accel_mag=1.0, bias_error=0.05)

t = [row[0] for row in log_with]
pos_true = [row[2] for row in log_with]
pos_gps = [row[1] for row in log_with]
pos_nogps = [row[1] for row in log_without]

fig1, ax1 = plt.subplots(figsize=(7.5, 4.6))
ax1.plot(t, pos_true, color=ink, linewidth=2, linestyle="--", label="True position")
ax1.plot(t, pos_nogps, color=color_nogps, linewidth=2, label="Dead reckoning only (no GPS)")
ax1.plot(t, pos_gps, color=color_gps, linewidth=2, label="With GPS correction")
ax1.set_xlabel("Time (s)", color=ink)
ax1.set_ylabel("North position (m)", color=ink)
ax1.set_title("Position tracking: gentle accel + accel bias", fontsize=11, color=ink)
style_axis(ax1)
ax1.legend(frameon=False, loc="upper left")
fig1.tight_layout()
fig1.savefig("gps_tracking_trajectory.png", dpi=150, facecolor="white")
print("wrote gps_tracking_trajectory.png")

#---- RMS bar chart across the scenario matrix ----
scenarios = [
    ("Gentle,\nclean", 1.0, 0.00, 0.0),
    ("Aggressive,\nclean", 4.0, 0.00, 0.0),
    ("Gentle,\nbiased", 1.0, 0.05, 0.0),
    ("Aggressive,\nbiased", 4.0, 0.05, 0.0),
]

rms_gps, rms_nogps = [], []
for label, accel_mag, bias_error, tilt in scenarios:
    rp_with, _, _ = run(gps_enabled=True, accel_mag=accel_mag, bias_error=bias_error, roll_tilt_deg=tilt)
    rp_without, _, _ = run(gps_enabled=False, accel_mag=accel_mag, bias_error=bias_error, roll_tilt_deg=tilt)
    rms_gps.append(rp_with)
    rms_nogps.append(rp_without)

labels = [s[0] for s in scenarios]
x = np.arange(len(labels))
width = 0.32
y_max = max(rms_nogps) * 1.2

fig2, ax2 = plt.subplots(figsize=(7.5, 4.6))
bars_nogps = ax2.bar(x - width/2, rms_nogps, width, label="No GPS", color=color_nogps)
bars_gps = ax2.bar(x + width/2, rms_gps, width, label="With GPS", color=color_gps)
for bars in (bars_nogps, bars_gps):
    for b in bars:
        ax2.text(b.get_x() + b.get_width()/2, b.get_height() + y_max*0.02,
                  f"{b.get_height():.3f}", ha="center", va="bottom", fontsize=8, color=ink)

ax2.set_xticks(x)
ax2.set_xticklabels(labels, color=ink, fontsize=9)
ax2.set_ylim(0, y_max)
ax2.set_ylabel("Position RMS error (m)", color=ink)
ax2.set_title("GPS correction vs. dead reckoning, by scenario", fontsize=11, color=ink)
style_axis(ax2)
ax2.legend(frameon=False, loc="upper right")
fig2.tight_layout()
fig2.savefig("gps_tracking_rms.png", dpi=150, facecolor="white")
print("wrote gps_tracking_rms.png")
