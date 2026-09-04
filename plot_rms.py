"""Generate rms_comparison.png from the ekf_rms.py results for the README."""

import matplotlib.pyplot as plt
import numpy as np

axes_labels = ["Roll", "Pitch", "Yaw"]

data = {
    "Original bias": {
        "Angle-based": [0.474, 0.427, 0.238],
        "Raw-vector + bias": [0.469, 0.296, 0.240],
    },
    "10x bias": {
        "Angle-based": [0.723, 0.510, 0.268],
        "Raw-vector + bias": [0.483, 0.299, 0.246],
    },
}

color_angle = "#2C7BB6"
color_raw = "#D95F02"
ink = "#3A3A3A"
grid_color = "#DDDDDD"

fig, axs = plt.subplots(1, 2, figsize=(9, 4.6), sharey=True)
y_max = max(v for panel in data.values() for series in panel.values() for v in series) * 1.2

for ax, (panel_title, series) in zip(axs, data.items()):
    x = np.arange(len(axes_labels))
    width = 0.32

    bars_angle = ax.bar(x - width/2, series["Angle-based"], width, label="Angle-based", color=color_angle)
    bars_raw = ax.bar(x + width/2, series["Raw-vector + bias"], width, label="Raw-vector + bias", color=color_raw)

    for bars in (bars_angle, bars_raw):
        for b in bars:
            ax.text(b.get_x() + b.get_width()/2, b.get_height() + y_max*0.02,
                     f"{b.get_height():.2f}", ha="center", va="bottom", fontsize=8, color=ink)

    ax.set_title(panel_title, fontsize=11, color=ink)
    ax.set_xticks(x)
    ax.set_xticklabels(axes_labels, color=ink)
    ax.set_ylim(0, y_max)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_color(grid_color)
    ax.spines["bottom"].set_color(grid_color)
    ax.tick_params(colors=ink)
    ax.yaxis.grid(True, color=grid_color, linewidth=0.8)
    ax.set_axisbelow(True)

axs[0].set_ylabel("RMS error (degrees)", color=ink)
handles, labels = axs[0].get_legend_handles_labels()
fig.legend(handles, labels, loc="upper center", ncol=2, frameon=False, bbox_to_anchor=(0.5, 1.0))
fig.tight_layout(rect=[0, 0, 1, 0.88])
fig.savefig("rms_comparison.png", dpi=150, facecolor="white")
print("wrote rms_comparison.png")
