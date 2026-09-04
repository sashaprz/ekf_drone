"""Generate rms_comparison.png from the ekf_rms.py results for the README."""

import matplotlib.pyplot as plt
import numpy as np

axes_labels = ["Roll", "Pitch", "Yaw"]
filters = ["Complementary", "Angle-based", "Raw-vector + bias"]

data = {
    "Original bias": {
        "Complementary": [0.954, 1.249, 0.327],
        "Angle-based": [0.445, 0.460, 0.230],
        "Raw-vector + bias": [0.443, 0.305, 0.229],
    },
    "10x bias": {
        "Complementary": [9.603, 5.029, 2.534],
        "Angle-based": [0.681, 0.521, 0.268],
        "Raw-vector + bias": [0.449, 0.308, 0.229],
    },
}

color_comp = "#1B9E77"
color_angle = "#2C7BB6"
color_raw = "#D95F02"
colors = [color_comp, color_angle, color_raw]
ink = "#3A3A3A"
grid_color = "#DDDDDD"

fig, axs = plt.subplots(1, 2, figsize=(9.5, 4.6)) #own y-scale per panel: complementary's 10x values dwarf the EKFs'

for ax, (panel_title, series) in zip(axs, data.items()):
    x = np.arange(len(axes_labels))
    width = 0.25
    y_max = max(v for vals in series.values() for v in vals) * 1.22

    for i, f in enumerate(filters):
        offset = (i - 1) * width
        bars = ax.bar(x + offset, series[f], width, label=f, color=colors[i])
        for b in bars:
            ax.text(b.get_x() + b.get_width()/2, b.get_height() + y_max*0.02,
                     f"{b.get_height():.2f}", ha="center", va="bottom", fontsize=7.5, color=ink)

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
fig.legend(handles, labels, loc="upper center", ncol=3, frameon=False, bbox_to_anchor=(0.5, 1.0))
fig.tight_layout(rect=[0, 0, 1, 0.88])
fig.savefig("rms_comparison.png", dpi=150, facecolor="white")
print("wrote rms_comparison.png")
