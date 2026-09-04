"""Generate static_comparison.png from the static-test results table for the README."""

import matplotlib.pyplot as plt
import numpy as np

color_comp = "#1B9E77"
color_angle = "#2C7BB6"
color_raw = "#D95F02"
ink = "#3A3A3A"
grid_color = "#DDDDDD"

filters = ["Complementary", "Angle-based", "Raw-vector + bias"]
colors = [color_comp, color_angle, color_raw]

#(true value, {filter: converged value or None if n/a})
panels = [
    ("Roll", 10.0, {"Complementary": 12.65, "Angle-based": 10.14, "Raw-vector + bias": 10.03}),
    ("Pitch", 0.0, {"Complementary": None, "Angle-based": 4.8, "Raw-vector + bias": 0.017}),
    ("Yaw", 0.0, {"Complementary": None, "Angle-based": 0.86, "Raw-vector + bias": 0.003}),
]

fig, axs = plt.subplots(1, 3, figsize=(10, 4.6))

for ax, (title, true_val, values) in zip(axs, panels):
    x = np.arange(len(filters))
    heights = [values[f] if values[f] is not None else 0 for f in filters]
    bars = ax.bar(x, heights, width=0.55, color=colors)

    for i, f in enumerate(filters):
        if values[f] is None:
            ax.text(i, true_val * 0.5 if true_val else 0.3, "n/a", ha="center", va="center",
                     fontsize=9, color=ink, style="italic")
        else:
            ax.text(i, values[f] + max(true_val, max(v for v in heights)) * 0.03,
                     f"{values[f]:.3g}°", ha="center", va="bottom", fontsize=8, color=ink)

    y_top = max(true_val, max(v for v in heights)) * 1.35 + 0.5
    if true_val != 0:
        ax.axhline(true_val, color=ink, linewidth=1, linestyle="--", alpha=0.6)
    ax.text(0.02, 0.96, f"true = {true_val:g}°", fontsize=8, color=ink, va="top", ha="left",
             transform=ax.transAxes)

    ax.set_title(title, fontsize=11, color=ink)
    ax.set_xticks(x)
    ax.set_xticklabels(["Comp.", "Angle", "Raw+bias"], color=ink, fontsize=9)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_color(grid_color)
    ax.spines["bottom"].set_color(grid_color)
    ax.tick_params(colors=ink)
    ax.yaxis.grid(True, color=grid_color, linewidth=0.8)
    ax.set_axisbelow(True)
    ax.set_ylim(0, y_top)

axs[0].set_ylabel("Converged value (degrees)", color=ink)

handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in colors]
fig.legend(handles, filters, loc="upper center", ncol=3, frameon=False, bbox_to_anchor=(0.5, 1.0))
fig.tight_layout(rect=[0, 0, 1, 0.88])
fig.savefig("static_comparison.png", dpi=150, facecolor="white")
print("wrote static_comparison.png")
