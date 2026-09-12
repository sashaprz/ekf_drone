"""Generate accel_bias_convergence.png from test_accel_bias_convergence.py's results for the README."""

import matplotlib.pyplot as plt
from test_accel_bias_convergence import run

color_x = "#2C7BB6"
color_y = "#D95F02"
color_z = "#1B9E77"
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

log, true_accel_bias, _ = run()

t = [row[0] for row in log]
bias_x_est = [row[1] for row in log]
bias_y_est = [row[2] for row in log]
bias_z_est = [row[3] for row in log]

fig, ax = plt.subplots(figsize=(7.5, 4.6))
ax.plot(t, bias_x_est, color=color_x, linewidth=2, label="accel_bias_x est")
ax.plot(t, bias_y_est, color=color_y, linewidth=2, label="accel_bias_y est")
ax.plot(t, bias_z_est, color=color_z, linewidth=2, label="accel_bias_z est")
ax.axhline(true_accel_bias[0], color=color_x, linewidth=1, linestyle="--", alpha=0.6)
ax.axhline(true_accel_bias[1], color=color_y, linewidth=1, linestyle="--", alpha=0.6)
ax.axhline(true_accel_bias[2], color=color_z, linewidth=1, linestyle="--", alpha=0.6)
ax.set_xlabel("Time (s)", color=ink)
ax.set_ylabel("accel_bias estimate (m/s²)", color=ink)
ax.set_title("accel_bias convergence under a rotating trajectory\n(dashed = true injected value)", fontsize=11, color=ink)
style_axis(ax)
ax.legend(frameon=False, loc="lower right")
fig.tight_layout()
fig.savefig("accel_bias_convergence.png", dpi=150, facecolor="white")
print("wrote accel_bias_convergence.png")
