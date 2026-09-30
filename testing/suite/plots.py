"""Per-run plot: truth vs estimate, error with the filter's own +/-3 sigma, fault window shaded."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

import metrics as M  # noqa: E402

MAX_POINTS = 4000


def plot_run(path, run, truth, info, title):
    t = run["t"] - run["t"][0]
    step = max(1, len(t) // MAX_POINTS)
    s = slice(None, None, step)
    att = M.quat_err_deg(run["q"], truth["q"])
    eu_e, eu_t = M.quat_to_euler_deg(run["q"]), M.quat_to_euler_deg(truth["q"])
    eu_e[:, 2], eu_t[:, 2] = np.degrees(np.unwrap(np.radians(eu_e[:, 2]))), np.degrees(np.unwrap(np.radians(eu_t[:, 2])))
    sig_att = np.degrees(np.sqrt(np.maximum(np.diagonal(run["P_att"], axis1=1, axis2=2), 0)))
    perr = run["pos"] - truth["pos"]
    sig_pos = np.sqrt(np.maximum(np.diagonal(run["P_pos"], axis1=1, axis2=2), 0))

    fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    cols = ["tab:red", "tab:green", "tab:blue"]
    for j, n in enumerate(("roll", "pitch", "yaw")):
        ax[0].plot(t[s], eu_t[s, j], color=cols[j], lw=1.6, alpha=0.45, label=f"{n} true")
        ax[0].plot(t[s], eu_e[s, j], color=cols[j], lw=0.8, ls="--", label=f"{n} est")
        ax[1].plot(t[s], att[s, j], color=cols[j], lw=0.8, label=f"{n} err")
        ax[1].fill_between(t[s], -3 * sig_att[s, j], 3 * sig_att[s, j], color=cols[j], alpha=0.08)
    ax[0].set_ylabel("attitude (deg)")
    ax[1].set_ylabel("att error (deg)\nband = 3 sigma(P)")
    lim = max(1.0, float(np.percentile(np.abs(att), 99.5)) * 1.3)
    ax[1].set_ylim(-lim, lim)
    for j, n in enumerate("xyz"):
        ax[2].plot(t[s], truth["pos"][s, j], color=cols[j], lw=1.6, alpha=0.45, label=f"{n} true")
        ax[2].plot(t[s], run["pos"][s, j], color=cols[j], lw=0.8, ls="--", label=f"{n} est")
        ax[3].plot(t[s], perr[s, j], color=cols[j], lw=0.8, label=f"{n} err")
        ax[3].fill_between(t[s], -3 * sig_pos[s, j], 3 * sig_pos[s, j], color=cols[j], alpha=0.08)
    ax[2].set_ylabel("position (m)")
    ax[3].set_ylabel("pos error (m)\nband = 3 sigma(P)")
    plim = max(0.5, float(np.percentile(np.abs(perr), 99.5)) * 1.3)
    ax[3].set_ylim(-plim, plim)
    ax[3].set_xlabel("t (s since first logged tick)")
    win = info.get("window")
    for a in ax:
        if win is not None:
            a.axvspan(win[0] - run["t"][0], win[1] - run["t"][0], color="0.5", alpha=0.15, lw=0)
        a.grid(alpha=0.3)
        a.legend(loc="upper right", fontsize=6, ncol=6)
    ax[0].set_title(title)
    fig.tight_layout()
    fig.savefig(path, dpi=70)
    plt.close(fig)
