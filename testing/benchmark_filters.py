"""Benchmark: FINAL_gps.py (this project's EKF) vs Madgwick vs Mahony vs PX4's EKF2.

Every filter sees the same recorded sensor stream and is scored against Gazebo ground truth:

  - ekf       FINAL_gps.DroneEKF, driven exactly like testing/suite (run_suite.replay)
  - madgwick  state estimation/madgwick.py   (attitude only - no position/velocity)
  - mahony    state estimation/mahony.py     (attitude only)
  - ekf2      PX4's own estimate, only in PX4-flown SITL logs (testing/ulog_to_csv.py writes
              it into the eq*/ep*/ev* columns; meta "truth" == "groundtruth")

Madgwick/Mahony get the same pre-flight information as the EKF: the calibration dwell
(<csv>.cal.csv) gives the starting gyro bias (mean gyro) and starting attitude (accel + mag
averaged, TRIAD), and the recording's magnetic-field direction gives magnetic north.

Each is run twice: with the reference C implementations' default gains, and with the best
gains from a grid search over the same recordings ("tuned" - a best case for them, since
they are tuned on the test data; the EKF is not retuned).

usage: python testing/benchmark_filters.py [csv ...] [--out DIR] [--no-sweep] [--plots]
       (default: every testing/data/flight_*.csv plus testing/data/shadow/sitl_flight*.csv)
Scoring window and error conventions are the suite's (testing/suite/metrics.py).
"""
import argparse
import glob
import json
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, "suite"))
sys.path.insert(0, os.path.join(ROOT, "state estimation"))
import replay_ekf
import run_suite
import metrics as M
from ulog_to_csv import R_to_quat
from madgwick import Madgwick
from mahony import Mahony

DEFAULTS = {"madgwick": {"beta": 0.1, "zeta": 0.0},   # MadgwickAHRS.c
            "mahony": {"kp": 0.5, "ki": 0.0}}         # MahonyAHRS.c
GRID = {"madgwick": [{"beta": b, "zeta": z} for b in (0.0, 0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2)
                     for z in (0.0, 0.002, 0.01)],
        "mahony": [{"kp": kp, "ki": ki} for kp in (0.0, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0)
                   for ki in (0.0, 0.01, 0.05, 0.2)]}
FILTERS = {"madgwick": Madgwick, "mahony": Mahony}


def default_recordings():
    paths = (sorted(glob.glob(os.path.join(ROOT, "testing", "data", "flight_*.csv")))
             + sorted(glob.glob(os.path.join(ROOT, "testing", "data", "shadow", "sitl_flight*.csv"))))
    return [p for p in paths if not p.endswith(".cal.csv")]  # skip the calibration sidecars


def triad(accel, mag, mag_ref):
    """Attitude from one 'up' and one 'north' observation: build an east/north/up basis in
    the body frame from the measurements and in the world frame from the references; the
    rotation between the two bases is the attitude."""
    up_b = accel / np.linalg.norm(accel)
    n_b = mag - (mag @ up_b) * up_b
    n_b /= np.linalg.norm(n_b)
    B = np.column_stack([np.cross(n_b, up_b), n_b, up_b])
    n_w = np.array([mag_ref[0], mag_ref[1], 0.0]) / np.hypot(mag_ref[0], mag_ref[1])
    W = np.column_stack([np.cross(n_w, [0.0, 0.0, 1.0]), n_w, [0.0, 0.0, 1.0]])
    return R_to_quat(W @ B.T)


_REC = {}


def load(path):
    """-> dict(cols, truth, cal, meta, window mask, init) for one recording (cached per process)."""
    if path in _REC:
        return _REC[path]
    cols = run_suite.load_recording(path)
    truth = M.truth_arrays(cols)
    cal = replay_ekf.load_cal(path)
    meta = replay_ekf.load_meta(path)
    mag_ref = np.array(meta.get("mag_reference", replay_ekf.MAG_REFERENCE_ENU), float)
    if cal:
        g0, a0, m0 = (np.mean(cal[k], axis=0) for k in ("gyro", "accel", "mag"))
    else:
        g0 = np.array([cols["gx"][0], cols["gy"][0], cols["gz"][0]])
        a0 = np.array([cols["ax"][0], cols["ay"][0], cols["az"][0]])
        m0 = np.array([cols["mx"][0], cols["my"][0], cols["mz"][0]])
    # scoring window, as in metrics.compute: from 2 s after takeoff to the end
    t = cols["t"]
    up = np.flatnonzero(truth["pos"][:, 2] > M.TAKEOFF_Z)
    t_start = t[up[0]] + M.SCORE_AFTER_TAKEOFF_S if len(up) else t[0]
    eq = np.stack([cols[k] for k in ("eqw", "eqx", "eqy", "eqz")], axis=1) if "eqw" in cols else None
    has_ekf2 = meta.get("truth") == "groundtruth" and eq is not None and np.abs(eq).sum() > 0
    _REC[path] = {"name": os.path.splitext(os.path.basename(path))[0], "path": path, "cols": cols,
                  "truth": truth, "cal": cal, "meta": meta, "mag_ref": mag_ref, "win": t >= t_start,
                  "q0": triad(a0, m0, mag_ref), "bias0": g0, "has_ekf2": has_ekf2}
    return _REC[path]


def run_ahrs(rec, kind, gains):
    c = rec["cols"]
    f = FILTERS[kind](q0=rec["q0"], gyro_bias0=rec["bias0"], mag_reference=rec["mag_ref"], **gains)
    g = np.stack([c["gx"], c["gy"], c["gz"]], axis=1)
    a = np.stack([c["ax"], c["ay"], c["az"]], axis=1)
    m = np.stack([c["mx"], c["my"], c["mz"]], axis=1)
    dts = np.diff(c["t"], prepend=c["t"][0] - np.median(np.diff(c["t"])))
    q = np.empty((len(g), 4))
    for i in range(len(g)):
        # the latest mag reading is used every step (held between samples) - how both
        # reference implementations are normally driven
        q[i] = f.update(g[i], a[i], m[i], float(np.clip(dts[i], 0.0, 0.05)))
    return {"q": q}


def score(rec, est):
    w = rec["win"]
    e = M.quat_err_deg(est["q"][w], rec["truth"]["q"][w])
    tilt, yaw = np.hypot(e[:, 0], e[:, 1]), np.abs(e[:, 2])
    s = {"tilt_rms": float(np.sqrt(np.mean(tilt ** 2))), "tilt_p95": float(np.percentile(tilt, 95)),
         "tilt_max": float(tilt.max()), "yaw_rms": float(np.sqrt(np.mean(yaw ** 2))), "yaw_max": float(yaw.max())}
    if "pos" in est:
        ph = np.linalg.norm((est["pos"] - rec["truth"]["pos"])[w][:, :2], axis=1)
        pv = np.abs((est["pos"] - rec["truth"]["pos"])[w][:, 2])
        ve = np.linalg.norm((est["vel"] - rec["truth"]["vel"])[w], axis=1)
        s.update(pos_h_rms=float(np.sqrt(np.mean(ph ** 2))), pos_v_rms=float(np.sqrt(np.mean(pv ** 2))),
                 vel_rms=float(np.sqrt(np.mean(ve ** 2))))
    return s


def task_ahrs(args):
    path, kind, gains = args
    rec = load(path)
    with np.errstate(all="ignore"):
        out = run_ahrs(rec, kind, gains)
    return path, kind, gains, score(rec, out), out["q"]


def task_ekf(path):
    rec = load(path)
    m = rec["meta"]
    out = run_suite.replay(rec["cols"], False, rec["cal"], m.get("gps_latency_s", 0.0), m.get("mag_reference"))
    res = {"ekf": (score(rec, out), out["q"])}
    if rec["has_ekf2"]:
        c = rec["cols"]
        ep = np.stack([c["epx"], c["epy"], c["epz"]], axis=1)
        e2 = {"q": np.stack([c[k] for k in ("eqw", "eqx", "eqy", "eqz")], axis=1),
              "pos": ep - ep[0], "vel": np.stack([c["evx"], c["evy"], c["evz"]], axis=1)}
        res["ekf2"] = (score(rec, e2), e2["q"])
    return path, res


CAVEATS = """## Read before quoting these numbers

- **Gazebo's gyros are nearly ideal** (tiny noise, no bias drift), and the dwell hands every filter the
  exact gyro bias. That is why the *tuned* Madgwick/Mahony gains collapse toward zero (beta=0.001, kp=0.02):
  in this sim, near-pure gyro integration is almost free. On real hardware the bias drifts and such
  low gains would let the attitude wander. The *default* gains are what those filters normally run with.
- **The default Madgwick/Mahony failure in manoeuvres (circle_fast, stops) is the thrust problem**: both
  treat the accelerometer as gravity, but in flight it measures thrust along body z, so they are pulled
  toward level whenever the drone tilts to accelerate. Your EKF turns accel-as-gravity off in flight and
  gets tilt from GPS velocity instead - that is the main reason it wins there.
- **EKF2's 5.6 deg heading on sitl_flight1 is mostly a reference mismatch, not filter error**:
  ulog_to_csv.py measured the field direction with the ground-truth attitude and handed it to your EKF,
  Madgwick and Mahony; EKF2 uses the World Magnetic Model declination, which does not match Gazebo's field.
- **Only one EKF2 log** (takeoff, ~25 s hover, land). EKF2's tilt advantage there (0.03 vs 0.24 deg) is real
  but measured on very little motion; a manoeuvring PX4-flown SITL flight would say much more.
"""


def fmt(v, nd=2):
    return "-" if v is None else f"{v:.{nd}f}"


def plot(rec, qs, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = rec["cols"]["t"] - rec["cols"]["t"][0]
    fig, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    for name, q in qs.items():
        e = M.quat_err_deg(q, rec["truth"]["q"])
        ax[0].plot(t, np.hypot(e[:, 0], e[:, 1]), lw=0.8, label=name)
        ax[1].plot(t, np.abs(e[:, 2]), lw=0.8, label=name)
    w0 = t[np.argmax(rec["win"])]
    for a_, lab in zip(ax, ("tilt error (deg)", "heading error (deg)")):
        a_.axvline(w0, color="k", lw=0.5, ls=":")
        a_.set_ylabel(lab); a_.set_yscale("log"); a_.grid(alpha=0.3)
    ax[0].legend(ncol=5, fontsize=8); ax[1].set_xlabel("time (s)  [dotted: scoring starts]")
    ax[0].set_title(rec["name"])
    fig.tight_layout(); fig.savefig(path, dpi=110); plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="*")
    ap.add_argument("--out", default=os.path.join(ROOT, "testing", "results", "benchmark"))
    ap.add_argument("--no-sweep", action="store_true")
    ap.add_argument("--plots", action="store_true")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    a = ap.parse_args()
    paths = [os.path.abspath(p) for p in (a.csv or default_recordings())]
    os.makedirs(a.out, exist_ok=True)

    with Pool(a.jobs) as pool:
        ekf = dict(pool.map(task_ekf, paths))
        configs = {k: ([DEFAULTS[k]] if a.no_sweep else GRID[k] + [DEFAULTS[k]]) for k in FILTERS}
        jobs = [(p, k, g) for k in FILTERS for g in configs[k] for p in paths]
        ahrs = pool.map(task_ahrs, jobs, chunksize=1)

    # tuned = the gain set with the lowest mean (tilt_rms + yaw_rms) over all recordings
    table = {}   # (kind, gains json) -> {path: (score, q)}
    for path, kind, gains, s, q in ahrs:
        table.setdefault((kind, json.dumps(gains, sort_keys=True)), {})[path] = (s, q)
    chosen = {}
    for kind in FILTERS:
        chosen[f"{kind} (default)"] = table[(kind, json.dumps(DEFAULTS[kind], sort_keys=True))]
        if not a.no_sweep:
            best = min((k for k in table if k[0] == kind),
                       key=lambda k: np.mean([v[0]["tilt_rms"] + v[0]["yaw_rms"] for v in table[k].values()]))
            chosen[f"{kind} (tuned {', '.join(f'{x}={y}' for x, y in json.loads(best[1]).items())})"] = table[best]

    lines = ["# Filter benchmark", "",
             "Same sensor data for every filter, scored against Gazebo ground truth from 2 s after takeoff "
             "(`testing/benchmark_filters.py`). Tilt = |roll/pitch error|, heading = |yaw error|, degrees. "
             "Madgwick/Mahony estimate attitude only. *Tuned* gains are the grid-search best over these same "
             "recordings (a best case for them); the EKF is not retuned. EKF2 exists only in PX4-flown SITL logs.", ""]
    for path in paths:
        rec = load(path)
        rows = [("**your EKF**", ekf[path]["ekf"][0])]
        if "ekf2" in ekf[path]:
            rows.append(("PX4 EKF2", ekf[path]["ekf2"][0]))
        rows += [(name, by_path[path][0]) for name, by_path in chosen.items()]
        dur = rec["cols"]["t"][rec["win"]]
        lines += [f"## {rec['name']}  ({dur[-1] - dur[0]:.0f} s scored)", "",
                  "| filter | tilt rms | tilt p95 | tilt max | heading rms | heading max | pos h rms (m) | pos v rms (m) | vel rms (m/s) |",
                  "|---|---|---|---|---|---|---|---|---|"]
        for name, s in rows:
            lines.append(f"| {name} | {fmt(s['tilt_rms'])} | {fmt(s['tilt_p95'])} | {fmt(s['tilt_max'])} | "
                         f"{fmt(s['yaw_rms'])} | {fmt(s['yaw_max'])} | {fmt(s.get('pos_h_rms'))} | "
                         f"{fmt(s.get('pos_v_rms'))} | {fmt(s.get('vel_rms'))} |")
        lines.append("")
        if a.plots:
            qs = {"your EKF": ekf[path]["ekf"][1]}
            if "ekf2" in ekf[path]:
                qs["PX4 EKF2"] = ekf[path]["ekf2"][1]
            qs.update({n.split(" (")[0] + (" tuned" if "tuned" in n else ""): bp[path][1] for n, bp in chosen.items()})
            plot(rec, qs, os.path.join(a.out, f"{rec['name']}.png"))
            lines += [f"![{rec['name']}]({rec['name']}.png)", ""]

    # summary across the Gazebo recordings (the SITL shadow log is reported separately above)
    lines += ["## Summary (mean over all recordings)", "", "| filter | tilt rms | tilt max | heading rms | heading max |",
              "|---|---|---|---|---|"]
    summary = [("**your EKF**", {p: ekf[p]["ekf"] for p in paths})] + list(chosen.items())
    for name, by_path in summary:
        ss = [by_path[p][0] for p in paths]
        lines.append(f"| {name} | {fmt(np.mean([s['tilt_rms'] for s in ss]))} | {fmt(np.mean([s['tilt_max'] for s in ss]))} | "
                     f"{fmt(np.mean([s['yaw_rms'] for s in ss]))} | {fmt(np.mean([s['yaw_max'] for s in ss]))} |")
    if not a.no_sweep:
        lines += ["", "### Gain sweep (mean tilt rms / heading rms over all recordings)", ""]
        for kind in FILTERS:
            lines += [f"**{kind}**", "", "| gains | tilt rms | heading rms |", "|---|---|---|"]
            for k in sorted((k for k in table if k[0] == kind), key=lambda k: k[1]):
                v = table[k].values()
                lines.append(f"| {k[1]} | {fmt(np.mean([x[0]['tilt_rms'] for x in v]))} | {fmt(np.mean([x[0]['yaw_rms'] for x in v]))} |")
            lines.append("")
    lines += ["", CAVEATS]
    report = os.path.join(a.out, "REPORT.md")
    with open(report, "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))
    print(f"\n-> {report}")


if __name__ == "__main__":
    main()
