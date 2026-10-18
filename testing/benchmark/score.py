"""Score the benchmark campaign: your EKF vs PX4 EKF2 vs Madgwick vs Mahony.

    python testing/benchmark/score.py [--jobs N] [--out testing/results/benchmark_v2]

Input: testing/data/benchmark/<scenario>__<model>__t<trial>__<fault>.csv (prepare.py).
Every filter sees the same corrupted sensor stream; truth is Gazebo ground truth.

Method (stated again in the report):
  - window: from 2 s after takeoff to the last airborne sample (truth > 0.3 m above start)
  - attitude error: body-frame small-angle vector (metrics.quat_err_deg); tilt = |x,y|,
    heading = |z|; "heading (offset removed)" subtracts the run's mean heading error, so a
    constant north-reference offset doesn't count
  - Madgwick/Mahony: reference-implementation default gains, and "CV-tuned" gains: for each
    trial k, the grid point with the lowest mean (tilt rms + heading rms) over all runs of
    the OTHER trials - no run is scored with gains chosen on its own trial
  - per-condition numbers: mean over trials with a 95% t-interval (n is small: read the
    interval, not the mean)
  - paired comparison vs your EKF: per run ratio other/yours; geometric mean ratio with a
    95% bootstrap interval resampling whole flights (runs of one flight are correlated)
  - every run that produced numbers counts in the statistics (a filter that is 30 deg off
    during a manoeuvre is scored 30 deg off, not dropped). "lost" = NaN, or still off in the
    last 10 s airborne: tilt > 10 deg, heading > 30 deg or horizontal position > 10 m
    (it did not recover); lost runs are counted separately AND kept in the error statistics
"""
import argparse
import glob
import json
import math
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "testing"))
sys.path.insert(0, os.path.join(ROOT, "testing", "suite"))
sys.path.insert(0, os.path.join(ROOT, "state estimation"))
import replay_ekf
import run_suite
import metrics as M
import benchmark_filters as BF

DATA = os.path.join(ROOT, "testing", "data", "benchmark")
GRID = {"madgwick": [{"beta": b, "zeta": z} for b in (0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2)
                     for z in (0.0, 0.002, 0.01)],
        "mahony": [{"kp": kp, "ki": ki} for kp in (0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0)
                   for ki in (0.0, 0.01, 0.05)]}
DEFAULTS = BF.DEFAULTS
FILTERS = ["ekf", "ekf2", "madgwick", "madgwick_cv", "mahony", "mahony_cv"]
LABEL = {"ekf": "**your EKF**", "ekf2": "PX4 EKF2", "madgwick": "Madgwick (default)", "madgwick_cv": "Madgwick (CV-tuned)",
         "mahony": "Mahony (default)", "mahony_cv": "Mahony (CV-tuned)"}
FAULT_WINDOWS = {"gps_dropout": (25.0, 40.0), "mag_interference": (20.0, 30.0)}  # s after takeoff (ulog_faults.py)
LOST = {"end_tilt": 10.0, "end_yaw": 30.0, "end_pos_h": 10.0}


def parse_name(path):
    scen, model, trial, fault = os.path.splitext(os.path.basename(path))[0].split("__")
    return {"scenario": scen, "model": model, "trial": int(trial[1:]), "fault": fault,
            "flight": f"{scen}__{model}__{trial}"}


def load(path):
    c = run_suite.load_recording(path)
    meta = replay_ekf.load_meta(path)
    cal = replay_ekf.load_cal(path)
    t = c["t"]
    q = np.stack([c["tqw"], c["tqx"], c["tqy"], c["tqz"]], 1)
    pos = np.stack([c["tpx"], c["tpy"], c["tpz"]], 1)
    truth = {"q": q, "pos": pos - pos[0], "vel": np.stack([c["tvx"], c["tvy"], c["tvz"]], 1)}
    air = np.flatnonzero(truth["pos"][:, 2] > 0.3)
    t_to = t[air[0]]
    win = (t >= t_to + 2.0) & (t <= t[air[-1]])
    mag_ref = np.array(meta["mag_reference"], float)
    g0, a0, m0 = (np.mean(cal[k], axis=0) for k in ("gyro", "accel", "mag"))
    return {"cols": c, "meta": meta, "cal": cal, "t": t, "t_takeoff": t_to, "truth": truth, "win": win,
            "mag_ref": mag_ref, "q0": BF.triad(a0, m0, mag_ref), "bias0": g0}


def metrics(rec, est, fault):
    w, t = rec["win"], rec["t"]
    if not np.all(np.isfinite(est["q"][w])):
        return {"nan": True}
    e = M.quat_err_deg(est["q"], rec["truth"]["q"])
    tilt, yaw = np.hypot(e[:, 0], e[:, 1]), e[:, 2]
    yw = yaw[w]
    end = w & (t >= t[w][-1] - 10.0)
    m = {"nan": False, "end_tilt": float(tilt[end].mean()), "end_yaw": float(np.abs(yaw[end]).mean()),
         "tilt_rms": float(np.sqrt(np.mean(tilt[w] ** 2))), "tilt_max": float(tilt[w].max()),
         "yaw_rms": float(np.sqrt(np.mean(yw ** 2))), "yaw_max": float(np.abs(yw).max()),
         "yaw_rms_offset_removed": float(np.std(yw))}
    if "pos" in est:
        if not (np.all(np.isfinite(est["pos"][w])) and np.all(np.isfinite(est["vel"][w]))):
            return {"nan": True}
        dp = est["pos"] - rec["truth"]["pos"]
        ph = np.linalg.norm(dp[:, :2], axis=1)
        dv = np.linalg.norm(est["vel"] - rec["truth"]["vel"], axis=1)
        m.update(pos_h_rms=float(np.sqrt(np.mean(ph[w] ** 2))), pos_h_max=float(ph[w].max()), end_pos_h=float(ph[end].mean()),
                 pos_v_rms=float(np.sqrt(np.mean(dp[w, 2] ** 2))), vel_rms=float(np.sqrt(np.mean(dv[w] ** 2))))
        if fault == "gps_dropout":
            lo, hi = FAULT_WINDOWS[fault]
            fw = w & (t >= rec["t_takeoff"] + lo) & (t <= rec["t_takeoff"] + hi)
            if fw.any():
                m["dropout_pos_h_max"] = float(ph[fw].max())
    if fault in FAULT_WINDOWS:
        lo, hi = FAULT_WINDOWS[fault]
        fw = w & (t >= rec["t_takeoff"] + lo) & (t <= rec["t_takeoff"] + hi + 10.0)  # + 10 s recovery
        if fw.any():
            m["fault_yaw_max"] = float(np.abs(yaw[fw]).max())
            m["fault_tilt_max"] = float(tilt[fw].max())
    return m


def lost(m):
    return m.get("nan", False) or any(m.get(k, 0.0) > v for k, v in LOST.items())


def usable(m):
    return not m.get("nan", False)


def task(path):
    info = parse_name(path)
    rec = load(path)
    out = {"info": info, "ahrs": {}}
    c, meta = rec["cols"], rec["meta"]
    with np.errstate(all="ignore"):
        r = run_suite.replay(c, False, rec["cal"], meta.get("gps_latency_s", 0.0), meta["mag_reference"])
        out["ekf"] = metrics(rec, r, info["fault"]) if r["error"] is None else {"nan": True, "error": r["error"]}
        ep = np.stack([c["epx"], c["epy"], c["epz"]], 1)
        e2 = {"q": np.stack([c["eqw"], c["eqx"], c["eqy"], c["eqz"]], 1), "pos": ep - ep[0],
              "vel": np.stack([c["evx"], c["evy"], c["evz"]], 1)}
        out["ekf2"] = metrics(rec, e2, info["fault"])
        for kind in ("madgwick", "mahony"):
            for g in GRID[kind] + ([DEFAULTS[kind]] if DEFAULTS[kind] not in GRID[kind] else []):
                q = BF.run_ahrs(rec, kind, g)["q"]
                out["ahrs"][f"{kind}|{json.dumps(g, sort_keys=True)}"] = metrics(rec, {"q": q}, info["fault"])
    return path, out


# ---- aggregation ---------------------------------------------------------------------
T95 = {1: 12.71, 2: 4.30, 3: 3.18, 4: 2.78, 5: 2.57, 6: 2.45, 7: 2.36, 8: 2.31, 9: 2.26}


def ci(x):
    x = np.asarray([v for v in x if v is not None and np.isfinite(v)], float)
    if len(x) == 0:
        return None, None, 0
    if len(x) == 1:
        return float(x[0]), None, 1
    return float(x.mean()), float(T95.get(len(x) - 1, 1.96) * x.std(ddof=1) / math.sqrt(len(x))), len(x)


def fmt_ci(x, nd=2):
    m, h, n = ci(x)
    if m is None:
        return "-"
    return f"{m:.{nd}f}" if h is None else f"{m:.{nd}f} ± {h:.{nd}f}"


def cv_select(runs):
    """Leave-one-trial-out gain choice -> runs[path]['madgwick_cv'|'mahony_cv'] metrics."""
    trials = sorted({r["info"]["trial"] for r in runs.values()})
    chosen = {}
    for kind in ("madgwick", "mahony"):
        keys = [k for k in next(iter(runs.values()))["ahrs"] if k.startswith(kind + "|")]
        for k_test in trials:
            train = [r for r in runs.values() if r["info"]["trial"] != k_test] or list(runs.values())

            def obj(key):
                v = [r["ahrs"][key] for r in train]
                return np.mean([(x["tilt_rms"] + x["yaw_rms"]) if usable(x) else 1e3 for x in v])
            best = min(keys, key=obj)
            chosen[(kind, k_test)] = best
            for r in runs.values():
                if r["info"]["trial"] == k_test:
                    r[kind + "_cv"] = r["ahrs"][best]
    for r in runs.values():
        for kind in ("madgwick", "mahony"):
            r[kind] = r["ahrs"][f"{kind}|{json.dumps(DEFAULTS[kind], sort_keys=True)}"]
    return chosen


def boot_ratio(runs, other, key, n_boot=4000, seed=0):
    """geometric-mean ratio other/ekf over runs, 95% CI resampling flights; + win rate of ekf"""
    by_flight = {}
    for r in runs:
        a, b = r["ekf"], r[other]
        if not usable(a) or not usable(b) or key not in a or key not in b:
            continue
        by_flight.setdefault(r["info"]["flight"], []).append(math.log(max(b[key], 1e-6) / max(a[key], 1e-6)))
    if not by_flight:
        return None
    flights = list(by_flight.values())
    allv = np.concatenate([np.array(f) for f in flights])
    rng = np.random.default_rng(seed)
    boots = []
    for _ in range(n_boot):
        pick = rng.integers(0, len(flights), len(flights))
        boots.append(np.concatenate([flights[i] for i in pick]).mean())
    lo, hi = np.percentile(boots, [2.5, 97.5])
    return {"ratio": float(np.exp(allv.mean())), "lo": float(np.exp(lo)), "hi": float(np.exp(hi)),
            "ekf_wins": float(np.mean(allv > 0)), "n": int(len(allv))}


def table(rows, header):
    out = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    out += ["| " + " | ".join(r) + " |" for r in rows]
    return out


def report(runs, chosen, out_dir):
    R = list(runs.values())
    L = ["# Filter benchmark v2 - your EKF vs PX4 EKF2 vs Madgwick vs Mahony", "",
         "All flights flown by **PX4 itself** in Gazebo SITL (none of this project's code in the loop); every filter "
         "sees the identical sensor stream (EKF2 via PX4's own replay of the same - possibly fault-injected - log). "
         "Truth = Gazebo ground truth. Method details: `testing/benchmark/score.py` docstring; setup: `testing/benchmark/README.md`.", ""]
    n_fl = len({r["info"]["flight"] for r in R})
    L += [f"**{len(R)} runs** from **{n_fl} flights** "
          f"({', '.join(sorted({r['info']['scenario'] for r in R}))}; trials: {sorted({r['info']['trial'] for r in R})}). "
          "Values: mean ± 95% CI over trials.", ""]
    L += ["CV-tuned gains chosen (per held-out trial): " + "; ".join(
        f"{k[0]} t{k[1]}: `{v.split('|')[1]}`" for k, v in sorted(chosen.items())), ""]

    def sel(**kw):
        return [r for r in R if all(r["info"][k] == v for k, v in kw.items())]

    # 1. headline, per condition group
    L += ["## 1. Headline", ""]
    groups = [("Stock x500 IMU (near-ideal), no fault", sel(model="x500", fault="none")),
              ("Realistic IMU, no fault", sel(model="x500_bench", fault="none")),
              ("Realistic IMU, all 8 faults pooled", [r for r in sel(model="x500_bench") if r["info"]["fault"] != "none"])]
    for title, rs in groups:
        if not rs:
            continue
        L += [f"### {title}  ({len(rs)} runs)", ""]
        rows = []
        for f in FILTERS:
            ms = [r[f] for r in rs]
            ok = [m for m in ms if usable(m)]
            rows.append([LABEL[f], fmt_ci([m["tilt_rms"] for m in ok]), fmt_ci([m["tilt_max"] for m in ok]),
                         fmt_ci([m["yaw_rms"] for m in ok]), fmt_ci([m["yaw_rms_offset_removed"] for m in ok]),
                         fmt_ci([m.get("pos_h_rms") for m in ok], 3) if f in ("ekf", "ekf2") else "-",
                         fmt_ci([m.get("vel_rms") for m in ok], 3) if f in ("ekf", "ekf2") else "-",
                         f"{sum(lost(m) for m in ms)}/{len(ms)}"])
        L += table(rows, ["filter", "tilt rms °", "tilt max °", "heading rms °", "heading rms, offset removed °",
                          "pos h rms m", "vel rms m/s", "lost"]) + [""]
        L += ["(mean ± CI here is over all runs in the group, not over trials)", ""]

    # 2. paired comparison
    L += ["## 2. Paired comparison against your EKF (realistic IMU, all conditions)", "",
          "Ratio = other filter's error / your EKF's error on the *same run*, geometric mean, 95% CI by resampling flights. "
          "> 1 means your EKF is better. 'EKF better' = share of runs where yours had the lower error. All runs with numbers "
          "included.", ""]
    rb = sel(model="x500_bench")
    rows = []
    for other in FILTERS[1:]:
        for key, lab in (("tilt_rms", "tilt rms"), ("yaw_rms", "heading rms"), ("yaw_rms_offset_removed", "heading (offset removed)"),
                         ("pos_h_rms", "pos h rms"), ("vel_rms", "vel rms")):
            if key in ("pos_h_rms", "vel_rms") and other != "ekf2":
                continue
            b = boot_ratio(rb, other, key)
            if b:
                rows.append([LABEL[other], lab, f"{b['ratio']:.2f} [{b['lo']:.2f}, {b['hi']:.2f}]", f"{100 * b['ekf_wins']:.0f}%", str(b["n"])])
    L += table(rows, ["vs", "metric", "ratio [95% CI]", "EKF better", "runs"]) + [""]

    # 3. per scenario
    L += ["## 3. Per scenario (realistic IMU, no fault) - mean ± 95% CI over trials", ""]
    for key, lab in (("tilt_rms", "tilt rms °"), ("yaw_rms", "heading rms °"), ("pos_h_rms", "pos h rms m")):
        scen = sorted({r["info"]["scenario"] for r in rb})
        rows = []
        for f in (FILTERS if key != "pos_h_rms" else ["ekf", "ekf2"]):
            rows.append([LABEL[f]] + [fmt_ci([r[f].get(key) for r in sel(model="x500_bench", fault="none", scenario=s)
                                              if usable(r[f])], 3 if key == "pos_h_rms" else 2) for s in scen])
        L += [f"**{lab}**", ""] + table(rows, ["filter"] + scen) + [""]

    # 4. per fault
    L += ["## 4. Per fault (realistic IMU, all scenarios pooled) - mean ± 95% CI over the 18 flights", ""]
    faults = sorted({r["info"]["fault"] for r in rb}, key=lambda f: (f != "none", f))
    for key, lab, fs in (("tilt_rms", "tilt rms °", FILTERS), ("tilt_max", "tilt max °", FILTERS),
                         ("yaw_rms", "heading rms °", FILTERS), ("pos_h_rms", "pos h rms m", ["ekf", "ekf2"])):
        rows = []
        for f in fs:
            cells = []
            for flt in faults:
                ms = [r[f] for r in sel(model="x500_bench", fault=flt)]
                ok = [m for m in ms if usable(m)]
                c = fmt_ci([m.get(key) for m in ok], 3 if "pos" in key else 2)
                n_lost = sum(lost(m) for m in ms)
                cells.append(c + (f" ({n_lost} lost)" if n_lost else ""))
            rows.append([LABEL[f]] + cells)
        L += [f"**{lab}**", ""] + table(rows, ["filter"] + faults) + [""]

    # 5. fault-window specifics
    L += ["## 5. During the fault (+10 s recovery)", ""]
    rows = []
    for f in FILTERS:
        cells = []
        for flt, key in (("gps_dropout", "dropout_pos_h_max"), ("gps_dropout", "fault_tilt_max"),
                         ("mag_interference", "fault_yaw_max"), ("mag_interference", "fault_tilt_max")):
            ms = [r[f] for r in sel(model="x500_bench", fault=flt) if usable(r[f])]
            cells.append(fmt_ci([m.get(key) for m in ms]) if any(key in m for m in ms) else "-")
        rows.append([LABEL[f]] + cells)
    L += table(rows, ["filter", "GPS dropout: max pos h err m", "GPS dropout: max tilt °", "mag interference: max heading err °",
                      "mag interference: max tilt °"]) + [""]

    L += ["## Caveats", "",
          "- Simulation only. The realistic IMU model (white noise 0.29 °/s, random turn-on gyro bias σ 0.5 °/s, Gauss-Markov "
          "bias wander, accel 0.1 m/s²) is a guess at hobby hardware; there's no vibration spectrum, no motor-induced mag "
          "interference, no temperature drift. Real flights are the real test.",
          "- PX4 flew every flight on its own EKF2 (live). The replayed EKF2 matches the live one to ≤0.06° after its first "
          "2 s, so its numbers are not inflated by replay; but EKF2 had the trajectory 'it caused', the others didn't - "
          "this matters only for closed-loop effects, which a replay can't capture for anyone.",
          "- Magnetic field: Gazebo's field is rotated onto the World Magnetic Model field EKF2 uses (EKF2 ignores the "
          "declination parameter when GPS is valid), so all filters share one north. The replayed filters get the field "
          "direction measured on the clean log against truth - slightly more information than EKF2's WMM lookup, which here "
          "equals the true field by construction.",
          "- Your EKF gets a 6 s ground dwell for calibration (gyro bias, attitude); Madgwick/Mahony get the same dwell's "
          "mean gyro (bias) and accel/mag (TRIAD attitude). EKF2 uses its own on-ground alignment from the same data.",
          "- Your EKF was developed on Gazebo recordings from the stock IMU, flown by your controller - none of these PX4 "
          "flights or the realistic-IMU model were used to tune it (held-out data). Madgwick/Mahony CV-tuned gains are "
          "chosen on other trials of these same scenarios - a mild advantage your EKF and EKF2 don't get.",
          "- 3 trials per condition: confidence intervals are wide; differences inside them are not established."]
    path = os.path.join(out_dir, "REPORT.md")
    open(path, "w", encoding="utf-8").write("\n".join(L) + "\n")
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(ROOT, "testing", "results", "benchmark_v2"))
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--data", default=DATA)
    ap.add_argument("--only", default=None, help="glob filter on recording names (debugging)")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    paths = sorted(p for p in glob.glob(os.path.join(a.data, "*__*__t*__*.csv")) if not p.endswith(".cal.csv"))
    if a.only:
        import fnmatch
        paths = [p for p in paths if fnmatch.fnmatch(os.path.basename(p), a.only)]
    cache_path = os.path.join(a.out, "runs.json")
    cache = json.load(open(cache_path)) if os.path.exists(cache_path) else {}
    todo = [p for p in paths if os.path.basename(p) not in cache]
    print(f"{len(paths)} recordings, {len(todo)} to run", flush=True)
    with Pool(a.jobs) as pool:
        for k, (p, out) in enumerate(pool.imap_unordered(task, todo), 1):
            cache[os.path.basename(p)] = out
            if k % 10 == 0 or k == len(todo):
                json.dump(cache, open(cache_path, "w"))
                print(f"  {k}/{len(todo)}", flush=True)
    runs = {k: v for k, v in cache.items() if k in {os.path.basename(p) for p in paths}}
    chosen = cv_select(runs)
    print("->", report(runs, chosen, a.out))


if __name__ == "__main__":
    main()
