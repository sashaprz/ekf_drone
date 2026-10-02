"""Tier A - offline replay of every (recording x fault) through DroneEKF (EKF_TEST_PLAN.md s3.6).

usage: python testing/suite/run_suite.py [--missions hover box ...] [--faults none gps_dropout ...]
                                         [--out testing/results/<dir>] [--jobs N] [--no-plots] [--full-patrol]

Recordings: testing/data/flight_<mission>.csv (+ .cal.csv sidecar), flown with ORACLE=1
by record.ps1. Also the 2026-09-30 reference flight testing/data/ref_sensors_live1.csv
(mission "ref_live1", first 12 s, mag remapped) - fault "none" only, as an anchor for
the 0.14/0.12/0.20 deg numbers. --faults matches a fault's name or its full key.

Writes <out>/results.json (deterministic - no timestamps, so two runs on the same code
and data are byte-identical), run_info.json (date, runtime), REPORT.md, plots/.
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "testing"))
import replay_ekf  # noqa: E402  (also puts "state estimation" on sys.path)
import faults as F  # noqa: E402
import metrics as M  # noqa: E402
import missions as MS  # noqa: E402

SUITE_VERSION = 2  # v2: mag faults additive (bridge no longer normalizes mag)
DATA = os.path.join(ROOT, "testing", "data")
REF = {"name": "ref_live1", "file": "ref_sensors_live1.csv", "remapped": True, "t_end": 12.0, "dyn_from_t0": 4.0}
# patrol_long is 10 min (150k rows, ~70 s per replay): by default it runs only the faults
# that target slow drift / long-run covariance health; --full-patrol runs everything
PATROL_FAULTS = ["none", "gps_dropout(duration_s=30)", "gps_noise", "gyro_drift(dps=0.5,over_s=60)",
                 "accel_bias(axis=x,ms2=0.05)", "mag_bias(frac=0.05)", "imu_noise", "imu_spikes",
                 "combined_realistic", "cal_ideal"]
KEY_METRICS = ["tilt_rms_deg", "tilt_max_deg", "yaw_rms_deg", "pos_h_rms_m", "nees_att", "nees_vel", "nees_pos",
               "tilt_drift_deg_s", "recovery_s", "gps_resets"]


# ---- recordings --------------------------------------------------------------------
# Recordings cut short because the flight itself went bad, even in ORACLE mode: name ->
# t_end in seconds from the first row. (Was yaw_steps/yaw_spin until the cascade.py
# world-frame attitude-error fix, 2026-09-30; both re-recorded clean since.)
TRUNCATE = {}


def recording_list(names=None, extra=()):
    recs = []
    for name, m in MS.MISSIONS.items():
        path = os.path.join(DATA, f"flight_{name}.csv")
        if os.path.exists(path) and (not names or name in names):
            recs.append({"name": name, "path": path, "remapped": False, "t_end": TRUNCATE.get(name)})
    ref = os.path.join(DATA, REF["file"])
    if os.path.exists(ref) and (not names or REF["name"] in names):
        recs.append({"name": REF["name"], "path": ref, "remapped": True, "t_end": REF["t_end"]})
    # extra recordings (e.g. live trials that crashed - replay reproduces them): scored on
    # fault "none" only, named after the file
    for path in extra:
        recs.append({"name": os.path.splitext(os.path.basename(path))[0], "path": os.path.abspath(path),
                     "remapped": False, "t_end": None})
    return recs


def sha1(path):
    h = hashlib.sha1()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()[:12]


def load_recording(path, t_end=None):
    with open(path) as f:
        header = f.readline().strip().split(",")
    arr = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    cols = {k: arr[:, j].copy() for j, k in enumerate(header)}
    if t_end is not None:
        keep = cols["t"] - cols["t"][0] <= t_end
        cols = {k: v[keep] for k, v in cols.items()}
    return cols


_CACHE = {}


def _get_rec(rec):
    key = rec["path"]
    if key not in _CACHE:
        if len(_CACHE) >= 2:
            _CACHE.pop(next(iter(_CACHE)))
        cols = load_recording(rec["path"], rec["t_end"])
        _CACHE[key] = (cols, M.truth_arrays(cols), replay_ekf.load_cal(rec["path"]))
    return _CACHE[key]


def with_cal_rows(cols, cal, remapped):
    """Prepend the calibration samples as rows (is_cal=1) so faults can corrupt them too.
    Recordings without a .cal.csv calibrate on row 0 repeated 200x (the old replay
    behaviour) - reproduced here explicitly. Non-sensor columns copy row 0."""
    n_cal = len(cal["gyro"]) if cal else 200
    out = {}
    for k, v in cols.items():
        out[k] = np.r_[np.full(n_cal, v[0]), v]
    if cal:
        g, a, m = np.array(cal["gyro"]), np.array(cal["accel"]), np.array(cal["mag"])
        if remapped:  # store in the csv's (remapped) convention, ArrayReplay undoes it
            m = np.stack([-m[:, 1], m[:, 0], -m[:, 2]], axis=1)
        for j, ax in enumerate("xyz"):
            out["g" + ax][:n_cal], out["a" + ax][:n_cal], out["m" + ax][:n_cal] = g[:, j], a[:, j], m[:, j]
    out["t"][:n_cal] = cols["t"][0] - 0.004 * (n_cal - np.arange(n_cal))
    out["is_cal"] = np.r_[np.ones(n_cal), np.zeros(len(cols["t"]))]
    return out


def split_cal(fcols, info, remapped):
    """Inverse of with_cal_rows after a fault: -> (flight cols, cal dict for ArrayReplay, info)."""
    n = int(fcols["is_cal"].sum())
    g = np.stack([fcols["gx"][:n], fcols["gy"][:n], fcols["gz"][:n]], axis=1)
    a = np.stack([fcols["ax"][:n], fcols["ay"][:n], fcols["az"][:n]], axis=1)
    m = np.stack([fcols["mx"][:n], fcols["my"][:n], fcols["mz"][:n]], axis=1)
    if remapped:
        m = np.stack([m[:, 1], -m[:, 0], -m[:, 2]], axis=1)
    cal = {"gyro": [tuple(r) for r in g.tolist()], "accel": [tuple(r) for r in a.tolist()],
           "mag": [tuple(r) for r in m.tolist()]}
    flight = {k: v[n:] for k, v in fcols.items() if k != "is_cal"}
    info = dict(info)
    for k in ("gyro_bias", "accel_bias", "gps_outlier", "keep"):
        if k in info:
            info[k] = info[k][n:]
    return flight, cal, info


def fault_context(rec, cols):
    t0 = cols["t"][0]
    if rec["name"] in MS.MISSIONS and "mt" in cols:
        mi = MS.MISSIONS[rec["name"]]
        k = np.searchsorted(cols["mt"], mi.profile_start + mi.dynamic_start)
        dyn_t = cols["t"][min(k, len(cols["t"]) - 1)]
        if rec.get("t_end") is not None:  # truncated: windows start at the profile start instead
            dyn_t = cols["t"][min(np.searchsorted(cols["mt"], mi.profile_start), len(cols["t"]) - 1)]
    else:
        dyn_t = t0 + REF["dyn_from_t0"]
    return {"t0": t0, "dyn_t": dyn_t}


# ---- one replay ----------------------------------------------------------------------
def replay(cols, remapped, cal):
    n = len(cols["t"])
    out = {"t": cols["t"], "q": np.zeros((n, 4)), "pos": np.zeros((n, 3)), "vel": np.zeros((n, 3)),
           "P_att": np.zeros((n, 3, 3)), "P_vel": np.zeros((n, 3, 3)), "P_pos": np.zeros((n, 3, 3)),
           "bg": np.zeros((n, 3)), "ba": np.zeros((n, 3)), "bm": np.zeros((n, 3)), "nan": False, "error": None}
    events = []
    last = {"n": 0, "rej": 0}

    def on_step(i, ekf, st):
        out["q"][i] = st["quat"]; out["pos"][i] = st["pos"]; out["vel"][i] = st["vel"]
        P = ekf.P
        out["P_att"][i] = P[0:3, 0:3]; out["P_vel"][i] = P[6:9, 6:9]; out["P_pos"][i] = P[9:12, 9:12]
        out["bg"][i] = (ekf.bias_x, ekf.bias_y, ekf.bias_z)
        out["ba"][i] = (ekf.accel_bias_x, ekf.accel_bias_y, ekf.accel_bias_z)
        out["bm"][i] = (ekf.mag_bias_x, ekf.mag_bias_y, ekf.mag_bias_z)
        s = ekf.stats
        nn = s["gps_updates"] + s["gps_rejected"]
        if nn != last["n"]:
            events.append((i, s["gps_rejected"] > last["rej"]))
            last["n"], last["rej"] = nn, s["gps_rejected"]

    src = replay_ekf.ArrayReplay({k: v.tolist() for k, v in cols.items()}, remapped=remapped, cal=cal)
    ekf = None
    with np.errstate(all="ignore"):
        try:
            ekf = replay_ekf.run_replay(src, cols["t"].tolist(), on_step, quiet=True)
        except Exception as e:  # a crash inside the filter is a result, not a suite bug
            out["error"] = f"{type(e).__name__}: {e}"
    out["stats"] = dict(ekf.stats) if ekf is not None else {}
    out["gps_events"] = np.array(events, dtype=float).reshape(-1, 2)
    finite = all(np.isfinite(out[k]).all() for k in ("q", "pos", "vel", "P_att", "P_vel", "P_pos"))
    if ekf is not None:
        finite = finite and np.isfinite(ekf.P).all()
    out["nan"] = not finite
    return out


def run_task(task):
    rec, fkey, p95, plot_dir = task
    fault = F.by_key()[fkey]
    cols, truth, cal = _get_rec(rec)
    ctx = fault_context(rec, cols)
    fcols, cal_f, info = split_cal(*fault.apply(with_cal_rows(cols, cal, rec["remapped"]), ctx), rec["remapped"])
    if "keep" in info:
        truth = {k: (v[info["keep"]] if isinstance(v, np.ndarray) and len(v) == len(info["keep"]) else v)
                 for k, v in truth.items()}
    run = replay(fcols, rec["remapped"], cal_f)
    m = M.compute(run, truth, info, fault, p95)
    grades, overall = M.grade(m, fault)
    res = {"mission": rec["name"], "fault": fault.name, "fault_key": fault.key, "params": fault.params,
           "seed": fault.seed, "metrics": _clean(m), "grades": grades, "overall": overall,
           "hypotheses": fault.hypotheses}
    if fkey == "none":
        res["_p95"] = {"tilt": m.get("tilt_p95_deg", 0.0), "yaw": m.get("yaw_p95_deg", 0.0),
                       "pos_h": m.get("pos_h_p95_m", 0.0)}
    if plot_dir:
        import plots
        plots.plot_run(os.path.join(plot_dir, f"{rec['name']}__{fault.key}.png"), run, truth, info,
                       f"{rec['name']} / {fault.key}  [{overall}]")
    return res


def _clean(d):
    out = {}
    for k, v in d.items():
        if isinstance(v, (np.floating, float)):
            v = float(v)
            out[k] = None if not np.isfinite(v) else round(v, 6)
        elif isinstance(v, (np.integer,)):
            out[k] = int(v)
        elif isinstance(v, list):
            out[k] = [round(float(x), 6) for x in v]
        else:
            out[k] = v
    return out


# ---- output --------------------------------------------------------------------------
def git_meta():
    def g(*a):
        try:
            return subprocess.run(["git", *a], cwd=ROOT, capture_output=True, text=True).stdout.strip()
        except OSError:
            return ""
    dirty = bool(g("status", "--porcelain", "--untracked-files=no"))
    files = {p: sha1(os.path.join(ROOT, p)) for p in ("state estimation/FINAL_gps.py", "state estimation/calibration.py")}
    return {"git": g("rev-parse", "--short", "HEAD"), "dirty": dirty, "filter_sha1": files}


SHORT = {"none": "none", "gps_dropout": "drop", "gps_stale": "stale", "gps_noise": "gpsN", "gps_outliers": "outl",
         "gps_rate": "1Hz", "gyro_bias": "gb", "gyro_drift": "gdrift", "accel_bias": "ab", "mag_bias": "mb",
         "mag_interference": "mint", "imu_noise": "imuN", "imu_spikes": "spike", "imu_dropouts": "idrop",
         "combined_realistic": "REAL", "cal_ideal": "calIdeal", "imu_gap": "gap", "gps_latency": "lat"}


def short_label(f):
    p = f.params
    s = SHORT[f.name]
    if f.name in ("gps_dropout", "gps_stale"):
        s += str(p["duration_s"])
    elif f.name == "gyro_bias":
        s += f"{p['dps']:g}" + ("ir" if p.get("from_cal") is False else "")
    elif f.name == "accel_bias":
        s += f"{p['axis']}{p['ms2']:g}" + ("ir" if p.get("from_cal") is False else "")
    elif f.name == "mag_bias":
        s += f"{p['frac']:g}"
    elif f.name == "mag_interference":
        s += f"{p['amp']:g}"
    elif f.name == "gps_latency":
        s += str(p["latency_ms"])
    elif f.name == "imu_gap":
        s += f"{p['gap_s']:g}"
    return s


def grid_text(results, missions, fault_list, md=False):
    by = {(r["mission"], r["fault_key"]): r for r in results}
    labels = [short_label(f) for f in fault_list]
    code = {"PASS": "P", "WARN": "W", "FAIL": "F"}
    lines = []
    if md:
        lines.append("| mission | " + " | ".join(labels) + " |")
        lines.append("|---|" + "---|" * len(labels))
    else:
        w = max(len(m) for m in missions) + 1
        lines.append(" " * w + " ".join(f"{l:>7s}" for l in labels))
    for m in missions:
        cells = []
        for f in fault_list:
            r = by.get((m, f.key))
            cells.append("" if r is None else (r["overall"] if md else code[r["overall"]]))
        if md:
            lines.append(f"| {m} | " + " | ".join(cells) + " |")
        else:
            lines.append(f"{m:<{w}s}" + " ".join(f"{c:>7s}" for c in cells))
    return "\n".join(lines)


def fmt(v, nd=2):
    if v is None:
        return "never"
    if isinstance(v, bool):
        return str(v)
    if isinstance(v, (int,)):
        return str(v)
    if isinstance(v, float):
        return f"{v:.{nd}f}" if abs(v) < 1000 else f"{v:.0f}"
    return str(v)


def write_report(path, meta, results, missions, fault_list):
    counts = {g: sum(r["overall"] == g for r in results) for g in ("PASS", "WARN", "FAIL")}
    L = [f"# EKF stress-test suite - results", "",
         f"code `{meta['git']}`{' (dirty)' if meta['dirty'] else ''}, filter sha1 "
         + ", ".join(f"`{k.split('/')[-1]}` {v}" for k, v in meta["filter_sha1"].items()),
         f"suite v{meta['suite_version']}, {len(results)} runs: **{counts['PASS']} PASS / {counts['WARN']} WARN / "
         f"{counts['FAIL']} FAIL**", "",
         "Auto-generated by `run_suite.py`. Thresholds: `metrics.THRESHOLDS`. Fault labels: "
         + ", ".join(f"`{short_label(f)}`=`{f.key}`" for f in fault_list), "", "## Summary grid", "",
         grid_text(results, missions, fault_list, md=True), "", "## Per-mission metrics", ""]
    cols = ["tilt_rms_deg", "tilt_max_deg", "yaw_rms_deg", "pos_h_rms_m", "vel_h_rms_ms", "nees_att", "nees_vel",
            "nees_pos", "tilt_drift_deg_s", "recovery_s", "gps_rejected", "gps_resets", "mag_rejected"]
    for mname in missions:
        L += [f"### {mname}", "", "| fault | overall | " + " | ".join(cols) + " | failing |",
              "|---|---|" + "---|" * len(cols) + "---|"]
        for r in results:
            if r["mission"] != mname:
                continue
            mm = r["metrics"]
            bad = [k for k, g in r["grades"].items() if g != "PASS"]
            L.append(f"| {r['fault_key']} | {r['overall']} | " + " | ".join(fmt(mm.get(c)) if c in mm else "" for c in cols)
                     + f" | {', '.join(f'{k}={r['grades'][k]}' for k in bad)} |")
        L.append("")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--missions", nargs="*")
    ap.add_argument("--faults", nargs="*")
    ap.add_argument("--out")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--full-patrol", action="store_true", help="run every fault on patrol_long, not PATROL_FAULTS")
    ap.add_argument("--extra", nargs="*", default=[], help="extra recordings (csv + .cal.csv), fault 'none' only")
    ap.add_argument("--no-plots", action="store_true")
    a = ap.parse_args()

    meta = git_meta()
    meta.update(suite_version=SUITE_VERSION, thresholds=M.THRESHOLDS, truth_lag_s=M.TRUTH_LAG_S)
    out = a.out or os.path.join(ROOT, "testing", "results", time.strftime("%Y%m%d_%H%M%S") + "_" + meta["git"]
                                + ("_dirty" if meta["dirty"] else ""))
    plot_dir = None if a.no_plots else os.path.join(out, "plots")
    os.makedirs(plot_dir or out, exist_ok=True)

    recs = recording_list(a.missions, a.extra)
    if not recs:
        sys.exit(f"no recordings found in {DATA}")
    meta["recordings"] = {r["name"]: sha1(r["path"]) for r in recs}
    fl = [f for f in F.ALL_FAULTS if not a.faults or f.name in a.faults or f.key in a.faults]
    if not any(f.key == "none" for f in fl):
        fl = [F.by_key()["none"]] + fl  # recovery thresholds need the no-fault run

    t_start = time.time()
    with Pool(a.jobs) as pool:
        base = pool.map(run_task, [(r, "none", None, plot_dir) for r in recs])
        p95 = {r["mission"]: r.pop("_p95") for r in base}
        tasks = [(r, f.key, p95[r["name"]], plot_dir) for r in recs if r["name"] in MS.MISSIONS
                 for f in fl if f.key != "none"
                 and (r["name"] != "patrol_long" or a.full_patrol or f.key in PATROL_FAULTS)]
        tasks.sort(key=lambda tk: tk[0]["name"] != "patrol_long")  # longest first
        rest = pool.map(run_task, tasks, chunksize=1)
    order = {f.key: i for i, f in enumerate(F.ALL_FAULTS)}
    mnames = [r["name"] for r in recs]
    results = sorted(base + rest, key=lambda r: (mnames.index(r["mission"]), order[r["fault_key"]]))
    elapsed = time.time() - t_start

    with open(os.path.join(out, "results.json"), "w") as f:
        json.dump({"meta": meta, "runs": results}, f, indent=1, sort_keys=True)
    with open(os.path.join(out, "run_info.json"), "w") as f:
        json.dump({"date": time.strftime("%Y-%m-%d %H:%M:%S"), "elapsed_s": round(elapsed, 1), "jobs": a.jobs,
                   "numpy": np.__version__, "python": sys.version.split()[0]}, f, indent=1)
    write_report(os.path.join(out, "REPORT.md"), meta, results, mnames, fl)
    print(grid_text(results, mnames, fl))
    c = {g: sum(r["overall"] == g for r in results) for g in ("PASS", "WARN", "FAIL")}
    print(f"\n{len(results)} runs in {elapsed:.0f}s: {c['PASS']} PASS / {c['WARN']} WARN / {c['FAIL']} FAIL -> {out}")


if __name__ == "__main__":
    main()
