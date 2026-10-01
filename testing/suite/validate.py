"""Tooling self-checks (EKF_TEST_PLAN.md s3.8, checks 1-3). Check 4 (determinism) is
compare.py on two run_suite outputs of the same code.

usage: python testing/suite/validate.py [--recording hover] [--out DIR]
Writes DIR/VALIDATION.md and DIR/fault_channels.png, exits nonzero if a check fails.
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import run_suite as RS  # noqa: E402
import faults as F  # noqa: E402
import metrics as M  # noqa: E402


def replay_with(rec, fault):
    cols, truth, cal = RS._get_rec(rec)
    ctx = RS.fault_context(rec, cols)
    fcols, cal_f, info = RS.split_cal(*fault.apply(RS.with_cal_rows(cols, cal, rec["remapped"]), ctx), rec["remapped"])
    return cols, fcols, cal_f, info, RS.replay(fcols, rec["remapped"], cal_f)


def check_reference(lines):
    rec = {"name": "ref_live1", "path": os.path.join(RS.DATA, RS.REF["file"]), "remapped": True, "t_end": 12.0}
    cols, _, _, _, run = replay_with(rec, F.by_key()["none"])
    raw_q = np.stack([cols["tqw"], cols["tqx"], cols["tqy"], cols["tqz"]], axis=1)
    err = np.abs(M.quat_err_deg(run["q"], raw_q))
    rms = np.sqrt((err ** 2).mean(0)).round(2)
    ok = list(rms) == [0.14, 0.12, 0.20]
    lines += ["## 1. Reference replay", "",
              f"`ref_sensors_live1.csv` (git show 33be40e), MAG_REMAPPED=1, first 12 s, all rows, raw truth column "
              f"(replay_ekf.py's convention): attitude RMS **{rms[0]:.2f} / {rms[1]:.2f} / {rms[2]:.2f} deg** "
              f"(expected 0.14 / 0.12 / 0.20) -> **{'PASS' if ok else 'FAIL'}**", ""]
    return ok


def check_faults(rec, lines, png):
    none = F.by_key()["none"]
    base_cols, base_f, base_cal, _, base_run = replay_with(rec, none)
    zero = F.Fault("gyro_bias", {"dps": 0.0}, 1)
    _, _, _, _, zrun = replay_with(rec, zero)
    same = all(np.array_equal(base_run[k], zrun[k]) for k in ("q", "pos", "vel", "P_att", "P_pos"))
    lines += ["## 2. Fault sanity", "",
              f"`gyro_bias(dps=0)` vs `none`: estimate and covariance traces bit-identical -> **{'PASS' if same else 'FAIL'}**", "",
              "Channels each fault changes (flight rows / calibration rows), vs `none`:", "",
              "| fault | flight channels changed | calibration rows changed | rows kept | gps_ok=0 rows |",
              "|---|---|---|---|---|"]
    cols, _, cal = RS._get_rec(rec)
    ctx = RS.fault_context(rec, cols)
    ref = RS.with_cal_rows(cols, cal, rec["remapped"])
    ok = same
    expected = {"gps": {"px", "py", "pz", "vx", "vy", "vz", "gps_ok"}, "gyro": {"gx", "gy", "gz"},
                "accel": {"ax", "ay", "az"}, "mag": {"mx", "my", "mz"}}
    allowed = {"none": set(), "gps_dropout": {"gps_ok"}, "gps_stale": expected["gps"], "gps_noise": expected["gps"],
               "gps_outliers": {"px", "py", "pz"}, "gps_rate": {"gps_ok"}, "gyro_bias": expected["gyro"],
               "gyro_drift": expected["gyro"], "accel_bias": expected["accel"], "mag_bias": expected["mag"],
               "mag_interference": expected["mag"], "imu_noise": expected["gyro"] | expected["accel"],
               "imu_spikes": expected["gyro"] | expected["accel"], "imu_dropouts": set(),
               "combined_realistic": expected["gps"] | expected["gyro"] | expected["accel"] | expected["mag"],
               "cal_ideal": set(), "imu_gap": set()}
    examples = {}
    for f in F.ALL_FAULTS:
        fc, info = f.apply(ref, ctx)
        n_cal = int(ref["is_cal"].sum())
        if "keep" in info:
            kept = int(info["keep"].sum())
            changed, cal_changed = set(), set()
        else:
            kept = len(ref["t"])
            changed = {k for k in fc if k not in ("is_cal",) and (k not in ref or not np.array_equal(fc[k][n_cal:], ref[k][n_cal:]))}
            if "gps_ok" in changed and np.all(fc["gps_ok"][n_cal:] == 1):
                changed.discard("gps_ok")
            cal_changed = {k for k in ref if k != "is_cal" and not np.array_equal(fc[k][:n_cal], ref[k][:n_cal])}
        bad = not changed <= allowed[f.name]
        ok &= not bad
        nofix = int((fc.get("gps_ok", np.ones(1))[n_cal:] == 0).sum()) if "gps_ok" in fc else 0
        lines.append(f"| {f.key} | {', '.join(sorted(changed)) or '-'}{' **UNEXPECTED**' if bad else ''} | "
                     f"{', '.join(sorted(cal_changed)) or '-'} | {kept}/{len(ref['t'])} | {nofix} |")
        examples.setdefault(f.name, (f, fc))
    lines.append("")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    names = [n for n in examples if n != "none"]
    fig, axs = plt.subplots(len(names), 1, figsize=(11, 2.1 * len(names)))
    n_cal = int(ref["is_cal"].sum())
    t = ref["t"][n_cal:] - ref["t"][n_cal]
    for ax, n in zip(axs, names):
        f, fc = examples[n]
        if "keep" in (info := f.apply(ref, ctx)[1]):
            k = info["keep"][n_cal:]
            ax.plot(t, np.cumsum(~k), lw=0.8); ax.set_ylabel("dropped\nrows", fontsize=7)
        else:
            chans = sorted(k for k in allowed[n] if k in fc and k != "gps_ok")[:6]
            for ch in chans:
                ax.plot(t, fc[ch][n_cal:] - ref[ch][n_cal:], lw=0.6, label=ch)
            if "gps_ok" in fc:
                ax.plot(t, 1 - fc["gps_ok"][n_cal:], lw=0.8, color="k", label="no fix")
            ax.legend(fontsize=6, ncol=7, loc="upper right")
            ax.set_ylabel("delta", fontsize=7)
        ax.set_title(f.key, fontsize=8)
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(png, dpi=60)
    plt.close(fig)
    lines += [f"Per-fault channel deltas (faulted - clean, recording `{rec['name']}`): `{os.path.basename(png)}`", ""]
    return ok


def check_nees(lines):
    rng = np.random.default_rng(0)
    A = rng.normal(size=(3, 3))
    P = A @ A.T + 0.1 * np.eye(3)
    e = rng.multivariate_normal(np.zeros(3), P, size=10000)
    n = M._nees(e, np.broadcast_to(P, (10000, 3, 3)).copy())
    mean = float(n.mean() / 3)
    out = float(np.mean((n < M.CHI2_3_95[0]) | (n > M.CHI2_3_95[1])))
    ok = abs(mean - 1) <= 0.05
    lines += ["## 3. NEES metric sanity", "",
              f"10k errors drawn from N(0, P) for a random SPD 3x3 P: mean NEES/dof = **{mean:.3f}** (need 1 +/- 0.05), "
              f"fraction outside chi2(3) 95% bounds = {out:.3f} (expect ~0.05) -> **{'PASS' if ok else 'FAIL'}**", ""]
    return ok


def truth_lag(cols, lags=np.arange(-0.04, 0.081, 0.002)):
    """Pose-message latency vs the IMU: the lag that best aligns the body rate implied by
    successive pose updates with the recorded gyro. Needs a recording with rotation."""
    t = cols["t"]
    tq = np.stack([cols["tqw"], cols["tqx"], cols["tqy"], cols["tqz"]], axis=1)
    ku = np.flatnonzero(np.r_[True, np.any(np.diff(tq, axis=0) != 0, axis=1)])
    q, tu = tq[ku], t[ku]
    dq = M._qmul(q[:-1] * np.array([1, -1, -1, -1]), q[1:])
    dq = np.where(dq[:, :1] < 0, -dq, dq)
    w = 2 * dq[:, 1:] / np.maximum(np.diff(tu), 1e-6)[:, None]
    tm = 0.5 * (tu[1:] + tu[:-1])
    g = np.stack([cols["gx"], cols["gy"], cols["gz"]], axis=1)
    if np.abs(g).max() < 0.2:
        return None, None
    errs = [np.mean(np.sum((w - np.stack([np.interp(tm - lag, t, g[:, j]) for j in range(3)], axis=1)) ** 2, 1))
            for lag in lags]
    k = int(np.argmin(errs))
    return float(-lags[k]), float(np.sqrt(errs[k]))


def check_recordings(names, lines):
    import missions as MS
    lines += ["## Recordings", "",
              "| recording | dur s | IMU Hz | pose Hz | GPS Hz | ended cleanly | max true tilt | min z airborne | "
              "tilt mean/max (profile) | speed mean/max | g*tan(tilt) mean/max | yaw rate max | path err mean/max | verdict |",
              "|---|" + "---|" * 13]
    ok = True
    lag_rows = []
    for rec in RS.recording_list(names):
        if rec["name"] not in MS.MISSIONS:
            continue
        mi = MS.MISSIONS[rec["name"]]
        cols = RS.load_recording(rec["path"])
        tr = M.truth_arrays(cols)
        t = cols["t"] - cols["t"][0]
        eu = M.quat_to_euler_deg(tr["q"])
        tilt = np.degrees(np.arccos(np.clip(1 - 2 * (tr["q"][:, 1] ** 2 + tr["q"][:, 2] ** 2), -1, 1)))
        z = tr["pos"][:, 2]
        prof = (cols["mt"] >= mi.profile_start) & (cols["mt"] <= mi.profile_start + mi.duration)
        up = np.flatnonzero(z > 0.3)
        air = (t >= t[up[0]] + 2) if len(up) else np.zeros(len(t), bool)
        spd = np.hypot(tr["vel"][:, 0], tr["vel"][:, 1])
        dt = np.median(np.diff(t))
        acc = 9.80665 * np.tan(np.radians(tilt))  # horizontal accel implied by true tilt (pose-rate diffs are too noisy)
        yawr = np.abs(np.gradient(np.unwrap(np.radians(eu[:, 2])), t))
        ideal = np.array([mi.ideal(mt)[0] for mt in cols["mt"][prof]])
        perr = np.hypot(*(tr["pos"][prof, :2] - ideal[:, :2]).T)
        log = os.path.join(RS.DATA, "logs", f"rec_{rec['name']}.log")
        ended = os.path.exists(log) and "MISSION_END" in open(log).read()
        gps_hz = np.sum(np.diff(cols["px"]) != 0) / t[-1]
        zmin = z[air & prof].min() if (air & prof).any() else float("nan")
        crashed = tilt[air].max() > 45 or (rec["name"] != "takeoff_land" and zmin < 0.3)
        good = ended and not crashed and len(t) / t[-1] > 200
        ok &= good
        lines.append(f"| {rec['name']} | {t[-1]:.0f} | {len(t)/t[-1]:.0f} | {tr['pose_rate_hz']:.0f} | {gps_hz:.0f} | {ended} | "
                     f"{tilt[air].max():.1f} | {zmin:.2f} | {tilt[prof].mean():.1f}/{tilt[prof].max():.1f} | "
                     f"{spd[prof].mean():.2f}/{spd[prof].max():.2f} | {acc[prof].mean():.2f}/{acc[prof].max():.2f} | "
                     f"{np.degrees(yawr[prof].max()):.0f} | {perr.mean():.2f}/{perr.max():.2f} | {'OK' if good else 'BAD'} |")
        lag, res = truth_lag(cols)
        if lag is not None:
            lag_rows.append(f"- {rec['name']}: best-fit pose latency {lag*1000:.0f} ms (residual {res:.3f} rad/s)")
    lines += ["", "Units: deg, m/s, m/s^2, deg/s, m. Speed/accel/tilt are Gazebo truth during the mission profile "
              "(not the settle/end-hold); path err = truth vs the mission's ideal path (no lead).", "",
              "Truth pose latency vs IMU (from aligning pose-derived body rate with the gyro):", ""] + lag_rows + [""]
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--recording", default="hover")
    ap.add_argument("--recordings", nargs="*", help="only validate these recordings (no names = all)")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    if a.recordings is not None:
        lines = ["# Recording validation", ""]
        ok = check_recordings(a.recordings or None, lines)
        with open(os.path.join(a.out, "RECORDINGS.md"), "w", encoding="utf-8") as f:
            f.write("\n".join(lines))
        print("\n".join(lines))
        sys.exit(0 if ok else 1)
    rec = [r for r in RS.recording_list([a.recording])][0]
    lines = ["# Suite tooling validation (EKF_TEST_PLAN.md s3.8)", ""]
    ok = check_reference(lines)
    ok &= check_faults(rec, lines, os.path.join(a.out, "fault_channels.png"))
    ok &= check_nees(lines)
    lines += ["## 4. Determinism", "", "See `compare.py` of two full runs (recorded in the report).", ""]
    with open(os.path.join(a.out, "VALIDATION.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print("\n".join(lines))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
