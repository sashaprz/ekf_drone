"""Error / consistency / recovery metrics and grading for one replay (EKF_TEST_PLAN.md s3.4-3.5).

Conventions (stated again in the report):
  - attitude error = body-frame small-angle vector 2*vec(q_true^-1 (x) q_est), deg - the
    same as replay_ekf.quat_err_deg. "tilt" = hypot(x, y), "yaw" = |z|.
  - truth pose arrives at ~50 Hz (Gazebo /world/default/pose/info) vs 250 Hz IMU, so the
    raw column is a staircase. truth_arrays() rebuilds it at every row by interpolating
    between pose updates (nlerp for attitude), shifted by TRUTH_LAG_S.
  - velocity truth = finite difference of the pose updates, 0.1 s moving average.
  - position truth is relative to the spawn point (the EKF's origin = first GPS fix).
  - scoring window: from 2 s after takeoff (true z > 0.3 m above spawn) to the end.
"""
import math
import numpy as np

# ---- grading thresholds - the owner may retune; keep them all here ---------------
THRESHOLDS = {
    # metric: (PASS if value < a, WARN if value < b, else FAIL)
    "tilt_rms_deg": (1.0, 3.0),
    "tilt_max_deg": (3.0, 8.0),
    "yaw_rms_deg": (2.0, 5.0),
    "pos_h_rms_m": (0.3, 1.0),                 # runs without a GPS fault
    "pos_h_rms_noise_x_sigma": (1.5, 3.0),     # GPS-noise runs: multiples of the injected sigma
    "gps_noise_sigma_h_m": 1.5,
    "tilt_drift_deg_s": (0.5, 2.0),            # |slope| during a GPS outage
    "recovery_s": (3.0, 10.0),
    "nees_pass": (0.3, 3.0),                   # mean NEES / dof inside -> PASS
    "nees_warn": (0.1, 10.0),                  # inside -> WARN, outside -> FAIL
    "gps_resets": (1, 2),                      # 0 PASS, 1 WARN, >1 FAIL (no-GPS-fault runs)
}
# recovery = time after the fault ends until the error stays below
# max(RECOVERY_FACTOR * no-fault p95, floor) for RECOVERY_HOLD_S
RECOVERY_FACTOR = 1.5
RECOVERY_FLOOR = {"tilt": 0.2, "yaw": 0.3, "pos_h": 0.2}
RECOVERY_HOLD_S = 1.0
TRANSIENT_WINDOW_S = 10.0
TRUTH_LAG_S = 0.0         # pose-message latency relative to IMU - see truth_arrays()
TAKEOFF_Z = 0.3
SCORE_AFTER_TAKEOFF_S = 2.0
CHI2_3_95 = (0.2158, 9.3484)  # two-sided 95% bounds of chi2 with 3 dof

GRADE_ORDER = {"PASS": 0, "WARN": 1, "FAIL": 2}


# ---- truth -----------------------------------------------------------------------
def _qmul(a, b):
    w1, x1, y1, z1 = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    w2, x2, y2, z2 = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2], axis=-1)


def quat_err_deg(q_est, q_true):
    # vectorized replay_ekf.quat_err_deg: rows of [w,x,y,z]
    qc = q_true * np.array([1.0, -1.0, -1.0, -1.0])
    e = _qmul(qc, q_est)
    e = np.where(e[:, :1] < 0, -e, e)
    return np.degrees(2 * e[:, 1:])


def quat_to_euler_deg(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.degrees(np.stack([roll, pitch, yaw], axis=1))


def truth_arrays(cols, lag=TRUTH_LAG_S):
    """Ground truth at every row: dict(q (N,4), pos (N,3) rel. to spawn, vel (N,3))."""
    t = cols["t"]
    tq = np.stack([cols["tqw"], cols["tqx"], cols["tqy"], cols["tqz"]], axis=1)
    tp = np.stack([cols["tpx"], cols["tpy"], cols["tpz"]], axis=1)
    new = np.r_[True, np.any(np.diff(tq, axis=0) != 0, axis=1) | np.any(np.diff(tp, axis=0) != 0, axis=1)]
    ku = np.flatnonzero(new)
    # recordings since 2026-10-03 carry the pose message's own stamp (tt, same clock as t):
    # place each pose where it was measured, not where it was logged
    tu = cols["tt"][ku] if "tt" in cols else t[ku] - lag
    qu, pu = tq[ku].copy(), tp[ku]
    for k in range(1, len(qu)):  # keep consecutive samples in the same hemisphere for nlerp
        if np.dot(qu[k], qu[k - 1]) < 0:
            qu[k] = -qu[k]
    idx = np.clip(np.searchsorted(tu, t, side="right") - 1, 0, len(tu) - 2) if len(tu) > 1 else np.zeros(len(t), int)
    if len(tu) > 1:
        f = np.clip((t - tu[idx]) / np.maximum(tu[idx + 1] - tu[idx], 1e-9), 0, 1)[:, None]
        q = qu[idx] * (1 - f) + qu[idx + 1] * f
        q /= np.linalg.norm(q, axis=1, keepdims=True)
        pos = np.stack([np.interp(t, tu, pu[:, j]) for j in range(3)], axis=1)
        dtu = np.maximum(np.diff(tu), 1e-6)
        vu = np.diff(pu, axis=0) / dtu[:, None]
        tm = 0.5 * (tu[1:] + tu[:-1])
        vel = np.stack([np.interp(t, tm, vu[:, j]) for j in range(3)], axis=1)
        dt_row = np.median(np.diff(t)) if len(t) > 1 else 0.004
        w = max(1, int(round(0.1 / dt_row)))
        kern = np.ones(w) / w
        vel = np.stack([np.convolve(np.pad(vel[:, j], (w // 2, w - 1 - w // 2), mode="edge"), kern, "valid")
                        for j in range(3)], axis=1)
    else:
        q, pos, vel = tq.copy(), tp.copy(), np.zeros_like(tp)
    return {"q": q, "pos": pos - tp[0], "vel": vel, "pose_rate_hz": len(ku) / max(t[-1] - t[0], 1e-9)}


# ---- metric helpers ----------------------------------------------------------------
def _stats(x, prefix, unit):
    if len(x) == 0:
        return {}
    return {f"{prefix}_rms_{unit}": float(np.sqrt(np.mean(x ** 2))),
            f"{prefix}_p95_{unit}": float(np.percentile(np.abs(x), 95)),
            f"{prefix}_max_{unit}": float(np.max(np.abs(x)))}


def _nees(e, P):
    # e (N,3), P (N,3,3) -> per-sample NEES
    try:
        sol = np.linalg.solve(P, e[:, :, None])[:, :, 0]
    except np.linalg.LinAlgError:
        return np.full(len(e), np.inf)
    return np.einsum("ij,ij->i", e, sol)


def _slope(t, y):
    if len(t) < 10:
        return None
    return float(np.polyfit(t - t[0], y, 1)[0])


def _recovery(t, y, t_end, thr):
    """seconds after t_end until y stays below thr for RECOVERY_HOLD_S; None = never."""
    after = t >= t_end
    ta, ya = t[after], y[after]
    if len(ta) == 0:
        return None
    start = None
    for k in range(len(ta)):
        if ya[k] < thr:
            if start is None:
                start = ta[k]
            if ta[k] - start >= RECOVERY_HOLD_S:
                return float(start - t_end)
        else:
            start = None
    # still below threshold when the data ends (run shorter than the hold): count it
    return None if start is None else float(start - t_end)


def compute(run, truth, info, fault, nofault_p95=None):
    """run: dict of arrays from run_suite (t, q, pos, vel, P_att, P_vel, P_pos, bg, ba, bm,
    nan, error, stats, gps_events). Returns a flat metrics dict."""
    t = run["t"]
    m = {"nan": bool(run["nan"]), "error": run.get("error")}
    m.update({k: int(v) for k, v in run["stats"].items()})
    if run.get("error") and len(t) == 0:
        return m

    z = truth["pos"][:, 2]
    up = np.flatnonzero(z > TAKEOFF_Z)
    t_takeoff = t[up[0]] if len(up) else t[0]
    w = t >= t_takeoff + SCORE_AFTER_TAKEOFF_S
    m["t_takeoff_s"] = float(t_takeoff - t[0])
    m["scored_s"] = float(t[w][-1] - t[w][0]) if w.any() else 0.0

    att = quat_err_deg(run["q"], truth["q"])
    tilt = np.hypot(att[:, 0], att[:, 1])
    yaw = np.abs(att[:, 2])
    perr = run["pos"] - truth["pos"]
    verr = run["vel"] - truth["vel"]
    pos_h, pos_v = np.hypot(perr[:, 0], perr[:, 1]), np.abs(perr[:, 2])
    vel_h, vel_v = np.hypot(verr[:, 0], verr[:, 1]), np.abs(verr[:, 2])

    for j, ax in enumerate(("roll", "pitch")):
        m[f"{ax}_rms_deg"] = float(np.sqrt(np.mean(att[w, j] ** 2)))
    m.update(_stats(tilt[w], "tilt", "deg"))
    m.update(_stats(yaw[w], "yaw", "deg"))
    m.update(_stats(pos_h[w], "pos_h", "m"))
    m.update(_stats(pos_v[w], "pos_v", "m"))
    m.update(_stats(vel_h[w], "vel_h", "ms"))
    m.update(_stats(vel_v[w], "vel_v", "ms"))
    m["tilt_mean_deg"] = float(np.mean(tilt[w]))
    m["yaw_mean_signed_deg"] = float(np.mean(att[w, 2]))

    # consistency: NEES / dof
    for name, e, P in (("att", np.radians(att), run["P_att"]), ("vel", verr, run["P_vel"]), ("pos", perr, run["P_pos"])):
        n = _nees(e[w], P[w])
        m[f"nees_{name}"] = float(np.mean(n) / 3.0)
        m[f"nees_{name}_out95"] = float(np.mean((n < CHI2_3_95[0]) | (n > CHI2_3_95[1])))
    sig = np.degrees(np.sqrt(np.maximum(np.diagonal(run["P_att"][w], axis1=1, axis2=2), 0)))
    for j, ax in enumerate(("roll", "pitch", "yaw")):
        m[f"sigma_{ax}_deg_median"] = float(np.median(sig[:, j]))
        m[f"{ax}_mean_signed_deg"] = float(np.mean(att[w, j]))

    # fault window
    win = info.get("window")
    if win is not None:
        ws, we = win
        inw = (t >= ws) & (t < we)
        m["window_s"] = [float(ws - t[0]), float(we - t[0])]
        if inw.sum() > 1:
            k_end = np.flatnonzero(inw)[-1]
            m["tilt_at_end_deg"] = float(tilt[k_end])
            m["yaw_at_end_deg"] = float(yaw[k_end])
            m["pos_h_at_end_m"] = float(pos_h[k_end])
            m["tilt_drift_deg_s"] = _slope(t[inw], tilt[inw])
            m["yaw_drift_deg_s"] = _slope(t[inw], yaw[inw])
            m["pos_h_drift_m_s"] = _slope(t[inw], pos_h[inw])
        if we > t[-1]:
            # recording ends inside the fault window (truncated recordings): no "after"
            m["window_truncated"] = True
            return _bias_and_gps(m, run, info, t)
        tr = (t >= we) & (t < we + TRANSIENT_WINDOW_S)
        if tr.any():
            m["transient_tilt_max_deg"] = float(tilt[tr].max())
            m["transient_pos_h_max_m"] = float(pos_h[tr].max())
        if nofault_p95:
            chans = {"tilt": tilt, "pos_h": pos_h} if fault.name.startswith("gps") else {"tilt": tilt, "yaw": yaw}
            recs = {}
            for ch, y in chans.items():
                thr = max(RECOVERY_FACTOR * nofault_p95[ch], RECOVERY_FLOOR[ch])
                recs[ch] = _recovery(t, y, we, thr)
                m[f"recovery_{ch}_s"] = recs[ch]
            m["recovery_s"] = None if any(v is None for v in recs.values()) else max(recs.values())
    return _bias_and_gps(m, run, info, t)


def _bias_and_gps(m, run, info, t):
    # injected biases vs the filter's estimates (Gazebo's own biases taken as zero)
    for key, est, unit, scale in (("gyro_bias", run["bg"], "dps", math.degrees(1)),
                                  ("accel_bias", run["ba"], "ms2", 1.0)):
        inj = info[key]
        if np.any(inj != 0):
            err = np.linalg.norm(est - inj, axis=1) * scale
            m[f"{key}_final_err_{unit}"] = float(err[-1])
            m[f"{key}_injected_{unit}"] = float(np.linalg.norm(inj[-1]) * scale)
            tol = max(0.2 * np.linalg.norm(inj[-1]) * scale, 0.02 if key == "gyro_bias" else 0.01)
            m[f"{key}_converge_s"] = _recovery(t, err, t[0], tol)
    if np.any(info["mag_bias"] != 0):
        m["mag_bias_injected"] = float(np.linalg.norm(info["mag_bias"]))
        m["mag_bias_final_err"] = float(np.linalg.norm(run["bm"][-1] - info["mag_bias"]))

    # GPS outlier accounting: which consumed fixes were injected outliers
    ev = run.get("gps_events")
    if ev is not None and len(ev) and np.any(info["gps_outlier"]):
        rows, rejected = ev[:, 0].astype(int), ev[:, 1].astype(bool)
        is_out = info["gps_outlier"][rows]
        m["outliers_consumed"] = int(is_out.sum())
        m["outliers_rejected"] = int((is_out & rejected).sum())
        m["good_fixes_rejected"] = int((~is_out & rejected).sum())
    return m


def gps_axis_sigma(fault):
    return THRESHOLDS["gps_noise_sigma_h_m"]


def _band(v, lo_hi):
    a, b = lo_hi
    return "PASS" if v < a else ("WARN" if v < b else "FAIL")


def grade(m, fault):
    """-> (grades dict, overall). Only metrics that apply to this fault are graded."""
    from faults import GPS_OUTAGE, GPS_NOISY, GPS_FAULTS, WINDOWED
    T = THRESHOLDS
    g = {}
    if m.get("nan") or m.get("error"):
        g["nan"] = "FAIL"
        return g, "FAIL"
    g["nan"] = "PASS"
    for k in ("tilt_rms_deg", "tilt_max_deg", "yaw_rms_deg"):
        if k in m:
            g[k] = _band(m[k], T[k])
    if "pos_h_rms_m" in m and fault.name not in GPS_OUTAGE:
        if fault.name in GPS_NOISY:
            g["pos_h_rms_m"] = _band(m["pos_h_rms_m"] / gps_axis_sigma(fault), T["pos_h_rms_noise_x_sigma"])
        else:
            g["pos_h_rms_m"] = _band(m["pos_h_rms_m"], T["pos_h_rms_m"])
    if fault.name in GPS_OUTAGE and m.get("tilt_drift_deg_s") is not None:
        g["tilt_drift_deg_s"] = _band(abs(m["tilt_drift_deg_s"]), T["tilt_drift_deg_s"])
    if fault.name in WINDOWED and "recovery_s" in m and not m.get("window_truncated"):
        g["recovery_s"] = "FAIL" if m["recovery_s"] is None else _band(m["recovery_s"], T["recovery_s"])
    # pos/vel NEES graded only where GPS noise is injected: Gazebo's GPS is noise-free, so
    # on clean-GPS runs R_gps (a real receiver's 2.5 m) makes them ~1e-3 by construction -
    # that measures the simulator, not the filter (still reported, just not graded)
    for k in ("nees_att",) + (("nees_vel", "nees_pos") if fault.name in GPS_NOISY else ()):
        if k in m:
            v = m[k]
            lo, hi = T["nees_pass"]
            wlo, whi = T["nees_warn"]
            g[k] = "PASS" if lo <= v <= hi else ("WARN" if wlo <= v <= whi else "FAIL")
    if fault.name not in GPS_FAULTS:
        g["gps_resets"] = _band(m.get("gps_resets", 0), T["gps_resets"])
    overall = max(g.values(), key=lambda s: GRADE_ORDER[s])
    return g, overall
