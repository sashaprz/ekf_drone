"""Tier B - closed-loop: the EKF actually flies each mission in Gazebo (EKF_TEST_PLAN.md s5).

usage: python testing/suite/run_live.py [--missions hover box ...] [--trials 3] [--out DIR] [--score-only]

Runs from Windows: each trial is one `wsl ... fly.sh` call (fresh Gazebo restart, no
ORACLE, SENSOR_LOG on -> testing/data/live/<mission>_t<k>.csv, replayable offline).
Per trial: survived (true tilt <= 45 deg and no unplanned ground contact), true tracking
error vs the mission's ideal path (and vs the ORACLE recording of the same mission, which
flew the same controller on truth - the difference is what estimator error costs), and
the error of the estimate that actually flew (logged live, so no replay assumptions).
Mean and spread across trials - never a single trial as evidence.
"""
import argparse
import json
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import run_suite as RS  # noqa: E402
import metrics as M  # noqa: E402
import missions as MS  # noqa: E402

DEFAULT = ["hover", "box", "circle_slow", "yaw_steps", "takeoff_land"]
LIVE = os.path.join(RS.DATA, "live")
FLY = "/mnt/c/Users/Sasha/repos/python_drone/testing/suite/fly.sh"
# planned ground contact in takeoff_land: profile 8-24 s descend/sit (+2 s to re-climb)
GROUND_OK = {"takeoff_land": (MS.SETTLE_S + 8, MS.SETTLE_S + 26)}


def fly(mission, k):
    csv = f"testing/data/live/{mission}_t{k}.csv"
    log = f"testing/data/logs/live_{mission}_t{k}.log"
    envs = [f"{k_}={v}" for k_, v in MS.AGGRESSIVE_LIMITS.items()] if MS.MISSIONS[mission].aggressive else []
    t0 = time.time()
    p = subprocess.run(["wsl", "-d", "Ubuntu-24.04", "--", "bash", FLY, mission, csv, log, *envs],
                       capture_output=True, text=True)
    last = [ln for ln in p.stdout.splitlines() if ln.startswith("FLY_RESULT")]
    print(f"  {mission} t{k}: {last[-1] if last else p.stdout[-300:]} ({time.time()-t0:.0f}s)", flush=True)
    return os.path.join(RS.ROOT, csv)


def tracking(cols, truth, mi):
    prof = (cols["mt"] >= MS.SETTLE_S) & (cols["mt"] <= MS.SETTLE_S + mi.duration)
    ideal = np.array([mi.ideal(mt)[0] for mt in cols["mt"][prof]])
    e = truth["pos"][prof] - ideal
    eh = np.hypot(e[:, 0], e[:, 1])
    return {"track_h_rms_m": float(np.sqrt(np.mean(eh ** 2))), "track_h_max_m": float(eh.max()),
            "track_v_rms_m": float(np.sqrt(np.mean(e[:, 2] ** 2)))}


def score(path, mi):
    cols = RS.load_recording(path)
    truth = M.truth_arrays(cols)
    t = cols["t"] - cols["t"][0]
    q = truth["q"]
    tilt_true = np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))
    z = truth["pos"][:, 2]
    up = np.flatnonzero(z > M.TAKEOFF_Z)
    air = t >= (t[up[0]] + M.SCORE_AFTER_TAKEOFF_S) if len(up) else np.zeros(len(t), bool)
    lo, hi = GROUND_OK.get(mi.name, (1e9, -1e9))
    unplanned = air & (z < 0.15) & ~((cols["mt"] >= lo) & (cols["mt"] <= hi))
    ended = cols["mt"][-1] >= mi.total - 0.5
    r = {"survived": bool(ended and tilt_true[air].max() <= 45 and not unplanned.any()),
         "completed": bool(ended), "max_true_tilt_deg": float(tilt_true[air].max()) if air.any() else None,
         "unplanned_ground_s": float(unplanned.sum() * np.median(np.diff(t))), "flight_s": float(t[-1])}
    r.update(tracking(cols, truth, mi))
    # the estimate that actually flew (logged live)
    eq = np.stack([cols["eqw"], cols["eqx"], cols["eqy"], cols["eqz"]], axis=1)
    att = M.quat_err_deg(eq, truth["q"])
    ep = np.stack([cols["epx"], cols["epy"], cols["epz"]], axis=1) - truth["pos"]
    ev = np.stack([cols["evx"], cols["evy"], cols["evz"]], axis=1) - truth["vel"]
    tilt, yaw = np.hypot(att[air, 0], att[air, 1]), np.abs(att[air, 2])
    r.update({"est_tilt_rms_deg": float(np.sqrt(np.mean(tilt ** 2))), "est_tilt_max_deg": float(tilt.max()),
              "est_yaw_rms_deg": float(np.sqrt(np.mean(yaw ** 2))), "est_yaw_mean_signed_deg": float(att[air, 2].mean()),
              "est_roll_mean_deg": float(att[air, 0].mean()), "est_pitch_mean_deg": float(att[air, 1].mean()),
              "est_pos_h_rms_m": float(np.sqrt(np.mean(np.hypot(ep[air, 0], ep[air, 1]) ** 2))),
              "est_vel_h_rms_ms": float(np.sqrt(np.mean(np.hypot(ev[air, 0], ev[air, 1]) ** 2)))})
    log = os.path.join(RS.DATA, "logs", "live_" + os.path.basename(path)[:-4] + ".log")
    r["gps_resets_printed"] = open(log).read().count("EKF_GPS_RESET") if os.path.exists(log) else None
    return r


def summarize(trials):
    keys = [k for k in trials[0] if isinstance(trials[0][k], (int, float)) and not isinstance(trials[0][k], bool)]
    s = {"n": len(trials), "survived": sum(t["survived"] for t in trials)}
    for k in keys:
        v = np.array([t[k] for t in trials if t.get(k) is not None], float)
        if len(v):
            s[k] = {"mean": float(v.mean()), "min": float(v.min()), "max": float(v.max())}
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--missions", nargs="*", default=DEFAULT)
    ap.add_argument("--trials", type=int, default=3)
    ap.add_argument("--out", required=True)
    ap.add_argument("--score-only", action="store_true")
    a = ap.parse_args()
    os.makedirs(LIVE, exist_ok=True)
    os.makedirs(a.out, exist_ok=True)
    res = {"meta": RS.git_meta(), "missions": {}}
    for m in a.missions:
        mi = MS.get(m)
        trials = []
        for k in range(1, a.trials + 1):
            path = os.path.join(LIVE, f"{m}_t{k}.csv")
            if not a.score_only:
                path = fly(m, k)
            if not os.path.exists(path):
                trials.append({"survived": False, "completed": False, "error": "no csv"})
                continue
            tr = score(path, mi)
            tr["trial"] = k
            trials.append(tr)
        oracle = os.path.join(RS.DATA, f"flight_{m}.csv")
        ref = tracking(RS.load_recording(oracle), M.truth_arrays(RS.load_recording(oracle)), mi) if os.path.exists(oracle) else {}
        good = [t for t in trials if "error" not in t]
        res["missions"][m] = {"trials": trials, "summary": summarize(good) if good else {"n": 0, "survived": 0},
                              "oracle_tracking": ref}
        print(f"{m}: survived {res['missions'][m]['summary']['survived']}/{len(trials)}", flush=True)
    with open(os.path.join(a.out, "tierB.json"), "w") as f:
        json.dump(res, f, indent=1, sort_keys=True)
    L = ["# Tier B - closed loop on the EKF", "", f"code `{res['meta']['git']}`{' (dirty)' if res['meta']['dirty'] else ''}", "",
         "| mission | survived | track_h rms (EKF) | track_h rms (ORACLE) | track_h max (EKF) | est tilt rms | est tilt max | "
         "est yaw rms | est yaw mean | est roll/pitch mean | est pos_h rms |", "|---|" + "---|" * 10]

    def mm(s, k, nd=2):
        v = s.get(k)
        return "-" if not v else f"{v['mean']:.{nd}f} [{v['min']:.{nd}f}..{v['max']:.{nd}f}]"
    for m, d in res["missions"].items():
        s = d["summary"]
        o = d["oracle_tracking"].get("track_h_rms_m")
        L.append(f"| {m} | {s['survived']}/{len(d['trials'])} | {mm(s, 'track_h_rms_m')} | {'-' if o is None else f'{o:.2f}'} | "
                 f"{mm(s, 'track_h_max_m')} | {mm(s, 'est_tilt_rms_deg')} | {mm(s, 'est_tilt_max_deg')} | "
                 f"{mm(s, 'est_yaw_rms_deg')} | {mm(s, 'est_yaw_mean_signed_deg', 1)} | "
                 f"{mm(s, 'est_roll_mean_deg')} / {mm(s, 'est_pitch_mean_deg')} | {mm(s, 'est_pos_h_rms_m')} |")
    L += ["", "Values: mean [min..max] over trials. est_* = the live EKF estimate vs Gazebo truth over the airborne window.",
          "track_h = true horizontal position vs the mission's ideal path during the profile."]
    with open(os.path.join(a.out, "TIERB.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main()
