"""PASS/FAIL for an OFFBOARD SITL flight (testing/suite/px4_offboard_flight.sh, 2026-11-02).

    python3 testing/suite/check_offboard.py testing/data/offboard/hover_none.csv [fault]

Reads the run's csv (SENSOR_LOG), run_sim log (<csv>.log) and PX4 console log (<csv>.px4.log).
The reference is PX4's own EKF2 estimate (the tq*/tp* columns in BRIDGE=px4 mode) - in SITL
that's within a few cm / tenths of a degree of truth, good enough to judge the flight.

fault none:  engaged, held the hover setpoint (z rms < 0.3 m, xy rms < 0.5 m once settled),
             tilt < 20 deg, landed and disarmed by our own landing
fault hang:  watchdog fired and handed control to PX4, PX4 landed + disarmed
fault nan:   FALLBACK handed control to PX4, PX4 landed + disarmed
fault kill:  run_sim killed; PX4's offboard-loss failsafe landed + disarmed
Exit code 0 = PASS - use it as the pre-flight regression check before every real flight.
"""
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import missions  # noqa: E402


def load(csv):
    d = np.genfromtxt(csv, delimiter=",", names=True)
    return {k: d[k] for k in d.dtype.names}


def tilt_deg(qw, qx, qy, qz):
    # angle between body z and world z
    return np.degrees(np.arccos(np.clip(1 - 2 * (qx ** 2 + qy ** 2), -1, 1)))


def main():
    csv = sys.argv[1]
    fault = sys.argv[2] if len(sys.argv) > 2 else "none"
    run = open(csv[:-4] + ".log", errors="replace").read()
    px4 = open(csv[:-4] + ".px4.log", errors="replace").read() if os.path.exists(csv[:-4] + ".px4.log") else ""
    checks = []

    def check(name, ok, detail=""):
        checks.append(ok)
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}{'  ' + detail if detail else ''}")

    m = re.search(r"MISSION (\w+):", run)
    mission = missions.get(m.group(1)) if m else None
    print(f"{os.path.basename(csv)}: mission={m.group(1) if m else '?'} fault={fault}")
    check("engaged OFFBOARD", "ENGAGED" in run)
    try:
        c = load(csv)
    except Exception as e:  # noqa: BLE001
        check("csv readable", False, repr(e))
        return 1
    mt = c["mt"]
    resets = np.nonzero(np.diff(mt) < -1.0)[0]
    e0 = resets[-1] + 1 if len(resets) else 0       # first row after engagement (mission clock reset)
    f = slice(e0, None)
    tz, sz = c["tpz"][f], c["spz"][f]
    txy = np.stack([c["tpx"][f], c["tpy"][f]], 1)
    sxy = np.stack([c["spx"][f], c["spy"][f]], 1)
    exyz = np.stack([c["epx"][f], c["epy"][f], c["epz"][f]], 1)
    txyz = np.stack([c["tpx"][f], c["tpy"][f], c["tpz"][f]], 1)
    tilt = tilt_deg(c["tqw"][f], c["tqx"][f], c["tqy"][f], c["tqz"][f])
    t = mt[f]
    fs = c["fs"][f]
    print(f"  flight: {t[-1]:.1f} s logged after engagement, max PX4 altitude {np.nanmax(tz):.2f} m, "
          f"max tilt {np.nanmax(tilt):.1f} deg")
    # settled hover window: after the climb, before any fault / the mission profile moving off (0,0)
    hi = missions.SETTLE_S + (mission.dynamic_start if mission else 10.0)
    if fault != "none":
        hi = min(hi, float(os.environ.get("FAULT_AT", "25")))
    w = (t > 6.0) & (t < hi) & (fs == 0)
    if w.sum() > 50:
        z_rms = float(np.sqrt(np.mean((tz[w] - sz[w]) ** 2)))
        xy_rms = float(np.sqrt(np.mean(np.sum((txy[w] - sxy[w]) ** 2, 1))))
        est = float(np.sqrt(np.mean(np.sum((exyz[w] - txyz[w]) ** 2, 1))))
        check("holds altitude", z_rms < 0.3, f"z rms {z_rms:.2f} m vs setpoint (t {6:.0f}-{hi:.0f} s)")
        check("holds position", xy_rms < 0.5, f"xy rms {xy_rms:.2f} m vs setpoint")
        print(f"  info: your EKF vs PX4 EKF2 position {est:.2f} m rms in that window")
    else:
        check("settled hover window", False, f"only {int(w.sum())} rows")
    check("tilt stays sane", np.nanmax(tilt[w | (t < hi)]) < 20.0, f"max {np.nanmax(tilt[w | (t < hi)]):.1f} deg before any fault")
    disarmed = re.search(r"disarm", px4, re.I) is not None
    if fault == "none":
        check("own landing + disarm", "landed - disarm requested" in run)
        check("ended on the ground", tz[-1] < 0.3, f"final PX4 altitude {tz[-1]:.2f} m")
        check("PX4 disarmed", disarmed)
    elif fault == "hang":
        check("watchdog fired", "WATCHDOG" in run)
        check("handed back to PX4", "handing control back" in run)
        check("PX4 landed + disarmed", disarmed)
    elif fault == "nan":
        check("FALLBACK", "FALLBACK" in run)
        check("handed back to PX4", "handing control back" in run)
        check("ended on the ground", tz[-1] < 0.3, f"final PX4 altitude {tz[-1]:.2f} m")
        check("PX4 disarmed", disarmed)
    elif fault == "kill":
        check("no hand-back (process died)", "handing control back" not in run)
        check("PX4 failsafe triggered", re.search(r"failsafe|offboard.*(lost|loss)|connection.*lost", px4, re.I) is not None)
        check("PX4 landed + disarmed", disarmed)
    ok = all(checks)
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
