"""Turn PX4-flown benchmark flights into scored-ready recordings. Runs INSIDE WSL:

    python3 testing/benchmark/prepare.py <flight.ulg> ... [--faults none,gps_dropout,...]

For each flight and fault:
  1. ulog_faults.py  - inject the fault into the raw sensor messages
  2. PX4 EKF2 replay - run PX4's EKF2 (the real C++ code, same build as SITL) on the
                       corrupted log -> *_replayed.ulg. Every EKF2 number in the benchmark comes
                       from replay, including fault 'none' (live EKF2 differs by <= 0.06 deg
                       after its first 2 s - checked on sitl_flight1).
     Magnetic field: EKF2 always uses the World Magnetic Model field (it ignores
     EKF2_MAG_DECL while GPS is valid), which is ~6 deg off Gazebo's field - that was the whole
     5.6 deg 'heading error' in the first benchmark. So before any fault, the mag samples are
     rotated so that the true field IS the WMM field (ulog_faults.py --align-field); the
     other filters then measure that same field from the clean aligned log against truth.
  3. ulog_to_csv.py  - convert the replayed log (it carries the corrupted sensors and EKF2's
                       output) to the replay csv the other filters use, with the clean-log
                       field direction (--mag-ref) and ground truth from the ORIGINAL log
                       (--truth-from: replay re-logs it at ~7 Hz, the flight log has 50 Hz).
Output: testing/data/benchmark/<flight>__<fault>.csv (+ .cal.csv, .meta.json)
"""
import argparse
import glob
import json
import os
import subprocess
import sys
import time
import zlib

import numpy as np

R = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PX = os.path.expanduser("~/PX4-Autopilot")
REPLAY_ROOT = os.path.join(PX, "build", "px4_sitl_default_replay", "rootfs")
WORK = os.path.expanduser("~/bench/work")
OUT = os.path.join(R, "testing", "data", "benchmark")
sys.path.insert(0, os.path.join(R, "testing", "benchmark"))
from ulog_faults import FAULTS

CAL_SAMPLES = 1500   # 6 s of the pre-arm dwell for the replayed filters' calibration


def run(cmd, **kw):
    r = subprocess.run(cmd, capture_output=True, text=True, **kw)
    if r.returncode != 0:
        raise RuntimeError(f"{cmd[0]} failed:\n{r.stdout[-2000:]}\n{r.stderr[-2000:]}")
    return r.stdout


def ekf2_replay(ulg):
    # no parameter overrides: EKF2 replays with the flight's own parameters
    p = os.path.join(REPLAY_ROOT, "replay_params.txt")
    if os.path.exists(p):
        os.remove(p)
    t0 = time.time()
    env = dict(os.environ, replay=ulg, replay_mode="ekf2")
    # ulg must be on the Linux filesystem: replay's reader crawls on /mnt/c (>120 s vs 8 s).
    # Output goes to a file: with stdin at EOF the pxh shell floods stdout until PX4 exits
    # (replay requests shutdown itself when done).
    with open(os.path.join(WORK, "replay.log"), "w") as log:
        r = subprocess.run(["make", "px4_sitl", "none"], cwd=PX, env=env, stdin=subprocess.DEVNULL,
                           stdout=log, stderr=subprocess.STDOUT, timeout=900)
    if r.returncode != 0:
        raise RuntimeError(f"replay failed, see {log.name}")
    new = [p for p in glob.glob(os.path.join(REPLAY_ROOT, "fs", "log", "*", "*_replayed.ulg")) if os.path.getmtime(p) >= t0]
    if not new:
        raise RuntimeError("replay wrote no log")
    return max(new, key=os.path.getmtime)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ulg", nargs="+")
    ap.add_argument("--faults", default=",".join(FAULTS))
    a = ap.parse_args()
    os.makedirs(WORK, exist_ok=True)
    os.makedirs(OUT, exist_ok=True)
    for ulg in a.ulg:
        name = os.path.splitext(os.path.basename(ulg))[0]
        # the WMM field EKF2 uses here: the declination it saved (EKF2_MAG_DECL, EKF2_DECL_TYPE
        # bit 1) and the inclination reference it logs; Gazebo's field is rotated onto it
        from pyulog import ULog
        lg = ULog(ulg, ["estimator_status"])
        decl = lg.initial_parameters["EKF2_MAG_DECL"]
        inc = lg.data_list[0].data["mag_inclination_ref_deg"]
        incl = float(np.nanmedian(inc[np.isfinite(inc) & (inc != 0)]))
        aligned = os.path.join(WORK, name + "__aligned.ulg")
        run([sys.executable, os.path.join(R, "testing", "benchmark", "ulog_faults.py"), ulg, aligned, "none",
             f"--align-field={decl},{incl}"])
        # field direction of the aligned clean log, measured against ground truth (ulog_to_csv's
        # estimate) - every fault variant's csv gets this, so a mag fault can't leak into it
        clean_csv = os.path.join(WORK, name + "__clean.csv")
        run([sys.executable, os.path.join(R, "testing", "ulog_to_csv.py"), aligned, clean_csv, "--cal-samples", str(CAL_SAMPLES)])
        meta = json.load(open(clean_csv + ".meta.json"))
        ref = ",".join(str(x) for x in meta["mag_reference"])
        for fault in a.faults.split(","):
            out_csv = os.path.join(OUT, f"{name}__{fault}.csv")
            if os.path.exists(out_csv + ".meta.json"):
                print(f"skip {out_csv} (done)", flush=True)
                continue
            t0 = time.time()
            patched = os.path.join(WORK, f"{name}__{fault}.ulg")
            info = json.loads(run([sys.executable, os.path.join(R, "testing", "benchmark", "ulog_faults.py"),
                                   ulg, patched, fault, "--seed", str(zlib.crc32(name.encode()) % 10000),
                                   f"--align-field={decl},{incl}"]))
            replayed = ekf2_replay(patched)
            run([sys.executable, os.path.join(R, "testing", "ulog_to_csv.py"), replayed, out_csv,
                 "--cal-samples", str(CAL_SAMPLES), f"--mag-ref={ref}", f"--truth-from={ulg}"])
            m = json.load(open(out_csv + ".meta.json"))
            m.update(flight=name, fault=fault, fault_info=info, ekf2_source="replay", wmm_decl_deg=decl, wmm_incl_deg=incl)
            json.dump(m, open(out_csv + ".meta.json", "w"), indent=1)
            os.remove(patched)
            os.remove(replayed)
            print(f"PREPARED {name} {fault} in {time.time() - t0:.0f}s", flush=True)


if __name__ == "__main__":
    main()
