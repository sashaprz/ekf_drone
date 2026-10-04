"""Timing benchmark: can this computer run the EKF + controller at the IMU rate?

Replays a recording through DroneEKF + Cascade exactly as run_sim.py's loop calls them
(one ekf.step() per IMU sample, one controller step per loop) and times every step. Needs
only numpy - no Gazebo - so it runs as-is on the Raspberry Pi:

  python3 testing/bench_timing.py                      # default recording, 250 Hz budget
  python3 testing/bench_timing.py testing/data/flight_circle_fast.csv --hz 250 --profile

Copy the repo (or just state estimation/, PID/, testing/, run_sim.py) plus one
testing/data/flight_*.csv (+ .cal.csv) to the Pi. Reports step-time percentiles, how often
a step overran the budget (1/hz), worst stall, GC pauses, and with --profile the functions
that cost the most. A loop needs comfortable headroom - p99 well under the budget - because
anything else on the Pi (logging, MAVLink, the OS) shares the same core.
"""
import argparse
import cProfile
import gc
import os
import platform
import pstats
import sys
import time
import types

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "testing"), os.path.join(ROOT, "PID"), ROOT]
sys.modules.setdefault("gz_bridge", types.SimpleNamespace(GazeboBridge=None))  # run_sim imports it; not needed here
import replay_ekf  # noqa: E402
import cascade  # noqa: E402
from run_sim import GAINS, LIMITS  # noqa: E402


def load(path):
    with open(path) as f:
        header = f.readline().strip().split(",")
    arr = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    return {k: arr[:, j] for j, k in enumerate(header)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?", default=os.path.join(ROOT, "testing", "data", "flight_circle_fast.csv"))
    ap.add_argument("--hz", type=float, default=250.0, help="IMU / loop rate to budget for")
    ap.add_argument("--seconds", type=float, default=30.0, help="how much of the recording to run")
    ap.add_argument("--profile", action="store_true")
    a = ap.parse_args()

    cols = load(a.csv)
    keep = cols["t"] - cols["t"][0] <= a.seconds
    cols = {k: v[keep] for k, v in cols.items()}
    src = replay_ekf.ArrayReplay({k: v.tolist() for k, v in cols.items()}, remapped=False, cal=replay_ekf.load_cal(a.csv))
    ctrl = cascade.Cascade({"GAINS": GAINS, "LIMITS": LIMITS})
    budget = 1.0 / a.hz
    t_ekf, t_ctrl = [], []
    gc_pauses = []
    gc_t0 = {}

    def gc_cb(phase, info):
        if phase == "start":
            gc_t0["t"] = time.perf_counter()
        elif "t" in gc_t0:
            gc_pauses.append(time.perf_counter() - gc_t0.pop("t"))
    gc.callbacks.append(gc_cb)

    setpoint = {"pos": np.array([0.0, 0.0, 2.0]), "yaw": 0.0}

    def on_step(i, ekf, st):
        # time the EKF step that run_replay just did (measured around ekf.step below)
        t0 = time.perf_counter()
        ctrl.step(setpoint, st, budget)
        t_ctrl.append(time.perf_counter() - t0)

    # wrap DroneEKF.step to time it without touching the filter
    import FINAL_gps
    orig_step = FINAL_gps.DroneEKF.step

    def timed_step(self):
        t0 = time.perf_counter()
        out = orig_step(self)
        t_ekf.append(time.perf_counter() - t0)
        return out
    FINAL_gps.DroneEKF.step = timed_step

    prof = cProfile.Profile() if a.profile else None
    wall0 = time.perf_counter()
    if prof:
        prof.enable()
    replay_ekf.run_replay(src, cols["t"].tolist(), on_step, quiet=True)
    if prof:
        prof.disable()
    wall = time.perf_counter() - wall0
    FINAL_gps.DroneEKF.step = orig_step
    gc.callbacks.remove(gc_cb)

    e, c = np.array(t_ekf) * 1000, np.array(t_ctrl) * 1000
    tot = e + c
    b = budget * 1000
    pct = lambda x, q: np.percentile(x, q)
    print(f"machine: {platform.machine()} {platform.processor() or ''} | python {platform.python_version()} | numpy {np.__version__}")
    print(f"recording: {os.path.basename(a.csv)}, {len(e)} steps ({a.seconds:.0f} s of flight), budget {b:.2f} ms/step at {a.hz:.0f} Hz\n")
    print(f"{'ms per step':>14s} {'mean':>7s} {'p50':>7s} {'p99':>7s} {'p99.9':>7s} {'max':>7s}")
    for name, x in (("EKF", e), ("controller", c), ("total", tot)):
        print(f"{name:>14s} {x.mean():7.3f} {pct(x,50):7.3f} {pct(x,99):7.3f} {pct(x,99.9):7.3f} {x.max():7.2f}")
    load_pct = tot.mean() / b * 100
    print(f"\nCPU load at {a.hz:.0f} Hz: {load_pct:.0f}% of one core (mean); steps over budget: "
          f"{np.mean(tot > b) * 100:.2f}% ; worst step {tot.max():.1f} ms")
    if gc_pauses:
        g = np.array(gc_pauses) * 1000
        print(f"GC pauses: {len(g)}, max {g.max():.2f} ms")
    verdict = ("OK - comfortable headroom" if load_pct < 50 and pct(tot, 99) < 0.7 * b else
               "MARGINAL - fits on average but little headroom" if load_pct < 85 and pct(tot, 99) < b else
               "TOO SLOW - lower the rate, optimize, or run the EKF in a faster language")
    print(f"verdict: {verdict}")
    print(f"(replayed {len(e)} steps in {wall:.1f} s = {len(e)/wall:.0f} steps/s incl. bookkeeping)")
    if prof:
        print("\nTop functions by own time:")
        pstats.Stats(prof).sort_stats("tottime").print_stats(12)


if __name__ == "__main__":
    main()
