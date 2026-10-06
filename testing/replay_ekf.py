"""Offline EKF replay against Gazebo ground truth.

Feeds a SENSOR_LOG csv (see run_sim.py) through DroneEKF with time.time() patched to
the recorded timestamps, and reports estimate-vs-truth error. Lets EKF changes be
checked in seconds instead of one Gazebo crash per try.

usage: python testing/replay_ekf.py sensors_oracle1.csv [print_every]

If <csv>.cal.csv exists (run_sim.py writes it since 2026-09-30 evening) calibration is
fed the live calibration's own samples; otherwise (or with CAL_ROW0=1) row 0 repeated.

The replay core (load_rows / Replay / run_replay) is also imported by testing/suite/
run_suite.py, so the stress-test suite and this tool drive the EKF identically.
"""
import sys
import os
import math
import csv
import types
import contextlib
import io
import numpy as np

# lives in testing/ - repo root is one level up (for run_sim.py, "state estimation/", "PID/")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "state estimation"))
sys.path.insert(0, os.path.join(ROOT, "PID"))
import FINAL_gps

# same value gz_bridge.py hands the EKF - imported by value to avoid needing gz.transport
MAG_REFERENCE_ENU = (0.450, 0.020, 0.893)


def quat_to_euler_deg(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.degrees([roll, pitch, yaw])


def quat_err_deg(q_est, q_true):
    # rotation angle between the two attitudes, per body axis (small-angle vector)
    w1, x1, y1, z1 = q_true
    qc = np.array([w1, -x1, -y1, -z1])
    e = FINAL_gps.quat_mult(qc, q_est)
    if e[0] < 0:
        e = -e
    return np.degrees(2 * e[1:])


def load_rows(path):
    with open(path) as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


def load_cal(csv_path):
    """The calibration samples run_sim.py saved next to a SENSOR_LOG (<csv>.cal.csv), as
    {"gyro": [...], "mag": [...], "accel": [...]} - or None for recordings made before
    2026-09-30 evening, which then calibrate on row 0 repeated (the old behaviour)."""
    path = csv_path + ".cal.csv"
    if not os.path.exists(path) or os.environ.get("CAL_ROW0") == "1":
        return None
    rows = load_rows(path)
    return {"gyro": [(r["gx"], r["gy"], r["gz"]) for r in rows],
            "mag": [(r["mx"], r["my"], r["mz"]) for r in rows],
            "accel": [(r["ax"], r["ay"], r["az"]) for r in rows]}


def load_meta(csv_path):
    # <csv>.meta.json written by run_sim.py since 2026-10-02 (e.g. gps_latency_s); {} if absent
    import json
    path = csv_path + ".meta.json"
    return json.load(open(path)) if os.path.exists(path) else {}


class Replay:
    mag_reference = MAG_REFERENCE_ENU
    gps_latency = 0.0   # seconds - the EKF reads it (delayed GPS fusion); see load_meta

    def __init__(self, rows, remapped=None, cal=None):
        self.rows = rows
        self.i = 0
        self.calibrating = True
        self.remapped = os.environ.get("MAG_REMAPPED", "1") == "1" if remapped is None else remapped
        self.cal = cal
        self._ci = {"gyro": 0, "mag": 0, "accel": 0}

    def _cal_read(self, name):
        # replay the live calibration's reads in order (same call sequence as calibrate())
        seq = self.cal[name]
        v = seq[min(self._ci[name], len(seq) - 1)]
        self._ci[name] += 1
        return v

    def _row(self):
        # calibration reads before any timestamps matter - serve the first (at-rest) row
        return self.rows[0] if self.calibrating else self.rows[self.i]

    def get_gyro(self):
        if self.calibrating and self.cal: return self._cal_read("gyro")
        r = self._row(); return (r["gx"], r["gy"], r["gz"])

    def get_accel(self):
        if self.calibrating and self.cal: return self._cal_read("accel")
        r = self._row(); return (r["ax"], r["ay"], r["az"])

    def get_mag(self):
        if self.calibrating and self.cal: return self._cal_read("mag")
        # csvs recorded before the 2026-09-30 mag fix hold the (wrong) remapped (-y, x, -z)
        # value - undo it to get the raw reading the bridge now passes through
        r = self._row()
        if self.remapped:
            return (r["my"], -r["mx"], -r["mz"])
        return (r["mx"], r["my"], r["mz"])

    def get_mag_time(self):
        # 'mgt' column (recordings since 2026-10-04); older recordings: a reading is new when
        # its value changes (Gazebo/PX4 mags are noisy, so a repeat = the same held reading)
        r = self._row()
        if "mgt" in r:
            return r["mgt"]
        m = (r["mx"], r["my"], r["mz"])
        if m != getattr(self, "_prev_mag", None):
            self._prev_mag = m
            self._mag_id = getattr(self, "_mag_id", 0) + 1
        return self._mag_id

    def get_baro(self):
        # optional bt/bh columns (recordings since 2026-10-03); NaN = no baro sample yet
        r = self._row()
        if "bh" not in r or r["bh"] != r["bh"]:
            return None
        return (r["bt"], r["bh"])

    def get_gps(self):
        # optional "gps_ok" column (testing/suite/faults.py): 0 = receiver has no fix
        r = self._row()
        if r.get("gps_ok", 1.0) == 0.0:
            return None
        return (r["px"], r["py"], r["pz"], r["vx"], r["vy"], r["vz"])

    def get_gps_time(self):
        # 'gpt' column (recordings since 2026-10-05); older recordings: a fix is new when its
        # value changes (same idea as get_mag_time)
        r = self._row()
        if "gpt" in r:
            return r["gpt"] if r["gpt"] == r["gpt"] else None
        g = _gps_key((r["px"], r["py"], r["pz"], r["vx"], r["vy"], r["vz"]))
        if g != getattr(self, "_prev_gps", None):
            self._prev_gps = g
            self._gps_id = getattr(self, "_gps_id", 0) + 1
        return self._gps_id


def _gps_key(fix):
    # NaN components (partial fixes) never compare equal - map them to None so a repeated
    # partial fix is still recognised as the same fix
    return tuple(v if v == v else None for v in fix)


class ArrayReplay(Replay):
    """Same sensor interface as Replay, backed by columns (name -> python list) instead of
    a list of row dicts - much lighter for long recordings (testing/suite)."""

    def __init__(self, cols, remapped=None, cal=None):
        super().__init__(None, remapped, cal)
        c = cols
        self._g = list(zip(c["gx"], c["gy"], c["gz"]))
        self._a = list(zip(c["ax"], c["ay"], c["az"]))
        m = (c["my"], [-v for v in c["mx"]], [-v for v in c["mz"]]) if self.remapped else (c["mx"], c["my"], c["mz"])
        self._m = list(zip(*m))
        self._p = list(zip(c["px"], c["py"], c["pz"], c["vx"], c["vy"], c["vz"]))
        self._ok = c.get("gps_ok")
        self._baro = list(zip(c["bt"], c["bh"])) if "bh" in c else None
        if "mgt" in c:
            self._mag_t = list(c["mgt"])
        else:  # infer: a new reading wherever the value changes
            ids, prev, k = [], None, 0
            for m in zip(c["mx"], c["my"], c["mz"]):
                if m != prev:
                    k, prev = k + 1, m
                ids.append(k)
            self._mag_t = ids
        if "gpt" in c:
            self._gps_t = [v if v == v else None for v in c["gpt"]]
        else:  # infer: a new fix wherever the value changes
            ids, prev, k = [], None, 0
            for p in self._p:
                p = _gps_key(p)
                if p != prev:
                    k, prev = k + 1, p
                ids.append(k)
            self._gps_t = ids

    def _k(self):
        return 0 if self.calibrating else self.i

    def get_gyro(self):
        if self.calibrating and self.cal: return self._cal_read("gyro")
        return self._g[self._k()]

    def get_accel(self):
        if self.calibrating and self.cal: return self._cal_read("accel")
        return self._a[self._k()]

    def get_mag(self):
        if self.calibrating and self.cal: return self._cal_read("mag")
        return self._m[self._k()]

    def get_mag_time(self):
        return None if self.calibrating else self._mag_t[self.i]

    def get_baro(self):
        if self._baro is None or self.calibrating:
            return None
        b = self._baro[self.i]
        return None if b[1] != b[1] else b

    def get_gps(self):
        k = self._k()
        if self._ok is not None and self._ok[k] == 0.0:
            return None
        return self._p[k]

    def get_gps_time(self):
        return None if self.calibrating else self._gps_t[self.i]


def apply_exp(ekf, exp):
    # experiment knobs: EXP="R_ACC=1e6 QAB=1e-10 QATT=1e-7"
    for kv in exp.split():
        k, v = kv.split("="); v = float(v)
        if k == "R_ACC": ekf.R_accel_base = np.eye(3) * v
        if k == "QAB": ekf.Q[12:15, 12:15] = np.eye(3) * v
        if k == "QATT": ekf.Q[0:3, 0:3] = np.eye(3) * v
        if k == "QVEL": ekf.Q[6:9, 6:9] = np.eye(3) * v
        if k == "RMAG": FINAL_gps.R_mag[:] = np.eye(3) * v
        if k == "K": ekf.k = v
        if k == "QMB": ekf.Q[15:18, 15:18] = np.eye(3) * v
        if k == "QGB": ekf.Q[3:6, 3:6] = np.eye(3) * v
        if k == "PMB": ekf.P[15:18, :] = 0; ekf.P[:, 15:18] = 0; ekf.P[15:18, 15:18] = np.eye(3) * v
        if k == "PAB": ekf.P[12:15, :] = 0; ekf.P[:, 12:15] = 0; ekf.P[12:15, 12:15] = np.eye(3) * v
        if k in ("ROLL0", "PITCH0", "YAW0"):
            # inject a known attitude error (deg) to test observability/convergence
            ax = {"ROLL0": 0, "PITCH0": 1, "YAW0": 2}[k]
            d = np.zeros(3); d[ax] = math.radians(v) / 2
            ekf.q = FINAL_gps.quat_mult(ekf.q, np.array([1.0, *d])); ekf.q /= np.linalg.norm(ekf.q)
            ekf.P[0:3, 0:3] += np.eye(3) * math.radians(abs(v)) ** 2


def run_replay(src, times, on_step, exp="", t_end=None, quiet=False):
    """Calibrate a DroneEKF on sample 0 (src.calibrating=True serves it), then step it
    through every sample with the EKF's clock set to times[i]. on_step(i, ekf, state) is
    called after each step. Only FINAL_gps's own `time` reference is patched (not the
    global time module). quiet=True swallows the EKF's prints (EKF_GPS_RESET - counted
    in ekf.stats anyway). Returns the ekf."""
    clock = {"t": times[0]}
    saved_time = FINAL_gps.time
    FINAL_gps.time = types.SimpleNamespace(time=lambda: clock["t"])
    out = io.StringIO() if quiet else sys.stdout
    try:
        with contextlib.redirect_stdout(out):
            src.calibrating = True
            src._ci = {"gyro": 0, "mag": 0, "accel": 0}
            ekf = FINAL_gps.DroneEKF(sensors=src)
            src.calibrating = False
            apply_exp(ekf, exp)
            t0 = times[0]
            for i, t in enumerate(times):
                if t_end is not None and t - t0 > t_end:
                    break
                src.i = i
                clock["t"] = t
                st = ekf.step()
                on_step(i, ekf, st)
    finally:
        FINAL_gps.time = saved_time
    return ekf


def main():
    path = sys.argv[1]
    every = int(sys.argv[2]) if len(sys.argv) > 2 else 250
    rows = load_rows(path)
    t0 = rows[0]["t"]
    att_err, pos_err = [], []

    def on_step(i, ekf, st):
        r = rows[i]
        tq = (r["tqw"], r["tqx"], r["tqy"], r["tqz"])
        tp = np.array([r["tpx"], r["tpy"], r["tpz"]])
        ae = quat_err_deg(st["quat"], tq)
        pe = st["pos"] - tp
        att_err.append(ae); pos_err.append(pe)
        if i % every == 0:
            er = quat_to_euler_deg(st["quat"]); tr = quat_to_euler_deg(tq)
            print(f"t={r['t']-t0:6.2f} est=({er[0]:6.1f},{er[1]:6.1f},{er[2]:6.1f}) "
                  f"true=({tr[0]:6.1f},{tr[1]:6.1f},{tr[2]:6.1f}) att_err=({ae[0]:5.1f},{ae[1]:5.1f},{ae[2]:5.1f}) "
                  f"pos_err=({pe[0]:5.2f},{pe[1]:5.2f},{pe[2]:6.2f}) "
                  f"abias=({ekf.accel_bias_x:+.3f},{ekf.accel_bias_y:+.3f},{ekf.accel_bias_z:+.3f})")

    src = Replay(rows, cal=load_cal(path))
    meta = load_meta(path)
    src.gps_latency = meta.get("gps_latency_s", 0.0)
    if "mag_reference" in meta:
        src.mag_reference = tuple(meta["mag_reference"])
    run_replay(src, [r["t"] for r in rows], on_step,
               exp=os.environ.get("EXP", ""), t_end=float(os.environ.get("T_END", "1e9")))
    att_err = np.abs(np.array(att_err)); pos_err = np.abs(np.array(pos_err))
    print(f"att_err deg  rms={np.sqrt((att_err**2).mean(0)).round(2)} max={att_err.max(0).round(1)}")
    print(f"pos_err m    rms={np.sqrt((pos_err**2).mean(0)).round(2)} max={pos_err.max(0).round(2)}")


if __name__ == "__main__":
    main()
