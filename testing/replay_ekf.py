"""Offline EKF replay against Gazebo ground truth.

Feeds a SENSOR_LOG csv (see run_sim.py) through DroneEKF with time.time() patched to
the recorded timestamps, and reports estimate-vs-truth error. Lets EKF changes be
checked in seconds instead of one Gazebo crash per try.

usage: python testing/replay_ekf.py sensors_oracle1.csv [print_every]
"""
import sys
import os
import math
import csv
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


class Replay:
    mag_reference = MAG_REFERENCE_ENU

    def __init__(self, rows):
        self.rows = rows
        self.i = 0
        self.calibrating = True
        self.remapped = os.environ.get("MAG_REMAPPED", "1") == "1"

    def _row(self):
        # calibration reads before any timestamps matter - serve the first (at-rest) row
        return self.rows[0] if self.calibrating else self.rows[self.i]

    def get_gyro(self):
        r = self._row(); return (r["gx"], r["gy"], r["gz"])

    def get_accel(self):
        r = self._row(); return (r["ax"], r["ay"], r["az"])

    def get_mag(self):
        # csvs recorded before the 2026-09-30 mag fix hold the (wrong) remapped (-y, x, -z)
        # value - undo it to get the raw reading the bridge now passes through
        r = self._row()
        if self.remapped:
            return (r["my"], -r["mx"], -r["mz"])
        return (r["mx"], r["my"], r["mz"])

    def get_gps(self):
        r = self._row(); return (r["px"], r["py"], r["pz"], r["vx"], r["vy"], r["vz"])


def main():
    path = sys.argv[1]
    every = int(sys.argv[2]) if len(sys.argv) > 2 else 250
    with open(path) as f:
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]

    src = Replay(rows)
    clock = {"t": rows[0]["t"]}
    FINAL_gps.time.time = lambda: clock["t"]
    ekf = FINAL_gps.DroneEKF(sensors=src)
    src.calibrating = False
    # experiment knobs: EXP="R_ACC=1e6 QAB=1e-10 QATT=1e-7"
    for kv in os.environ.get("EXP", "").split():
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

    t0 = rows[0]["t"]
    att_err, pos_err = [], []
    t_end = float(os.environ.get("T_END", "1e9"))
    for i, r in enumerate(rows):
        if r["t"] - rows[0]["t"] > t_end:
            break
        src.i = i
        clock["t"] = r["t"]
        st = ekf.step()
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
    att_err = np.abs(np.array(att_err)); pos_err = np.abs(np.array(pos_err))
    print(f"att_err deg  rms={np.sqrt((att_err**2).mean(0)).round(2)} max={att_err.max(0).round(1)}")
    print(f"pos_err m    rms={np.sqrt((pos_err**2).mean(0)).round(2)} max={pos_err.max(0).round(2)}")


if __name__ == "__main__":
    main()
