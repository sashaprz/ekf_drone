"""PX4 flight log (.ulg) -> suite recording, for SHADOW MODE (2026-10-03).

Fly the drone on PX4; afterwards convert its log and replay your EKF on the real sensor
data, scored against a reference attitude/position:

  python3 testing/ulog_to_csv.py flight.ulg testing/data/shadow/flight1.csv
  python3 testing/replay_ekf.py testing/data/shadow/flight1.csv              # quick look
  python3 testing/suite/run_suite.py --missions none --extra testing/data/shadow/flight1.csv

Needs `pip install pyulog`. PX4 settings for useful logs (one-time, then reboot):
  SDLOG_PROFILE = 3   default set + "Estimator replay": IMU, mag, baro, GPS at FULL rate
                      (the default set logs mag/baro at only 5 Hz)
  SDLOG_MODE    = 1   log from boot, so the log contains the vehicle at rest before
                      arming - that stretch becomes the calibration dwell (.cal.csv)

Writes <csv> (same columns as run_sim.py's SENSOR_LOG), <csv>.cal.csv (calibration dwell)
and <csv>.meta.json (gps_latency_s, mag_reference, what "truth" is).

Frames: PX4 uses FRD body / NED world; this project uses FLU body / ENU world (gz_bridge).
  body FRD -> FLU: (x, -y, -z)          world NED -> ENU: (n, e, d) -> (e, n, -d)
  attitude: R_enu_flu = T_ned2enu @ R_ned_frd @ T_flu2frd
"Truth": SITL logs *_groundtruth topics (real truth); on hardware there are none and PX4's
own estimate (vehicle_attitude / vehicle_local_position) is the reference - meta says which.
The eq*/ep*/ev* columns hold PX4's estimate either way (so run_live-style comparisons work).
"""
import argparse
import json
import math
import os
import sys

import numpy as np

EARTH_RADIUS = 6371000.0  # same flat-earth conversion as gz_bridge.py
T_NED2ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
FRD2FLU = np.array([1.0, -1.0, -1.0])


def quat_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    tr = np.trace(R)
    if tr > 0:
        s = 2.0 * math.sqrt(tr + 1.0)
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
        q = [0.0] * 4
        q[0] = (R[k, j] - R[j, k]) / s
        q[1 + i] = 0.25 * s
        q[1 + j] = (R[j, i] + R[i, j]) / s
        q[1 + k] = (R[k, i] + R[i, k]) / s
    q = np.array(q)
    return q / np.linalg.norm(q)


def q_ned_frd_to_enu_flu(q):
    # PX4 vehicle_attitude.q rotates FRD body -> NED world
    return R_to_quat(T_NED2ENU @ quat_to_R(q) @ np.diag(FRD2FLU))


def ds(ulog, name):
    try:
        return ulog.get_dataset(name).data
    except (KeyError, IndexError, ValueError):
        return None


def vec(d, base, n=3):
    return np.stack([d[f"{base}[{i}]"] for i in range(n)], axis=1)


def hold(t_src, values, t_query):
    # zero-order hold: latest source sample at or before each query time (first sample before that)
    idx = np.clip(np.searchsorted(t_src, t_query, side="right") - 1, 0, len(t_src) - 1)
    return values[idx], idx


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ulg")
    ap.add_argument("csv")
    ap.add_argument("--cal-samples", type=int, default=200)
    a = ap.parse_args()
    from pyulog import ULog

    ulog = ULog(a.ulg)
    imu = ds(ulog, "sensor_combined")
    mag = ds(ulog, "vehicle_magnetometer")
    gps = ds(ulog, "vehicle_gps_position") or ds(ulog, "sensor_gps")
    baro = ds(ulog, "vehicle_air_data")
    att_gt, pos_gt = ds(ulog, "vehicle_attitude_groundtruth"), ds(ulog, "vehicle_local_position_groundtruth")
    att_ekf, pos_ekf = ds(ulog, "vehicle_attitude"), ds(ulog, "vehicle_local_position")
    if imu is None or mag is None or gps is None:
        sys.exit("log lacks sensor_combined / vehicle_magnetometer / vehicle_gps_position - see SDLOG_PROFILE above")

    t = imu["timestamp"] * 1e-6
    gyro = vec(imu, "gyro_rad") * FRD2FLU
    accel = vec(imu, "accelerometer_m_s2") * FRD2FLU
    tm = mag.get("timestamp_sample", mag["timestamp"]) * 1e-6
    m_raw, mi = hold(tm, vec(mag, "magnetometer_ga") * FRD2FLU, t)
    mgt = tm[mi]  # each row's mag-reading stamp - the EKF fuses each reading once

    # calibration dwell: first run of cal-samples IMU samples at rest (before arming)
    rest = (np.linalg.norm(gyro, axis=1) < 0.03) & (np.abs(np.linalg.norm(accel, axis=1) - 9.80665) < 0.3)
    run = np.convolve(rest.astype(int), np.ones(a.cal_samples, int), "valid")
    starts = np.flatnonzero(run == a.cal_samples)
    if len(starts) == 0:
        sys.exit("no at-rest stretch found for calibration - log from boot (SDLOG_MODE=1)")
    c0 = starts[0]
    cal = slice(c0, c0 + a.cal_samples)
    mag_scale = float(np.mean(np.linalg.norm(m_raw[cal], axis=1)))  # latched like gz_bridge
    m = m_raw / mag_scale
    flight = slice(c0 + a.cal_samples, len(t))

    # GPS -> local ENU (home = first 3D fix), zero-order held onto the IMU timeline
    tg = gps.get("timestamp_sample", gps["timestamp"]) * 1e-6
    good = gps.get("fix_type", np.full(len(tg), 3)) >= 3
    tg = tg[good]
    lat, lon = gps["latitude_deg"][good], gps["longitude_deg"][good]
    alt = gps["altitude_msl_m"][good]
    lat0, lon0, alt0 = lat[0], lon[0], alt[0]
    north = np.radians(lat - lat0) * EARTH_RADIUS
    east = np.radians(lon - lon0) * EARTH_RADIUS * math.cos(math.radians(lat0))
    g_enu = np.stack([east, north, alt - alt0, gps["vel_e_m_s"][good], gps["vel_n_m_s"][good], -gps["vel_d_m_s"][good]], axis=1)
    g_rows, _ = hold(tg, g_enu, t)

    # baro: height relative to the dwell, with its own sample stamp
    if baro is not None:
        tb = baro.get("timestamp_sample", baro["timestamp"]) * 1e-6
        b_h = baro["baro_alt_meter"] - np.interp(t[c0], tb, baro["baro_alt_meter"])
        bh, bi = hold(tb, b_h, t)
        bt = tb[bi]
        bh[t < tb[0]] = np.nan
    else:
        bt = bh = np.full(len(t), np.nan)

    # reference attitude/position: groundtruth if logged (SITL), else PX4's estimate
    def att_pos(att, pos):
        ta = att["timestamp"] * 1e-6
        q_ned = vec(att, "q", 4)
        qa, ia = hold(ta, q_ned, t)
        q = np.array([q_ned_frd_to_enu_flu(x) for x in qa])
        tp_ = pos["timestamp"] * 1e-6
        p_ned = np.stack([pos["x"], pos["y"], pos["z"]], axis=1)
        pp, ip = hold(tp_, p_ned, t)
        p = pp @ T_NED2ENU.T
        v = None
        if "vx" in pos:
            vv, _ = hold(tp_, np.stack([pos["vx"], pos["vy"], pos["vz"]], axis=1), t)
            v = vv @ T_NED2ENU.T
        return q, p, v, ta[ia]
    truth_src = "groundtruth" if (att_gt is not None and pos_gt is not None) else "px4_ekf2"
    tq, tpos, _, tt = att_pos(att_gt, pos_gt) if truth_src == "groundtruth" else att_pos(att_ekf, pos_ekf)
    eq, epos, evel, _ = att_pos(att_ekf, pos_ekf)

    # mag reference: the earth-field direction in ENU, measured like MAG_REFERENCE_ENU was
    # (rotate body readings by the reference attitude and average)
    w = np.array([quat_to_R(q) @ mm for q, mm in zip(tq[flight][::10], m[flight][::10])])
    mag_ref = w.mean(0)
    mag_ref = mag_ref / np.linalg.norm(mag_ref)
    spread = float(np.degrees(np.arccos(np.clip(w @ mag_ref / np.linalg.norm(w, axis=1), -1, 1))).std())

    params = ulog.initial_parameters
    gps_delay_ms = params.get("EKF2_GPS_DELAY", 110.0)

    os.makedirs(os.path.dirname(os.path.abspath(a.csv)), exist_ok=True)
    cols = ["t", "gx", "gy", "gz", "ax", "ay", "az", "mx", "my", "mz", "px", "py", "pz", "vx", "vy", "vz",
            "tqw", "tqx", "tqy", "tqz", "tpx", "tpy", "tpz", "mt", "spx", "spy", "spz", "spyaw",
            "eqw", "eqx", "eqy", "eqz", "epx", "epy", "epz", "evx", "evy", "evz", "tw", "tt", "bt", "bh", "mgt"]
    f0 = flight.start
    n = len(t) - f0
    z = np.zeros(n)
    evel = evel if evel is not None else np.zeros((len(t), 3))
    data = np.column_stack([t[f0:], gyro[f0:], accel[f0:], m[f0:], g_rows[f0:], tq[f0:], tpos[f0:],
                            t[f0:] - t[f0], z, z, z, z, eq[f0:], epos[f0:], evel[f0:], t[f0:], tt[f0:], bt[f0:], bh[f0:], mgt[f0:]])
    np.savetxt(a.csv, data, delimiter=",", header=",".join(cols), comments="", fmt="%.6f")
    with open(a.csv + ".cal.csv", "w") as fcal:
        fcal.write("gx,gy,gz,mx,my,mz,ax,ay,az\n")
        for k in range(cal.start, cal.stop):
            fcal.write(",".join(f"{v:.6f}" for v in (*gyro[k], *m[k], *accel[k])) + "\n")
    meta = {"source": os.path.basename(a.ulg), "truth": truth_src, "gps_latency_s": gps_delay_ms / 1000.0,
            "mag_reference": [round(float(x), 4) for x in mag_ref], "mag_reference_spread_deg": round(spread, 2),
            "mag_scale_gauss": mag_scale, "cal_window_s": [float(t[cal.start] - t[0]), float(t[cal.stop - 1] - t[0])],
            "imu_hz": float(1.0 / np.median(np.diff(t))), "mag_hz": float(1.0 / np.median(np.diff(tm))),
            "gps_hz": float(1.0 / np.median(np.diff(tg))),
            "baro_hz": float(1.0 / np.median(np.diff(tb))) if baro is not None else None}
    with open(a.csv + ".meta.json", "w") as fm:
        json.dump(meta, fm, indent=1)
    print(f"{n} rows -> {a.csv}")
    print(json.dumps(meta, indent=1))


if __name__ == "__main__":
    main()
