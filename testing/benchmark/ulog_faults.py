"""Inject a sensor fault into a PX4 .ulg by rewriting sensor values IN PLACE (same bytes,
same message sizes, same timestamps), so that PX4's EKF2 replay and this project's filters
(via ulog_to_csv.py) read exactly the same corrupted data.

    python3 testing/benchmark/ulog_faults.py in.ulg out.ulg <fault> [--seed N]

Faults (magnitudes follow testing/suite/faults.py - "cheap hobby hardware"):
  none                 copy unchanged
  gyro_bias_step       +-1 deg/s on every axis, appearing at takeoff (after any calibration)
  gyro_drift           bias ramping 0 -> +-0.5 deg/s over 60 s from takeoff
  accel_bias_step      +0.2 m/s^2 on body x from takeoff
  gps_dropout          no fix (fix_type 0) for 15 s, starting 25 s after takeoff
  gps_outliers         2 % of fixes jump 10-50 m horizontally, in flight
  mag_bias             hard-iron offset of 15 % of the field, whole log (incl. the dwell)
  mag_interference     10 s disturbance of 50 % of the field, fixed body direction, from
                       takeoff + 20 s (e.g. a current-carrying wire near the compass)
  imu_spikes           0.1 % of IMU samples hit by 5 rad/s gyro / 30 m/s^2 accel spikes, in flight

Field alignment (--align-field DECL,INCL, applied before any fault, for every condition):
  EKF2 always takes Earth's field from the World Magnetic Model at the GPS position
  (mag_control.cpp resets its field states to the WMM vector; EKF2_MAG_DECL is ignored when
  GPS is valid), but Gazebo's simulated field points ~6 deg elsewhere. Each magnetometer
  sample is rotated into the world frame with the ground-truth attitude, rotated from
  Gazebo's field direction onto the WMM direction, and rotated back - so the true field
  becomes the one EKF2 assumes, and the other filters measure that same field from the log.

ULog format (docs.px4.io "ULog File Format"): 16-byte header, then messages
[uint16 size][uint8 type][payload]. 'F' = format "name:type field;..." , 'A' = subscription
(multi_id, msg_id, name), 'D' = data (msg_id + packed fields in format order).
"""
import argparse
import hashlib
import math
import struct
import sys

import numpy as np

SIZES = {"int8_t": 1, "uint8_t": 1, "bool": 1, "char": 1, "int16_t": 2, "uint16_t": 2, "int32_t": 4,
         "uint32_t": 4, "float": 4, "int64_t": 8, "uint64_t": 8, "double": 8}
CODES = {"int8_t": "b", "uint8_t": "B", "bool": "?", "char": "c", "int16_t": "h", "uint16_t": "H",
         "int32_t": "i", "uint32_t": "I", "float": "f", "int64_t": "q", "uint64_t": "Q", "double": "d"}
G = 9.80665
FAULTS = ["none", "gyro_bias_step", "gyro_drift", "accel_bias_step", "gps_dropout", "gps_outliers",
          "mag_bias", "mag_interference", "imu_spikes"]


class ULogPatch:
    def __init__(self, data):
        self.b = bytearray(data)
        assert self.b[:7] == b"ULog\x01\x12\x35", "not a ULog file"
        self.formats, self.subs, self.msgs = {}, {}, {}
        p = 16
        while p + 3 <= len(self.b):
            size, typ = struct.unpack_from("<HB", self.b, p)
            pay = p + 3
            if pay + size > len(self.b):
                break
            if typ == ord("F"):
                name, fields = self.b[pay:pay + size].decode().split(":", 1)
                self.formats[name] = [f.split(" ") for f in fields.split(";") if f]
            elif typ == ord("A"):
                multi, mid = struct.unpack_from("<BH", self.b, pay)
                self.subs[mid] = (self.b[pay + 3:pay + size].decode(), multi)
            elif typ == ord("D"):
                mid = struct.unpack_from("<H", self.b, pay)[0]
                self.msgs.setdefault(mid, []).append((pay + 2, size - 2))
            p = pay + size

    def _size(self, fmt):
        return sum(self._field_size(t) for t, _ in self.formats[fmt])

    def _field_size(self, t):
        base, n = (t.split("[")[0], int(t.split("[")[1][:-1])) if "[" in t else (t, 1)
        return n * (SIZES[base] if base in SIZES else self._size(base))

    def offset(self, topic, field):
        """byte offset of `field` inside one data payload of `topic`, and its (type, count)"""
        off = 0
        for t, name in self.formats[topic]:
            if name == field:
                base, n = (t.split("[")[0], int(t.split("[")[1][:-1])) if "[" in t else (t, 1)
                return off, base, n
            off += self._field_size(t)
        raise KeyError(f"{topic}.{field}")

    def instances(self, topic):
        return [mid for mid, (n, _) in self.subs.items() if n == topic]

    def read(self, topic, field, multi=None):
        """-> (positions, values (N, n)) for every data message of topic (all instances)"""
        off, base, n = self.offset(topic, field)
        fmt = "<" + CODES[base] * n
        pos, vals = [], []
        for mid in self.instances(topic):
            if multi is not None and self.subs[mid][1] != multi:
                continue
            for p, size in self.msgs.get(mid, []):
                if off + SIZES[base] * n <= size:
                    pos.append(p)
                    vals.append(struct.unpack_from(fmt, self.b, p + off))
        order = np.argsort(pos)
        return np.array(pos)[order], np.array(vals, dtype=float)[order]

    def write(self, topic, field, pos, vals):
        off, base, n = self.offset(topic, field)
        fmt = "<" + CODES[base] * n
        for p, v in zip(pos, vals):
            v = [int(round(x)) if base not in ("float", "double") else float(x) for x in np.atleast_1d(v)]
            struct.pack_into(fmt, self.b, int(p) + off, *v)


def _rng(seed, name):
    return np.random.default_rng(int(hashlib.sha1(f"{seed}:{name}".encode()).hexdigest()[:8], 16))


def _signs(rng):
    return np.where(rng.random(3) < 0.5, -1.0, 1.0)


def takeoff_time(u):
    """first groundtruth sample more than 0.3 m above the start (us); falls back to land detector"""
    for topic in ("vehicle_local_position_groundtruth", "vehicle_local_position"):
        if u.instances(topic):
            _, ts = u.read(topic, "timestamp")
            _, z = u.read(topic, "z")
            up = np.flatnonzero(z[:, 0] < z[0, 0] - 0.3)
            if len(up):
                return ts[up[0], 0]
    sys.exit("no takeoff found")


def _quat_to_R(q):
    w, x, y, z = q.T
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
                     np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
                     np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], 1)


def _rot_between(a, b):
    # smallest rotation taking unit vector a onto unit vector b (Rodrigues)
    v, c = np.cross(a, b), float(a @ b)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K / (1 + c)


def align_field(u, decl_deg, incl_deg):
    """Rotate every mag sample so the world-frame field points along the WMM direction
    (NED: declination east of north, inclination down). Returns the rotation angle (deg)."""
    _, tg = u.read("vehicle_attitude_groundtruth", "timestamp")
    _, qg = u.read("vehicle_attitude_groundtruth", "q")
    for k in range(1, len(qg)):  # same hemisphere for interpolation
        if qg[k] @ qg[k - 1] < 0:
            qg[k] = -qg[k]
    pos, tm = u.read("vehicle_magnetometer", "timestamp")
    _, m = u.read("vehicle_magnetometer", "magnetometer_ga")
    q = np.stack([np.interp(tm[:, 0], tg[:, 0], qg[:, j]) for j in range(4)], 1)
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    R = _quat_to_R(q)                                  # body FRD -> NED
    h = np.einsum("nij,nj->ni", R, m)                  # measured field in NED
    gz_dir = np.median(h / np.linalg.norm(h, axis=1, keepdims=True), axis=0)
    gz_dir /= np.linalg.norm(gz_dir)
    D, I = math.radians(decl_deg), math.radians(incl_deg)
    wmm_dir = np.array([math.cos(I) * math.cos(D), math.cos(I) * math.sin(D), math.sin(I)])
    Q = _rot_between(gz_dir, wmm_dir)
    m_new = np.einsum("nji,nj->ni", R, h @ Q.T)        # R^T (Q h)
    u.write("vehicle_magnetometer", "magnetometer_ga", pos, m_new)
    return math.degrees(math.acos(np.clip(gz_dir @ wmm_dir, -1, 1)))


def apply(u, fault, seed):
    rng = _rng(seed, fault)
    t_to = takeoff_time(u)
    info = {"fault": fault, "seed": seed, "takeoff_us": int(t_to)}
    if fault == "none":
        return info

    if fault in ("gyro_bias_step", "gyro_drift", "accel_bias_step", "imu_spikes"):
        pos, ts = u.read("sensor_combined", "timestamp")
        t = (ts[:, 0] - t_to) * 1e-6
        on = t >= 0
        if fault == "gyro_bias_step":
            b = math.radians(1.0) * _signs(rng)
            _, g = u.read("sensor_combined", "gyro_rad")
            g[on] += b
            u.write("sensor_combined", "gyro_rad", pos, g)
            info["bias_rad_s"] = b.tolist()
        elif fault == "gyro_drift":
            b = math.radians(0.5) * _signs(rng)
            _, g = u.read("sensor_combined", "gyro_rad")
            g += np.clip(t / 60.0, 0, 1)[:, None] * b
            u.write("sensor_combined", "gyro_rad", pos, g)
            info["final_bias_rad_s"] = b.tolist()
        elif fault == "accel_bias_step":
            _, a = u.read("sensor_combined", "accelerometer_m_s2")
            a[on, 0] += 0.2
            u.write("sensor_combined", "accelerometer_m_s2", pos, a)
        else:
            _, g = u.read("sensor_combined", "gyro_rad")
            _, a = u.read("sensor_combined", "accelerometer_m_s2")
            hit = (rng.random(len(t)) < 0.001) & on
            g[hit] += rng.choice([-1, 1], (hit.sum(), 3)) * 5.0
            a[hit] += rng.choice([-1, 1], (hit.sum(), 3)) * 30.0
            u.write("sensor_combined", "gyro_rad", pos, g)
            u.write("sensor_combined", "accelerometer_m_s2", pos, a)
            info["spikes"] = int(hit.sum())
        return info

    if fault in ("gps_dropout", "gps_outliers"):
        # the topic EKF2 replay fuses and ulog_to_csv.py reads (sensor_gps only if it's absent)
        for topic in [next(t for t in ("vehicle_gps_position", "sensor_gps") if u.instances(t))]:
            pos, ts = u.read(topic, "timestamp")
            t = (ts[:, 0] - t_to) * 1e-6
            r = _rng(seed, fault)  # same draw for both topics: they carry the same fixes
            if fault == "gps_dropout":
                off = (t >= 25.0) & (t < 40.0)
                _, fix = u.read(topic, "fix_type")
                fix[off] = 0
                u.write(topic, "fix_type", pos, fix)
                info["window_s_after_takeoff"] = [25.0, 40.0]
            else:
                hit = (r.random(len(t)) < 0.02) & (t >= 0)
                _, lat = u.read(topic, "latitude_deg")
                _, lon = u.read(topic, "longitude_deg")
                ang = r.uniform(0, 2 * math.pi, len(t))
                dist = r.uniform(10, 50, len(t))
                lat[hit, 0] += np.degrees(dist[hit] * np.cos(ang[hit]) / 6371000.0)
                lon[hit, 0] += np.degrees(dist[hit] * np.sin(ang[hit]) / 6371000.0 / math.cos(math.radians(lat[0, 0])))
                u.write(topic, "latitude_deg", pos, lat)
                u.write(topic, "longitude_deg", pos, lon)
                info["outliers"] = int(hit.sum())
        return info

    if fault in ("mag_bias", "mag_interference"):
        pos, ts = u.read("vehicle_magnetometer", "timestamp")
        _, m = u.read("vehicle_magnetometer", "magnetometer_ga")
        strength = float(np.median(np.linalg.norm(m, axis=1)))
        d = rng.normal(size=3)
        d /= np.linalg.norm(d)
        if fault == "mag_bias":
            m += 0.15 * strength * d
        else:
            t = (ts[:, 0] - t_to) * 1e-6
            on = (t >= 20.0) & (t < 30.0)
            m[on] += 0.5 * strength * d
            info["window_s_after_takeoff"] = [20.0, 30.0]
        u.write("vehicle_magnetometer", "magnetometer_ga", pos, m)
        info["direction_body"] = d.tolist()
        return info
    raise ValueError(fault)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("inp")
    ap.add_argument("out")
    ap.add_argument("fault", choices=FAULTS)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--align-field", default=None, help="DECL_DEG,INCL_DEG of the WMM field EKF2 uses")
    a = ap.parse_args()
    u = ULogPatch(open(a.inp, "rb").read())
    rot = None
    if a.align_field:
        rot = align_field(u, *(float(x) for x in a.align_field.split(",")))
    info = apply(u, a.fault, a.seed)
    info["field_rotated_deg"] = rot
    open(a.out, "wb").write(u.b)
    import json
    print(json.dumps(info))


if __name__ == "__main__":
    main()
