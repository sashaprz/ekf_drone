"""PX4 <-> this project's EKF, over MAVLink - LIVE SHADOW MODE (2026-10-04).

Same interface as gz_bridge.GazeboBridge, so run_sim.py's loop, logging and failsafes run
unchanged on the Raspberry Pi while PX4 flies the drone:

  BRIDGE=px4 PX4_URL=/dev/serial0 PX4_BAUD=921600 SENSOR_LOG=shadow1.csv RUN_SECONDS=300 python3 run_sim.py
  BRIDGE=px4 PX4_URL=udpin:0.0.0.0:14540 ...      # PX4 SITL's onboard link (testing)

SHADOW ONLY: publish_motors() never sends anything. Your EKF + controller run and are logged
next to PX4's own estimate (logged as the "truth"/reference columns), so the recording replays
through testing/replay_ekf.py and the suite like any other.

Data (PX4 -> MAVLink, requested at connect with MAV_CMD_SET_MESSAGE_INTERVAL):
  HIGHRES_IMU          250 Hz  gyro/accel (FRD), mag (gauss, FRD), abs_pressure - with
                               time_usec and fields_updated (which fields are NEW this message)
  GPS_RAW_INT           10 Hz  lat/lon/alt + ground speed/course (no vertical velocity -> NaN,
                               the EKF fuses partial fixes)
  ATTITUDE_QUATERNION,
  LOCAL_POSITION_NED    50 Hz  PX4's estimate = the reference
All stamps are PX4 boot time (time_usec / time_boot_ms), so the EKF's dt comes from the sensor,
not from when a message happened to arrive over the link.

PX4 side (Pixhawk TELEM2 -> Pi UART): MAV_1_CONFIG = TELEM 2, MAV_1_MODE = Onboard,
SER_TEL2_BAUD = 921600. The link has to carry ~250 HIGHRES_IMU/s (~16 kB/s) - fine at 921600.

Frames: PX4 FRD body / NED world -> this project's FLU body / ENU world (same as
testing/ulog_to_csv.py). Mag field direction: measured at startup from PX4's attitude while at
rest (or PX4_MAG_REF="x,y,z" in ENU). GPS latency: PX4's EKF2_GPS_DELAY (or EKF_GPS_LATENCY_MS).
"""
import collections
import math
import os
import threading
import time

import numpy as np
from pymavlink import mavutil

EARTH_RADIUS = 6371000.0
T_NED2ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
FRD2FLU = np.array([1.0, -1.0, -1.0])
MSG_RATES_HZ = {"HIGHRES_IMU": 250, "GPS_RAW_INT": 10, "ATTITUDE_QUATERNION": 50, "LOCAL_POSITION_NED": 50}
MSG_IDS = {"HIGHRES_IMU": 105, "GPS_RAW_INT": 24, "ATTITUDE_QUATERNION": 31, "LOCAL_POSITION_NED": 32}
HIGHRES_ACC_GYRO = 0x3F    # fields_updated bits 0-5
HIGHRES_MAG = 0x1C0        # bits 6-8
HIGHRES_PRESS = 0x200      # bit 9


def _quat_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _R_to_quat(R):
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


class _NoDegrade:
    # run_sim prints bridge.degrade.describe(); sensors here are real
    active = realistic = False
    latency = 0.0

    def describe(self):
        return "SENSORS: PX4 over MAVLink (real data) - SHADOW MODE, motor commands are NOT sent"


class Px4Bridge:
    shadow = True            # run_sim: never let the failsafe end the run / never send motors
    use_timestamps = True    # run_sim: queue IMU samples, EKF dt from PX4's stamps

    def __init__(self, url=None, baud=None, startup_timeout=60.0):
        url = url or os.environ.get("PX4_URL", "udpin:0.0.0.0:14540")
        baud = baud or int(os.environ.get("PX4_BAUD", "921600"))
        self.degrade = _NoDegrade()
        self._m = mavutil.mavlink_connection(url, baud=baud, source_system=1, source_component=191)
        print(f"PX4 bridge: waiting for heartbeat on {url} ...", flush=True)
        if self._m.wait_heartbeat(timeout=startup_timeout) is None:
            raise RuntimeError(f"no MAVLink heartbeat on {url}")
        self._imu_event = threading.Event()
        self._imu_queue = collections.deque(maxlen=5000)
        self._imu_queued = False
        self._imu_time = None
        self._gyro = (0.0, 0.0, 0.0)
        self._accel = (0.0, 0.0, 0.0)
        self._mag = (1.0, 0.0, 0.0)
        self._mag_time = None
        self._mag_scale = None
        self._baro = None
        self._p0 = None
        self._gps = None
        self._gps_time = None
        self._home = None
        self._ref_q = self._ref_pos = self._ref_stamp = None
        self._motor_warned = False
        self.counts = collections.Counter()
        self._request_streams()
        self.gps_latency = self._gps_latency()
        self._stop = False
        threading.Thread(target=self._reader, daemon=True).start()
        self.mag_reference = self._mag_reference(startup_timeout)
        print(f"PX4 bridge ready: mag_reference {np.round(self.mag_reference, 3)} (ENU), "
              f"gps_latency {self.gps_latency * 1000:.0f} ms", flush=True)

    # ---- setup -------------------------------------------------------------------------
    def _request_streams(self):
        for name, hz in MSG_RATES_HZ.items():
            self._m.mav.command_long_send(self._m.target_system, self._m.target_component,
                                          mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                          MSG_IDS[name], int(1e6 / hz), 0, 0, 0, 0, 0)

    def _gps_latency(self):
        if os.environ.get("EKF_GPS_LATENCY_MS"):
            return float(os.environ["EKF_GPS_LATENCY_MS"]) / 1000.0
        # parameters live on the autopilot component (1), whatever component sent the first heartbeat
        for _ in range(3):
            self._m.mav.param_request_read_send(self._m.target_system, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                                                b"EKF2_GPS_DELAY", -1)
            t_end = time.time() + 2
            while time.time() < t_end:
                msg = self._m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
                if msg is not None and str(msg.param_id).strip("\x00") == "EKF2_GPS_DELAY":
                    return float(msg.param_value) / 1000.0
        # PX4 <= 1.15 has EKF2_GPS_DELAY; newer PX4 (e.g. 1.18-dev) dropped it and time-stamps GPS
        # samples itself - set EKF_GPS_LATENCY_MS to your receiver's latency if you know it
        print("PX4 bridge: no EKF2_GPS_DELAY on this PX4 (newer versions dropped it) - assuming 110 ms; "
              "override with EKF_GPS_LATENCY_MS", flush=True)
        return 0.110

    def _mag_reference(self, timeout):
        # earth-field direction in ENU: rotate body mag readings by PX4's attitude while at rest
        # (the same measurement that produced gz_bridge.MAG_REFERENCE_ENU)
        env = os.environ.get("PX4_MAG_REF")
        if env:
            v = np.array([float(x) for x in env.split(",")])
            return tuple(v / np.linalg.norm(v))
        acc, t_end = [], time.time() + timeout
        while len(acc) < 100 and time.time() < t_end:
            time.sleep(0.02)
            if self._ref_q is not None and self._mag_time is not None:
                acc.append(_quat_to_R(self._ref_q) @ np.asarray(self._mag))
        if len(acc) < 20:
            raise RuntimeError("no attitude/mag data from PX4 to measure the field direction - set PX4_MAG_REF")
        v = np.mean(acc, axis=0)
        return tuple(v / np.linalg.norm(v))

    # ---- receive -----------------------------------------------------------------------
    def _reader(self):
        handlers = {"HIGHRES_IMU": self._on_imu, "GPS_RAW_INT": self._on_gps,
                    "ATTITUDE_QUATERNION": self._on_att, "LOCAL_POSITION_NED": self._on_pos}
        while not self._stop:
            msg = self._m.recv_match(type=list(handlers), blocking=True, timeout=0.5)
            if msg is None:
                continue
            self.counts[msg.get_type()] += 1
            handlers[msg.get_type()](msg)

    def _on_imu(self, msg):
        stamp = msg.time_usec * 1e-6
        f = msg.fields_updated
        if f & HIGHRES_MAG:
            m = np.array([msg.xmag, msg.ymag, msg.zmag]) * FRD2FLU
            n = np.linalg.norm(m)
            if n > 0:
                self._mag_scale = self._mag_scale or n   # latched like gz_bridge
                self._mag = tuple(m / self._mag_scale)
                self._mag_time = stamp
        if f & HIGHRES_PRESS and msg.abs_pressure > 0:
            self._p0 = self._p0 or msg.abs_pressure
            self._baro = (stamp, 44330.0 * (1.0 - (msg.abs_pressure / self._p0) ** (1.0 / 5.255)))
        if f & HIGHRES_ACC_GYRO:
            sample = (stamp, tuple(np.array([msg.xgyro, msg.ygyro, msg.zgyro]) * FRD2FLU),
                      tuple(np.array([msg.xacc, msg.yacc, msg.zacc]) * FRD2FLU))
            self._imu_queue.append(sample)
            if not self._imu_queued:
                self._imu_time, self._gyro, self._accel = sample
            self._imu_event.set()

    def _on_gps(self, msg):
        if msg.fix_type < 3:
            return
        lat, lon, alt = msg.lat * 1e-7, msg.lon * 1e-7, msg.alt * 1e-3
        if self._home is None:
            self._home = (lat, lon, alt)
        lat0, lon0, alt0 = self._home
        north = math.radians(lat - lat0) * EARTH_RADIUS
        east = math.radians(lon - lon0) * EARTH_RADIUS * math.cos(math.radians(lat0))
        if msg.vel != 65535 and msg.cog != 65535:
            spd, cog = msg.vel / 100.0, math.radians(msg.cog / 100.0)
            ve, vn = spd * math.sin(cog), spd * math.cos(cog)
        else:
            ve = vn = float("nan")
        # GPS_RAW_INT has no vertical velocity -> NaN (FINAL_gps fuses partial fixes)
        self._gps = (east, north, alt - alt0, ve, vn, float("nan"))
        self._gps_time = msg.time_usec * 1e-6  # the EKF rejects a repeated fix (get_gps_time)

    def _on_att(self, msg):
        q_ned_frd = (msg.q1, msg.q2, msg.q3, msg.q4)
        self._ref_q = tuple(_R_to_quat(T_NED2ENU @ _quat_to_R(q_ned_frd) @ np.diag(FRD2FLU)))
        self._ref_stamp = msg.time_boot_ms * 1e-3

    def _on_pos(self, msg):
        self._ref_pos = tuple(T_NED2ENU @ np.array([msg.x, msg.y, msg.z]))

    # ---- gz_bridge interface -------------------------------------------------------------
    def wait_for_imu(self, timeout=0.1):
        got = self._imu_event.wait(timeout)
        self._imu_event.clear()
        return got

    def drain_imu(self, timeout=0.1):
        self._imu_queued = True
        if not self._imu_queue:
            self._imu_event.wait(timeout)
        self._imu_event.clear()
        out = []
        while self._imu_queue:
            out.append(self._imu_queue.popleft())
        return out

    def use_imu_sample(self, sample):
        self._imu_time, self._gyro, self._accel = sample

    def get_imu_time(self):
        return self._imu_time

    def get_gyro(self):
        return self._gyro

    def get_accel(self):
        return self._accel

    def get_mag(self):
        return self._mag

    def get_mag_time(self):
        return self._mag_time

    def get_baro(self):
        return self._baro

    def get_gps(self):
        return self._gps

    def get_gps_time(self):
        return self._gps_time

    def get_true_pose(self):
        # PX4's own estimate - the reference in shadow mode (there is no ground truth on hardware)
        return self._ref_q, self._ref_pos

    def get_true_stamp(self):
        return self._ref_stamp

    def publish_motors(self, m1, m2, m3, m4):
        # SHADOW MODE: never command the vehicle from here
        if not self._motor_warned:
            print("PX4 bridge (shadow): motor commands are computed and logged, NOT sent", flush=True)
            self._motor_warned = True

    def close(self):
        self._stop = True
