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

# ---- OFFBOARD=1 (2026-11-02): your EKF + position/velocity loops fly, PX4's attitude/rate
# loops execute. run_sim.py sends one attitude quaternion + collective thrust per control
# tick (SET_ATTITUDE_TARGET); PX4 tracks it with its own (tuned) inner loops. Your cascade's
# attitude/rate loops and mixer are not used in this mode - motor-level control comes later.
#
# Engagement, all from the main loop (offboard_tick): stream neutral setpoints PRESTREAM_S
# (PX4 refuses OFFBOARD without a live stream) -> request OFFBOARD -> arm -> ENGAGED.
# After that, if PX4 leaves OFFBOARD for any reason we didn't ask for (RC mode switch, PX4
# failsafe, low battery), we go to OVERRIDE and never send a command again - the pilot or
# PX4 always wins. Setpoints are only ever sent from the main loop, so a hung loop stops
# the stream and PX4's own offboard-loss failsafe (COM_OF_LOSS_T) takes over.
OFFBOARD = os.environ.get("OFFBOARD") == "1"
PX4_MAIN_POSCTL, PX4_MAIN_AUTO, PX4_MAIN_OFFBOARD = 3, 4, 6
PX4_AUTO_LOITER, PX4_AUTO_LAND = 3, 6
PRESTREAM_S = 1.0
ENGAGE_TIMEOUT_S = 15.0
SETPOINT_HZ = 50.0
MAX_THRUST = 0.95
# where control goes when WE give it up (FALLBACK, watchdog, exception): PX4 lands by
# itself ("land", default) or holds position ("hold") for the pilot to take over
FALLBACK_MODES = {"land": (PX4_MAIN_AUTO, PX4_AUTO_LAND), "hold": (PX4_MAIN_AUTO, PX4_AUTO_LOITER)}
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


def _yaw_quat(yaw):
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def thrust_command(rel, hover):
    """collective thrust for SET_ATTITUDE_TARGET from the cascade's thrust.
    rel = cascade thrust / VEL_Z_HOVER_THRUST_FF - the cascade works in Gazebo rotor speed
    (rad/s), and rotor force goes with speed squared, so rel**2 is the force as a fraction
    of hover force. PX4 treats its normalized thrust as proportional to force (that's what
    THR_MDL_FAC calibrates), so the command is hover * rel**2. A wrong `hover` shows up as
    a steady vel_z error that the vel_z integrator trims out."""
    return float(np.clip(hover * max(rel, 0.0) ** 2, 0.0, MAX_THRUST))


class _NoDegrade:
    # run_sim prints bridge.degrade.describe(); sensors here are real
    active = realistic = False
    latency = 0.0

    def describe(self):
        if OFFBOARD:
            return "SENSORS: PX4 over MAVLink - OFFBOARD: your EKF + outer loops command PX4's attitude controller"
        return "SENSORS: PX4 over MAVLink (real data) - SHADOW MODE, motor commands are NOT sent"


class Px4Bridge:
    use_timestamps = True    # run_sim: queue IMU samples, EKF dt from PX4's stamps
    # shadow (run_sim: never let the failsafe end the run / never send motors) unless OFFBOARD=1
    shadow = not OFFBOARD
    offboard = OFFBOARD

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
        self._send_lock = threading.Lock()   # the watchdog thread may send (hand_back) too
        self.px4_armed = False
        self.px4_main = self.px4_sub = None
        self.ob_state = "IDLE"
        self._ob_t0 = self._ob_start = self._ob_retry = None
        self._last_sp_t = self._last_hb_t = -math.inf
        self._request_streams()
        self.gps_latency = self._gps_latency()
        if OFFBOARD:
            # hover thrust: OFFBOARD_HOVER_THR (measure it - PX4 logs hover_thrust_estimate in
            # every hover), else PX4's MPC_THR_HOVER. SITL x500: true hover is ~0.71 (757 rad/s
            # on gz's linear 150..1000 rad/s command map) while MPC_THR_HOVER defaults to 0.6.
            env = os.environ.get("OFFBOARD_HOVER_THR")
            self.hover_thrust = float(env) if env else (self._read_param("MPC_THR_HOVER") or 0.5)
            self.fallback_mode = FALLBACK_MODES[os.environ.get("OFFBOARD_FALLBACK", "land")]
            print(f"PX4 bridge: OFFBOARD - hover thrust {self.hover_thrust:.3f} "
                  f"({'OFFBOARD_HOVER_THR' if env else 'MPC_THR_HOVER'}), "
                  f"fallback {os.environ.get('OFFBOARD_FALLBACK', 'land')}", flush=True)
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

    def _read_param(self, name, tries=3):
        # parameters live on the autopilot component (1), whatever component sent the first
        # heartbeat. Only before the reader thread starts (both would read the link).
        for _ in range(tries):
            self._m.mav.param_request_read_send(self._m.target_system, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                                                name.encode(), -1)
            t_end = time.time() + 2
            while time.time() < t_end:
                msg = self._m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
                if msg is not None and str(msg.param_id).strip("\x00") == name:
                    return float(msg.param_value)
        return None

    def _gps_latency(self):
        if os.environ.get("EKF_GPS_LATENCY_MS"):
            return float(os.environ["EKF_GPS_LATENCY_MS"]) / 1000.0
        delay_ms = self._read_param("EKF2_GPS_DELAY")
        if delay_ms is not None:
            return delay_ms / 1000.0
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
                    "ATTITUDE_QUATERNION": self._on_att, "LOCAL_POSITION_NED": self._on_pos,
                    "HEARTBEAT": self._on_heartbeat, "STATUSTEXT": self._on_statustext,
                    "COMMAND_ACK": self._on_ack}
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

    def _on_heartbeat(self, msg):
        # PX4's own heartbeat only (a GCS on the same link sends them too)
        if msg.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1 or msg.type == mavutil.mavlink.MAV_TYPE_GCS:
            return
        self.px4_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        # PX4 custom_mode: bits 16-23 main mode, 24-31 sub mode
        self.px4_main, self.px4_sub = (msg.custom_mode >> 16) & 0xFF, (msg.custom_mode >> 24) & 0xFF

    def _on_statustext(self, msg):
        # PX4's own explanations ("Arming denied: ...", "Failsafe activated", ...)
        if OFFBOARD:
            print(f"PX4: {msg.text}", flush=True)

    def _on_ack(self, msg):
        if OFFBOARD and msg.command in (mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM) \
                and msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print(f"PX4 bridge: command {msg.command} rejected (result {msg.result})", flush=True)

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
        # never commands the vehicle - shadow mode, and OFFBOARD sends attitude setpoints
        # (publish_attitude) instead of motor speeds
        if not self._motor_warned:
            print("PX4 bridge: motor commands are computed and logged, NOT sent", flush=True)
            self._motor_warned = True

    # ---- OFFBOARD ------------------------------------------------------------------------
    def _command(self, cmd, *params):
        p = list(params) + [0.0] * (7 - len(params))
        with self._send_lock:
            self._m.mav.command_long_send(self._m.target_system, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, cmd, 0, *p)

    def _set_mode(self, main, sub=0):
        self._command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main, sub)

    def _send_attitude(self, q_enu_flu, thrust):
        # this project's ENU world / FLU body -> PX4's NED / FRD (both maps are their own inverse)
        q = _R_to_quat(T_NED2ENU @ _quat_to_R(q_enu_flu) @ np.diag(FRD2FLU))
        ignore_rates = 0b111   # type_mask: body roll/pitch/yaw rates not set -> PX4 tracks q
        with self._send_lock:
            self._m.mav.set_attitude_target_send(int(time.monotonic() * 1000) & 0xFFFFFFFF, self._m.target_system,
                                                 mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, ignore_rates,
                                                 [float(x) for x in q], 0.0, 0.0, 0.0, float(thrust))

    def _heartbeat(self, now):
        # 1 Hz "companion computer alive", from the main loop only (a hung loop goes silent)
        if now - self._last_hb_t >= 1.0:
            with self._send_lock:
                self._m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                           mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
            self._last_hb_t = now

    def _neutral(self):
        # level at PX4's current heading, zero thrust: what the pre-engage stream asks for
        yaw = 0.0
        if self._ref_q is not None:
            w, x, y, z = self._ref_q
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return _yaw_quat(yaw), 0.0

    def offboard_tick(self, now, ready=True):
        """engagement state machine - call every main-loop iteration; returns the state.
        IDLE -> PRESTREAM -> MODE (OFFBOARD requested) -> ARM -> ENGAGED, then OVERRIDE if PX4
        leaves OFFBOARD/disarms without us asking. FAILED = didn't engage in ENGAGE_TIMEOUT_S.
        ready=False holds in IDLE (run_sim: only engage while the health monitor is NORMAL)."""
        self._heartbeat(now)
        s = self.ob_state
        if s == "IDLE" and ready:
            self._ob_start = self._ob_t0 = now
            self.ob_state = "PRESTREAM"
            print("OFFBOARD: streaming neutral setpoints", flush=True)
        elif s == "PRESTREAM" and now - self._ob_t0 >= PRESTREAM_S:
            self._set_mode(PX4_MAIN_OFFBOARD)
            self._ob_retry, self.ob_state = now, "MODE"
            print("OFFBOARD: requesting OFFBOARD mode", flush=True)
        elif s == "MODE":
            if self.px4_main == PX4_MAIN_OFFBOARD:
                self._command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
                self._ob_retry, self.ob_state = now, "ARM"
                print("OFFBOARD: in OFFBOARD, arming", flush=True)
            elif now - self._ob_retry > 1.0:
                self._set_mode(PX4_MAIN_OFFBOARD)
                self._ob_retry = now
        elif s == "ARM":
            if self.px4_armed and self.px4_main == PX4_MAIN_OFFBOARD:
                self.ob_state = "ENGAGED"
                print("OFFBOARD: ENGAGED - your controller is flying", flush=True)
            elif self.px4_main != PX4_MAIN_OFFBOARD:
                self.ob_state = "MODE"   # PX4 dropped out of OFFBOARD before arming - ask again
                self._ob_retry = -math.inf
            elif now - self._ob_retry > 1.0:
                self._command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
                self._ob_retry = now
        elif s == "ENGAGED" and (self.px4_main != PX4_MAIN_OFFBOARD or not self.px4_armed):
            self.ob_state = "OVERRIDE"
            print(f"OFFBOARD: PX4 left OFFBOARD on its own (mode {self.px4_main}.{self.px4_sub}, "
                  f"armed={self.px4_armed}) - pilot/PX4 has control, sending nothing more", flush=True)
        if self.ob_state in ("PRESTREAM", "MODE", "ARM"):
            if now - self._ob_start > ENGAGE_TIMEOUT_S:
                self.ob_state = "FAILED"
                print("OFFBOARD: FAILED to engage - disarming", flush=True)
                self._command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
            elif now - self._last_sp_t >= 1.0 / SETPOINT_HZ:
                self._send_attitude(*self._neutral())
                self._last_sp_t = now
        return self.ob_state

    def publish_attitude(self, q_enu_flu, thrust_rel, now):
        """the cascade's attitude setpoint (ENU/FLU quat) + thrust relative to hover (see
        thrust_command), rate-limited to SETPOINT_HZ. Only while ENGAGED - never after an
        override or hand-back."""
        if self.ob_state != "ENGAGED":
            return
        self._heartbeat(now)
        if now - self._last_sp_t >= 1.0 / SETPOINT_HZ:
            self._send_attitude(q_enu_flu, thrust_command(thrust_rel, self.hover_thrust))
            self._last_sp_t = now

    def hand_back(self, reason):
        """give control to PX4 (FALLBACK, watchdog, exception): switch to the fallback mode
        and stop sending. Idempotent, safe from the watchdog thread."""
        if self.ob_state not in ("PRESTREAM", "MODE", "ARM", "ENGAGED", "DONE"):
            return
        self.ob_state = "HANDED_BACK"
        main, sub = self.fallback_mode
        print(f"OFFBOARD: handing control back to PX4 (mode {main}.{sub}): {reason}", flush=True)
        for _ in range(3):   # UDP/serial can drop one; PX4 ignores repeats
            self._set_mode(main, sub)

    def disarm(self):
        """after our own landing. PX4 refuses a (non-forced) disarm in the air, so a wrong
        'landed' decision can't drop the vehicle - the stream stops and PX4's offboard-loss
        failsafe lands it instead."""
        if self.ob_state == "ENGAGED":
            self.ob_state = "DONE"
            self._command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
            print("OFFBOARD: landed - disarm requested", flush=True)

    def close(self):
        self._stop = True
