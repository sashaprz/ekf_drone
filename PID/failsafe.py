"""Flight-safety layer for run_sim.py (2026-10-04): a health monitor that can take control
away from the mission, and a watchdog for a hung/crashed loop.

Modes:  NORMAL -> LAND -> DISARMED ;  FALLBACK = LAND flown on a backup estimator.
  FALLBACK  the EKF itself is broken (NaN/inf, or attitude uncertainty > TILT_SIGMA_MAX
            for TILT_SIGMA_HOLD_S). In sim the backup is Gazebo truth; on hardware it is
            PX4's own estimator/controller taking over (switch out of offboard).
  LAND      the EKF is fine but the flight isn't: GPS lost > GPS_LOSS_S, >= RESETS_MAX GPS
            lockout resets within RESETS_WINDOW_S, outside the geofence, a loop stall
            > LOOP_GAP_S, or the mission ended. Holds x/y where it started, descends at
            LAND_RATE, and disarms once landed.
  DISARMED  motors off.
The Watchdog thread cuts the motors if the main loop stops calling kick() for WATCHDOG_S -
the last line of defence for a hang or an exception. On hardware PX4's offboard-loss
failsafe (COM_OF_LOSS_T / COM_OBL_RC_ACT) does this job, and an RC kill switch outranks all
of it.
"""
import math
import threading
import time

import numpy as np

TILT_SIGMA_MAX = math.radians(20)
TILT_SIGMA_HOLD_S = 1.0
GPS_LOSS_S = 3.0
RESETS_MAX, RESETS_WINDOW_S = 3, 10.0
GEOFENCE_R, GEOFENCE_ALT = 50.0, 15.0
LOOP_GAP_S = 0.5
LAND_RATE = 0.6          # m/s
LANDED_ALT = 0.25        # m (estimated) ...
LANDED_VZ = 0.3          # m/s ... both, for LANDED_HOLD_S
LANDED_HOLD_S = 1.5
# PX4-style "can't descend any further": commanded at least STUCK_BELOW m below the estimate
# but |vz| < LANDED_VZ -> on the ground, whatever the (possibly drifted) height estimate says.
# The height-only rule cut the motors at 4.5 m true height after a no-GPS touchdown went wrong.
STUCK_BELOW = 0.4
LAND_TIMEOUT_S = 25.0    # disarm anyway after this long in LAND
AIRBORNE_ALT = 0.5       # GPS-loss / geofence checks only once airborne

MODE_CODE = {"NORMAL": 0, "LAND": 1, "FALLBACK": 2, "DISARMED": 3}
ALLOWED = {("NORMAL", "LAND"), ("NORMAL", "FALLBACK"), ("NORMAL", "DISARMED"),
           ("LAND", "FALLBACK"), ("LAND", "DISARMED"), ("FALLBACK", "DISARMED")}


class HealthMonitor:
    def __init__(self, log=print):
        self.mode = "NORMAL"
        self.reason = ""
        self._log = log
        self._last_t = None
        self._last_gps_updates = 0
        self._last_gps_t = None
        self._reset_times = []
        self._last_resets = 0
        self._sigma_bad_since = None
        self._airborne = False
        self._land_t0 = None
        self._land_start = None
        self._landed_since = None

    def _go(self, mode, reason, now, state):
        if (self.mode, mode) not in ALLOWED:
            return  # never back to NORMAL; FALLBACK already includes landing
        self._log(f"FAILSAFE {self.mode} -> {mode}: {reason}")
        self.mode, self.reason = mode, reason
        if mode in ("LAND", "FALLBACK") and self._land_t0 is None:
            self._land_t0 = now
            self._land_start = np.array(state["pos"], float).copy() if np.all(np.isfinite(state["pos"])) else None

    def update(self, now, ekf, state, mission_done=False, backup_state=None):
        """call once per control iteration (after the EKF steps); returns the mode.
        backup_state: the backup estimator's state (sim: truth, hardware: PX4's) - used for
        the landing in FALLBACK, when our own estimate can't be trusted"""
        if self.mode == "DISARMED":
            return self.mode
        # loop stall
        if self._last_t is not None and now - self._last_t > LOOP_GAP_S:
            self._go("LAND", f"control loop stalled {now - self._last_t:.2f} s", now, state)
        self._last_t = now
        # estimator health
        finite = all(np.all(np.isfinite(x)) for x in (ekf.q, ekf.position, ekf.velocity, ekf.P))
        if not finite:
            self._go("FALLBACK", "estimator produced NaN/inf", now, state)
        else:
            sig = math.sqrt(max(ekf.P[0, 0] + ekf.P[1, 1], 0.0))
            if sig > TILT_SIGMA_MAX:
                self._sigma_bad_since = self._sigma_bad_since or now
                if now - self._sigma_bad_since > TILT_SIGMA_HOLD_S:
                    self._go("FALLBACK", f"tilt uncertainty {math.degrees(sig):.0f} deg", now, state)
            else:
                self._sigma_bad_since = None
        pos = np.asarray(state["pos"], float)
        if finite and pos[2] > AIRBORNE_ALT:
            self._airborne = True
        # GPS health
        s = ekf.stats
        if s["gps_updates"] != self._last_gps_updates or self._last_gps_t is None:
            self._last_gps_updates, self._last_gps_t = s["gps_updates"], now
        self.gps_ok = now - self._last_gps_t <= GPS_LOSS_S
        if self._airborne and now - self._last_gps_t > GPS_LOSS_S:
            self._go("LAND", f"no accepted GPS for {now - self._last_gps_t:.1f} s", now, state)
        if s["gps_resets"] != self._last_resets:
            self._reset_times += [now] * (s["gps_resets"] - self._last_resets)
            self._last_resets = s["gps_resets"]
        self._reset_times = [t for t in self._reset_times if now - t < RESETS_WINDOW_S]
        if len(self._reset_times) >= RESETS_MAX:
            self._go("LAND", f"{len(self._reset_times)} GPS lockout resets in {RESETS_WINDOW_S:.0f} s", now, state)
        # geofence
        if finite and self._airborne and (math.hypot(pos[0], pos[1]) > GEOFENCE_R or pos[2] > GEOFENCE_ALT):
            self._go("LAND", f"geofence ({math.hypot(pos[0], pos[1]):.0f} m, {pos[2]:.1f} m up)", now, state)
        if mission_done:
            self._go("LAND", "mission complete", now, state)
        nav = backup_state if (self.mode == "FALLBACK" and backup_state is not None) else state
        if self.mode == "FALLBACK" and self._land_start is None and nav is not None:
            self._land_start = np.array(nav["pos"], float).copy()
        # landed?
        if self.mode in ("LAND", "FALLBACK"):
            pos = np.asarray(nav["pos"], float)
            vz = float(np.asarray(nav["vel"], float)[2])
            sp_z = self.setpoint(now, {"pos": pos, "yaw": 0.0}, nav)["pos"][2]
            stuck = sp_z < pos[2] - STUCK_BELOW
            if (pos[2] < LANDED_ALT or stuck) and abs(vz) < LANDED_VZ:
                self._landed_since = self._landed_since or now
                if now - self._landed_since > LANDED_HOLD_S:
                    self._go("DISARMED", "landed", now, state)
            else:
                self._landed_since = None
            if now - self._land_t0 > LAND_TIMEOUT_S:
                self._go("DISARMED", "land timeout", now, state)
        return self.mode

    def setpoint(self, now, base, state):
        """the setpoint to fly: the mission's in NORMAL, a vertical descent in LAND/FALLBACK"""
        if self.mode == "NORMAL":
            return base
        start = self._land_start if self._land_start is not None else np.array([0.0, 0.0, 2.0])
        z = max(start[2] - LAND_RATE * (now - self._land_t0), -0.5)
        # without GPS (and not on the FALLBACK backup) the horizontal estimate drifts:
        # descend level instead of chasing it (cascade.step's level_xy)
        level = self.mode == "LAND" and not getattr(self, "gps_ok", True)
        return {"pos": np.array([start[0], start[1], z]), "yaw": base["yaw"], "level_xy": level}


class Watchdog:
    """cuts the motors (publish 0) if kick() isn't called for timeout_s"""

    def __init__(self, publish_motors, timeout_s=1.0, log=print):
        self._pub = publish_motors
        self._timeout = timeout_s
        self._log = log
        self._last = time.time()
        self._stop = False
        self.tripped = False
        threading.Thread(target=self._run, daemon=True).start()

    def kick(self):
        self._last = time.time()

    def stop(self):
        self._stop = True

    def _run(self):
        while not self._stop:
            time.sleep(0.05)
            if time.time() - self._last > self._timeout:
                if not self.tripped:
                    self._log(f"WATCHDOG: control loop silent for {time.time() - self._last:.2f} s - motors cut")
                    self.tripped = True
                self._pub(0.0, 0.0, 0.0, 0.0)
