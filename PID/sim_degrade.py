"""Live sensor degradation for Gazebo flights (2026-10-02) - makes the sim's near-perfect
sensors look like cheap hardware, so the CONTROLLER flies on realistic data too (the
offline suite only tests the estimator open-loop). Off unless an env var is set:

  SIM_REALISTIC=1       same set as testing/suite/faults.py combined_realistic:
                        gyro white 0.005 rad/s + bias 0.2 deg/s, accel white 0.05 m/s^2 +
                        bias 0.05 m/s^2 (body x), mag hard iron 0.05 (fixed body vector),
                        GPS white 1.5 m horiz / 3 m vert + 0.1 m/s + 0.5 m/sqrt(min) wander,
                        baro white 0.3 m + 0.3 m/sqrt(min) drift (temperature/weather)
  SIM_GPS_LATENCY_MS=N  every fix is delivered N ms after it was measured
  SIM_GPS_LOSS_AT=s     GPS stops (get_gps -> None, "no fix") s seconds after the first fix -
                        for testing the failsafe (PID/failsafe.py)
  SIM_SEED=k            RNG seed (default 1) - fixes the bias directions/signs; the noise
                        itself still differs run to run (sample timing isn't deterministic)

Biases are present from power-on (calibration sees them), like real hardware. Everything
is applied in the bridge's units, so SENSOR_LOG recordings hold the degraded data and
replay exactly what the EKF saw.
"""
import collections
import math
import os
import time

import numpy as np

G = 9.80665


class SensorDegrader:
    def __init__(self):
        self.realistic = os.environ.get("SIM_REALISTIC") == "1"
        self.latency = float(os.environ.get("SIM_GPS_LATENCY_MS", "0")) / 1000.0
        seed = int(os.environ.get("SIM_SEED", "1"))
        self.gps_loss_at = float(os.environ.get("SIM_GPS_LOSS_AT", "0"))
        self._first_gps_t = None
        self.active = self.realistic or self.latency > 0 or self.gps_loss_at > 0
        # one generator per sensor - the callbacks may run on different gz-transport threads
        self._rng = {k: np.random.default_rng([seed, i]) for i, k in enumerate(("imu", "mag", "gps", "fixed"))}
        r = self._rng["fixed"]
        self.gyro_bias = math.radians(0.2) * r.choice([-1.0, 1.0], 3)
        self.accel_bias = np.array([0.05, 0.0, 0.0])
        d = r.normal(0, 1, 3)
        self.mag_bias = 0.05 * d / np.linalg.norm(d)
        self._walk = np.zeros(3)
        self._baro_walk = 0.0
        self._last_baro_t = None
        self._rng["baro"] = np.random.default_rng([seed, 9])
        self._last_gps_t = None
        self._gps_queue = collections.deque(maxlen=200)

    def describe(self):
        if not self.active:
            return "SENSORS: clean (Gazebo)"
        parts = []
        if self.realistic:
            parts.append(f"realistic noise+bias (gyro bias {np.degrees(self.gyro_bias).round(2)} deg/s, "
                         f"accel bias {self.accel_bias}, mag hard iron {self.mag_bias.round(3)})")
        if self.latency:
            parts.append(f"GPS latency {self.latency*1000:.0f} ms")
        if self.gps_loss_at:
            parts.append(f"GPS LOST {self.gps_loss_at:.0f} s after the first fix")
        return "SENSORS DEGRADED: " + ", ".join(parts)

    def imu(self, gyro, accel):
        if not self.realistic:
            return gyro, accel
        r = self._rng["imu"]
        g = np.asarray(gyro) + self.gyro_bias + r.normal(0, 0.005, 3)
        a = np.asarray(accel) + self.accel_bias + r.normal(0, 0.05, 3)
        return tuple(g), tuple(a)

    def mag(self, m):
        if not self.realistic:
            return m
        return tuple(np.asarray(m) + self.mag_bias)  # additive - the bridge scales, doesn't normalize

    def baro(self, h):
        if not self.realistic:
            return h
        r = self._rng["baro"]
        now = time.time()
        if self._last_baro_t is not None:
            self._baro_walk += r.normal() * 0.3 * math.sqrt(max(now - self._last_baro_t, 0) / 60.0)
        self._last_baro_t = now
        return h + self._baro_walk + r.normal(0, 0.3)

    def gps_in(self, fix):
        # called on every fix the bridge receives; returns nothing - get_gps reads gps_out
        now = time.time()
        self._first_gps_t = self._first_gps_t or now
        if self.realistic:
            r = self._rng["gps"]
            if self._last_gps_t is not None:
                dt = now - self._last_gps_t
                self._walk += r.normal(0, 1, 3) * 0.5 * math.sqrt(max(dt, 0) / 60.0)
            self._last_gps_t = now
            pos = np.asarray(fix[0:3]) + r.normal(0, 1, 3) * np.array([1.5, 1.5, 3.0]) + self._walk
            vel = np.asarray(fix[3:6]) + r.normal(0, 0.1, 3)
            fix = (*pos, *vel)
        self._gps_queue.append((now, tuple(fix)))

    def gps_out(self, default):
        # newest fix that is at least `latency` old
        if self.gps_loss_at and self._first_gps_t and time.time() - self._first_gps_t > self.gps_loss_at:
            return None
        if not self._gps_queue:
            return default
        cutoff = time.time() - self.latency
        for t, fix in reversed(self._gps_queue):
            if t <= cutoff:
                return fix
        return default
