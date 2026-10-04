import collections
import math
import os
import threading
import numpy as np

from gz.transport13 import Node
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.magnetometer_pb2 import Magnetometer
from gz.msgs10.navsat_pb2 import NavSat
from gz.msgs10.actuators_pb2 import Actuators
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.fluid_pressure_pb2 import FluidPressure

from sim_degrade import SensorDegrader

# topic/model names match the x500 SITL default world - see `gz topic -l` while
# `make px4_sitl gz_x500` is running
IMU_TOPIC = "/world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu"
MAG_TOPIC = "/world/default/model/x500_0/link/base_link/sensor/magnetometer_sensor/magnetometer"
GPS_TOPIC = "/world/default/model/x500_0/link/base_link/sensor/navsat_sensor/navsat"
BARO_TOPIC = "/world/default/model/x500_0/link/base_link/sensor/air_pressure_sensor/air_pressure"
POSE_TOPIC = "/world/default/pose/info"  # ground truth, diagnostics ONLY - never fed to the EKF/controller
MOTOR_TOPIC = "/x500_0/command/motor_speed"  # NOT /model/x500_0/... - that one exists but has zero real subscribers

EARTH_RADIUS = 6371000.0  # meters - flat-earth approximation for lat/lon -> local ENU, fine over the small distances a sim flight covers

# 2026-09-30: the EKF's world frame is now Gazebo's own world frame, ENU (X=east,
# Y=north, Z=up - see default.sdf's <world_frame_orientation>). The x500 spawns facing
# +X, so yaw=0 still means "facing the way it spawned", same as before.
#
# Mag reference = the field direction gz-sim actually uses in that world frame,
# MEASURED against ground truth rather than derived from real-world physics: rotating
# every raw mag reading by the true attitude (sensors_live1.csv, first 14s of a flight
# with up to 30deg tilt and 35deg of yaw) gives the same world vector every tick,
# std 0.001 per axis. It points east-and-UP, which is not Earth's real field at Zurich
# (north-and-down) - the gz-sim quirk PX4's bridge comments on - but what matters to
# the EKF is only that it's consistent with how the raw reading rotates, and it is.
# FINAL_gps.py used to hardcode horizontal (1,0,0), so the +0.89 vertical part got
# absorbed into mag_bias_z (every CAL_DIAG run: mag_bias_z ~0.83-0.90) - fine while
# level, but a body-fixed bias can't follow a world-fixed vector as the vehicle tilts,
# so every tilt leaked into bogus heading/attitude corrections.
MAG_REFERENCE_ENU = (0.450, 0.020, 0.893)


class GazeboBridge:
    # get_gyro/get_accel/get_mag/get_gps here match FINAL_gps.py's simulated versions'
    # return shapes exactly, so a DroneEKF can take these as its `sensors` instead
    def __init__(self):
        self._node = Node()

        # set by _on_imu() (fires on gz-transport's own background thread), waited on by
        # wait_for_imu() - see that method's comment for why this exists.
        self._imu_event = threading.Event()

        self._gyro = (0.0, 0.0, 0.0)
        self._accel = (0.0, 0.0, 0.0)
        self._gyro_clean = (0.0, 0.0, 0.0)
        # IMU sample queue + timestamps, 2026-10-03. Every IMU message is kept with Gazebo's
        # own timestamp (header.stamp, sim time) so a consumer that calls drain_imu() can
        # step the EKF once per sample with the TRUE sample spacing. Before, only the latest
        # sample was kept and dt came from arrival time: when gz-transport delivered a burst
        # after a 100 ms hiccup, ~100 ms of motion was integrated as a few ms (live crash
        # stops_real_ct_t3). Callers that never drain (test_*_loop.py) see the old
        # latest-sample behaviour. EKF_IMU_TIMESTAMPS=0 turns the whole thing off (A/B).
        self.use_timestamps = os.environ.get("EKF_IMU_TIMESTAMPS", "1") == "1"
        self._imu_queue = collections.deque(maxlen=5000)
        self._imu_queued = False   # becomes True on the first drain_imu()
        self._imu_time = None      # timestamp of the sample get_gyro/get_accel serve
        self._true_stamp = None
        # barometer, 2026-10-03: (stamp, height in m relative to the first reading) - the
        # EKF fuses it as position_z + baro_bias. EKF_USE_BARO=0 hides it (A/B).
        self.use_baro = os.environ.get("EKF_USE_BARO", "1") == "1"
        self._baro = None
        self._baro_p0 = None
        self._accel_clean = (0.0, 0.0, 0.0)
        self._mag = (1.0, 0.0, 0.0)
        self._mag_scale = None  # field strength of the first reading - see _on_mag
        # world-frame field direction for FINAL_gps.py's mag model - see MAG_REFERENCE_ENU
        self.mag_reference = MAG_REFERENCE_ENU
        self._gps = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        # optional live sensor degradation (SIM_REALISTIC / SIM_GPS_LATENCY_MS env vars) -
        # a no-op by default, see sim_degrade.py
        self.degrade = SensorDegrader()
        # GPS latency the EKF should compensate (s) - read by DroneEKF (delayed fusion).
        # EKF_GPS_LATENCY_MS overrides (on hardware: the receiver's known latency); in sim it
        # defaults to the latency sim_degrade injects (0 unless SIM_GPS_LATENCY_MS is set).
        _lat = os.environ.get("EKF_GPS_LATENCY_MS")
        self.gps_latency = float(_lat) / 1000.0 if _lat else self.degrade.latency
        self._gps_seen = False

        # local-frame origin, set from the first GPS fix received (matches
        # FINAL_gps.py's assumption that position/velocity start at zero)
        self._home_lat = None
        self._home_lon = None
        self._home_alt = None

        self._node.subscribe(IMU, IMU_TOPIC, self._on_imu)
        self._node.subscribe(Magnetometer, MAG_TOPIC, self._on_mag)
        self._node.subscribe(NavSat, GPS_TOPIC, self._on_gps)
        self._node.subscribe(FluidPressure, BARO_TOPIC, self._on_baro)
        # ground truth (2026-09-30): Gazebo's world frame is ENU and the model link is FLU
        # - the same frames the EKF now uses - so this is directly comparable to
        # state['quat']/state['pos']. For logging only.
        self._true_quat = None
        self._true_pos = None
        self._node.subscribe(Pose_V, POSE_TOPIC, self._on_pose)
        self._motor_pub = self._node.advertise(MOTOR_TOPIC, Actuators)

    def _on_imu(self, msg):
        # sanity bound, 2026-09-26: same corrupted-message risk as the accel bound
        # below (same IMU message, gz-transport occasionally delivers a corrupted one)
        # but this field was never guarded - FINAL_gps.py's `state['rate']` is a
        # straight passthrough of this value (no gating, unlike accel's chi-squared
        # gate), so a single bad gyro reading hits the rate loop completely
        # unprotected. Confirmed via a full run_sim.py flight log: rate_meas jumped
        # from ~0 to (-9.84,-22.36,-3.29) rad/s in a single tick, immediately
        # saturating all 3 torque channels, right at the point where the 2026-09-26
        # full-cascade divergence begins. 50 rad/s (~2865 deg/s) is far beyond
        # anything this vehicle's real torque authority could produce in one ~200Hz
        # control tick even fully saturated, and far below what's needed to reject
        # any real (if fast) rotation.
        gyro = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        if math.sqrt(gyro[0]**2 + gyro[1]**2 + gyro[2]**2) <= 50.0:
            self._gyro_clean = gyro

        # sanity bound, 2026-09-26: gz-transport occasionally delivers a genuinely
        # corrupted IMU message with an accel magnitude in the hundreds of m/s^2 (40+g,
        # impossible for this vehicle) - confirmed via FINAL_gps.py's EKF_ACCEL
        # diagnostic, correlated with the intermittent single-tick attitude corruption
        # seen throughout tuning (test_attitude_loop.py, test_velocity_loop.py). The
        # EKF's own chi-squared gate catches most of these, but the transient covariance
        # disruption right around a spike lets at least one moderately-bad reading slip
        # through with a large one-step correction. 6g is generous headroom above any
        # real maneuver this vehicle does (confirmed accel deviations during normal,
        # even aggressive, testing stay under ~1 m/s^2) and far below the observed
        # garbage values (422, 713 m/s^2) - reject and hold the last good reading rather
        # than feed this to the EKF at all.
        accel = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        if math.sqrt(accel[0]**2 + accel[1]**2 + accel[2]**2) <= 6 * 9.80665:
            self._accel_clean = accel
        # degrade from the clean held values, so a guard-rejected sample can't get its
        # bias applied twice
        gyro_d, accel_d = self.degrade.imu(self._gyro_clean, self._accel_clean)
        st = msg.header.stamp
        sample = (st.sec + st.nsec * 1e-9, gyro_d, accel_d)
        self._imu_queue.append(sample)
        if not self._imu_queued:
            self._imu_time, self._gyro, self._accel = sample

        self._imu_event.set()

    def _on_mag(self, msg):
        # Raw passthrough - the reading IS already body-frame FLU, same as the IMU
        # (verified against ground truth, see MAG_REFERENCE_ENU). 2026-09-30 note: an
        # earlier attempt the same day remapped this as (-y, x, -z), following PX4's
        # GZBridge.cpp comment that gz's mag is "left handed". That was wrong for this
        # use - it produced a mirrored vector whose yaw moved OPPOSITE to the true yaw
        # in flight (true yaw -5 -> +18deg while the estimate went 0 -> -14deg). PX4's
        # remap is about its own FRD/NED conventions, not a body-frame reflection.
        #
        # Scaled by ONE fixed field strength (latched from the first reading), not
        # normalized per reading, 2026-09-30. A hard-iron offset is additive in the raw
        # field; normalizing every reading turns it into an offset that changes with
        # orientation, which the EKF's constant mag_bias state can't learn - in replay, a
        # 0.05 hard iron was learned exactly during a yaw turn when scaled, and not at all
        # when normalized (EKF_TEST_REPORT.md, in-flight calibration). Gazebo's field
        # strength is constant, so with no hard iron this is identical to normalizing.
        v = np.array([msg.field_tesla.x, msg.field_tesla.y, msg.field_tesla.z])
        norm = np.linalg.norm(v)
        if norm > 0 and self._mag_scale is None:
            self._mag_scale = norm
        if self._mag_scale:
            v = v / self._mag_scale
        self._mag = self.degrade.mag(tuple(v))

    def _on_gps(self, msg):
        if self._home_lat is None:
            self._home_lat = msg.latitude_deg
            self._home_lon = msg.longitude_deg
            self._home_alt = msg.altitude

        lat_rad = math.radians(self._home_lat)
        north = math.radians(msg.latitude_deg - self._home_lat) * EARTH_RADIUS
        east = math.radians(msg.longitude_deg - self._home_lon) * EARTH_RADIUS * math.cos(lat_rad)
        up = msg.altitude - self._home_alt

        # ENU order (x=east, y=north), 2026-09-30 - was (north, east, up). The EKF's x
        # axis is whatever the vehicle faced at spawn (east in Gazebo's ENU world), so
        # (north, east) swapped x/y. A swap is a mirror, not a rotation: position/
        # velocity feedback through it is negative along one diagonal but POSITIVE
        # along the other - a good fit for "any nonzero pos_xy gain tumbles it".
        self._gps = (east, north, up, msg.velocity_east, msg.velocity_north, msg.velocity_up)
        self._gps_seen = True
        if self.degrade.active:
            self.degrade.gps_in(self._gps)

    def _on_baro(self, msg):
        if self._baro_p0 is None:
            self._baro_p0 = msg.pressure
        # standard-atmosphere pressure -> height above the boot reading
        h = 44330.0 * (1.0 - (msg.pressure / self._baro_p0) ** (1.0 / 5.255))
        st = msg.header.stamp
        self._baro = (st.sec + st.nsec * 1e-9, self.degrade.baro(h))

    def get_baro(self):
        return self._baro if self.use_baro else None

    def _on_pose(self, msg):
        for p in msg.pose:
            if p.name == "x500_0":
                st = msg.header.stamp
                self._true_stamp = st.sec + st.nsec * 1e-9
                self._true_quat = (p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)
                self._true_pos = (p.position.x, p.position.y, p.position.z)
                break

    def get_true_pose(self):
        # (quat [w,x,y,z], pos) or (None, None) before the first message
        return self._true_quat, self._true_pos

    def wait_for_imu(self, timeout=0.1):
        # 2026-09-27: every control loop (run_sim.py, test_*_loop.py) used to call
        # ekf.step() in a bare `while True` with no pacing at all - measured at up to
        # ~3000Hz (dt~0.3ms), while Gazebo's real IMU topic publishes far slower. Most
        # iterations were re-processing the exact same cached sensor reading with
        # whatever tiny, OS-scheduling-dependent dt the busy loop happened to have at
        # that instant, and the moment a genuinely new reading landed (delivered
        # asynchronously by gz-transport's own thread) was a race against the main
        # loop's own timing - non-deterministic by construction, and a good fit for
        # this project's long-documented "run-to-run noise" (identical gains,
        # identical code, wildly different divergence times) and the intermittent
        # single-tick sensor "corruption" events: a real sensor jump divided by
        # whatever near-random dt the loop happened to have produces an
        # effectively-random-magnitude derivative kick. Blocking here ties the loop's
        # rate to real IMU arrival instead - dt then reflects actual sensor timing,
        # not scheduler noise. `timeout` is just a safety net against a stalled topic
        # (e.g. Gazebo not running yet) - the loop still proceeds so a hang here can't
        # wedge the vehicle with stale motor commands.
        got_new = self._imu_event.wait(timeout)
        self._imu_event.clear()
        return got_new

    def drain_imu(self, timeout=0.1):
        # every IMU sample received since the last call, oldest first: [(stamp, gyro, accel)].
        # Waits up to `timeout` for at least one. Call use_imu_sample() on each before
        # ekf.step(), so get_gyro/get_accel/get_imu_time serve that sample.
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
        # sim-time stamp of the current IMU sample - DroneEKF uses it as its clock
        return self._imu_time

    def get_true_stamp(self):
        return self._true_stamp

    def get_gyro(self):
        return self._gyro

    def get_accel(self):
        return self._accel

    def get_mag(self):
        return self._mag

    def get_gps(self):
        if self.degrade.active and self._gps_seen:
            return self.degrade.gps_out((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        return self._gps

    def publish_motors(self, m1, m2, m3, m4):
        # velocity field is rad/s directly (x500's MulticopterMotorModel plugin,
        # motorType=velocity, maxRotVelocity=1000.0) - NOT a normalized 0-1 command
        msg = Actuators()
        msg.velocity.extend([float(m1), float(m2), float(m3), float(m4)])
        self._motor_pub.publish(msg)
