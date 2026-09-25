import math
import numpy as np

from gz.transport13 import Node
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.magnetometer_pb2 import Magnetometer
from gz.msgs10.navsat_pb2 import NavSat
from gz.msgs10.actuators_pb2 import Actuators

# topic/model names match the x500 SITL default world - see `gz topic -l` while
# `make px4_sitl gz_x500` is running
IMU_TOPIC = "/world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu"
MAG_TOPIC = "/world/default/model/x500_0/link/base_link/sensor/magnetometer_sensor/magnetometer"
GPS_TOPIC = "/world/default/model/x500_0/link/base_link/sensor/navsat_sensor/navsat"
MOTOR_TOPIC = "/x500_0/command/motor_speed"  # NOT /model/x500_0/... - that one exists but has zero real subscribers

EARTH_RADIUS = 6371000.0  # meters - flat-earth approximation for lat/lon -> local NED, fine over the small distances a sim flight covers


class GazeboBridge:
    # get_gyro/get_accel/get_mag/get_gps here match FINAL_gps.py's simulated versions'
    # return shapes exactly, so a DroneEKF can take these as its `sensors` instead
    def __init__(self):
        self._node = Node()

        self._gyro = (0.0, 0.0, 0.0)
        self._accel = (0.0, 0.0, 0.0)
        self._mag = (1.0, 0.0, 0.0)
        self._gps = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        # local-frame origin, set from the first GPS fix received (matches
        # FINAL_gps.py's assumption that position/velocity start at zero)
        self._home_lat = None
        self._home_lon = None
        self._home_alt = None

        self._node.subscribe(IMU, IMU_TOPIC, self._on_imu)
        self._node.subscribe(Magnetometer, MAG_TOPIC, self._on_mag)
        self._node.subscribe(NavSat, GPS_TOPIC, self._on_gps)
        self._motor_pub = self._node.advertise(MOTOR_TOPIC, Actuators)

    def _on_imu(self, msg):
        self._gyro = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        self._accel = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)

    def _on_mag(self, msg):
        v = np.array([msg.field_tesla.x, msg.field_tesla.y, msg.field_tesla.z])
        norm = np.linalg.norm(v)
        if norm > 0:
            v = v / norm
        self._mag = tuple(v)

    def _on_gps(self, msg):
        if self._home_lat is None:
            self._home_lat = msg.latitude_deg
            self._home_lon = msg.longitude_deg
            self._home_alt = msg.altitude

        lat_rad = math.radians(self._home_lat)
        north = math.radians(msg.latitude_deg - self._home_lat) * EARTH_RADIUS
        east = math.radians(msg.longitude_deg - self._home_lon) * EARTH_RADIUS * math.cos(lat_rad)
        up = msg.altitude - self._home_alt

        self._gps = (north, east, up, msg.velocity_north, msg.velocity_east, msg.velocity_up)

    def get_gyro(self):
        return self._gyro

    def get_accel(self):
        return self._accel

    def get_mag(self):
        return self._mag

    def get_gps(self):
        return self._gps

    def publish_motors(self, m1, m2, m3, m4):
        # velocity field is rad/s directly (x500's MulticopterMotorModel plugin,
        # motorType=velocity, maxRotVelocity=1000.0) - NOT a normalized 0-1 command
        msg = Actuators()
        msg.velocity.extend([float(m1), float(m2), float(m3), float(m4)])
        self._motor_pub.publish(msg)
