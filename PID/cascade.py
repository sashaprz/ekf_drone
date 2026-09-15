import numpy as np

# gains + limits live here as plain constants/dict for now
# each GAINS entry is {"kp": ..., "ki": ..., "kd": ...} so it can be unpacked straight into PID(**GAINS["pos_xy"], ...)
GAINS = {
    "pos_xy": {"kp": ..., "ki": ..., "kd": ...},
    "pos_z": {"kp": ..., "ki": ..., "kd": ...},
    "vel_xy": {"kp": ..., "ki": ..., "kd": ...},
    "vel_z": {"kp": ..., "ki": ..., "kd": ...},
    "att_rp": {"kp": ..., "ki": ..., "kd": ...},
    "att_yaw": {"kp": ..., "ki": ..., "kd": ...},
    "rate_rp": {"kp": ..., "ki": ..., "kd": ...},
    "rate_yaw": {"kp": ..., "ki": ..., "kd": ...},
}
LIMITS = {"max_tilt_rad": ..., "max_rate": ..., "thrust_range": ..., "motor_range": ..., "max_vel_xy": ..., "max_vel_z": ..., "max_accel_xy": ..., "max_rate_yaw": ..., "torque_range_rp": ..., "torque_range_yaw": ...}
FRAME = "quad_x"

#cascade is the thing that turns "i want drone at this position" into "here's how hard each motor should spin"
#chains 4 pid loops -> each loop only has to solve one narrow porblem, and each loop can run at the frequency suited to how fast that physical quanitity actually changes
#position changes slowly (10-50Hz). velocity (50-100hz), attitude (100-250hz), rate/gyro (500-1khz)
#each outer loop is slower and "trusts" that the inner loop has converged -> ie it just says give x velcoity and trusts the inner loops to deliver it


class Cascade:
    def __init__(self, config):
        #creating 12 instances of the PID class, each with different inputs
        #runs once, sets the object's state so every other method can just assume everything it needs already exists. purely setup

        gains = config["GAINS"]
        limits = config["LIMITS"]
        self.frame = config.get("FRAME", FRAME)

        # position loop -> outputs velocity setpoint (m/s)
        self.pos_pid_x = PID(**gains["pos_xy"], output_limits=(-limits["max_vel_xy"], limits["max_vel_xy"]))
        self.pos_pid_y = PID(**gains["pos_xy"], output_limits=(-limits["max_vel_xy"], limits["max_vel_xy"]))
        self.pos_pid_z = PID(**gains["pos_z"], output_limits=(-limits["max_vel_z"], limits["max_vel_z"]))

        # velocity loop -> outputs desired accel (m/s^2), x/y become tilt later, z becomes thrust
        self.vel_pid_x = PID(**gains["vel_xy"], output_limits=(-limits["max_accel_xy"], limits["max_accel_xy"]))
        self.vel_pid_y = PID(**gains["vel_xy"], output_limits=(-limits["max_accel_xy"], limits["max_accel_xy"]))
        self.vel_pid_z = PID(**gains["vel_z"], output_limits=limits["thrust_range"])

        # attitude loop -> operates on quaternion error vector components (not euler angles), outputs body rate setpoint (rad/s)
        self.att_pid_x = PID(**gains["att_rp"], output_limits=(-limits["max_rate"], limits["max_rate"]))
        self.att_pid_y = PID(**gains["att_rp"], output_limits=(-limits["max_rate"], limits["max_rate"]))
        self.att_pid_z = PID(**gains["att_yaw"], output_limits=(-limits["max_rate_yaw"], limits["max_rate_yaw"]))

        # rate loop -> gyro rate error, outputs torque command
        self.rate_pid_roll = PID(**gains["rate_rp"], output_limits=limits["torque_range_rp"])
        self.rate_pid_pitch = PID(**gains["rate_rp"], output_limits=limits["torque_range_rp"])
        self.rate_pid_yaw = PID(**gains["rate_yaw"], output_limits=limits["torque_range_yaw"])

    def position_loop(self, pos_setpoint, state, dt):
        # pos error -> velocity setpoint
        # assumes state is a dict with state["pos"] as a flat np.array([x, y, z]),
        # matching the EKF's position layout. pos_setpoint is the same shape.
        vx = self.pos_pid_x.update(pos_setpoint[0], state["pos"][0], dt)
        vy = self.pos_pid_y.update(pos_setpoint[1], state["pos"][1], dt)
        vz = self.pos_pid_z.update(pos_setpoint[2], state["pos"][2], dt)
        return np.array([vx, vy, vz])

    def velocity_loop(self, vel_setpoint, state, dt):
        # vel error -> desired accel -> (roll_sp, pitch_sp, thrust)
        # remember: rotate accel by -yaw before mapping to pitch/roll

    def attitude_loop(self, att_setpoint, state, dt):
        # attitude error (quaternion-aware) -> rate setpoint

    def rate_loop(self, rate_setpoint, state, dt):
        # rate error -> torque command (roll_tau, pitch_tau, yaw_tau)

    def step(self, setpoint, state, dt):
        # calls the above in sequence at the right sub-rates
        # returns (thrust, roll_tau, pitch_tau, yaw_tau)


def mix(thrust, roll_tau, pitch_tau, yaw_tau, frame="quad_x"):
    # returns per-motor commands, clamped to actuator range

    m1 = thrust + roll_tau - pitch_tau - yaw_tau
    m2 = thrust - roll_tau - pitch_tau + yaw_tau
    m3 = thrust - roll_tau + pitch_tau - yaw_tau
    m4 = thrust + roll_tau + pitch_tau + yaw_tau
