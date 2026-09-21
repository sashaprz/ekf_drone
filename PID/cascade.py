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
GRAVITY = 9.80665  # matches FINAL_gps.py, so accel<->tilt conversion agrees with the EKF's frame

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
        self.max_tilt_rad = limits["max_tilt_rad"]

        # step() is called once per rate-loop tick (fastest loop); position/velocity/attitude
        # run slower, so each one accumulates elapsed time and only recomputes (holding its
        # last output otherwise) once its own period has passed.
        sub_rates_hz = config.get("SUB_RATES_HZ", {"position": 30, "velocity": 75, "attitude": 200})
        self._periods = {name: 1.0 / hz for name, hz in sub_rates_hz.items()}
        self._elapsed = {"position": 0.0, "velocity": 0.0, "attitude": 0.0}

        # held setpoints, reused between recomputes of their owning loop
        self._vel_sp = np.zeros(3)
        self._att_sp = np.array([1.0, 0.0, 0.0, 0.0])  # identity quat = level
        self._thrust = 0.0
        self._rate_sp = np.zeros(3)

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
        #pulls the current xyz position estimation from the EKF state vector, feeds each axis into its own PID against setpoint
        #then feed velocity the outputs in an array
        vx = self.pos_pid_x.update(pos_setpoint[0], state["pos"][0], dt)
        vy = self.pos_pid_y.update(pos_setpoint[1], state["pos"][1], dt)
        vz = self.pos_pid_z.update(pos_setpoint[2], state["pos"][2], dt)
        return np.array([vx, vy, vz])

    def velocity_loop(self, vel_setpoint, state, dt):
        # vel error -> desired accel -> (roll_sp, pitch_sp, thrust)
        # x/y accel gets rotated by -yaw (world frame -> heading frame) then mapped to tilt;
        # z accel is thrust directly, the vel_pid_z output_limits already clamp it to thrust_range
        ax = self.vel_pid_x.update(vel_setpoint[0], state["vel"][0], dt)
        ay = self.vel_pid_y.update(vel_setpoint[1], state["vel"][1], dt)
        thrust = self.vel_pid_z.update(vel_setpoint[2], state["vel"][2], dt)

        yaw = yaw_from_quat(state["quat"])
        a_fwd = ax * np.cos(yaw) + ay * np.sin(yaw)
        a_right = -ax * np.sin(yaw) + ay * np.cos(yaw)

        pitch_sp = np.arctan2(a_fwd, GRAVITY)
        roll_sp = np.arctan2(-a_right, GRAVITY)
        pitch_sp = np.clip(pitch_sp, -self.max_tilt_rad, self.max_tilt_rad)
        roll_sp = np.clip(roll_sp, -self.max_tilt_rad, self.max_tilt_rad)

        return roll_sp, pitch_sp, thrust


    def attitude_loop(self, att_setpoint, state, dt):
        # attitude error (quaternion-aware) -> rate setpoint
        # att_setpoint is a target quaternion [w, x, y, z] (same convention as state["quat"])
        q_err = quat_mult(att_setpoint, quat_conjugate(state["quat"]))
        if q_err[0] < 0:
            q_err = -q_err  # shortest-path fix: q and -q are the same rotation
        err_roll, err_pitch, err_yaw = q_err[1:]  # small-angle approx of the rotation needed

        rate_roll_sp = self.att_pid_x.update(err_roll, 0.0, dt)
        rate_pitch_sp = self.att_pid_y.update(err_pitch, 0.0, dt)
        rate_yaw_sp = self.att_pid_z.update(err_yaw, 0.0, dt)
        return np.array([rate_roll_sp, rate_pitch_sp, rate_yaw_sp])

    def rate_loop(self, rate_setpoint, state, dt):
        # rate error -> torque command (roll_tau, pitch_tau, yaw_tau)
        # setpoint and measurement are both already body-frame gyro rates, so no rotation needed
        roll_tau = self.rate_pid_roll.update(rate_setpoint[0], state["rate"][0], dt)
        pitch_tau = self.rate_pid_pitch.update(rate_setpoint[1], state["rate"][1], dt)
        yaw_tau = self.rate_pid_yaw.update(rate_setpoint[2], state["rate"][2], dt)
        return np.array([roll_tau, pitch_tau, yaw_tau])

    def step(self, setpoint, state, dt):
        # calls the above in sequence at the right sub-rates
        # setpoint = {"pos": np.array([x, y, z]), "yaw": desired heading (rad)}
        # returns (thrust, roll_tau, pitch_tau, yaw_tau)
        self._elapsed["position"] += dt
        if self._elapsed["position"] >= self._periods["position"]:
            self._vel_sp = self.position_loop(setpoint["pos"], state, self._elapsed["position"])
            self._elapsed["position"] = 0.0

        self._elapsed["velocity"] += dt
        if self._elapsed["velocity"] >= self._periods["velocity"]:
            roll_sp, pitch_sp, self._thrust = self.velocity_loop(self._vel_sp, state, self._elapsed["velocity"])
            self._att_sp = euler_to_quat(roll_sp, pitch_sp, setpoint["yaw"])
            self._elapsed["velocity"] = 0.0

        self._elapsed["attitude"] += dt
        if self._elapsed["attitude"] >= self._periods["attitude"]:
            self._rate_sp = self.attitude_loop(self._att_sp, state, self._elapsed["attitude"])
            self._elapsed["attitude"] = 0.0

        roll_tau, pitch_tau, yaw_tau = self.rate_loop(self._rate_sp, state, dt)
        return self._thrust, roll_tau, pitch_tau, yaw_tau


def yaw_from_quat(q):
    # q = [w, x, y, z], same convention as FINAL_gps.py / quaternarions.py
    w, x, y, z = q
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def quat_conjugate(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def quat_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def euler_to_quat(roll, pitch, yaw):
    # roll/pitch/yaw (rad) -> quaternion [w, x, y, z], ZYX convention (q = q_yaw * q_pitch * q_roll)
    # matches the Hamilton product convention in quat_mult/rotate_by_quat above and in FINAL_gps.py
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def mix(thrust, roll_tau, pitch_tau, yaw_tau, motor_range, frame="quad_x"):
    # returns per-motor commands, clamped to actuator range
    m1 = thrust + roll_tau - pitch_tau - yaw_tau
    m2 = thrust - roll_tau - pitch_tau + yaw_tau
    m3 = thrust - roll_tau + pitch_tau - yaw_tau
    m4 = thrust + roll_tau + pitch_tau + yaw_tau
    return tuple(np.clip([m1, m2, m3, m4], motor_range[0], motor_range[1]))
