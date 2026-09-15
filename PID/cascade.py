# gains + limits live here as plain constants/dict for now
# GAINS = {"pos_xy": ..., "pos_z": ..., "vel_xy": ..., "vel_z": ..., "att_rp": ..., "att_yaw": ..., "rate_rp": ..., "rate_yaw": ...}
# LIMITS = {"max_tilt_rad": ..., "max_rate": ..., "thrust_range": ..., "motor_range": ...}
# FRAME = "quad_x"

class Cascade:
    def __init__(self, config):
        # instantiate PID objects for each loop from config
        # self.pos_pid_x, self.pos_pid_y, self.pos_pid_z = ...
        # self.vel_pid_x, self.vel_pid_y, self.vel_pid_z = ...
        # self.att_pid_roll, self.att_pid_pitch, self.att_pid_yaw = ...
        # self.rate_pid_roll, self.rate_pid_pitch, self.rate_pid_yaw = ...

    def position_loop(self, pos_setpoint, state, dt):
        # pos error -> velocity setpoint

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
