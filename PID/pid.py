#position -> outputs desired veloicity
#velocity -> outputs desired acceleration, translates to desired roll/pitch angles (x/y) and thrust (z)
#attitude (roll, pitch, yaw) -> outputs desired angular rates
#angular rate -> outputs torque commands -> motor mixing

#the inner loop, angular rate, runs fastest while the outer loop (position) is slowest. 
#the attitude feeds the attitude loop
#the angular velocity feeds the rate rool
#the position/velcoity estimate feeds outer loops

class PID:
    def __init__(self, kp, ki, kd, output_limits, integral_limits=None, d_filter_alpha=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.integral_limits = integral_limits
        self.d_filter_alpha = d_filter_alpha

        self.integral = 0.0
        self.last_measurement = None
        self.filtered_derivative = 0.0

    def reset(self):
        #clear integral + the last measurement
        self.integral = 0.0
        self.last_measurement = None
        self.filtered_derivative = 0.0

    def update(self, setpoint, measurement, dt):
        #return clamped output
        error = setpoint - measurement

        self.integral += error * dt
        if self.integral_limits is not None:
            lo, hi = self.integral_limits
            self.integral = max(lo, min(hi, self.integral))

        # derivative on measurement, not error - avoids a derivative-kick spike when
        # setpoint jumps instantaneously (measurement can't jump the way a command can)
        if self.last_measurement is None:
            raw_derivative = 0.0  # first call, no history yet - don't fake a spike
        else:
            raw_derivative = -(measurement - self.last_measurement) / dt
        self.last_measurement = measurement

        if self.d_filter_alpha is not None:
            # low-pass the raw derivative - undamped, it amplifies sensor/estimator noise
            self.filtered_derivative = (self.d_filter_alpha * raw_derivative
                                         + (1 - self.d_filter_alpha) * self.filtered_derivative)
            derivative = self.filtered_derivative
        else:
            derivative = raw_derivative

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        lo, hi = self.output_limits
        return max(lo, min(hi, output))