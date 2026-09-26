"""
  SoC EKF, generalized:
  - State: SoC
  - Input (drives prediction): current, via Coulomb counting — SoC_k+1 =
    SoC_k + (I/Q)*dt
  - Measurement model (nonlinear, maps state → expected sensor reading):
    OCV(SoC) curve, compared against measured terminal voltage
  - Jacobians of both, for covariance propagation

  input: gyrp. how fast orientation is changing. take curent orientation estimate, integrate gyro
    rate over dt, get new orientation. same as coulomb counting. drift accumulates, just like it does for
    coulomb counting.
measrement model: accel and mag. they give an absolute reference to correct drift. accelerometer measures specific force.
    when drone isn't accelerating that's just gravity, and gravity is "down." so given a hypothesized orientation you can predict
    where gravity SHOULD point, and compare that to measured accel value. mismatch tells you roll/pitch error.
    magnometer does same for yaw. given a hypothesized orientation, predict what direction earth's magnetic
    field SHOULD be pointing, compare to actual reading.
    accelerometer doesn't work when drone accelerating hard (bc then the gravity down assumption isn't valid) and megnometer is sensitive to
    magnetic interference.
the hypothesized orietnation you're comparing accel/mag against is the gyro's measurement
  """

import time
import math
import numpy as np
import calibration

#variable definition
k = 1 #how agressively to increase accel measurement noise when drone is accelerating.
chi2_threshold = 11.34 #chi2 threshold for 3 DOF, 99% confidence interval - accel and mag corrections (3 DOF each)
chi2_threshold_gps = 16.81 #chi2 threshold for 6 DOF, 99% confidence interval - GPS correction (position+velocity, 6 DOF)
GRAVITY = 9.80665

#---- ground truth (separate from the filter's own estimate) - a genuinely dynamic
#trajectory instead of frozen constants. A live sensor reports whatever the vehicle is
#actually doing at that instant; frozen constants paired with a nonzero gyro rate meant
#"the vehicle is rotating" and "the vehicle stays at a fixed tilt forever" were both true
#at once, which is a contradiction the filter eventually has to notice and can't resolve.
#Same sinusoidal roll/pitch/yaw convention as testing/ekf_rms.py and
#testing/test_accel_bias_convergence.py, so this is the same trajectory already
#validated there, just driving this file's own live loop instead of a standalone test.
sim_time = 0.0
TRUE_AMP_ROLL = math.radians(15); TRUE_FREQ_ROLL = 0.5
TRUE_AMP_PITCH = math.radians(10); TRUE_FREQ_PITCH = 0.3; TRUE_PHASE_PITCH = math.pi / 2
TRUE_AMP_YAW = math.radians(5); TRUE_FREQ_YAW = 0.1; TRUE_PHASE_YAW = math.pi

#true (unknown-to-the-filter) sensor imperfections + noise - what the filter is actually
#trying to estimate/reject. accel/mag/gyro noise stds match testing/ekf_rms.py's
#already-validated values, NOT R_accel_base/R_mag's old 0.1 placeholder: 0.1 variance
#(std~0.316) is a fine ~3% relative accel noise (scale ~9.8 m/s²) but a wildly unrealistic
#~30%+ relative mag noise (scale ~1, unit vector) - injecting noise at that scale made
#accel_bias diverge outright (verified: R_mag=0.1 alone was enough to break convergence
#in testing/test_accel_bias_convergence.py; switching to these realistic stds fixed it).
#R_accel_base/R_mag below are updated to match these, so the filter's assumed noise
#matches what's actually injected - the gates and covariance are only well-calibrated
#when those agree.
TRUE_GYRO_BIAS = np.array([math.radians(2), math.radians(-1), math.radians(0.5)])
TRUE_ACCEL_BIAS = np.array([0.05, -0.02, 0.03])
TRUE_MAG_BIAS = np.array([0.04, 0.06, -0.03])   #hard-iron style sensor offset - see
                                                 #calibration.py's docstring caveat: this
                                                 #treats it as constant, real interference
                                                 #may be current/throttle-dependent instead
GYRO_NOISE_STD = math.radians(0.5)
ACCEL_NOISE_STD = 0.02
MAG_NOISE_STD = 0.01
GPS_POS_NOISE_STD = np.array([2.5, 2.5, 5.0])     #matches R_gps's position diagonal
GPS_VEL_NOISE_STD = np.array([0.15, 0.15, 0.3])   #matches R_gps's velocity diagonal

def true_roll(t): return TRUE_AMP_ROLL * math.sin(TRUE_FREQ_ROLL * t)
def true_roll_dot(t): return TRUE_AMP_ROLL * TRUE_FREQ_ROLL * math.cos(TRUE_FREQ_ROLL * t)
def true_pitch(t): return TRUE_AMP_PITCH * math.sin(TRUE_FREQ_PITCH * t + TRUE_PHASE_PITCH)
def true_pitch_dot(t): return TRUE_AMP_PITCH * TRUE_FREQ_PITCH * math.cos(TRUE_FREQ_PITCH * t + TRUE_PHASE_PITCH)
def true_yaw(t): return TRUE_AMP_YAW * math.sin(TRUE_FREQ_YAW * t + TRUE_PHASE_YAW)
def true_yaw_dot(t): return TRUE_AMP_YAW * TRUE_FREQ_YAW * math.cos(TRUE_FREQ_YAW * t + TRUE_PHASE_YAW)

#filter tuning (never reassigned, so these stay plain module constants even though the
#per-instance filter state below (q, P, biases, ...) moved onto DroneEKF)
Q = np.diag([0.01, 0.01, 0.01, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6]) #how much new uncertainty is added by the prediction step (how uncertain you are abt gyro)
R_accel_base = np.diag([ACCEL_NOISE_STD**2] * 3) #matches ACCEL_NOISE_STD above - the old 0.1 placeholder didn't match either sensor's real noise scale
R_mag = np.diag([MAG_NOISE_STD**2] * 3) #matches MAG_NOISE_STD above - 0.1 was ~30%+ relative noise on a unit vector, wildly unrealistic
I = np.eye(18) #identity matrix for updating the covariance
#how much uncertainty is added by the GPS measurement step. Grounded in typical consumer-GPS specs
#rather than an arbitrary placeholder: ~2.5m 1-sigma horizontal, ~5m 1-sigma vertical (altitude is
#usually ~2x worse than horizontal from a single constellation), ~0.15 m/s 1-sigma horizontal velocity,
#~0.3 m/s vertical (Doppler-derived velocity is typically much better than position, but still worse
#vertically) - variances, so these are the accuracy figures squared. Still not calibrated to a specific
#receiver's datasheet - swap in real numbers once you know the actual hardware.
R_gps = np.diag([6.25, 6.25, 25.0, 0.0225, 0.0225, 0.09])
gps_period = 0.2

#math helpers
def rotate_by_quat(q, v):
    # Rotate vector v by quaternion q
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
    v_quat = np.array([0, v[0], v[1], v[2]])
    rotated_v_quat = quat_mult(quat_mult(q, v_quat), q_conj)
    return rotated_v_quat[1:]

def quat_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_mult(q1, q2):
    w = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]
    x = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]
    y = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]
    z = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]
    return np.array([w, x, y, z])

def skew(v):
    return np.array([[0,-v[2],v[1]], [v[2],0,-v[0]], [-v[1],v[0],0]])

def quat_to_R(q):
    #body->world rotation matrix, built by rotating the standard basis vectors through rotate_by_quat -
    #reuses the already-verified rotation function instead of a separate closed-form formula, so it's
    #guaranteed consistent with rotate_by_quat's own quaternion convention
    return np.array([rotate_by_quat(q, e) for e in np.eye(3)]).T

#sensor reading functions (simulated for now) - each computes the true body-frame
#reading from the ground truth at the current sim_time, then adds the true (unknown to
#the filter) bias/noise, exactly what a live sensor would hand the filter: a fresh,
#already-corrupted reading every call, not a stored constant
def get_gyro():
    roll, pitch = true_roll(sim_time), true_pitch(sim_time)
    roll_dot, pitch_dot, yaw_dot = true_roll_dot(sim_time), true_pitch_dot(sim_time), true_yaw_dot(sim_time)
    wx = roll_dot - yaw_dot * math.sin(pitch)
    wy = pitch_dot * math.cos(roll) + yaw_dot * math.sin(roll) * math.cos(pitch)
    wz = -pitch_dot * math.sin(roll) + yaw_dot * math.cos(roll) * math.cos(pitch)
    noise = np.random.normal(0, GYRO_NOISE_STD, 3)
    return (wx + TRUE_GYRO_BIAS[0] + noise[0],
            wy + TRUE_GYRO_BIAS[1] + noise[1],
            wz + TRUE_GYRO_BIAS[2] + noise[2])

def get_accel():
    #hovering - specific force is gravity-only, rotated into the body frame by the true attitude
    roll, pitch = true_roll(sim_time), true_pitch(sim_time)
    ax = -GRAVITY * math.sin(pitch)
    ay = GRAVITY * math.sin(roll) * math.cos(pitch)
    az = GRAVITY * math.cos(roll) * math.cos(pitch)
    noise = np.random.normal(0, ACCEL_NOISE_STD, 3)
    return (ax + TRUE_ACCEL_BIAS[0] + noise[0],
            ay + TRUE_ACCEL_BIAS[1] + noise[1],
            az + TRUE_ACCEL_BIAS[2] + noise[2])

def get_mag():
    roll, pitch, yaw = true_roll(sim_time), true_pitch(sim_time), true_yaw(sim_time)
    mx = math.cos(pitch) * math.cos(yaw)
    my = math.sin(roll) * math.sin(pitch) * math.cos(yaw) - math.cos(roll) * math.sin(yaw)
    mz = math.cos(roll) * math.sin(pitch) * math.cos(yaw) + math.sin(roll) * math.sin(yaw)
    noise = np.random.normal(0, MAG_NOISE_STD, 3)
    return (mx + TRUE_MAG_BIAS[0] + noise[0],
            my + TRUE_MAG_BIAS[1] + noise[1],
            mz + TRUE_MAG_BIAS[2] + noise[2])

def get_gps():
    #hovering translationally (true position/velocity are always zero) - this file's
    #focus is attitude/bias consistency, not a translating trajectory; testing/test_gps_tracking.py
    #and testing/test_accel_bias_convergence.py already cover translational motion
    noise_pos = np.random.normal(0, GPS_POS_NOISE_STD)
    noise_vel = np.random.normal(0, GPS_VEL_NOISE_STD)
    return (noise_pos[0], noise_pos[1], noise_pos[2], noise_vel[0], noise_vel[1], noise_vel[2])

#---- pre-flight calibration wiggle: a separate set of sensor functions (_cal_get_*),
#used only for the calibration.calibrate() call below, so it sees a genuinely changing
#attitude (dwell, then a small deliberate roll/pitch/yaw wiggle) instead of the constant
#"in-flight" stubs the main loop uses - without this, accel_bias/mag_bias and
#attitude/heading tilt are indistinguishable from one orientation (see calibration.py)
#and the calibration call would just be going through the motions with nothing to
#disambiguate them. Yaw wiggle specifically is what mag_bias needs (same role
#roll/pitch play for accel_bias) - without it mag_bias vs. heading error is exactly as
#ambiguous as accel_bias vs. tilt was before the roll/pitch wiggle existed ----
_calibration_step = 0
_CAL_DWELL_STEPS = 200
_CAL_DT = 0.01
_CAL_WIGGLE_AMP_ROLL = math.radians(8)
_CAL_WIGGLE_FREQ_ROLL = 0.8
_CAL_WIGGLE_AMP_PITCH = math.radians(6)
_CAL_WIGGLE_FREQ_PITCH = 0.5
_CAL_WIGGLE_AMP_YAW = math.radians(10)
_CAL_WIGGLE_FREQ_YAW = 0.4

def _cal_attitude(step):
    if step < _CAL_DWELL_STEPS:
        return 0.0, 0.0, 0.0
    t = (step - _CAL_DWELL_STEPS) * _CAL_DT
    roll = _CAL_WIGGLE_AMP_ROLL * math.sin(_CAL_WIGGLE_FREQ_ROLL * t)
    pitch = _CAL_WIGGLE_AMP_PITCH * math.sin(_CAL_WIGGLE_FREQ_PITCH * t)
    yaw = _CAL_WIGGLE_AMP_YAW * math.sin(_CAL_WIGGLE_FREQ_YAW * t)
    return roll, pitch, yaw

def _cal_rates(step):
    if step < _CAL_DWELL_STEPS:
        return 0.0, 0.0, 0.0
    t = (step - _CAL_DWELL_STEPS) * _CAL_DT
    roll_dot = _CAL_WIGGLE_AMP_ROLL * _CAL_WIGGLE_FREQ_ROLL * math.cos(_CAL_WIGGLE_FREQ_ROLL * t)
    pitch_dot = _CAL_WIGGLE_AMP_PITCH * _CAL_WIGGLE_FREQ_PITCH * math.cos(_CAL_WIGGLE_FREQ_PITCH * t)
    yaw_dot = _CAL_WIGGLE_AMP_YAW * _CAL_WIGGLE_FREQ_YAW * math.cos(_CAL_WIGGLE_FREQ_YAW * t)
    return roll_dot, pitch_dot, yaw_dot

def _cal_get_gyro():
    roll, pitch, _ = _cal_attitude(_calibration_step)
    roll_dot, pitch_dot, yaw_dot = _cal_rates(_calibration_step)
    wx = roll_dot - yaw_dot * math.sin(pitch)
    wy = pitch_dot * math.cos(roll) + yaw_dot * math.sin(roll) * math.cos(pitch)
    wz = -pitch_dot * math.sin(roll) + yaw_dot * math.cos(roll) * math.cos(pitch)
    return wx, wy, wz

def _cal_get_accel():
    global _calibration_step
    roll, pitch, _ = _cal_attitude(_calibration_step)
    ax = -GRAVITY * math.sin(pitch)
    ay = GRAVITY * math.sin(roll) * math.cos(pitch)
    az = GRAVITY * math.cos(roll) * math.cos(pitch)
    _calibration_step += 1   #advance once per calibrate() iteration - calibrate() calls
                              #get_gyro then get_mag then get_accel each step, so
                              #incrementing here (the last of the three) keeps all three
                              #consistent within a step before moving to the next one
    return ax, ay, az

def _cal_get_mag():
    roll, pitch, yaw = _cal_attitude(_calibration_step)
    mx = math.cos(pitch) * math.cos(yaw)
    my = math.sin(roll) * math.sin(pitch) * math.cos(yaw) - math.cos(roll) * math.sin(yaw)
    mz = math.cos(roll) * math.sin(pitch) * math.cos(yaw) + math.sin(roll) * math.sin(yaw)
    v = np.array([mx, my, mz])
    v = v / np.linalg.norm(v)
    return tuple(v)

def update_F(w, dt, F, q, accel_body):
    #F is a matrix of partial derivatives - a Jacobian
    # w = [wx, wy, wz] # corrected gyro vector
    F[0:3, 0:3] = np.eye(3) - dt * skew(w)
    F[0:3, 3:6] = -dt * np.eye(3)
    F[3:6, 0:3] = 0
    F[3:6, 3:6] = np.eye(3)
    #how attitude error perturbs predicted world-frame accel (verified numerically via finite difference:
    #-R(q) @ skew(accel_body) matched the numeric Jacobian to ~1e-6; skew(accel_world) alone did not)
    F[6:9, 0:3] = -dt * quat_to_R(q) @ skew(accel_body)
    F[9:12, 6:9] = dt * np.eye(3)
    F[12:15, 12:15] = np.eye(3)
    F[6:9, 12:15] = -dt * quat_to_R(q)
    F[15:18, 15:18] = np.eye(3)   # mag_bias persists - no coupling into velocity/position
                                   # the way accel_bias has (mag doesn't drive propagation)
    return F

def update_H_accel(predicted_accel, H):
    H[0:3, 0:3] = skew(predicted_accel)
    H[0:3, 3:6] = 0
    H[0:3, 12:15] = np.eye(3)   # accel rows' Jacobian wrt accel_bias error
    H[0:3, 15:18] = 0            # explicit - accel doesn't see mag bias
    return H

def update_H_mag(predicted_mag, H):
    H[0:3, 0:3] = skew(predicted_mag)
    H[0:3, 3:6] = 0
    H[0:3, 12:15] = 0            # explicit - mag doesn't see accel bias
    H[0:3, 15:18] = np.eye(3)    # mag rows' Jacobian wrt mag_bias error - same
                                  # derivation as accel_bias's +I (see calibration.py)
    return H

def update_H_gps(H):
    #gps measures position AND velocity directly (linear model, no skew()/Jacobian needed) -
    #rows 0:3 pick out the d_position columns, rows 3:6 pick out the d_velocity columns
    H[:, 0:6] = 0
    H[0:3, 9:12] = np.eye(3)
    H[3:6, 6:9] = np.eye(3)
    return H


class DroneEKF:
    def __init__(self, sensors=None):
        # sensors: object exposing get_gyro/get_accel/get_mag/get_gps (same shapes as
        # this module's own simulated versions) - defaults to those simulated stand-ins,
        # but step() can instead be driven by e.g. gz_bridge.GazeboBridge for real sim data.
        # Calibration always uses its own synthetic _cal_get_* wiggle functions regardless
        # (it needs a deliberate dwell+wiggle sequence, not whatever the live sensors report).
        self._get_gyro = get_gyro if sensors is None else sensors.get_gyro
        self._get_accel = get_accel if sensors is None else sensors.get_accel
        self._get_mag = get_mag if sensors is None else sensors.get_mag
        self._get_gps = get_gps if sensors is None else sensors.get_gps

        #state variables - placeholders, overwritten by the pre-flight calibration call below
        #(get_gyro/get_accel/get_mag have to be defined first, so the actual calibrate() call
        #happens further down, right after those are defined)
        self.q = np.array([1.0, 0.0, 0.0, 0.0]) #identity quaternion, [w, x, y, z]
        self.bias_x = 0
        self.bias_y = 0
        self.bias_z = 0
        self.accel_bias_x = 0
        self.accel_bias_y = 0
        self.accel_bias_z = 0
        self.mag_bias_x = 0
        self.mag_bias_y = 0
        self.mag_bias_z = 0
        self.velocity = np.array([0.0, 0.0, 0.0]) # Initialize velocity
        self.position = np.array([0.0, 0.0, 0.0])
        self.rate = np.array([0.0, 0.0, 0.0])  # bias-corrected gyro, refreshed each step() - exposed via get_state()

        #filter matrices - 18 states: attitude(0:3), gyro_bias(3:6), velocity(6:9),
        #position(9:12), accel_bias(12:15), mag_bias(15:18)
        self.P = np.eye(18) #placeholder - overwritten below, after calibration, from calibration's own converged P
        self.F = np.eye(18) #state transition matrix, how the state evolves from one step to the next without control input (identity for this case)
        self.H_accel = np.zeros((3, 18)) #measurement matrix for accel - separate from H_mag now, since each correction re-linearizes independently (see apply_correction)
        self.H_mag = np.zeros((3, 18)) #measurement matrix for mag
        self.H_gps = np.zeros((6, 18)) #measurement matrix for GPS, how the measurements relate to the state (position rows 0:3, velocity rows 3:6)

        #pre-flight calibration - seeds q/gyro_bias/accel_bias/mag_bias/P instead of starting
        #from identity/zero/np.eye(18) (see calibration.py: accel_bias/mag_bias and
        #attitude/heading tilt are otherwise unobservable from a single fixed orientation,
        #which is exactly what the accel_bias/mag_bias columns in update_H_accel/update_H_mag
        #below need at least a decent starting point for). Seeding P from calibration's own
        #converged uncertainty - not a blind np.eye(18) guess - matters specifically because
        #np.eye(15) (this file's old 15-state size) gave the pooled accel+mag correction below
        #a ~30% chance of overcorrecting badly enough to lock the accel gate on permanently
        #(see testing/test_gate_monte_carlo.py; the sequential apply_correction design below
        #also independently closes this, but a good P0 remains cheap insurance).
        global _calibration_step
        _calibration_step = 0  # reset so a later DroneEKF() doesn't inherit an earlier one's leftover count
        if sensors is None:
            # synthetic self-contained mode: use the scripted dwell+wiggle functions,
            # which are specifically designed to recover this file's own TRUE_GYRO_BIAS/
            # TRUE_ACCEL_BIAS/TRUE_MAG_BIAS constants
            self.q, (self.bias_x, self.bias_y, self.bias_z), (self.accel_bias_x, self.accel_bias_y, self.accel_bias_z), \
                (self.mag_bias_x, self.mag_bias_y, self.mag_bias_z), _P_cal = \
                calibration.calibrate(_cal_get_gyro, _cal_get_accel, _cal_get_mag,
                                       dwell_steps=_CAL_DWELL_STEPS, wiggle_steps=400, dt=_CAL_DT,
                                       gravity=GRAVITY, accel_noise_var=ACCEL_NOISE_STD**2, mag_noise_var=MAG_NOISE_STD**2)
        else:
            # live-bridge mode: calibrating against the synthetic _cal_get_* functions here
            # would learn a bias correction for TRUE_GYRO_BIAS etc., which have nothing to do
            # with this sensor's actual (near-zero) bias - inject a real, wrong, persistent
            # rate error into every future step(). Use the real sensors instead. Dwell-only
            # (no wiggle - nothing here can command an actual wiggle maneuver), so accel_bias/
            # mag_bias may keep some tilt/heading ambiguity, but gyro_bias (this file's real
            # problem) is fully observable at rest regardless - see calibration.py's docstring.
            self.q, (self.bias_x, self.bias_y, self.bias_z), (self.accel_bias_x, self.accel_bias_y, self.accel_bias_z), \
                (self.mag_bias_x, self.mag_bias_y, self.mag_bias_z), _P_cal = \
                calibration.calibrate(self._get_gyro, self._get_accel, self._get_mag,
                                       dwell_steps=_CAL_DWELL_STEPS, wiggle_steps=0, dt=_CAL_DT,
                                       gravity=GRAVITY, accel_noise_var=ACCEL_NOISE_STD**2, mag_noise_var=MAG_NOISE_STD**2)
        #main loop below uses get_gyro/get_accel/get_mag (the constant "in-flight" stubs), not the _cal_* functions

        #map calibration's 12x12 P (attitude, gyro_bias, accel_bias, mag_bias, in that order)
        #onto this file's 18x18 layout (attitude, gyro_bias, velocity, position, accel_bias,
        #mag_bias) - velocity/position get a modest independent prior since calibration never
        #modeled them (the vehicle was assumed stationary throughout)
        self.P = np.eye(18) * 0.1
        self.P[0:3, 0:3] = _P_cal[0:3, 0:3]      #attitude-attitude
        self.P[0:3, 3:6] = _P_cal[0:3, 3:6]      #attitude-gyro_bias
        self.P[3:6, 0:3] = _P_cal[3:6, 0:3]
        self.P[3:6, 3:6] = _P_cal[3:6, 3:6]      #gyro_bias-gyro_bias
        self.P[0:3, 12:15] = _P_cal[0:3, 6:9]    #attitude-accel_bias
        self.P[12:15, 0:3] = _P_cal[6:9, 0:3]
        self.P[3:6, 12:15] = _P_cal[3:6, 6:9]    #gyro_bias-accel_bias
        self.P[12:15, 3:6] = _P_cal[6:9, 3:6]
        self.P[12:15, 12:15] = _P_cal[6:9, 6:9]  #accel_bias-accel_bias
        self.P[0:3, 15:18] = _P_cal[0:3, 9:12]   #attitude-mag_bias
        self.P[15:18, 0:3] = _P_cal[9:12, 0:3]
        self.P[3:6, 15:18] = _P_cal[3:6, 9:12]   #gyro_bias-mag_bias
        self.P[15:18, 3:6] = _P_cal[9:12, 3:6]
        self.P[12:15, 15:18] = _P_cal[6:9, 9:12] #accel_bias-mag_bias
        self.P[15:18, 12:15] = _P_cal[9:12, 6:9]
        self.P[15:18, 15:18] = _P_cal[9:12, 9:12] #mag_bias-mag_bias

        #timing starts here, AFTER calibration - see the comment above where these used to live
        self.last_time = time.time()
        self.last_gps_time = time.time()

    def apply_correction(self, correction):
        #injects one measurement's correction immediately and re-normalizes q, instead of
        #pooling GPS+accel+mag against one frozen q and injecting once at the end. Pooling
        #let two large-gain corrections (accel+mag) double-count against the same stale
        #linearization when P was large, occasionally overcorrecting badly enough to lock
        #the accel gate on permanently (see testing/test_gate_monte_carlo.py's ~30% lockout
        #rate with np.eye(15) P0) - re-linearizing between each correction removes that
        #mechanism outright, rather than just avoiding its trigger via a better-tuned P0.
        d_theta = correction[0:3]
        d_bias = correction[3:6]
        d_velocity = correction[6:9]
        d_position = correction[9:12]
        d_accel_bias = correction[12:15]
        d_mag_bias = correction[15:18]

        self.bias_x = self.bias_x + d_bias[0]
        self.bias_y = self.bias_y + d_bias[1]
        self.bias_z = self.bias_z + d_bias[2]
        self.accel_bias_x += d_accel_bias[0]
        self.accel_bias_y += d_accel_bias[1]
        self.accel_bias_z += d_accel_bias[2]
        self.mag_bias_x += d_mag_bias[0]
        self.mag_bias_y += d_mag_bias[1]
        self.mag_bias_z += d_mag_bias[2]
        self.velocity += d_velocity
        self.position += d_position

        dq = np.array([1.0, d_theta[0]/2, d_theta[1]/2, d_theta[2]/2])
        self.q = quat_mult(self.q, dq)
        self.q = self.q / np.linalg.norm(self.q)

    def step(self):
        #one iteration of what used to be the module-level main loop: read sensors,
        #predict, then apply gps/accel/mag corrections as they come available
        global sim_time

        #read the sensors
        gyro_x, gyro_y, gyro_z = self._get_gyro()
        mag_x, mag_y, mag_z = self._get_mag()
        accel_x, accel_y, accel_z = self._get_accel()

        corrected_gyro_x = gyro_x - self.bias_x
        corrected_gyro_y = gyro_y - self.bias_y
        corrected_gyro_z = gyro_z - self.bias_z
        self.rate = np.array([corrected_gyro_x, corrected_gyro_y, corrected_gyro_z])

        corrected_accel_x = accel_x - self.accel_bias_x
        corrected_accel_y = accel_y - self.accel_bias_y
        corrected_accel_z = accel_z - self.accel_bias_z

        #adaptive accel noise - computed from the BIAS-CORRECTED accel, not raw (2026-09-26
        # fix). Using raw accel here let deviation/R_accel and the gate's accept/reject
        # decision run on a different signal than what the prediction step and the
        # correction's own residual actually use (corrected_accel) - a real, sustained
        # acceleration could get partially absorbed into accel_bias one small accepted
        # correction at a time (each individually under the gate, computed from a raw
        # deviation that didn't reflect how much the bias had already crept), while the
        # bias-corrected accel fed to prediction stayed anomalously close to gravity-only.
        # Confirmed via test_velocity_loop.py: state['vel'] froze bit-for-bit for ~0.6s of
        # real, sustained climb thrust while raw deviation climbed smoothly through 0.08
        # to 0.43 the whole time - the gate was deciding on a signal that didn't match
        # what prediction was actually integrating.
        accel_magnitude = math.sqrt(corrected_accel_x**2 + corrected_accel_y**2 + corrected_accel_z**2)
        deviation = abs(accel_magnitude - GRAVITY) #how far off from 1g is the (bias-corrected) accel reading?
        R_accel = R_accel_base * (1 + k * deviation ** 2) #increase accel measurement noise if drone is accelerating

        corrected_mag_x = mag_x - self.mag_bias_x
        corrected_mag_y = mag_y - self.mag_bias_y
        corrected_mag_z = mag_z - self.mag_bias_z

        #predict: integrate gyro into current angle estimation
        now = time.time()
        dt = now - self.last_time #sampling as fast as the hardware can handle

        #using gyro to advance the attitude estimation forward by one timestep.
        #multiplying 2 unit quaternions
        w_quat = [0, corrected_gyro_x, corrected_gyro_y, corrected_gyro_z] #building a quaternarion with scalar value 0
        q_dot = 0.5 * quat_mult(self.q, w_quat) #q dot is how fast the quaternarion is moving, but we need to convert it into quaternarion space so we can integrate it in the next step
        self.q = self.q + q_dot * dt #integrating over time
        self.q = self.q / np.linalg.norm(self.q) #renormalize so the quaternarion magnitude = 1, and its still on the unit sphere, making it a pure rotation

        #velocity/position prediction
        corrected_accel = [corrected_accel_x, corrected_accel_y, corrected_accel_z]
        accel_world = rotate_by_quat(self.q, corrected_accel) #put in world frame
        accel_world = accel_world - [0, 0, GRAVITY] #subtract gravity
        self.velocity += accel_world * dt #integrate velcoity
        self.position += self.velocity * dt #integrate position

        # Update the state transition matrix based on the current state and time step
        w = np.array([corrected_gyro_x, corrected_gyro_y, corrected_gyro_z])
        self.F = update_F(w, dt, self.F, self.q, corrected_accel) #describes how error state vector evolves.
        self.P = self.F @ self.P @ self.F.T + Q #update covariance error matrix, uncertainty increases because we are just integrating the model

        #Each correction below is applied immediately (apply_correction) and re-linearizes
        #the next one against the just-updated q, rather than pooling all three against one
        #frozen q and injecting once at the end - see apply_correction's comment for why.

        #gated bc gps sample rate is lower than accel/mag/gyro
        if now - self.last_gps_time >= gps_period:
            #get gps data
            north, east, up, vel_north, vel_east, vel_up = self._get_gps()
            gps_measurement = np.array([north, east, up, vel_north, vel_east, vel_up]) #current GPS position + velocity, compared against predicted position/velocity in the gps correction step

            #gps correction
            self.H_gps = update_H_gps(self.H_gps) #builds measurment jacobian for this update
            S_gps = self.H_gps @ self.P @ self.H_gps.T + R_gps #how much uncertainty you'd expect in residual, combining current uncertainty with sensor noise R_gps
            K_gps = self.P @ self.H_gps.T @ np.linalg.inv(S_gps) #computing kalman fain
            predicted_gps = np.concatenate([self.position, self.velocity]) #what you expect gps to report
            residual_gps = gps_measurement - predicted_gps #what gps reported vs what you expected
            #gate against a bad fix (multipath, momentary bad geometry) - R_gps isn't
            #adaptively inflated like R_accel, so no separate "base" R is needed here
            d_squared_gps = residual_gps.T @ np.linalg.inv(S_gps) @ residual_gps

            #to catch unreasonable measurements and not let them corrupt the state.
            if d_squared_gps <= chi2_threshold_gps:
                self.apply_correction(K_gps @ residual_gps)
                #Joseph form - numerically robust to floating-point drift (keeps P symmetric/PSD),
                #vs the algebraically-equivalent but fragile (I-KH)@P
                self.P = (I - K_gps @ self.H_gps) @ self.P @ (I - K_gps @ self.H_gps).T + K_gps @ R_gps @ K_gps.T
                #this is the second p update, after we incorporate a measuremnt uncertainty goes down bc that is another measurement source
            #else: skip entirely - state and P stay exactly as the predict step left them
            self.last_gps_time = now

        #accel correction - predicted_accel uses q as GPS may have just updated it (via P's
        #attitude-position cross-covariance, even though H_gps itself has no attitude columns)
        predicted_accel = rotate_by_quat(quat_conjugate(self.q), np.array([0.0, 0.0, GRAVITY]))
        self.H_accel = update_H_accel(predicted_accel, self.H_accel)

        S_accel = self.H_accel @ self.P @ self.H_accel.T + R_accel
        K_accel = self.P @ self.H_accel.T @ np.linalg.inv(S_accel)
        residual_accel = np.array([corrected_accel_x, corrected_accel_y, corrected_accel_z]) - predicted_accel
        #gate against BASE noise, not the already-inflated adaptive R_accel - otherwise
        #adaptive R inflates in lockstep with the residual and the gate can never fire
        #(d_squared asymptotes to ~1/k for large outliers regardless of severity)
        S_accel_gate = self.H_accel @ self.P @ self.H_accel.T + R_accel_base
        d_squared = residual_accel.T @ np.linalg.inv(S_accel_gate) @ residual_accel

        # TEMP DIAGNOSTIC (2026-09-26) - chasing a sudden single-tick attitude jump seen
        # in test_attitude_loop.py while state['rate'] (raw gyro, doesn't depend on q)
        # stays smooth through the same tick - theory is a bad accel correction slipping
        # through during real (non-hover) acceleration. Remove once resolved.
        correction_accel = K_accel @ residual_accel
        if d_squared <= chi2_threshold:
            print(f"EKF_ACCEL t={now:.6f} FIRED   dev={deviation:.4f} d2={d_squared:8.2f} "
                  f"thr={chi2_threshold:.2f} dtheta={np.linalg.norm(correction_accel[0:3]):.4f}", flush=True)
        else:
            print(f"EKF_ACCEL t={now:.6f} REJECTED dev={deviation:.4f} d2={d_squared:8.2f} "
                  f"thr={chi2_threshold:.2f}", flush=True)

        if d_squared <= chi2_threshold:
          self.apply_correction(correction_accel)
          #this is the second p update, after we incorporate a measuremnt uncertainty goes down bc that is another measurement source
          self.P = (I - K_accel @ self.H_accel) @ self.P @ (I - K_accel @ self.H_accel).T + K_accel @ R_accel @ K_accel.T
        #else: skip entirely - state and P stay exactly as the predict step left them

        #mag correction - predicted_mag uses q as it stands AFTER the accel correction just
        #applied, not the stale pre-accel q - this is what actually removes the overcorrection
        #risk, not just avoiding a bad P0 (see apply_correction's comment)
        predicted_mag = rotate_by_quat(quat_conjugate(self.q), np.array([1.0, 0.0, 0.0]))
        self.H_mag = update_H_mag(predicted_mag, self.H_mag)

        S_mag = self.H_mag @ self.P @ self.H_mag.T + R_mag
        K_mag = self.P @ self.H_mag.T @ np.linalg.inv(S_mag)
        residual_mag = np.array([corrected_mag_x, corrected_mag_y, corrected_mag_z]) - predicted_mag
        #gate against magnetic interference (motors/ESCs) - R_mag is static, not
        #adaptively inflated like R_accel, so no separate "base" R is needed here
        d_squared_mag = residual_mag.T @ np.linalg.inv(S_mag) @ residual_mag

        if d_squared_mag <= chi2_threshold:
            self.apply_correction(K_mag @ residual_mag)
            #this is the second p update, after we incorporate a measuremnt uncertainty goes down bc that is another measurement source
            self.P = (I - K_mag @ self.H_mag) @ self.P @ (I - K_mag @ self.H_mag).T + K_mag @ R_mag @ K_mag.T
        #else: skip entirely - state and P stay exactly as the predict step left them

        #loop timing
        self.last_time = now
        sim_time += dt   #advances the ground truth by the same dt the filter just integrated with

        return self.get_state()

    def get_state(self):
        #matches the state dict shape PID/cascade.py's Cascade.step() expects
        return {"pos": self.position, "vel": self.velocity, "quat": self.q, "rate": self.rate}


if __name__ == "__main__":
    ekf = DroneEKF()
    while True:
        ekf.step()
