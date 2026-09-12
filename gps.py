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

#state variables - placeholders, overwritten by the pre-flight calibration call below
#(get_gyro/get_accel/get_mag have to be defined first, so the actual calibrate() call
#happens further down, right after those are defined)
q = np.array([1.0, 0.0, 0.0, 0.0]) #identity quaternion, [w, x, y, z]
bias_x = 0
bias_y = 0
bias_z = 0
accel_bias_x = 0
accel_bias_y = 0
accel_bias_z = 0
velocity = np.array([0.0, 0.0, 0.0]) # Initialize velocity
position = np.array([0.0, 0.0, 0.0])

#filter matrices
P = np.eye(15) #how uncertain you currently are about each state, and how uncertainties are correlated
Q = np.diag([0.01, 0.01, 0.01, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6]) #how much new uncertainty is added by the prediction step (how uncertain you are abt gyro)
R_accel_base = np.diag([ACCEL_NOISE_STD**2] * 3) #matches ACCEL_NOISE_STD above - the old 0.1 placeholder didn't match either sensor's real noise scale
R_mag = np.diag([MAG_NOISE_STD**2] * 3) #matches MAG_NOISE_STD above - 0.1 was ~30%+ relative noise on a unit vector, wildly unrealistic
F = np.eye(15) #state transition matrix, how the state evolves from one step to the next without control input (identity for this case)
I = np.eye(15) #identity matrix for updating the covariance
H_accel_mag = np.zeros((6, 15)) #measurement matrix, how the measurements relate to the state
H_gps = np.zeros((6, 15)) #measurement matrix for GPS, how the measurements relate to the state (position rows 0:3, velocity rows 3:6)
#how much uncertainty is added by the GPS measurement step. Grounded in typical consumer-GPS specs
#rather than an arbitrary placeholder: ~2.5m 1-sigma horizontal, ~5m 1-sigma vertical (altitude is
#usually ~2x worse than horizontal from a single constellation), ~0.15 m/s 1-sigma horizontal velocity,
#~0.3 m/s vertical (Doppler-derived velocity is typically much better than position, but still worse
#vertically) - variances, so these are the accuracy figures squared. Still not calibrated to a specific
#receiver's datasheet - swap in real numbers once you know the actual hardware.
R_gps = np.diag([6.25, 6.25, 25.0, 0.0225, 0.0225, 0.09])

#timing variables - last_time/last_gps_time are set AFTER calibration below, not here:
#calibration.calibrate() runs hundreds of iterations before the main loop starts, and
#setting these before that call made the main loop's first dt include that whole
#calibration duration (~40ms instead of ~10ms) as if it were one integration step
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
    return (mx + noise[0], my + noise[1], mz + noise[2])

def get_gps():
    #hovering translationally (true position/velocity are always zero) - this file's
    #focus is attitude/bias consistency, not a translating trajectory; testing/test_gps_tracking.py
    #and testing/test_accel_bias_convergence.py already cover translational motion
    noise_pos = np.random.normal(0, GPS_POS_NOISE_STD)
    noise_vel = np.random.normal(0, GPS_VEL_NOISE_STD)
    return (noise_pos[0], noise_pos[1], noise_pos[2], noise_vel[0], noise_vel[1], noise_vel[2])

#---- pre-flight calibration wiggle: a separate set of sensor functions (_cal_get_*),
#used only for the calibration.calibrate() call below, so it sees a genuinely changing
#attitude (dwell, then a small deliberate roll/pitch wiggle) instead of the constant
#"in-flight" stubs the main loop uses - without this, accel_bias and attitude tilt are
#indistinguishable from one orientation (see calibration.py) and the calibration call
#would just be going through the motions with nothing to disambiguate them ----
_calibration_step = 0
_CAL_DWELL_STEPS = 200
_CAL_DT = 0.01
_CAL_WIGGLE_AMP_ROLL = math.radians(8)
_CAL_WIGGLE_FREQ_ROLL = 0.8
_CAL_WIGGLE_AMP_PITCH = math.radians(6)
_CAL_WIGGLE_FREQ_PITCH = 0.5

def _cal_attitude(step):
    if step < _CAL_DWELL_STEPS:
        return 0.0, 0.0
    t = (step - _CAL_DWELL_STEPS) * _CAL_DT
    roll = _CAL_WIGGLE_AMP_ROLL * math.sin(_CAL_WIGGLE_FREQ_ROLL * t)
    pitch = _CAL_WIGGLE_AMP_PITCH * math.sin(_CAL_WIGGLE_FREQ_PITCH * t)
    return roll, pitch

def _cal_rates(step):
    if step < _CAL_DWELL_STEPS:
        return 0.0, 0.0
    t = (step - _CAL_DWELL_STEPS) * _CAL_DT
    roll_dot = _CAL_WIGGLE_AMP_ROLL * _CAL_WIGGLE_FREQ_ROLL * math.cos(_CAL_WIGGLE_FREQ_ROLL * t)
    pitch_dot = _CAL_WIGGLE_AMP_PITCH * _CAL_WIGGLE_FREQ_PITCH * math.cos(_CAL_WIGGLE_FREQ_PITCH * t)
    return roll_dot, pitch_dot

def _cal_get_gyro():
    roll_dot, pitch_dot = _cal_rates(_calibration_step)
    return roll_dot, pitch_dot, 0.0

def _cal_get_accel():
    global _calibration_step
    roll, pitch = _cal_attitude(_calibration_step)
    ax = -GRAVITY * math.sin(pitch)
    ay = GRAVITY * math.sin(roll) * math.cos(pitch)
    az = GRAVITY * math.cos(roll) * math.cos(pitch)
    _calibration_step += 1   #advance once per calibrate() iteration - calibrate() calls
                              #get_gyro then get_mag then get_accel each step, so
                              #incrementing here (the last of the three) keeps all three
                              #consistent within a step before moving to the next one
    return ax, ay, az

def _cal_get_mag():
    roll, pitch = _cal_attitude(_calibration_step)
    mx = math.cos(pitch)
    my = math.sin(roll) * math.sin(pitch)
    mz = math.cos(roll) * math.sin(pitch)
    v = np.array([mx, my, mz])
    v = v / np.linalg.norm(v)
    return tuple(v)

#pre-flight calibration - seeds q/gyro_bias/accel_bias instead of starting from
#identity/zero (see calibration.py: accel_bias and attitude tilt are otherwise
#unobservable from a single fixed orientation, which is exactly what the accel_bias
#column in update_H_mag_accel below needs at least a decent starting point for).
q, (bias_x, bias_y, bias_z), (accel_bias_x, accel_bias_y, accel_bias_z) = \
    calibration.calibrate(_cal_get_gyro, _cal_get_accel, _cal_get_mag,
                           dwell_steps=_CAL_DWELL_STEPS, wiggle_steps=400, dt=_CAL_DT,
                           gravity=GRAVITY, accel_noise_var=ACCEL_NOISE_STD**2, mag_noise_var=MAG_NOISE_STD**2)
#main loop below uses get_gyro/get_accel/get_mag (the constant "in-flight" stubs), not the _cal_* functions

#timing starts here, AFTER calibration - see the comment above where these used to live
last_time = time.time()
last_gps_time = time.time()

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
    return F

def update_H_mag_accel(predicted_accel, predicted_mag, H):
    #accel rows: jacobian of predicted_accel wrt state
    H[0:3, 0:3] = skew(predicted_accel)
    H[0:3, 3:6] = 0
    H[0:3, 12:15] = np.eye(3)   # accel rows' Jacobian wrt accel_bias error
    H[3:6, 0:3] = skew(predicted_mag)
    H[3:6, 3:6] = 0
    H[3:6, 12:15] = 0            # explicit, for symmetry — mag doesn't see accel bias
    return H

def update_H_gps(H):
    #gps measures position AND velocity directly (linear model, no skew()/Jacobian needed) -
    #rows 0:3 pick out the d_position columns, rows 3:6 pick out the d_velocity columns
    H[:, 0:6] = 0
    H[0:3, 9:12] = np.eye(3)
    H[3:6, 6:9] = np.eye(3)
    return H

#main loop
while True:

    #read the sensors
    gyro_x, gyro_y, gyro_z = get_gyro()
    mag_x, mag_y, mag_z = get_mag()
    accel_x, accel_y, accel_z = get_accel()

    #adaptive accel noise + bias-corrected gyro + accel
    accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
    deviation = abs(accel_magnitude - GRAVITY) #how far off from 1g is the accel reading?
    R_accel = R_accel_base * (1 + k * deviation ** 2) #increase accel measurement noise if drone is accelerating
    corrected_gyro_x = gyro_x - bias_x
    corrected_gyro_y = gyro_y - bias_y
    corrected_gyro_z = gyro_z - bias_z
    
    corrected_accel_x = accel_x - accel_bias_x
    corrected_accel_y = accel_y - accel_bias_y
    corrected_accel_z = accel_z - accel_bias_z

    #predict: integrate gyro into current angle estimation
    now = time.time()
    dt = now - last_time #sampling as fast as the hardware can handle

    error_state = np.zeros(15)  # Initialize error state vector

    #using gyro to advance the attitude estimation forward by one timestep. 
    #multiplying 2 unit quaternions
    w_quat = [0, corrected_gyro_x, corrected_gyro_y, corrected_gyro_z] #building a quaternarion with scalar value 0
    q_dot = 0.5 * quat_mult(q, w_quat) #q dot is how fast the quaternarion is moving, but we need to convert it into quaternarion space so we can integrate it in the next step
    q = q + q_dot * dt #integrating over time
    q = q / np.linalg.norm(q) #renormalize so the quaternarion magnitude = 1, and its still on the unit sphere, making it a pure rotation

    #velocity/position prediction
    corrected_accel = [corrected_accel_x, corrected_accel_y, corrected_accel_z]
    accel_world = rotate_by_quat(q, corrected_accel) #put in world frame
    accel_world = accel_world - [0, 0, GRAVITY] #subtract gravity
    velocity += accel_world * dt #integrate velcoity
    position += velocity * dt #integrate position

    # Update the state transition matrix based on the current state and time step
    w = np.array([corrected_gyro_x, corrected_gyro_y, corrected_gyro_z])
    F = update_F(w, dt, F, q, corrected_accel) #describes how error state vector evolves. 
    P = F @ P @ F.T + Q #update covariance error matrix, uncertainty increases because we are just integrating the model

    #gated bc gps sample rate is lower than accel/mag/gyro
    if now - last_gps_time >= gps_period:
        #get gps data
        north, east, up, vel_north, vel_east, vel_up = get_gps()
        gps_measurement = np.array([north, east, up, vel_north, vel_east, vel_up]) #current GPS position + velocity, compared against predicted position/velocity in the gps correction step
        
        #gps correction
        H_gps = update_H_gps(H_gps) #builds measurment jacobian for this update
        S_gps = H_gps @ P @ H_gps.T + R_gps #how much uncertainty you'd expect in residual, combining current uncertainty with sensor noise R_gps
        K_gps = P @ H_gps.T @ np.linalg.inv(S_gps) #computing kalman fain
        predicted_gps = np.concatenate([position, velocity]) #what you expect gps to report
        residual_gps = gps_measurement - predicted_gps #what gps reported vs what you expected
        #gate against a bad fix (multipath, momentary bad geometry) - R_gps isn't
        #adaptively inflated like R_accel, so no separate "base" R is needed here
        d_squared_gps = residual_gps.T @ np.linalg.inv(S_gps) @ residual_gps

        #to catch unreasonable measurements and not let them corrupt the state. 
        if d_squared_gps <= chi2_threshold_gps:
            error_state = error_state + K_gps @ residual_gps #apply the correction.
            #Joseph form - numerically robust to floating-point drift (keeps P symmetric/PSD),
            #vs the algebraically-equivalent but fragile (I-KH)@P
            P = (I - K_gps @ H_gps) @ P @ (I - K_gps @ H_gps).T + K_gps @ R_gps @ K_gps.T
            #this is the second p update, after we incorporate a measuremnt uncertainty goes down bc that is another measurement source
        #else: skip entirely - state and P stay exactly as the predict step left them
        last_gps_time = now

    #predicted accel/mag + error_state init
    #compute the expected measuremnets so you can compare them with what you actually got. 
    predicted_accel = rotate_by_quat(quat_conjugate(q), np.array([0.0, 0.0, GRAVITY]))
    predicted_mag = rotate_by_quat(quat_conjugate(q), np.array([1.0, 0.0, 0.0]))
    H_accel_mag = update_H_mag_accel(predicted_accel, predicted_mag, H_accel_mag)
    H_accel = H_accel_mag[0:3, :]

    #accel correction
    S_accel = H_accel @ P @ H_accel.T + R_accel
    K_accel = P @ H_accel.T @ np.linalg.inv(S_accel)
    residual_accel = np.array([corrected_accel_x, corrected_accel_y, corrected_accel_z]) - predicted_accel
    #gate against BASE noise, not the already-inflated adaptive R_accel - otherwise
    #adaptive R inflates in lockstep with the residual and the gate can never fire
    #(d_squared asymptotes to ~1/k for large outliers regardless of severity)
    S_accel_gate = H_accel @ P @ H_accel.T + R_accel_base
    d_squared = residual_accel.T @ np.linalg.inv(S_accel_gate) @ residual_accel

    if d_squared <= chi2_threshold:
      error_state = error_state + K_accel @ residual_accel
      #this is the second p update, after we incorporate a measuremnt uncertainty goes down bc that is another measurement source
      P = (I - K_accel @ H_accel) @ P @ (I - K_accel @ H_accel).T + K_accel @ R_accel @ K_accel.T
    #else: skip entirely - state and P stay exactly as the predict step left them

    #mag correction
    H_mag = H_accel_mag[3:]
    S_mag = H_mag @ P @ H_mag.T + R_mag
    K_mag = P @ H_mag.T @ np.linalg.inv(S_mag)
    residual_mag = np.array([mag_x, mag_y, mag_z]) - predicted_mag
    #gate against magnetic interference (motors/ESCs) - R_mag is static, not
    #adaptively inflated like R_accel, so no separate "base" R is needed here
    d_squared_mag = residual_mag.T @ np.linalg.inv(S_mag) @ residual_mag

    if d_squared_mag <= chi2_threshold:
        error_state = error_state + K_mag @ residual_mag
        #this is the second p update, after we incorporate a measuremnt uncertainty goes down bc that is another measurement source
        P = (I - K_mag @ H_mag) @ P @ (I - K_mag @ H_mag).T + K_mag @ R_mag @ K_mag.T
    #else: skip entirely - state and P stay exactly as the predict step left them

    #d meaning delta, small change
    #its a correction to the state
    d_theta = error_state[0:3]
    d_bias  = error_state[3:6]
    d_velocity = error_state[6:9]
    d_position = error_state[9:12]
    d_accel_bias = error_state[12:15]

    bias_x = bias_x + d_bias[0]
    bias_y = bias_y + d_bias[1]
    bias_z = bias_z + d_bias[2]
    accel_bias_x += d_accel_bias[0]
    accel_bias_y += d_accel_bias[1]
    accel_bias_z += d_accel_bias[2]
    velocity += d_velocity
    position += d_position

    #the multiplicative (MEKF) bit, composing the corrections by multiplying them 
    dq = np.array([1.0, d_theta[0]/2, d_theta[1]/2, d_theta[2]/2])
    q = quat_mult(q, dq)
    q = q / np.linalg.norm(q)

    print("q: ", q, "velocity: ", velocity, "position: ", position, "bias: ", [bias_x, bias_y, bias_z])

    #loop timing
    last_time = now
    sim_time += dt   #advances the ground truth by the same dt the filter just integrated with
    time.sleep(0.01) #sleep for 10ms to simulate sensor reading rate

