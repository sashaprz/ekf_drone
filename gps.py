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

#variable definition
k = 1 #how agressively to increase accel measurement noise when drone is accelerating.
chi2_threshold = 11.34 #chi2 threshold for 3 DOF, 99% confidence interval

#measurement variables
gyro_x_dps = 5 #raw gyro constants, in deg/s - never overwritten by the loop
gyro_y_dps = 3 #nonzero, to exercise coupling into pitch_dot and (via roll) into yaw_dot
gyro_z_dps = 0
accel_x = 0
accel_y = math.sin(math.radians(10)) #simulate 10 deg roll
accel_z = math.cos(math.radians(10)) #simulate 10 deg roll
mag_x = 1.0
mag_y = 0
mag_z = 0
gps_north = 0.0 #raw GPS constants, in the same local-tangent-plane meters frame as position - never overwritten by the loop
gps_east = 0.0
gps_up = 0.0
gps_vel_north = 0.0 #raw GPS velocity constants (from Doppler, same units as velocity state) - never overwritten by the loop
gps_vel_east = 0.0
gps_vel_up = 0.0

#state variables
q = np.array([1.0, 0.0, 0.0, 0.0]) #identity quaternion, [w, x, y, z]
bias_x = 0
bias_y = 0
bias_z = 0
velocity = np.array([0.0, 0.0, 0.0]) # Initialize velocity
position = np.array([0.0, 0.0, 0.0])

#filter matrices
P = np.eye(12) #how uncertain you currently are about each state, and how uncertainties are correlated
Q = np.diag([0.01, 0.01, 0.01, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6]) #how much new uncertainty is added by the prediction step (how uncertain you are abt gyro)
R_accel_base = np.diag([0.1, 0.1, 0.1]) #how much uncertainty is added by the measurement step (how uncertain you are abt accel/mag)
R_mag = np.diag([0.1, 0.1, 0.1]) #how much uncertainty is added by the measurement step (how uncertain you are abt accel/mag)
F = np.eye(12) #state transition matrix, how the state evolves from one step to the next without control input (identity for this case)
I = np.eye(12) #identity matrix for updating the covariance
H_accel_mag = np.zeros((6, 12)) #measurement matrix, how the measurements relate to the state
H_gps = np.zeros((6, 12)) #measurement matrix for GPS, how the measurements relate to the state (position rows 0:3, velocity rows 3:6)
R_gps = np.diag([1.0, 1.0, 1.0, 0.5, 0.5, 0.5]) #how much uncertainty is added by the GPS measurement step (placeholder - needs tuning). Position and velocity blocks kept separately tunable - GNSS velocity (from Doppler) is often more accurate than position, but noisier at low speed

#timing variables
last_time = time.time()
gps_period = 0.2
last_gps_time = time.time()

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

#sensor reading functions (simulated for now)
def get_gyro():
    #raw values are in deg/s, convert to rad/s before returning

    return math.radians(gyro_x_dps), math.radians(gyro_y_dps), math.radians(gyro_z_dps)

def get_accel():

    return accel_x, accel_y, accel_z

def get_mag():
    return mag_x, mag_y, mag_z

def get_gps():
    return gps_north, gps_east, gps_up, gps_vel_north, gps_vel_east, gps_vel_up

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
    return F

def update_H_mag_accel(predicted_accel, predicted_mag, H):
    #accel rows: jacobian of predicted_accel wrt state
    H[0:3, 0:3] = skew(predicted_accel)
    H[0:3, 3:6] = 0
    H[3:6, 0:3] = skew(predicted_mag)
    H[3:6, 3:6] = 0
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

    #adaptive accel noise + bias-corrected gyro
    accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
    deviation = abs(accel_magnitude - 1.0) #how far off from 1g is the accel reading?
    R_accel = R_accel_base * (1 + k * deviation ** 2) #increase accel measurement noise if drone is accelerating
    corrected_gyro_x = gyro_x - bias_x
    corrected_gyro_y = gyro_y - bias_y
    corrected_gyro_z = gyro_z - bias_z

    #predict: integrate gyro into current angle estimation
    now = time.time()
    dt = now - last_time #sampling as fast as the hardware can handle

    error_state = np.zeros(12)  # Initialize error state vector

    w_quat = [0, corrected_gyro_x, corrected_gyro_y, corrected_gyro_z]
    q_dot = 0.5 * quat_mult(q, w_quat)
    q = q + q_dot * dt
    q = q / np.linalg.norm(q)          # renormalize — new step

    #velocity/position prediction
    accel = np.array([accel_x, accel_y, accel_z])
    accel_world = rotate_by_quat(q, accel)
    accel_world = accel_world - [0, 0, 1] #subtract gravity
    velocity += accel_world * dt
    position += velocity * dt

    # Update the state transition matrix based on the current state and time step
    w = np.array([corrected_gyro_x, corrected_gyro_y, corrected_gyro_z])
    F = update_F(w, dt, F, q, accel)
    P = F @ P @ F.T + Q

    if now - last_gps_time >= gps_period:
        #get gps data
        north, east, up, vel_north, vel_east, vel_up = get_gps()
        gps_measurement = np.array([north, east, up, vel_north, vel_east, vel_up]) #current GPS position + velocity, compared against predicted position/velocity in the gps correction step
        
        #gps correction
        H_gps = update_H_gps(H_gps)
        S_gps = H_gps @ P @ H_gps.T + R_gps
        K_gps = P @ H_gps.T @ np.linalg.inv(S_gps)
        predicted_gps = np.concatenate([position, velocity])
        residual_gps = gps_measurement - predicted_gps
        error_state = error_state + K_gps @ residual_gps
        P = (I - K_gps @ H_gps) @ P
        last_gps_time = now

    #predicted accel/mag + error_state init
    predicted_accel = rotate_by_quat(quat_conjugate(q), np.array([0.0, 0.0, 1.0]))
    predicted_mag = rotate_by_quat(quat_conjugate(q), np.array([1.0, 0.0, 0.0]))
    H_accel_mag = update_H_mag_accel(predicted_accel, predicted_mag, H_accel_mag)
    H_accel = H_accel_mag[0:3, :]

    #accel correction
    S_accel = H_accel @ P @ H_accel.T + R_accel
    K_accel = P @ H_accel.T @ np.linalg.inv(S_accel)
    residual_accel = np.array([accel_x, accel_y, accel_z]) - predicted_accel
    #gate against BASE noise, not the already-inflated adaptive R_accel - otherwise
    #adaptive R inflates in lockstep with the residual and the gate can never fire
    #(d_squared asymptotes to ~1/k for large outliers regardless of severity)
    S_accel_gate = H_accel @ P @ H_accel.T + R_accel_base
    d_squared = residual_accel.T @ np.linalg.inv(S_accel_gate) @ residual_accel

    if d_squared <= chi2_threshold:
      error_state = error_state + K_accel @ residual_accel
      P = (I - K_accel @ H_accel) @ P
    #else: skip entirely - state and P stay exactly as the predict step left them

    #mag correction
    H_mag = H_accel_mag[3:]
    K_mag = P @ H_mag.T @ np.linalg.inv(H_mag @ P @ H_mag.T + R_mag)
    residual_mag = np.array([mag_x, mag_y, mag_z]) - predicted_mag
    error_state = error_state + K_mag @ residual_mag
    P = (I - K_mag @ H_mag) @ P

    d_theta = error_state[0:3]
    d_bias  = error_state[3:6]
    d_velocity = error_state[6:9]
    d_position = error_state[9:12]

    bias_x = bias_x + d_bias[0]
    bias_y = bias_y + d_bias[1]
    bias_z = bias_z + d_bias[2]

    dq = np.array([1.0, d_theta[0]/2, d_theta[1]/2, d_theta[2]/2])
    q = quat_mult(q, dq)
    q = q / np.linalg.norm(q)

    velocity += d_velocity
    position += d_position

    print("q: ", q, "velocity: ", velocity, "position: ", position, "bias: ", [bias_x, bias_y, bias_z])

    #loop timing
    last_time = now
    time.sleep(0.01) #sleep for 10ms to simulate sensor reading rate

