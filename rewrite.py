import time
import math
import numpy as np
import calibration

#variable definition
k = 1 #how aggressively i distrust accel when drone is accelerating. will be tuned
chi2_threshold = 11.34 #threshold for 99% confidence, accel and mag
chi2_threshold_gps = 16.81 #threshold for 99% confidene, gps, 6 dof
GRAVITY = 9.80665 #constant

#measrement variables
accel_x = 0
accel_y = GRAVITY * math.sin(math.radians(10))
accel_z = GRAVITY * math.cos(math.radians(10))
accel_bias_x = 0
accel_bias_y = 0
accel_bias_z = 0
mag_x = 1.0
mag_y = 0
mag_z = 0
gyro_x = 0
gyro_y = 0
gyro_z = 0
gyro_bias_x = 0
gyro_bias_y = 0
gyro_bias_z = 0
gps_north = 0.0
gps_east = 0.0
gps_up = 0.0
gps_vel_north = 0.0
gps_vel_east = 0.0
gps_vel_up = 0.0
velocity = np.array([0.0, 0.0, 0.0])
position = np.array([0.0, 0.0, 0.0])
q = np.array([1.0, 0.0, 0.0, 0.0]) #identity quaternion

#filter matrices
P = np.eye(15) #how uncertain you currently are about each state, and how uncertainties are correlated
Q = np.diag([0.01, 0.01, 0.01, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6]) #how much new uncertainty is added by the prediction step (how uncertain you are abt gyro)
R_accel_base = np.diag([0.1, 0.1, 0.1]) #how much uncertainty is added by the measurement step (how uncertain you are abt accel/mag)
R_mag = np.diag([0.1, 0.1, 0.1]) #how much uncertainty is added by the measurement step (how uncertain you are abt accel/mag)
F = np.eye(15) #state transition matrix, how the state evolves from one step to the next without control input (identity for this case)
I = np.eye(15) #identity matrix for updating the covariance
H_accel_mag = np.zeros((6, 15)) #measurement matrix, how the measurements relate to the state
H_gps = np.zeros((6, 15)) #measurement matrix for GPS, how the measurements relate to the state (position rows 0:3, velocity rows 3:6)
R_gps = np.diag([1.0, 1.0, 1.0, 0.5, 0.5, 0.5]) #how much uncertainty is added by the GPS measurement step (placeholder - needs tuning). Position and velocity blocks kept separately tunable - GNSS velocity (from Doppler) is often more accurate than position, but noisier at low speed

#timing variables
last_gps_time = time.time()
gps_period = 0.2
last_time = time.time()

#math helpers
def rotate_by_quat(q, v):
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

def quat_to_R(q):
    return np.array([rotate_by_quat(q, e) for e in np.eye(3)]).T

#pre-flight calibration - seeds q/gyro_bias/accel_bias instead of starting from
#identity/zero (see calibration.py: accel_bias and attitude tilt are otherwise
#unobservable from a single fixed orientation, which is exactly what the accel_bias
#column in update_H_mag_accel below needs at least a decent starting point for).
#NOTE: get_gyro/get_accel/get_mag here just return this file's constant "in-flight"
#sensor stubs (gyro_x_dps etc. never stop, there's no separate stationary phase
#simulated yet) - so this wiring doesn't get the wiggle's disambiguation benefit
#until these sensor functions (or real hardware) actually go through a stationary
#dwell + wiggle before the main loop starts.

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

while True:

    #read sensors
    gyro_x, gyro_y, gyro_z = get_gyro()
    mag_x, mag_y, mag_z = get_mag()
    accel_x, accel_y, accel_z = get_accel()

    accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
    deviation = abs(accel_magnitude - GRAVITY) #how far off from 1g is the accel reading?
    R_accel = R_accel_base * (1 + k * deviation ** 2) #increase accel measurement noise if drone is accelerating

    corrected_gyro_x = gyro_x - gyro_bias_x
    corrected_gyro_y = gyro_y - gyro_bias_y
    corrected_gyro_z = gyro_z - gyro_bias_z
    corrected_accel_x = accel_x - accel_bias_x
    corrected_accel_y = accel_y - accel_bias_y
    corrected_accel_z = accel_z - accel_bias_z

    now = time.time()
    dt = now - last_time
    error_state = np.zeros() #reset each iteration

    w_quat = [0, corrected_gyro_x, corrected_gyro_y, corrected_gyro_z]
    q_dot = 0.5 * quat_mult(q, w_quat)
    q = q + q_dot * dt
    q = q / np.linalg.norm(q) 

    corrected_accel = [corrected_accel_x, corrected_accel_y, corrected_accel_z]
    accel_world = rotate_by_quat(q, corrected_accel)
    accel_world = accel_world - [0, 0, GRAVITY] #subtract gravity
    velocity += accel_world * dt
    position += velocity * dt