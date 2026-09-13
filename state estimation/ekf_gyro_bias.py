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

import math
import numpy as np

#variable definition

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

#state variables
roll = 0
pitch = 0
yaw = 0
bias_x = 0
bias_y = 0
bias_z = 0

#oscillation constants
amplitude_roll = math.radians(15)  # how far the oscillation swings,in radians
freq_roll = 0.5                    # how fast it oscillates, in rad/s
phase_roll = 0                     # phase offset for roll oscillation

amplitude_pitch = math.radians(10) # how far the oscillation swings, in radians
freq_pitch = 0.3                   # how fast it oscillates, in rad/s
phase_pitch = math.pi/2            # phase offset for pitch oscillation

amplitude_yaw = math.radians(5)    # how far the oscillation swings, in radians
freq_yaw = 0.1                     # how fast it oscillates, in rad/s
phase_yaw = math.pi                  # phase offset for yaw oscillation

#adding noise for sim
true_bias_x = math.radians(2)
true_bias_y = math.radians(-1)
true_bias_z = math.radians(0.5)
gyro_noise_std = math.radians(0.5)
accel_noise_std = 0.02
mag_noise_std = 0.01

#dynamically update weighting
P = np.eye(6) #how uncertain you currently are about each state, and how uncertainties are correlated
Q = np.diag([0.01, 0.01, 0.01, 1e-6, 1e-6, 1e-6]) #how much new uncertainty is added by the prediction step (how uncertain you are abt gyro)
R = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]) #how much uncertainty is added by the measurement step (how uncertain you are abt accel/mag)
K = np.zeros((6, 6)) #how much to trust the measurement vs the prediction, kalman gain

F = np.eye(6) #state transition matrix, how the state evolves from one step to the next without control input (identity for this case)
I = np.eye(6) #identity matrix for updating the covariance
H = np.zeros((6, 6)) #measurement matrix, how the measurements relate to the state

dt = 0.01 #fixed simulated timestep - deterministic, not wall-clock, so truth and filter can't desync
t = 0.0   #simulated elapsed time, advanced by exactly dt each iteration

#sub functions to find variables for loop
def get_gyro(t):
    simulated_gyro_x = true_roll_dot(t) - true_yaw_dot(t) * math.sin(true_pitch(t))
    simulated_gyro_y = true_pitch_dot(t) * math.cos(true_roll(t)) + true_yaw_dot(t) * math.sin(true_roll(t)) * math.cos(true_pitch(t))
    simulated_gyro_z = -true_pitch_dot(t) * math.sin(true_roll(t)) + true_yaw_dot(t) * math.cos(true_roll(t)) * math.cos(true_pitch(t))
    return (simulated_gyro_x + true_bias_x + np.random.normal(0, gyro_noise_std),
            simulated_gyro_y + true_bias_y + np.random.normal(0, gyro_noise_std),
            simulated_gyro_z + true_bias_z + np.random.normal(0, gyro_noise_std))

def get_accel(t):
    true_ax = -math.sin(true_pitch(t))
    true_ay = math.sin(true_roll(t)) * math.cos(true_pitch(t))
    true_az = math.cos(true_roll(t)) * math.cos(true_pitch(t))
    return (true_ax + np.random.normal(0, accel_noise_std),
              true_ay + np.random.normal(0, accel_noise_std),
              true_az + np.random.normal(0, accel_noise_std))

def get_mag(t):
    #matches the filter's own predicted_mag convention directly now that the
    #atan2/compensation path is gone - no sign flip needed (see chat for why)
    true_mx = math.cos(true_pitch(t)) * math.cos(true_yaw(t))
    true_my = math.sin(true_roll(t)) * math.sin(true_pitch(t)) * math.cos(true_yaw(t)) - math.cos(true_roll(t)) * math.sin(true_yaw(t))
    true_mz = math.cos(true_roll(t)) * math.sin(true_pitch(t)) * math.cos(true_yaw(t)) + math.sin(true_roll(t)) * math.sin(true_yaw(t))
    return (true_mx + np.random.normal(0, mag_noise_std),
            true_my + np.random.normal(0, mag_noise_std),
            true_mz + np.random.normal(0, mag_noise_std))

def update_F(pitch_dot, pitch, yaw_dot, dt, roll, F):
    #F is a matrix of partial derivatives - a Jacobian
    #jacobian needs to be recomputed live if function it's derived from is non-linear
    #since the process model has roll dot (with sin/cos), its now nonlinear
    F[0][0] = 1 + dt * np.tan(pitch) * pitch_dot
    F[0][1] = dt * yaw_dot / np.cos(pitch)
    F[0][2] = 0
    F[1][0] = -dt * yaw_dot * np.cos(pitch)
    F[1][1] = 1
    F[1][2] = 0
    F[2][0] = dt * pitch_dot / np.cos(pitch)
    F[2][1] = dt * yaw_dot * np.tan(pitch)
    F[2][2] = 1
    F[0][3] = -dt
    F[0][4] = -dt * np.sin(roll) * np.tan(pitch)
    F[0][5] = -dt * np.cos(roll) * np.tan(pitch)
    F[1][3] = 0 
    F[1][4] = -dt * np.cos(roll)
    F[1][5] = dt * np.sin(roll)
    F[2][3] = 0
    F[2][4] = -dt * np.sin(roll) / np.cos(pitch)
    F[2][5] = -dt * np.cos(roll) / np.cos(pitch)
    return F

def update_H(roll, pitch, yaw):
    #accel rows: jacobian of predicted_accel wrt state
    H[0] = [0, -math.cos(pitch), 0, 0, 0, 0]
    H[1] = [math.cos(roll)*math.cos(pitch), -math.sin(roll)*math.sin(pitch), 0, 0, 0, 0]
    H[2] = [-math.sin(roll)*math.cos(pitch), -math.cos(roll)*math.sin(pitch), 0, 0, 0, 0]

    #mag rows: jacobian of predicted_mag wrt state
    my = math.sin(roll)*math.sin(pitch)*math.cos(yaw) - math.cos(roll)*math.sin(yaw)
    mz = math.cos(roll)*math.sin(pitch)*math.cos(yaw) + math.sin(roll)*math.sin(yaw)
    H[3] = [0, -math.sin(pitch)*math.cos(yaw), -math.cos(pitch)*math.sin(yaw), 0, 0, 0]
    H[4] = [mz, math.sin(roll)*math.cos(pitch)*math.cos(yaw), -math.sin(roll)*math.sin(pitch)*math.sin(yaw) - math.cos(roll)*math.cos(yaw), 0, 0, 0]
    H[5] = [-my, math.cos(roll)*math.cos(pitch)*math.cos(yaw), -math.cos(roll)*math.sin(pitch)*math.sin(yaw) + math.sin(roll)*math.cos(yaw), 0, 0, 0]
    return H

def true_roll(t):
    return amplitude_roll * math.sin(freq_roll * t)

def true_roll_dot(t):
    return amplitude_roll * freq_roll * math.cos(freq_roll * t)

def true_pitch(t):
    return amplitude_pitch * math.sin(freq_pitch * t + phase_pitch)

def true_pitch_dot(t):
    return amplitude_pitch * freq_pitch * math.cos(freq_pitch * t + phase_pitch)

def true_yaw(t):
    return amplitude_yaw * math.sin(freq_yaw * t + phase_yaw)

def true_yaw_dot(t):
    return amplitude_yaw * freq_yaw * math.cos(freq_yaw * t + phase_yaw)

#main loop
while True:

    #read gyro, mag, accel
    gyro_x, gyro_y, gyro_z = get_gyro(t)
    mag_x, mag_y, mag_z = get_mag(t)
    accel_x, accel_y, accel_z = get_accel(t)

    corrected_gyro_x = gyro_x - bias_x
    corrected_gyro_y = gyro_y - bias_y
    corrected_gyro_z = gyro_z - bias_z

    #predict: integrate gyro into current angle estimation
    #dt is fixed (set above) - no wall-clock read here anymore

    #youre just adding how fastyou've moved multiplied by how long you moved it
    # so if you went 3 deg/s clockwise for 1 sec then youre adding 3 degrees to current angle 
    # since you do this many many times for a tiny timestep, that is technically integration 
    #it needs to many tiny timesteps bc the drone changes speed many times so each tiny step is
    #roughly accurate only bc the rate is constant in that tiny of a window. 
    #need roll dot because roll is not just gyro_x, it is also affected by gyro_y and gyro_z when pitch is not zero. 
    # the gyrp readings are measured around the dron's body axes. pitch and yaw though, are defined relative to the ground. 
    # a rotation rate the gyro reports around its own axes dont match how fast pitch/yaw is changing in the real world. 
    # this is because roll pitch and yaw are human bookkeeping. they are defined in the world, and once the drone isn't 
    #level, the axes no longer line up. the dot equations incprporate the fact that the body and world axes don't line up
    #old code assumed roll += gyro_x * dt which is only correct when body frame = world frame, when drone is level
    roll_dot = corrected_gyro_x + corrected_gyro_y * math.sin(roll) * math.tan(pitch) + corrected_gyro_z * math.cos(roll) * math.tan(pitch)
    pitch_dot = corrected_gyro_y * math.cos(roll) - corrected_gyro_z * math.sin(roll)
    yaw_dot = (corrected_gyro_y * math.sin(roll) + corrected_gyro_z * math.cos(roll)) / math.cos(pitch)

    # Update the state transition matrix based on the current state and time step
    F = update_F(pitch_dot, pitch, yaw_dot, dt, roll, F)
    H = update_H(roll, pitch, yaw)

    roll += roll_dot * dt
    pitch += pitch_dot * dt
    yaw += yaw_dot * dt

    predicted_accel = np.array([-math.sin(pitch), math.sin(roll)*math.cos(pitch), math.cos(roll)*math.cos(pitch)])
    predicted_mag = np.array([math.cos(pitch)*math.cos(yaw),
                               math.sin(roll)*math.sin(pitch)*math.cos(yaw) - math.cos(roll)*math.sin(yaw),
                               math.cos(roll)*math.sin(pitch)*math.cos(yaw) + math.sin(roll)*math.sin(yaw)])

    #predict covariance, then compute gain (canonical order: predict state/cov -> gain -> correct state -> correct cov)
    P = F @ P @ F.T + Q
    K = P @ H.T @ np.linalg.inv(H @ P @ H.T + R)

    measurement_vector = np.array([accel_x, accel_y, accel_z, mag_x, mag_y, mag_z])
    predicted_state = np.array([roll, pitch, yaw, bias_x, bias_y, bias_z])
    predicted_measurement = np.concatenate([predicted_accel, predicted_mag])
    residual =  measurement_vector - predicted_measurement

    corrected_state = predicted_state + K @ residual
    P = (I - K @ H) @ P

    roll, pitch, yaw, bias_x, bias_y, bias_z = corrected_state
    
    roll_error = math.degrees(roll) - math.degrees(true_roll(t))
    pitch_error = math.degrees(pitch) - math.degrees(true_pitch(t))
    yaw_error = math.degrees(yaw) - math.degrees(true_yaw(t))

    print("roll: ", math.degrees(roll), "(true: ", math.degrees(true_roll(t)), " err: ", roll_error, ")",
          "pitch: ", math.degrees(pitch), "(true: ", math.degrees(true_pitch(t)), " err: ", pitch_error, ")",
          "yaw: ", math.degrees(yaw), "(true: ", math.degrees(true_yaw(t)), " err: ", yaw_error, ")")

    t += dt #advance simulated time by exactly one fixed step
