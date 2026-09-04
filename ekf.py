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

#guessing variables
accel_roll = 0
accel_pitch = 0
accel_yaw = 0

#state variables
roll = 0
pitch = 0
yaw = 0
bias_x = 0
bias_y = 0
bias_z = 0

#dynamically update weighting
P = np.eye(6) #how uncertain you currently are about each state, and how uncertainties are correlated
Q = np.diag([0.01, 0.01, 0.01, 1e-6, 1e-6, 1e-6]) #how much new uncertainty is added by the prediction step (how uncertain you are abt gyro)
R = np.diag([0.1, 0.1, 0.1, 0.1]) #how much uncertainty is added by the measurement step (how uncertain you are abt accel/mag)
K = np.zeros((6, 4)) #how much to trust the measurement vs the prediction, kalman gain

F = np.eye(6) #state transition matrix, how the state evolves from one step to the next without control input (identity for this case)
I = np.eye(6) #identity matrix for updating the covariance
H = np.zeros((4, 6)) #measurement matrix, how the measurements relate to the state

last_time = time.time()

#sub functions to find variables for loop
def get_gyro():
    #raw values are in deg/s, convert to rad/s before returning

    return math.radians(gyro_x_dps), math.radians(gyro_y_dps), math.radians(gyro_z_dps)

def get_accel():

    return accel_x, accel_y, accel_z

def get_mag():
    return mag_x, mag_y, mag_z

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

def update_H(roll, pitch):
    H[0] = [0, -math.cos(pitch), 0, 0, 0, 0]
    H[1] = [math.cos(roll)*math.cos(pitch), -math.sin(roll)*math.sin(pitch), 0, 0, 0, 0]
    H[2] = [-np.sin(roll)*math.cos(pitch), -math.cos(roll)*math.sin(pitch), 0, 0, 0, 0]
    H[3] = [0, 0, 1, 0, 0, 0]
    return H

#main loop
while True:

    #read gyro, mag, accel
    gyro_x, gyro_y, gyro_z = get_gyro()
    mag_x, mag_y, mag_z = get_mag()
    accel_x, accel_y, accel_z = get_accel()

    mag_x_compensated = mag_x * math.cos(pitch) + mag_y * math.sin(roll) * math.sin(pitch) + mag_z * math.cos(roll) * math.sin(pitch)
    mag_y_compensated = mag_y * math.cos(roll) - mag_z * math.sin(roll)

    corrected_gyro_x = gyro_x - bias_x
    corrected_gyro_y = gyro_y - bias_y  
    corrected_gyro_z = gyro_z - bias_z

    #predict: integrate gyro into current angle estimation
    now = time.time()
    dt = now - last_time #sampling as fast as the hardware can handle

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
    H = update_H(roll, pitch)

    roll += roll_dot * dt
    pitch += pitch_dot * dt
    yaw += yaw_dot * dt

    #compute accel based roll/pitch and mag based yaw independently
    accel_roll = math.atan2(accel_y, accel_z)
    accel_pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))
    accel_yaw = math.atan2(mag_y_compensated, mag_x_compensated)
    predicted_accel = np.array([-math.sin(pitch),math.sin(roll)*math.cos(pitch), math.cos(roll)*math.cos(pitch)])

    #update covariances
    P = F @ P @ F.T + Q
    K = P @ H.T @ np.linalg.inv(H @ P @ H.T + R)
    P = (I - K @ H) @ P

    measurement_vector = np.array([accel_x, accel_y, accel_z, accel_yaw])
    predicted_state = np.array([roll, pitch, yaw, bias_x, bias_y, bias_z])
    predicted_measurement = np.array([predicted_accel[0], predicted_accel[1], predicted_accel[2], yaw])
    residual =  measurement_vector - predicted_measurement

    corrected_state = predicted_state + K @ residual

    roll, pitch, yaw, bias_x, bias_y, bias_z = corrected_state
    
    print("roll: ", math.degrees(roll), "pitch: ", math.degrees(pitch), "yaw: ", math.degrees(yaw))

    last_time = now
    time.sleep(0.01) #sleep for 10ms to simulate sensor reading rate
