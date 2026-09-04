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
weight = 0.98 #weight for complementary filter, how much to trust gyro vs accel/mag

#measurement variables
gyro_x_dps = 5 #raw gyro constants, in deg/s - never overwritten by the loop
gyro_y_dps = 0
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

#dynamically update weighting
state_covariance = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) #how uncertain you currently are about each state, and how uncertainties are correlated
process_noise = np.array([[0.01, 0, 0], [0, 0.01, 0], [0, 0, 0.01]]) #how much new uncertainty is added by the prediction step (how uncertain you are abt gyro)
measurement_noise = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]]) #how much uncertainty is added by the measurement step (how uncertain you are abt accel/mag)
kalman_gain = np.zeros((3, 3)) #how much to trust the measurement vs the prediction

last_time = time.time()

#sub functions to find variables for loop
def get_gyro():
    #raw values are in deg/s, convert to rad/s before returning

    return math.radians(gyro_x_dps), math.radians(gyro_y_dps), math.radians(gyro_z_dps)

def get_accel():

    return accel_x, accel_y, accel_z

def get_mag():
    return mag_x, mag_y, mag_z

#main loop
while True:

    #read gyro, mag, accel
    gyro_x, gyro_y, gyro_z = get_gyro()
    mag_x, mag_y, mag_z = get_mag()
    accel_x, accel_y, accel_z = get_accel()

    mag_x_compensated = mag_x * math.cos(pitch) + mag_y * math.sin(roll) * math.sin(pitch) + mag_z * math.cos(roll) * math.sin(pitch)
    mag_y_compensated = mag_y * math.cos(roll) - mag_z * math.sin(roll)

    #predict: integrate gyro into current angle estimation
    now = time.time()
    dt = now - last_time #sampling as fast as the hardware can handle

    #youre just adding how fastyou've moved multiplied by how long you moved it
    # so if you went 3 deg/s clockwise for 1 sec then youre adding 3 degrees to current angle 
    # since you do this many many times for a tiny timestep, that is technically integration 
    #it needs to many tiny timesteps bc the drone changes speed many times so each tiny step is
    #roughly accurate only bc the rate is constant in that tiny of a window. 
    roll += gyro_x * dt
    pitch += gyro_y * dt
    yaw += gyro_z * dt

    #compute accel based roll/pitch and mag based yaw independently
    accel_roll = math.atan2(accel_y, accel_z)
    accel_pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))
    accel_yaw = math.atan2(mag_y_compensated, mag_x_compensated)

    #blend prediction with those (complementary filter) 
    #later this is kalman gain update step
    roll = (weight) * roll + (1 - weight) * accel_roll
    pitch = (weight) * pitch + (1 - weight) * accel_pitch
    yaw = (weight) * yaw + (1 - weight) * accel_yaw

    print("roll: ", math.degrees(roll), "pitch: ", math.degrees(pitch), "yaw: ", math.degrees(yaw))

    #store result as new "current angle estimate" for next loop

    last_time = now
    time.sleep(0.01) #sleep for 10ms to simulate sensor reading rate
