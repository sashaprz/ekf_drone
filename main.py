"""
  Your SoC EKF, generalized:
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

#variable definition
weight = 0.98 #weight for complementary filter, how much to trust gyro vs accel/mag

gyro = 0
accel = 0
mag = 0

roll = 0
pitch = 0
yaw = 0

last_time = time.time()


#sub functions to find variables for loop
def get_gyro():

    return gyro_x, gyro_y, gyro_z

def get_accel():

def get_mag():
    s

#main loop
while True:

    #read gyro, mag, accel
    gyro_x, gyro_y, gyro_z = get_gyro()
    mag_cur = get_mag()
    accel_cur = get_accel()

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

    #blend prediction with those (complementary filter) 
    #later this is kalman gain update step
    roll = (weight) * roll + (1 - weight) * accel_roll

    #store result as new "current angle estimate" for next loop

    last_time = now
