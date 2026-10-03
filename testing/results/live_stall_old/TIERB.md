# Tier B - closed loop on the EKF

code `6074c75` (dirty) - tag `stall_old`, env `SIM_REALISTIC=1 SIM_LOOP_STALL=0.005,150 EKF_IMU_TIMESTAMPS=0 CAL_TURN=1`

| mission | survived | track_h rms (EKF) | track_h rms (ORACLE) | track_h max (EKF) | est tilt rms | est tilt max | est yaw rms | est yaw mean | est roll/pitch mean | est pos_h rms |
|---|---|---|---|---|---|---|---|---|---|---|
| stops | 3/3 | 2.77 [2.71..2.81] | 3.34 | 5.02 [4.79..5.20] | 0.70 [0.55..0.88] | 2.48 [2.33..2.64] | 1.41 [1.07..1.80] | -0.1 [-0.3..0.0] | -0.09 [-0.16..0.04] / -0.08 [-0.14..-0.03] | 0.76 [0.67..0.84] |

Values: mean [min..max] over trials. est_* = the live EKF estimate vs Gazebo truth over the airborne window.
track_h = true horizontal position vs the mission's ideal path during the profile.
