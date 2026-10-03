# Tier B - closed loop on the EKF

code `6074c75` (dirty) - tag `stall_lat_new`, env `SIM_REALISTIC=1 SIM_GPS_LATENCY_MS=150 SIM_LOOP_STALL=0.005,150 CAL_TURN=1`

| mission | survived | track_h rms (EKF) | track_h rms (ORACLE) | track_h max (EKF) | est tilt rms | est tilt max | est yaw rms | est yaw mean | est roll/pitch mean | est pos_h rms |
|---|---|---|---|---|---|---|---|---|---|---|
| circle_fast | 3/3 | 2.21 [2.05..2.34] | 4.63 | 3.64 [3.47..3.82] | 0.62 [0.60..0.63] | 1.98 [1.90..2.03] | 1.01 [0.97..1.07] | 0.0 [-0.0..0.0] | -0.01 [-0.02..-0.01] / -0.26 [-0.28..-0.25] | 0.82 [0.76..0.91] |

Values: mean [min..max] over trials. est_* = the live EKF estimate vs Gazebo truth over the airborne window.
track_h = true horizontal position vs the mission's ideal path during the profile.
