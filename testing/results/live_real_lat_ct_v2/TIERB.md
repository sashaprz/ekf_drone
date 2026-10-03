# Tier B - closed loop on the EKF

code `6074c75` (dirty) - tag `real_lat_ct_v2`, env `SIM_REALISTIC=1 SIM_GPS_LATENCY_MS=150 CAL_TURN=1`

| mission | survived | track_h rms (EKF) | track_h rms (ORACLE) | track_h max (EKF) | est tilt rms | est tilt max | est yaw rms | est yaw mean | est roll/pitch mean | est pos_h rms |
|---|---|---|---|---|---|---|---|---|---|---|
| circle_fast | 3/3 | 1.92 [1.83..1.97] | 4.63 | 3.75 [3.67..3.81] | 0.60 [0.59..0.61] | 1.84 [1.68..1.96] | 1.04 [0.99..1.07] | -0.0 [-0.0..0.0] | 0.00 [-0.01..0.01] / -0.27 [-0.29..-0.26] | 0.95 [0.85..1.02] |

Values: mean [min..max] over trials. est_* = the live EKF estimate vs Gazebo truth over the airborne window.
track_h = true horizontal position vs the mission's ideal path during the profile.
