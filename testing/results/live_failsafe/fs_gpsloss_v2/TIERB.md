# Tier B - closed loop on the EKF

code `2ea4f28` (dirty) - tag `fs_gpsloss_v2`, env `SIM_GPS_LOSS_AT=20`

| mission | survived | track_h rms (EKF) | track_h rms (ORACLE) | track_h max (EKF) | est tilt rms | est tilt max | est yaw rms | est yaw mean | est roll/pitch mean | est pos_h rms | track_v rms | est pos_v rms / max |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| hover | 2/2 | 0.01 [0.01..0.02] | 0.00 | 0.02 [0.02..0.02] | 0.46 [0.39..0.53] | 3.17 [2.49..3.85] | 0.91 [0.77..1.05] | -0.0 [-0.1..0.1] | 0.00 [-0.06..0.06] / 0.00 [0.00..0.00] | 1.79 [1.27..2.32] | 1.18 [1.17..1.18] | 0.24 [0.17..0.30] / 0.40 [0.38..0.41] |

Values: mean [min..max] over trials. est_* = the live EKF estimate vs Gazebo truth over the airborne window.
track_h = true horizontal position vs the mission's ideal path during the profile.
