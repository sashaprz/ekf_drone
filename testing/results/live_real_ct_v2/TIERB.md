# Tier B - closed loop on the EKF

code `6074c75` (dirty) - tag `real_ct_v2`, env `SIM_REALISTIC=1 CAL_TURN=1`

| mission | survived | track_h rms (EKF) | track_h rms (ORACLE) | track_h max (EKF) | est tilt rms | est tilt max | est yaw rms | est yaw mean | est roll/pitch mean | est pos_h rms |
|---|---|---|---|---|---|---|---|---|---|---|
| stops | 3/3 | 2.86 [2.76..2.99] | 3.34 | 5.30 [5.01..5.64] | 0.39 [0.39..0.39] | 1.30 [1.20..1.40] | 0.70 [0.69..0.72] | 0.1 [0.0..0.1] | -0.01 [-0.01..-0.00] / -0.22 [-0.22..-0.21] | 0.71 [0.58..0.85] |

Values: mean [min..max] over trials. est_* = the live EKF estimate vs Gazebo truth over the airborne window.
track_h = true horizontal position vs the mission's ideal path during the profile.
