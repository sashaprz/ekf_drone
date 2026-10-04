# Tier B - closed loop on the EKF

code `2ea4f28` (dirty) - tag `fs_fp_circle`, env `SIM_REALISTIC=1 SIM_GPS_LATENCY_MS=150 CAL_TURN=1`

| mission | survived | track_h rms (EKF) | track_h rms (ORACLE) | track_h max (EKF) | est tilt rms | est tilt max | est yaw rms | est yaw mean | est roll/pitch mean | est pos_h rms | track_v rms | est pos_v rms / max |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| circle_fast | 1/1 | 1.90 [1.90..1.90] | 4.63 | 3.50 [3.50..3.50] | 0.56 [0.56..0.56] | 1.83 [1.83..1.83] | 0.94 [0.94..0.94] | -0.0 [-0.0..-0.0] | -0.02 [-0.02..-0.02] / -0.28 [-0.28..-0.28] | 0.97 [0.97..0.97] | 0.23 [0.23..0.23] | 0.23 [0.23..0.23] / 0.49 [0.49..0.49] |

Values: mean [min..max] over trials. est_* = the live EKF estimate vs Gazebo truth over the airborne window.
track_h = true horizontal position vs the mission's ideal path during the profile.
