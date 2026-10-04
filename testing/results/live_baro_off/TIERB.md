# Tier B - closed loop on the EKF

code `2ea4f28` (dirty) - tag `baro_off`, env `SIM_REALISTIC=1 EKF_USE_BARO=0`

| mission | survived | track_h rms (EKF) | track_h rms (ORACLE) | track_h max (EKF) | est tilt rms | est tilt max | est yaw rms | est yaw mean | est roll/pitch mean | est pos_h rms | track_v rms | est pos_v rms / max |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| hover | 3/3 | 0.57 [0.53..0.63] | 0.00 | 1.02 [0.96..1.09] | 0.43 [0.42..0.43] | 0.99 [0.95..1.05] | 0.92 [0.83..1.02] | 0.7 [0.5..0.8] | 0.01 [0.01..0.01] / -0.30 [-0.30..-0.29] | 0.70 [0.66..0.75] | 0.55 [0.46..0.60] | 0.48 [0.40..0.53] / 0.92 [0.80..1.09] |
| box | 3/3 | 1.21 [1.20..1.23] | 0.93 | 2.18 [2.17..2.19] | 0.43 [0.42..0.44] | 1.01 [0.98..1.03] | 0.85 [0.78..0.94] | 0.5 [0.5..0.7] | 0.00 [-0.00..0.01] / -0.30 [-0.30..-0.29] | 0.70 [0.67..0.72] | 0.53 [0.52..0.54] | 0.46 [0.46..0.48] / 0.83 [0.74..0.97] |
| takeoff_land | 3/3 | 0.46 [0.30..0.61] | 0.00 | 0.86 [0.65..1.08] | 0.44 [0.43..0.44] | 1.20 [1.08..1.37] | 0.99 [0.93..1.05] | 0.7 [0.7..0.8] | 0.01 [0.00..0.01] / -0.29 [-0.29..-0.28] | 0.56 [0.44..0.67] | 0.61 [0.52..0.67] | 0.56 [0.50..0.64] / 1.17 [1.09..1.30] |

Values: mean [min..max] over trials. est_* = the live EKF estimate vs Gazebo truth over the airborne window.
track_h = true horizontal position vs the mission's ideal path during the profile.
