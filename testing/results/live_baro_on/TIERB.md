# Tier B - closed loop on the EKF

code `2ea4f28` (dirty) - tag `baro_on`, env `SIM_REALISTIC=1`

| mission | survived | track_h rms (EKF) | track_h rms (ORACLE) | track_h max (EKF) | est tilt rms | est tilt max | est yaw rms | est yaw mean | est roll/pitch mean | est pos_h rms | track_v rms | est pos_v rms / max |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| hover | 2/3 | 0.54 [0.38..0.64] | 0.00 | 0.94 [0.74..1.04] | 0.42 [0.41..0.44] | 0.95 [0.78..1.08] | 0.93 [0.86..0.99] | 0.7 [0.6..0.8] | 0.00 [0.00..0.01] / -0.29 [-0.29..-0.29] | 0.62 [0.48..0.72] | 0.14 [0.08..0.22] | 0.13 [0.09..0.21] / 0.28 [0.21..0.38] |
| box | 3/3 | 1.16 [1.08..1.20] | 0.93 | 2.02 [1.91..2.09] | 0.42 [0.40..0.43] | 0.98 [0.97..1.01] | 0.74 [0.68..0.79] | 0.4 [0.3..0.5] | 0.01 [0.00..0.01] / -0.29 [-0.29..-0.29] | 0.66 [0.60..0.72] | 0.13 [0.10..0.18] | 0.13 [0.10..0.19] / 0.26 [0.22..0.33] |
| takeoff_land | 3/3 | 0.36 [0.29..0.41] | 0.00 | 0.72 [0.65..0.80] | 0.42 [0.41..0.43] | 0.90 [0.87..0.94] | 1.00 [0.94..1.06] | 0.8 [0.7..0.9] | 0.01 [0.01..0.01] / -0.29 [-0.29..-0.29] | 0.45 [0.43..0.51] | 0.35 [0.34..0.36] | 0.11 [0.08..0.16] / 0.27 [0.22..0.31] |

Values: mean [min..max] over trials. est_* = the live EKF estimate vs Gazebo truth over the airborne window.
track_h = true horizontal position vs the mission's ideal path during the profile.
