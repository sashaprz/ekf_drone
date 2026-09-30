# Tier B - closed loop on the EKF

code `86193e6` (dirty)

| mission | survived | track_h rms (EKF) | track_h rms (ORACLE) | track_h max (EKF) | est tilt rms | est tilt max | est yaw rms | est yaw mean | est roll/pitch mean | est pos_h rms |
|---|---|---|---|---|---|---|---|---|---|---|
| hover | 3/3 | 0.25 [0.14..0.40] | 0.00 | 0.57 [0.38..0.94] | 0.64 [0.41..0.92] | 0.79 [0.53..0.95] | 5.75 [4.15..7.33] | -1.9 [-7.3..5.8] | 0.03 [-0.39..0.75] / 0.07 [-0.53..0.45] | 0.04 [0.02..0.07] |
| box | 3/3 | 0.95 [0.93..0.96] | 0.93 | 1.40 [1.38..1.42] | 0.61 [0.09..0.93] | 0.66 [0.14..0.96] | 1.26 [0.19..1.88] | -0.0 [-1.7..1.9] | -0.15 [-0.37..0.01] / 0.53 [-0.09..0.92] | 0.01 [0.01..0.01] |
| circle_slow | 3/3 | 0.54 [0.39..0.80] | 0.48 | 0.83 [0.55..1.23] | 0.86 [0.46..1.24] | 1.07 [0.70..1.32] | 4.77 [1.74..6.33] | -0.6 [-6.3..6.2] | -0.10 [-0.48..0.36] / -0.78 [-1.14..-0.40] | 0.05 [0.02..0.06] |
| takeoff_land | 3/3 | 0.17 [0.05..0.37] | 0.00 | 0.45 [0.12..0.94] | 0.74 [0.21..1.35] | 0.83 [0.24..1.51] | 2.85 [1.18..5.25] | -0.7 [-5.3..2.1] | -0.23 [-0.40..-0.13] / 0.68 [0.17..1.34] | 0.01 [0.00..0.04] |

Values: mean [min..max] over trials. est_* = the live EKF estimate vs Gazebo truth over the airborne window.
track_h = true horizontal position vs the mission's ideal path during the profile.
