# Tier B - closed loop on the EKF

code `4d25c07` (dirty)

| mission | survived | track_h rms (EKF) | track_h rms (ORACLE) | track_h max (EKF) | est tilt rms | est tilt max | est yaw rms | est yaw mean | est roll/pitch mean | est pos_h rms |
|---|---|---|---|---|---|---|---|---|---|---|
| hover_ct | 3/3 | 0.03 [0.02..0.05] | 0.00 | 0.07 [0.05..0.08] | 0.02 [0.01..0.02] | 0.07 [0.07..0.07] | 0.06 [0.06..0.07] | -0.0 [-0.0..-0.0] | 0.00 [0.00..0.00] / 0.01 [0.01..0.01] | 0.01 [0.01..0.01] |
| box_ct | 3/3 | 0.93 [0.93..0.93] | 0.92 | 1.36 [1.34..1.38] | 0.02 [0.02..0.02] | 0.08 [0.05..0.13] | 0.07 [0.06..0.07] | -0.0 [-0.0..-0.0] | 0.00 [0.00..0.00] / 0.01 [0.00..0.01] | 0.01 [0.01..0.01] |
| hover | 3/3 | 0.01 [0.01..0.02] | 0.00 | 0.02 [0.01..0.02] | 0.01 [0.01..0.01] | 0.02 [0.02..0.02] | 0.02 [0.01..0.02] | -0.0 [-0.0..0.0] | -0.00 [-0.00..0.00] / 0.00 [0.00..0.00] | 0.00 [0.00..0.01] |

Values: mean [min..max] over trials. est_* = the live EKF estimate vs Gazebo truth over the airborne window.
track_h = true horizontal position vs the mission's ideal path during the profile.
