# Tier B - closed loop on the EKF

code `479064b` (dirty) - tag `clean_ct`, env `CAL_TURN=1`

| mission | survived | track_h rms (EKF) | track_h rms (ORACLE) | track_h max (EKF) | est tilt rms | est tilt max | est yaw rms | est yaw mean | est roll/pitch mean | est pos_h rms |
|---|---|---|---|---|---|---|---|---|---|---|
| circle_fast | 3/3 | 1.33 [1.32..1.34] | 4.63 | 3.24 [3.22..3.27] | 0.15 [0.15..0.15] | 0.68 [0.56..0.77] | 0.29 [0.27..0.31] | -0.0 [-0.1..-0.0] | -0.01 [-0.02..-0.00] / -0.02 [-0.03..-0.02] | 0.06 [0.05..0.08] |
| stops | 3/3 | 2.65 [2.65..2.65] | 3.34 | 4.44 [4.39..4.50] | 0.05 [0.05..0.05] | 0.26 [0.24..0.29] | 0.08 [0.07..0.08] | -0.0 [-0.0..-0.0] | 0.00 [-0.00..0.00] / 0.00 [0.00..0.01] | 0.04 [0.04..0.04] |
| yaw_steps | 3/3 | 0.07 [0.03..0.10] | 0.00 | 0.11 [0.06..0.15] | 0.05 [0.04..0.05] | 0.21 [0.15..0.31] | 0.11 [0.10..0.12] | -0.0 [-0.1..-0.0] | 0.00 [0.00..0.00] / 0.00 [-0.00..0.00] | 0.02 [0.01..0.03] |

Values: mean [min..max] over trials. est_* = the live EKF estimate vs Gazebo truth over the airborne window.
track_h = true horizontal position vs the mission's ideal path during the profile.
