# Tier B - closed loop on the EKF

code `2ea4f28` (dirty) - tag `fs_hang`, env `SIM_HANG_AT=20`

| mission | survived | track_h rms (EKF) | track_h rms (ORACLE) | track_h max (EKF) | est tilt rms | est tilt max | est yaw rms | est yaw mean | est roll/pitch mean | est pos_h rms | track_v rms | est pos_v rms / max |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| hover | 0/1 | 0.00 [0.00..0.00] | 0.00 | 0.00 [0.00..0.00] | 0.00 [0.00..0.00] | 0.01 [0.01..0.01] | 0.02 [0.02..0.02] | -0.0 [-0.0..-0.0] | 0.00 [0.00..0.00] / 0.00 [0.00..0.00] | 0.00 [0.00..0.00] | 0.28 [0.28..0.28] | 0.25 [0.25..0.25] / 0.30 [0.30..0.30] |

Values: mean [min..max] over trials. est_* = the live EKF estimate vs Gazebo truth over the airborne window.
track_h = true horizontal position vs the mission's ideal path during the profile.
