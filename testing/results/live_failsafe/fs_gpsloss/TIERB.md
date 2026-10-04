# Tier B - closed loop on the EKF

code `2ea4f28` (dirty) - tag `fs_gpsloss`, env `SIM_GPS_LOSS_AT=20`

| mission | survived | track_h rms (EKF) | track_h rms (ORACLE) | track_h max (EKF) | est tilt rms | est tilt max | est yaw rms | est yaw mean | est roll/pitch mean | est pos_h rms | track_v rms | est pos_v rms / max |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| hover | 0/1 | 16.43 [16.43..16.43] | 0.00 | 37.85 [37.85..37.85] | 9.36 [9.36..9.36] | 64.19 [64.19..64.19] | 11.21 [11.21..11.21] | 2.4 [2.4..2.4] | 0.28 [0.28..0.28] / -0.95 [-0.95..-0.95] | 36.99 [36.99..36.99] | 2.26 [2.26..2.26] | 2.74 [2.74..2.74] / 7.47 [7.47..7.47] |

Values: mean [min..max] over trials. est_* = the live EKF estimate vs Gazebo truth over the airborne window.
track_h = true horizontal position vs the mission's ideal path during the profile.
