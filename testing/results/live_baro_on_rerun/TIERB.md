# Tier B - closed loop on the EKF

code `2ea4f28` (dirty) - tag `baro_on_rerun`, env `SIM_REALISTIC=1`

| mission | survived | track_h rms (EKF) | track_h rms (ORACLE) | track_h max (EKF) | est tilt rms | est tilt max | est yaw rms | est yaw mean | est roll/pitch mean | est pos_h rms | track_v rms | est pos_v rms / max |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| hover | 1/1 | 0.50 [0.50..0.50] | 0.00 | 0.95 [0.95..0.95] | 0.42 [0.42..0.42] | 1.04 [1.04..1.04] | 1.04 [1.04..1.04] | 0.8 [0.8..0.8] | 0.00 [0.00..0.00] / -0.29 [-0.29..-0.29] | 0.64 [0.64..0.64] | 0.12 [0.12..0.12] | 0.12 [0.12..0.12] / 0.27 [0.27..0.27] |

Values: mean [min..max] over trials. est_* = the live EKF estimate vs Gazebo truth over the airborne window.
track_h = true horizontal position vs the mission's ideal path during the profile.
