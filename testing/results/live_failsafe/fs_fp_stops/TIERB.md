# Tier B - closed loop on the EKF

code `2ea4f28` (dirty) - tag `fs_fp_stops`, env `SIM_REALISTIC=1 CAL_TURN=1`

| mission | survived | track_h rms (EKF) | track_h rms (ORACLE) | track_h max (EKF) | est tilt rms | est tilt max | est yaw rms | est yaw mean | est roll/pitch mean | est pos_h rms | track_v rms | est pos_v rms / max |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| stops | 1/1 | 2.94 [2.94..2.94] | 3.34 | 5.63 [5.63..5.63] | 0.40 [0.40..0.40] | 1.12 [1.12..1.12] | 0.68 [0.68..0.68] | 0.0 [0.0..0.0] | -0.01 [-0.01..-0.01] / -0.21 [-0.21..-0.21] | 0.88 [0.88..0.88] | 0.50 [0.50..0.50] | 0.52 [0.52..0.52] / 2.55 [2.55..2.55] |

Values: mean [min..max] over trials. est_* = the live EKF estimate vs Gazebo truth over the airborne window.
track_h = true horizontal position vs the mission's ideal path during the profile.
