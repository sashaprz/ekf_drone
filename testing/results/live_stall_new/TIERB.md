# Tier B - closed loop on the EKF

code `6074c75` (dirty) - tag `stall_new`, env `SIM_REALISTIC=1 SIM_LOOP_STALL=0.005,150 CAL_TURN=1`

| mission | survived | track_h rms (EKF) | track_h rms (ORACLE) | track_h max (EKF) | est tilt rms | est tilt max | est yaw rms | est yaw mean | est roll/pitch mean | est pos_h rms |
|---|---|---|---|---|---|---|---|---|---|---|
| stops | 3/3 | 3.07 [2.96..3.25] | 3.34 | 6.36 [5.98..6.80] | 0.43 [0.40..0.45] | 1.32 [1.15..1.54] | 0.71 [0.66..0.76] | 0.1 [0.1..0.1] | -0.01 [-0.02..-0.01] / -0.22 [-0.22..-0.21] | 0.86 [0.82..0.89] |

Values: mean [min..max] over trials. est_* = the live EKF estimate vs Gazebo truth over the airborne window.
track_h = true horizontal position vs the mission's ideal path during the profile.
