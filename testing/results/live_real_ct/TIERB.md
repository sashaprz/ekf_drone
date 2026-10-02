# Tier B - closed loop on the EKF

code `479064b` (dirty) - tag `real_ct`, env `SIM_REALISTIC=1 CAL_TURN=1`

| mission | survived | track_h rms (EKF) | track_h rms (ORACLE) | track_h max (EKF) | est tilt rms | est tilt max | est yaw rms | est yaw mean | est roll/pitch mean | est pos_h rms |
|---|---|---|---|---|---|---|---|---|---|---|
| circle_fast | 3/3 | 1.91 [1.86..2.01] | 4.63 | 3.56 [3.43..3.65] | 0.49 [0.47..0.51] | 1.56 [1.50..1.67] | 0.87 [0.84..0.90] | 0.0 [-0.0..0.0] | -0.01 [-0.02..-0.00] / -0.26 [-0.26..-0.25] | 0.92 [0.79..1.00] |
| stops | 2/3 | 12.59 [2.93..31.90] | 3.34 | 27.46 [5.48..71.37] | 10.27 [0.40..30.00] | 25.43 [1.04..74.09] | 5.92 [0.69..16.38] | -0.6 [-2.0..0.1] | -0.55 [-1.63..-0.01] / 0.51 [-0.21..1.95] | 1.20 [0.80..1.99] |

Values: mean [min..max] over trials. est_* = the live EKF estimate vs Gazebo truth over the airborne window.
track_h = true horizontal position vs the mission's ideal path during the profile.
