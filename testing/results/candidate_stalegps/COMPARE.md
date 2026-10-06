# EKF suite comparison

baseline `6f70846` dirty (filter {'state estimation/FINAL_gps.py': 'b12ef7484f1b', 'state estimation/calibration.py': 'f891872e7efd'}) vs candidate `0be7988` (filter {'state estimation/FINAL_gps.py': 'd9265e258345', 'state estimation/calibration.py': 'f891872e7efd'})

**164 metric improvements, 58 metric regressions, 6 overall-grade changes** across 361 matched runs.


## Regressions

- **hover / gps_stale(duration_s=30)** `pos_h_rms_m`: 0.00312 -> 0.473 (None -> None)
- **hover / gps_stale(duration_s=30)** `vel_h_rms_ms`: 0.00118 -> 0.0597 (None -> None)
- **box / gps_stale(duration_s=5)** `nees_att`: 1.32 -> 0.000182 (PASS -> FAIL)
- **box / gps_stale(duration_s=5)** `nees_vel`: 2.17 -> 0.142 (None -> None)
- **box / gps_stale(duration_s=5)** `nees_pos`: 2.83 -> 0.00118 (None -> None)
- **box / gps_stale(duration_s=15)** `nees_att`: 1.38 -> 0.000175 (PASS -> FAIL)
- **box / gps_stale(duration_s=15)** `nees_pos`: 19.1 -> 0.00166 (None -> None)
- **box / gps_stale(duration_s=30)** `nees_att`: 4.74 -> 0.00017 (WARN -> FAIL)
- **box / gps_stale(duration_s=30)** `nees_pos`: 78.9 -> 0.00352 (None -> None)
- **circle_slow / gps_stale(duration_s=5)** `nees_att`: 0.274 -> 0.00402 (WARN -> FAIL)
- **circle_slow / gps_stale(duration_s=5)** `nees_vel`: 3.84 -> 0.12 (None -> None)
- **circle_slow / gps_stale(duration_s=5)** `nees_pos`: 2.11 -> 0.00117 (None -> None)
- **circle_slow / gps_stale(duration_s=15)** `nees_att`: 0.676 -> 0.00272 (PASS -> FAIL)
- **circle_slow / gps_stale(duration_s=15)** `nees_pos`: 12.8 -> 0.00137 (None -> None)
- **circle_slow / gps_stale(duration_s=30)** `pos_h_rms_m`: 3.65 -> 7.22 (None -> None)
- **circle_slow / gps_stale(duration_s=30)** `nees_att`: 1.16 -> 0.00157 (PASS -> FAIL)
- **circle_slow / gps_stale(duration_s=30)** `nees_pos`: 18.6 -> 0.00196 (None -> None)
- **circle_fast / gps_stale(duration_s=5)** `nees_att`: 1.9 -> 0.0512 (PASS -> FAIL)
- **circle_fast / gps_stale(duration_s=5)** `nees_pos`: 0.241 -> 0.0204 (None -> None)
- **circle_fast / gps_stale(duration_s=15)** `nees_att`: 6.44 -> 0.0383 (WARN -> FAIL)
- **circle_fast / gps_stale(duration_s=15)** `nees_pos`: 0.509 -> 0.0131 (None -> None)
- **circle_fast / gps_stale(duration_s=30)** `nees_att`: 11.4 -> 0.0301 (FAIL -> FAIL)
- **circle_fast / gps_stale(duration_s=30)** `nees_pos`: 0.951 -> 0.0139 (None -> None)
- **stops / gps_stale(duration_s=5)** `nees_pos`: 0.354 -> 0.00805 (None -> None)
- **stops / gps_stale(duration_s=15)** `nees_att`: 0.0252 -> 0.0119 (FAIL -> FAIL)
- **stops / gps_stale(duration_s=15)** `nees_pos`: 0.999 -> 0.00455 (None -> None)
- **stops / gps_stale(duration_s=30)** `nees_att`: 0.041 -> 0.0116 (FAIL -> FAIL)
- **stops / gps_stale(duration_s=30)** `nees_pos`: 1.93 -> 0.00337 (None -> None)
- **yaw_steps / gps_stale(duration_s=30)** `pos_h_rms_m`: 0.0173 -> 0.924 (None -> None)
- **yaw_steps / gps_stale(duration_s=30)** `vel_h_rms_ms`: 0.00799 -> 0.113 (None -> None)
- **yaw_steps / gps_stale(duration_s=30)** `recovery_s`: 0.0036 -> 0.612 (PASS -> PASS)
- **yaw_spin / gps_stale(duration_s=15)** `pos_h_rms_m`: 0.0248 -> 0.142 (None -> None)
- **yaw_spin / gps_stale(duration_s=15)** `recovery_s`: 0.00385 -> 0.608 (PASS -> PASS)
- **yaw_spin / gps_stale(duration_s=30)** `pos_h_rms_m`: 0.0246 -> 0.648 (None -> None)
- **yaw_spin / gps_stale(duration_s=30)** `vel_h_rms_ms`: 0.00977 -> 0.0541 (None -> None)
- **yaw_spin / gps_stale(duration_s=30)** `recovery_s`: 1.9e-05 -> 0.809 (PASS -> PASS)
- **takeoff_land / gps_stale(duration_s=15)** `nees_pos`: 0.883 -> 0.0623 (None -> None)
- **takeoff_land / gps_stale(duration_s=30)** `tilt_rms_deg`: 0.00484 -> 0.112 (PASS -> PASS)
- **takeoff_land / gps_stale(duration_s=30)** `tilt_max_deg`: 0.0123 -> 0.413 (PASS -> PASS)
- **takeoff_land / gps_stale(duration_s=30)** `yaw_rms_deg`: 0.0204 -> 0.231 (PASS -> PASS)
- **takeoff_land / gps_stale(duration_s=30)** `pos_h_rms_m`: 0.00602 -> 1.75 (None -> None)
- **takeoff_land / gps_stale(duration_s=30)** `vel_h_rms_ms`: 0.000941 -> 0.217 (None -> None)
- **takeoff_land / gps_stale(duration_s=30)** `nees_pos`: 0.719 -> 0.443 (None -> None)
- **takeoff_land / gps_outliers(frac=0.01)** `gps_rejected`: 1 -> 6 (None -> None)
- **takeoff_land / accel_bias(axis=x,from_cal=False,ms2=0.2)** `pos_h_rms_m`: 0.393 -> 0.558 (WARN -> WARN)
- **takeoff_land / imu_spikes** `pos_h_rms_m`: 0.0188 -> 0.124 (PASS -> PASS)
- **takeoff_land / imu_spikes** `vel_h_rms_ms`: 0.0317 -> 0.0556 (None -> None)
- **hover_ct / gps_stale(duration_s=30)** `pos_h_rms_m`: 0.00344 -> 0.428 (None -> None)
- **hover_ct / gps_stale(duration_s=30)** `vel_h_rms_ms`: 0.00139 -> 0.0568 (None -> None)
- **box_ct / gps_stale(duration_s=5)** `nees_att`: 0.0187 -> 0.00242 (FAIL -> FAIL)
- **box_ct / gps_stale(duration_s=5)** `nees_vel`: 1.64 -> 0.0374 (None -> None)
- **box_ct / gps_stale(duration_s=5)** `nees_pos`: 2.23 -> 0.000695 (None -> None)
- **box_ct / gps_stale(duration_s=15)** `nees_att`: 0.0743 -> 0.00242 (FAIL -> FAIL)
- **box_ct / gps_stale(duration_s=15)** `nees_vel`: 8.54 -> 0.0341 (None -> None)
- **box_ct / gps_stale(duration_s=15)** `nees_pos`: 15.3 -> 0.00096 (None -> None)
- **box_ct / gps_stale(duration_s=30)** `nees_att`: 0.136 -> 0.00241 (WARN -> FAIL)
- **box_ct / gps_stale(duration_s=30)** `nees_vel`: 12.8 -> 0.025 (None -> None)
- **box_ct / gps_stale(duration_s=30)** `nees_pos`: 63.7 -> 0.0016 (None -> None)

## Overall grade changes

- box / gps_stale(duration_s=15): WARN -> FAIL
- box / gps_stale(duration_s=30): WARN -> FAIL
- circle_slow / gps_stale(duration_s=5): WARN -> FAIL
- circle_slow / gps_stale(duration_s=15): WARN -> FAIL
- circle_slow / gps_stale(duration_s=30): WARN -> FAIL
- box_ct / gps_stale(duration_s=30): WARN -> FAIL

## Improvements

- hover / gps_stale(duration_s=30) `nees_pos`: 0.000276 -> 0.00153 (None -> None)
- box / gps_stale(duration_s=5) `tilt_rms_deg`: 0.189 -> 0.00895 (PASS -> PASS)
- box / gps_stale(duration_s=5) `tilt_max_deg`: 1.19 -> 0.0449 (PASS -> PASS)
- box / gps_stale(duration_s=5) `yaw_rms_deg`: 2.04 -> 0.021 (WARN -> PASS)
- box / gps_stale(duration_s=5) `pos_h_rms_m`: 0.811 -> 0.0119 (None -> None)
- box / gps_stale(duration_s=5) `vel_h_rms_ms`: 0.106 -> 0.0302 (None -> None)
- box / gps_stale(duration_s=5) `recovery_s`: never -> 3.9e-05 (FAIL -> PASS)
- box / gps_stale(duration_s=15) `tilt_rms_deg`: 0.415 -> 0.0103 (PASS -> PASS)
- box / gps_stale(duration_s=15) `tilt_max_deg`: 2.03 -> 0.0425 (PASS -> PASS)
- box / gps_stale(duration_s=15) `yaw_rms_deg`: 1.91 -> 0.0214 (PASS -> PASS)
- box / gps_stale(duration_s=15) `pos_h_rms_m`: 2.11 -> 0.0232 (None -> None)
- box / gps_stale(duration_s=15) `vel_h_rms_ms`: 0.253 -> 0.0307 (None -> None)
- box / gps_stale(duration_s=15) `tilt_drift_deg_s`: 0.0551 -> 0.000682 (PASS -> PASS)
- box / gps_stale(duration_s=15) `recovery_s`: 5.46 -> 0.000454 (WARN -> PASS)
- box / gps_stale(duration_s=15) `gps_resets`: 1 -> 0 (None -> None)
- box / gps_stale(duration_s=15) `gps_rejected`: 5 -> 0 (None -> None)
- box / gps_stale(duration_s=30) `tilt_rms_deg`: 0.498 -> 0.0279 (PASS -> PASS)
- box / gps_stale(duration_s=30) `tilt_max_deg`: 2.13 -> 0.181 (PASS -> PASS)
- box / gps_stale(duration_s=30) `yaw_rms_deg`: 3.12 -> 0.063 (WARN -> PASS)
- box / gps_stale(duration_s=30) `pos_h_rms_m`: 4.29 -> 0.379 (None -> None)
- box / gps_stale(duration_s=30) `vel_h_rms_ms`: 0.362 -> 0.0577 (None -> None)
- box / gps_stale(duration_s=30) `recovery_s`: 2.39 -> 0.00522 (PASS -> PASS)
- box / gps_stale(duration_s=30) `gps_resets`: 1 -> 0 (None -> None)
- box / gps_stale(duration_s=30) `gps_rejected`: 5 -> 0 (None -> None)
- circle_slow / gps_stale(duration_s=5) `tilt_rms_deg`: 0.65 -> 0.0445 (PASS -> PASS)
- circle_slow / gps_stale(duration_s=5) `tilt_max_deg`: 3.02 -> 0.243 (WARN -> PASS)
- circle_slow / gps_stale(duration_s=5) `yaw_rms_deg`: 1.34 -> 0.104 (PASS -> PASS)
- circle_slow / gps_stale(duration_s=5) `pos_h_rms_m`: 0.728 -> 0.0404 (None -> None)
- circle_slow / gps_stale(duration_s=5) `vel_h_rms_ms`: 0.348 -> 0.0375 (None -> None)
- circle_slow / gps_stale(duration_s=5) `tilt_drift_deg_s`: 0.467 -> 0.0359 (PASS -> PASS)
- circle_slow / gps_stale(duration_s=5) `recovery_s`: 5.4 -> 0.00015 (WARN -> PASS)
- circle_slow / gps_stale(duration_s=5) `gps_resets`: 1 -> 0 (None -> None)
- circle_slow / gps_stale(duration_s=5) `gps_rejected`: 5 -> 0 (None -> None)
- circle_slow / gps_stale(duration_s=15) `tilt_rms_deg`: 0.925 -> 0.16 (PASS -> PASS)
- circle_slow / gps_stale(duration_s=15) `tilt_max_deg`: 3.02 -> 0.522 (WARN -> PASS)
- circle_slow / gps_stale(duration_s=15) `yaw_rms_deg`: 1.8 -> 0.291 (PASS -> PASS)
- circle_slow / gps_stale(duration_s=15) `pos_h_rms_m`: 2.86 -> 0.889 (None -> None)
- circle_slow / gps_stale(duration_s=15) `vel_h_rms_ms`: 0.781 -> 0.186 (None -> None)
- circle_slow / gps_stale(duration_s=15) `nees_vel`: 70.8 -> 0.101 (None -> None)
- circle_slow / gps_stale(duration_s=15) `recovery_s`: 5.32 -> 0.0039 (WARN -> PASS)
- circle_slow / gps_stale(duration_s=15) `gps_resets`: 2 -> 0 (None -> None)
- circle_slow / gps_stale(duration_s=15) `gps_rejected`: 10 -> 0 (None -> None)
- circle_slow / gps_stale(duration_s=30) `tilt_rms_deg`: 1.27 -> 0.384 (WARN -> PASS)
- circle_slow / gps_stale(duration_s=30) `tilt_max_deg`: 3.14 -> 1.01 (WARN -> PASS)
- circle_slow / gps_stale(duration_s=30) `yaw_rms_deg`: 2.61 -> 0.714 (WARN -> PASS)
- circle_slow / gps_stale(duration_s=30) `vel_h_rms_ms`: 1.04 -> 0.8 (None -> None)
- circle_slow / gps_stale(duration_s=30) `nees_vel`: 108 -> 0.0698 (None -> None)
- circle_slow / gps_stale(duration_s=30) `recovery_s`: 5.48 -> 0.408 (WARN -> PASS)
- circle_slow / gps_stale(duration_s=30) `gps_resets`: 4 -> 0 (None -> None)
- circle_slow / gps_stale(duration_s=30) `gps_rejected`: 20 -> 0 (None -> None)
- circle_fast / gps_stale(duration_s=5) `tilt_rms_deg`: 4.75 -> 0.151 (FAIL -> PASS)
- circle_fast / gps_stale(duration_s=5) `tilt_max_deg`: 25.2 -> 0.575 (FAIL -> PASS)
- circle_fast / gps_stale(duration_s=5) `yaw_rms_deg`: 7.02 -> 0.296 (FAIL -> PASS)
- circle_fast / gps_stale(duration_s=5) `pos_h_rms_m`: 1.1 -> 0.0911 (None -> None)
- circle_fast / gps_stale(duration_s=5) `vel_h_rms_ms`: 0.991 -> 0.111 (None -> None)
- circle_fast / gps_stale(duration_s=5) `nees_vel`: 27.3 -> 1.3 (None -> None)
- circle_fast / gps_stale(duration_s=5) `tilt_drift_deg_s`: 1.37 -> 0.00319 (WARN -> PASS)
- circle_fast / gps_stale(duration_s=5) `recovery_s`: 8.46 -> 0.00445 (WARN -> PASS)
- circle_fast / gps_stale(duration_s=5) `gps_resets`: 6 -> 0 (None -> None)
- circle_fast / gps_stale(duration_s=5) `gps_rejected`: 30 -> 0 (None -> None)
- circle_fast / gps_stale(duration_s=15) `tilt_rms_deg`: 6.28 -> 0.138 (FAIL -> PASS)
- circle_fast / gps_stale(duration_s=15) `tilt_max_deg`: 23.1 -> 0.433 (FAIL -> PASS)
- circle_fast / gps_stale(duration_s=15) `yaw_rms_deg`: 10.8 -> 0.282 (FAIL -> PASS)
- circle_fast / gps_stale(duration_s=15) `pos_h_rms_m`: 1.58 -> 0.159 (None -> None)
- circle_fast / gps_stale(duration_s=15) `vel_h_rms_ms`: 1.5 -> 0.109 (None -> None)
- circle_fast / gps_stale(duration_s=15) `nees_vel`: 59.8 -> 1.06 (None -> None)
- circle_fast / gps_stale(duration_s=15) `tilt_drift_deg_s`: 1.05 -> 0.0102 (WARN -> PASS)
- circle_fast / gps_stale(duration_s=15) `recovery_s`: 5.33 -> 0.000125 (WARN -> PASS)
- circle_fast / gps_stale(duration_s=15) `gps_resets`: 10 -> 0 (None -> None)
- circle_fast / gps_stale(duration_s=15) `gps_rejected`: 50 -> 0 (None -> None)
- circle_fast / gps_stale(duration_s=30) `tilt_rms_deg`: 8.62 -> 0.152 (FAIL -> PASS)
- circle_fast / gps_stale(duration_s=30) `tilt_max_deg`: 23.1 -> 0.551 (FAIL -> PASS)
- circle_fast / gps_stale(duration_s=30) `yaw_rms_deg`: 14.6 -> 0.304 (FAIL -> PASS)
- circle_fast / gps_stale(duration_s=30) `pos_h_rms_m`: 2.23 -> 1.04 (None -> None)
- circle_fast / gps_stale(duration_s=30) `vel_h_rms_ms`: 2.13 -> 0.183 (None -> None)
- circle_fast / gps_stale(duration_s=30) `nees_vel`: 115 -> 0.742 (None -> None)
- circle_fast / gps_stale(duration_s=30) `tilt_drift_deg_s`: 0.123 -> 0.0053 (PASS -> PASS)
- circle_fast / gps_stale(duration_s=30) `recovery_s`: 5.07 -> 0.204 (WARN -> PASS)
- circle_fast / gps_stale(duration_s=30) `gps_resets`: 20 -> 0 (None -> None)
- circle_fast / gps_stale(duration_s=30) `gps_rejected`: 100 -> 0 (None -> None)
- stops / gps_stale(duration_s=5) `tilt_rms_deg`: 0.139 -> 0.0302 (PASS -> PASS)
- stops / gps_stale(duration_s=5) `tilt_max_deg`: 0.757 -> 0.192 (PASS -> PASS)
- stops / gps_stale(duration_s=5) `yaw_rms_deg`: 0.265 -> 0.104 (PASS -> PASS)
- stops / gps_stale(duration_s=5) `pos_h_rms_m`: 0.876 -> 0.0474 (None -> None)
- stops / gps_stale(duration_s=5) `vel_h_rms_ms`: 0.675 -> 0.0803 (None -> None)
- stops / gps_stale(duration_s=5) `nees_vel`: 40.8 -> 1.47 (None -> None)
- stops / gps_stale(duration_s=5) `tilt_drift_deg_s`: 0.14 -> 0.00542 (PASS -> PASS)
- stops / gps_stale(duration_s=5) `recovery_s`: 10.3 -> 0.00409 (FAIL -> PASS)
- stops / gps_stale(duration_s=5) `gps_resets`: 2 -> 0 (None -> None)
- stops / gps_stale(duration_s=5) `gps_rejected`: 10 -> 0 (None -> None)
- stops / gps_stale(duration_s=15) `tilt_rms_deg`: 0.227 -> 0.0321 (PASS -> PASS)
- stops / gps_stale(duration_s=15) `tilt_max_deg`: 0.76 -> 0.193 (PASS -> PASS)
- stops / gps_stale(duration_s=15) `yaw_rms_deg`: 0.422 -> 0.109 (PASS -> PASS)
- stops / gps_stale(duration_s=15) `pos_h_rms_m`: 1.52 -> 0.0432 (None -> None)
- stops / gps_stale(duration_s=15) `vel_h_rms_ms`: 1.16 -> 0.0795 (None -> None)
- stops / gps_stale(duration_s=15) `nees_vel`: 119 -> 1.19 (None -> None)
- stops / gps_stale(duration_s=15) `recovery_s`: 11.2 -> 0.00355 (FAIL -> PASS)
- stops / gps_stale(duration_s=15) `gps_resets`: 6 -> 0 (None -> None)
- stops / gps_stale(duration_s=15) `gps_rejected`: 30 -> 0 (None -> None)
- stops / gps_stale(duration_s=30) `tilt_rms_deg`: 0.331 -> 0.0497 (PASS -> PASS)
- stops / gps_stale(duration_s=30) `tilt_max_deg`: 0.839 -> 0.249 (PASS -> PASS)
- stops / gps_stale(duration_s=30) `yaw_rms_deg`: 0.599 -> 0.116 (PASS -> PASS)
- stops / gps_stale(duration_s=30) `pos_h_rms_m`: 2.13 -> 0.673 (None -> None)
- stops / gps_stale(duration_s=30) `vel_h_rms_ms`: 1.64 -> 0.105 (None -> None)
- stops / gps_stale(duration_s=30) `nees_vel`: 240 -> 0.695 (None -> None)
- stops / gps_stale(duration_s=30) `recovery_s`: 1.66 -> 0.404 (PASS -> PASS)
- stops / gps_stale(duration_s=30) `gps_resets`: 12 -> 0 (None -> None)
- stops / gps_stale(duration_s=30) `gps_rejected`: 62 -> 0 (None -> None)
- yaw_steps / gps_stale(duration_s=30) `nees_vel`: 0.0166 -> 0.04 (None -> None)
- yaw_steps / gps_stale(duration_s=30) `nees_pos`: 0.00142 -> 0.0143 (None -> None)
- yaw_spin / gps_stale(duration_s=15) `nees_pos`: 0.0027 -> 0.0131 (None -> None)
- yaw_spin / gps_stale(duration_s=30) `nees_vel`: 0.0249 -> 0.0845 (None -> None)
- yaw_spin / gps_stale(duration_s=30) `nees_pos`: 0.00265 -> 0.0489 (None -> None)
- takeoff_land / none `nees_pos`: 0.0594 -> 0.156 (None -> None)
- takeoff_land / gps_dropout(duration_s=5) `nees_pos`: 0.0274 -> 0.0916 (None -> None)
- takeoff_land / gps_dropout(duration_s=15) `nees_pos`: 0.023 -> 0.0601 (None -> None)
- takeoff_land / gps_stale(duration_s=5) `nees_pos`: 0.0399 -> 0.0933 (None -> None)
- takeoff_land / gps_stale(duration_s=15) `nees_vel`: 0.531 -> 0.999 (None -> None)
- takeoff_land / gps_outliers(frac=0.01) `nees_pos`: 0.0701 -> 0.135 (None -> None)
- takeoff_land / gps_latency(latency_ms=100) `nees_pos`: 0.0721 -> 0.173 (None -> None)
- takeoff_land / gps_latency(latency_ms=200) `nees_pos`: 0.0745 -> 0.178 (None -> None)
- takeoff_land / gps_latency(ekf_latency_ms=0,latency_ms=200) `nees_pos`: 0.0667 -> 0.169 (None -> None)
- takeoff_land / gyro_bias(dps=0.2) `nees_pos`: 0.0594 -> 0.156 (None -> None)
- takeoff_land / gyro_bias(dps=1.0) `nees_pos`: 0.0594 -> 0.156 (None -> None)
- takeoff_land / gyro_bias(dps=1.0,from_cal=False) `nees_pos`: 0.0594 -> 0.156 (None -> None)
- takeoff_land / gyro_drift(dps=0.5,over_s=60) `nees_pos`: 0.0594 -> 0.156 (None -> None)
- takeoff_land / accel_bias(axis=x,ms2=0.05) `nees_pos`: 0.0595 -> 0.156 (None -> None)
- takeoff_land / accel_bias(axis=x,ms2=0.2) `nees_pos`: 0.0593 -> 0.155 (None -> None)
- takeoff_land / accel_bias(axis=x,ms2=0.5) `nees_pos`: 0.0581 -> 0.154 (None -> None)
- takeoff_land / accel_bias(axis=z,ms2=0.05) `nees_pos`: 0.0537 -> 0.145 (None -> None)
- takeoff_land / accel_bias(axis=z,ms2=0.2) `nees_pos`: 0.0386 -> 0.115 (None -> None)
- takeoff_land / accel_bias(axis=z,ms2=0.5) `nees_pos`: 0.0177 -> 0.0672 (None -> None)
- takeoff_land / accel_bias(axis=x,from_cal=False,ms2=0.2) `nees_pos`: 0.691 -> 1.1 (None -> None)
- takeoff_land / mag_bias(frac=0.05) `nees_pos`: 0.0594 -> 0.156 (None -> None)
- takeoff_land / mag_bias(frac=0.15) `nees_pos`: 0.0594 -> 0.156 (None -> None)
- takeoff_land / mag_interference(amp=0.3) `nees_pos`: 0.0594 -> 0.156 (None -> None)
- takeoff_land / mag_interference(amp=1.0) `nees_pos`: 0.0594 -> 0.156 (None -> None)
- takeoff_land / imu_noise `nees_pos`: 0.0694 -> 0.169 (None -> None)
- takeoff_land / imu_spikes `nees_pos`: 0.287 -> 0.661 (None -> None)
- takeoff_land / imu_dropouts `nees_pos`: 0.0585 -> 0.159 (None -> None)
- takeoff_land / imu_gap(gap_s=0.2) `nees_pos`: 0.0444 -> 0.125 (None -> None)
- takeoff_land / imu_gap(gap_s=1.0) `nees_pos`: 0.0451 -> 0.118 (None -> None)
- takeoff_land / cal_ideal `nees_pos`: 0.0586 -> 0.154 (None -> None)
- box_ct / gps_stale(duration_s=5) `tilt_rms_deg`: 0.16 -> 0.0168 (PASS -> PASS)
- box_ct / gps_stale(duration_s=5) `tilt_max_deg`: 1.12 -> 0.0894 (PASS -> PASS)
- box_ct / gps_stale(duration_s=5) `yaw_rms_deg`: 0.308 -> 0.045 (PASS -> PASS)
- box_ct / gps_stale(duration_s=5) `pos_h_rms_m`: 0.719 -> 0.015 (None -> None)
- box_ct / gps_stale(duration_s=5) `vel_h_rms_ms`: 0.0895 -> 0.0166 (None -> None)
- box_ct / gps_stale(duration_s=5) `recovery_s`: never -> 0.00347 (FAIL -> PASS)
- box_ct / gps_stale(duration_s=15) `tilt_rms_deg`: 0.372 -> 0.0551 (PASS -> PASS)
- box_ct / gps_stale(duration_s=15) `tilt_max_deg`: 2 -> 0.241 (PASS -> PASS)
- box_ct / gps_stale(duration_s=15) `yaw_rms_deg`: 0.745 -> 0.121 (PASS -> PASS)
- box_ct / gps_stale(duration_s=15) `pos_h_rms_m`: 1.89 -> 0.289 (None -> None)
- box_ct / gps_stale(duration_s=15) `vel_h_rms_ms`: 0.223 -> 0.0635 (None -> None)
- box_ct / gps_stale(duration_s=15) `recovery_s`: 5.22 -> 0.00306 (WARN -> PASS)
- box_ct / gps_stale(duration_s=15) `gps_resets`: 1 -> 0 (None -> None)
- box_ct / gps_stale(duration_s=15) `gps_rejected`: 5 -> 0 (None -> None)
- box_ct / gps_stale(duration_s=30) `tilt_rms_deg`: 0.444 -> 0.131 (PASS -> PASS)
- box_ct / gps_stale(duration_s=30) `tilt_max_deg`: 2.12 -> 0.388 (PASS -> PASS)
- box_ct / gps_stale(duration_s=30) `yaw_rms_deg`: 0.88 -> 0.277 (PASS -> PASS)
- box_ct / gps_stale(duration_s=30) `pos_h_rms_m`: 3.86 -> 2.44 (None -> None)
- box_ct / gps_stale(duration_s=30) `recovery_s`: 1.55 -> 0.00229 (PASS -> PASS)
- box_ct / gps_stale(duration_s=30) `gps_resets`: 1 -> 0 (None -> None)
- box_ct / gps_stale(duration_s=30) `gps_rejected`: 5 -> 0 (None -> None)

## All matched runs (metrics that changed at all)

| mission | fault | overall | changes |
|---|---|---|---|
| hover | none | FAIL | - |
| hover | gps_dropout(duration_s=5) | FAIL | - |
| hover | gps_dropout(duration_s=15) | FAIL | - |
| hover | gps_dropout(duration_s=30) | FAIL | - |
| hover | gps_stale(duration_s=5) | FAIL | tilt_rms_deg 0.00473->0.00471; tilt_max_deg 0.0136->0.0136; yaw_rms_deg 0.0189->0.0188; pos_h_rms_m 0.00164->0.00242; vel_h_rms_ms 0.0011->0.00119; nees_vel 0.00159->0.00169; nees_pos 0.000243->0.000294; tilt_drift_deg_s -0.000253->-0.000387 |
| hover | gps_stale(duration_s=15) | FAIL | tilt_rms_deg 0.00475->0.00494; tilt_max_deg 0.0136->0.0166; yaw_rms_deg 0.019->0.02; pos_h_rms_m 0.00235->0.00994; vel_h_rms_ms 0.00113->0.00239; nees_vel 0.0016->0.0021; nees_pos 0.000256->0.000591; tilt_drift_deg_s 3.1e-05->0.000211 |
| hover | gps_stale(duration_s=30) | FAIL | tilt_rms_deg 0.00483->0.032; tilt_max_deg 0.0137->0.17; yaw_rms_deg 0.019->0.0708; pos_h_rms_m 0.00312->0.473 **WORSE**; vel_h_rms_ms 0.00118->0.0597 **WORSE**; nees_att 0.000156->0.000157; nees_vel 0.00163->0.00316; nees_pos 0.000276->0.00153 **BETTER**; tilt_drift_deg_s -1.4e-05->0.00352 |
| hover | gps_noise | FAIL | - |
| hover | gps_outliers(frac=0.01) | FAIL | - |
| hover | gps_rate(hz=1.0) | FAIL | - |
| hover | gps_latency(latency_ms=100) | FAIL | - |
| hover | gps_latency(latency_ms=200) | FAIL | - |
| hover | gps_latency(ekf_latency_ms=0,latency_ms=200) | FAIL | - |
| hover | gyro_bias(dps=0.2) | FAIL | - |
| hover | gyro_bias(dps=1.0) | FAIL | - |
| hover | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| hover | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| hover | accel_bias(axis=x,ms2=0.05) | PASS | - |
| hover | accel_bias(axis=x,ms2=0.2) | FAIL | - |
| hover | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| hover | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| hover | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| hover | accel_bias(axis=z,ms2=0.5) | FAIL | - |
| hover | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | - |
| hover | mag_bias(frac=0.05) | WARN | - |
| hover | mag_bias(frac=0.15) | FAIL | - |
| hover | mag_interference(amp=0.3) | FAIL | - |
| hover | mag_interference(amp=1.0) | FAIL | - |
| hover | imu_noise | FAIL | - |
| hover | imu_spikes | WARN | - |
| hover | imu_dropouts | FAIL | - |
| hover | imu_gap(gap_s=0.2) | FAIL | - |
| hover | imu_gap(gap_s=1.0) | FAIL | - |
| hover | combined_realistic | WARN | - |
| hover | cal_ideal | FAIL | - |
| box | none | FAIL | - |
| box | gps_dropout(duration_s=5) | FAIL | - |
| box | gps_dropout(duration_s=15) | FAIL | - |
| box | gps_dropout(duration_s=30) | FAIL | - |
| box | gps_stale(duration_s=5) | FAIL | tilt_rms_deg 0.189->0.00895 **BETTER**; tilt_max_deg 1.19->0.0449 **BETTER**; yaw_rms_deg 2.04->0.021 **BETTER**; pos_h_rms_m 0.811->0.0119 **BETTER**; vel_h_rms_ms 0.106->0.0302 **BETTER**; nees_att 1.32->0.000182 **WORSE**; nees_vel 2.17->0.142 **WORSE**; nees_pos 2.83->0.00118 **WORSE**; tilt_drift_deg_s -0.000738->0.00102; recovery_s never->3.9e-05 **BETTER** |
| box | gps_stale(duration_s=15) | WARN -> FAIL | tilt_rms_deg 0.415->0.0103 **BETTER**; tilt_max_deg 2.03->0.0425 **BETTER**; yaw_rms_deg 1.91->0.0214 **BETTER**; pos_h_rms_m 2.11->0.0232 **BETTER**; vel_h_rms_ms 0.253->0.0307 **BETTER**; nees_att 1.38->0.000175 **WORSE**; nees_vel 10.6->0.139; nees_pos 19.1->0.00166 **WORSE**; tilt_drift_deg_s 0.0551->0.000682 **BETTER**; recovery_s 5.46->0.000454 **BETTER**; gps_resets 1->0 **BETTER**; gps_rejected 5->0 **BETTER** |
| box | gps_stale(duration_s=30) | WARN -> FAIL | tilt_rms_deg 0.498->0.0279 **BETTER**; tilt_max_deg 2.13->0.181 **BETTER**; yaw_rms_deg 3.12->0.063 **BETTER**; pos_h_rms_m 4.29->0.379 **BETTER**; vel_h_rms_ms 0.362->0.0577 **BETTER**; nees_att 4.74->0.00017 **WORSE**; nees_vel 16.3->0.0854; nees_pos 78.9->0.00352 **WORSE**; tilt_drift_deg_s 0.00989->0.0024; recovery_s 2.39->0.00522 **BETTER**; gps_resets 1->0 **BETTER**; gps_rejected 5->0 **BETTER** |
| box | gps_noise | FAIL | - |
| box | gps_outliers(frac=0.01) | FAIL | - |
| box | gps_rate(hz=1.0) | FAIL | - |
| box | gps_latency(latency_ms=100) | FAIL | - |
| box | gps_latency(latency_ms=200) | FAIL | - |
| box | gps_latency(ekf_latency_ms=0,latency_ms=200) | FAIL | - |
| box | gyro_bias(dps=0.2) | FAIL | - |
| box | gyro_bias(dps=1.0) | FAIL | - |
| box | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| box | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| box | accel_bias(axis=x,ms2=0.05) | PASS | - |
| box | accel_bias(axis=x,ms2=0.2) | FAIL | - |
| box | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| box | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| box | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| box | accel_bias(axis=z,ms2=0.5) | FAIL | - |
| box | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | - |
| box | mag_bias(frac=0.05) | WARN | - |
| box | mag_bias(frac=0.15) | FAIL | - |
| box | mag_interference(amp=0.3) | FAIL | - |
| box | mag_interference(amp=1.0) | FAIL | - |
| box | imu_noise | FAIL | - |
| box | imu_spikes | WARN | - |
| box | imu_dropouts | FAIL | - |
| box | imu_gap(gap_s=0.2) | FAIL | - |
| box | imu_gap(gap_s=1.0) | FAIL | - |
| box | combined_realistic | WARN | - |
| box | cal_ideal | FAIL | - |
| circle_slow | none | FAIL | - |
| circle_slow | gps_dropout(duration_s=5) | FAIL | - |
| circle_slow | gps_dropout(duration_s=15) | FAIL | - |
| circle_slow | gps_dropout(duration_s=30) | FAIL | - |
| circle_slow | gps_stale(duration_s=5) | WARN -> FAIL | tilt_rms_deg 0.65->0.0445 **BETTER**; tilt_max_deg 3.02->0.243 **BETTER**; yaw_rms_deg 1.34->0.104 **BETTER**; pos_h_rms_m 0.728->0.0404 **BETTER**; vel_h_rms_ms 0.348->0.0375 **BETTER**; nees_att 0.274->0.00402 **WORSE**; nees_vel 3.84->0.12 **WORSE**; nees_pos 2.11->0.00117 **WORSE**; tilt_drift_deg_s 0.467->0.0359 **BETTER**; recovery_s 5.4->0.00015 **BETTER**; gps_resets 1->0 **BETTER**; gps_rejected 5->0 **BETTER** |
| circle_slow | gps_stale(duration_s=15) | WARN -> FAIL | tilt_rms_deg 0.925->0.16 **BETTER**; tilt_max_deg 3.02->0.522 **BETTER**; yaw_rms_deg 1.8->0.291 **BETTER**; pos_h_rms_m 2.86->0.889 **BETTER**; vel_h_rms_ms 0.781->0.186 **BETTER**; nees_att 0.676->0.00272 **WORSE**; nees_vel 70.8->0.101 **BETTER**; nees_pos 12.8->0.00137 **WORSE**; tilt_drift_deg_s 0.0321->0.0321; recovery_s 5.32->0.0039 **BETTER**; gps_resets 2->0 **BETTER**; gps_rejected 10->0 **BETTER** |
| circle_slow | gps_stale(duration_s=30) | WARN -> FAIL | tilt_rms_deg 1.27->0.384 **BETTER**; tilt_max_deg 3.14->1.01 **BETTER**; yaw_rms_deg 2.61->0.714 **BETTER**; pos_h_rms_m 3.65->7.22 **WORSE**; vel_h_rms_ms 1.04->0.8 **BETTER**; nees_att 1.16->0.00157 **WORSE**; nees_vel 108->0.0698 **BETTER**; nees_pos 18.6->0.00196 **WORSE**; tilt_drift_deg_s -0.00768->0.029; recovery_s 5.48->0.408 **BETTER**; gps_resets 4->0 **BETTER**; gps_rejected 20->0 **BETTER** |
| circle_slow | gps_noise | FAIL | - |
| circle_slow | gps_outliers(frac=0.01) | FAIL | - |
| circle_slow | gps_rate(hz=1.0) | FAIL | - |
| circle_slow | gps_latency(latency_ms=100) | FAIL | - |
| circle_slow | gps_latency(latency_ms=200) | FAIL | - |
| circle_slow | gps_latency(ekf_latency_ms=0,latency_ms=200) | PASS | - |
| circle_slow | gyro_bias(dps=0.2) | FAIL | - |
| circle_slow | gyro_bias(dps=1.0) | FAIL | - |
| circle_slow | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| circle_slow | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| circle_slow | accel_bias(axis=x,ms2=0.05) | PASS | - |
| circle_slow | accel_bias(axis=x,ms2=0.2) | FAIL | - |
| circle_slow | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| circle_slow | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| circle_slow | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| circle_slow | accel_bias(axis=z,ms2=0.5) | FAIL | - |
| circle_slow | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | - |
| circle_slow | mag_bias(frac=0.05) | PASS | - |
| circle_slow | mag_bias(frac=0.15) | FAIL | - |
| circle_slow | mag_interference(amp=0.3) | FAIL | - |
| circle_slow | mag_interference(amp=1.0) | FAIL | - |
| circle_slow | imu_noise | FAIL | - |
| circle_slow | imu_spikes | WARN | - |
| circle_slow | imu_dropouts | FAIL | - |
| circle_slow | imu_gap(gap_s=0.2) | FAIL | - |
| circle_slow | imu_gap(gap_s=1.0) | FAIL | - |
| circle_slow | combined_realistic | PASS | - |
| circle_slow | cal_ideal | FAIL | - |
| circle_fast | none | FAIL | - |
| circle_fast | gps_dropout(duration_s=5) | FAIL | - |
| circle_fast | gps_dropout(duration_s=15) | FAIL | - |
| circle_fast | gps_dropout(duration_s=30) | FAIL | - |
| circle_fast | gps_stale(duration_s=5) | FAIL | tilt_rms_deg 4.75->0.151 **BETTER**; tilt_max_deg 25.2->0.575 **BETTER**; yaw_rms_deg 7.02->0.296 **BETTER**; pos_h_rms_m 1.1->0.0911 **BETTER**; vel_h_rms_ms 0.991->0.111 **BETTER**; nees_att 1.9->0.0512 **WORSE**; nees_vel 27.3->1.3 **BETTER**; nees_pos 0.241->0.0204 **WORSE**; tilt_drift_deg_s 1.37->0.00319 **BETTER**; recovery_s 8.46->0.00445 **BETTER**; gps_resets 6->0 **BETTER**; gps_rejected 30->0 **BETTER** |
| circle_fast | gps_stale(duration_s=15) | FAIL | tilt_rms_deg 6.28->0.138 **BETTER**; tilt_max_deg 23.1->0.433 **BETTER**; yaw_rms_deg 10.8->0.282 **BETTER**; pos_h_rms_m 1.58->0.159 **BETTER**; vel_h_rms_ms 1.5->0.109 **BETTER**; nees_att 6.44->0.0383 **WORSE**; nees_vel 59.8->1.06 **BETTER**; nees_pos 0.509->0.0131 **WORSE**; tilt_drift_deg_s 1.05->0.0102 **BETTER**; recovery_s 5.33->0.000125 **BETTER**; gps_resets 10->0 **BETTER**; gps_rejected 50->0 **BETTER** |
| circle_fast | gps_stale(duration_s=30) | FAIL | tilt_rms_deg 8.62->0.152 **BETTER**; tilt_max_deg 23.1->0.551 **BETTER**; yaw_rms_deg 14.6->0.304 **BETTER**; pos_h_rms_m 2.23->1.04 **BETTER**; vel_h_rms_ms 2.13->0.183 **BETTER**; nees_att 11.4->0.0301 **WORSE**; nees_vel 115->0.742 **BETTER**; nees_pos 0.951->0.0139 **WORSE**; tilt_drift_deg_s 0.123->0.0053 **BETTER**; recovery_s 5.07->0.204 **BETTER**; gps_resets 20->0 **BETTER**; gps_rejected 100->0 **BETTER** |
| circle_fast | gps_noise | WARN | - |
| circle_fast | gps_outliers(frac=0.01) | FAIL | - |
| circle_fast | gps_rate(hz=1.0) | FAIL | - |
| circle_fast | gps_latency(latency_ms=100) | FAIL | - |
| circle_fast | gps_latency(latency_ms=200) | FAIL | - |
| circle_fast | gps_latency(ekf_latency_ms=0,latency_ms=200) | WARN | - |
| circle_fast | gyro_bias(dps=0.2) | FAIL | - |
| circle_fast | gyro_bias(dps=1.0) | FAIL | - |
| circle_fast | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| circle_fast | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| circle_fast | accel_bias(axis=x,ms2=0.05) | PASS | - |
| circle_fast | accel_bias(axis=x,ms2=0.2) | FAIL | - |
| circle_fast | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| circle_fast | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| circle_fast | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| circle_fast | accel_bias(axis=z,ms2=0.5) | FAIL | - |
| circle_fast | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | - |
| circle_fast | mag_bias(frac=0.05) | WARN | - |
| circle_fast | mag_bias(frac=0.15) | WARN | - |
| circle_fast | mag_interference(amp=0.3) | FAIL | - |
| circle_fast | mag_interference(amp=1.0) | FAIL | - |
| circle_fast | imu_noise | FAIL | - |
| circle_fast | imu_spikes | WARN | - |
| circle_fast | imu_dropouts | FAIL | - |
| circle_fast | imu_gap(gap_s=0.2) | FAIL | - |
| circle_fast | imu_gap(gap_s=1.0) | FAIL | - |
| circle_fast | combined_realistic | PASS | - |
| circle_fast | cal_ideal | FAIL | - |
| stops | none | FAIL | - |
| stops | gps_dropout(duration_s=5) | FAIL | - |
| stops | gps_dropout(duration_s=15) | FAIL | - |
| stops | gps_dropout(duration_s=30) | FAIL | - |
| stops | gps_stale(duration_s=5) | FAIL | tilt_rms_deg 0.139->0.0302 **BETTER**; tilt_max_deg 0.757->0.192 **BETTER**; yaw_rms_deg 0.265->0.104 **BETTER**; pos_h_rms_m 0.876->0.0474 **BETTER**; vel_h_rms_ms 0.675->0.0803 **BETTER**; nees_att 0.0163->0.0123; nees_vel 40.8->1.47 **BETTER**; nees_pos 0.354->0.00805 **WORSE**; tilt_drift_deg_s 0.14->0.00542 **BETTER**; recovery_s 10.3->0.00409 **BETTER**; gps_resets 2->0 **BETTER**; gps_rejected 10->0 **BETTER** |
| stops | gps_stale(duration_s=15) | FAIL | tilt_rms_deg 0.227->0.0321 **BETTER**; tilt_max_deg 0.76->0.193 **BETTER**; yaw_rms_deg 0.422->0.109 **BETTER**; pos_h_rms_m 1.52->0.0432 **BETTER**; vel_h_rms_ms 1.16->0.0795 **BETTER**; nees_att 0.0252->0.0119 **WORSE**; nees_vel 119->1.19 **BETTER**; nees_pos 0.999->0.00455 **WORSE**; tilt_drift_deg_s 0.017->0.000956; recovery_s 11.2->0.00355 **BETTER**; gps_resets 6->0 **BETTER**; gps_rejected 30->0 **BETTER** |
| stops | gps_stale(duration_s=30) | FAIL | tilt_rms_deg 0.331->0.0497 **BETTER**; tilt_max_deg 0.839->0.249 **BETTER**; yaw_rms_deg 0.599->0.116 **BETTER**; pos_h_rms_m 2.13->0.673 **BETTER**; vel_h_rms_ms 1.64->0.105 **BETTER**; nees_att 0.041->0.0116 **WORSE**; nees_vel 240->0.695 **BETTER**; nees_pos 1.93->0.00337 **WORSE**; tilt_drift_deg_s 0.00257->0.00292; recovery_s 1.66->0.404 **BETTER**; gps_resets 12->0 **BETTER**; gps_rejected 62->0 **BETTER** |
| stops | gps_noise | WARN | - |
| stops | gps_outliers(frac=0.01) | FAIL | - |
| stops | gps_rate(hz=1.0) | FAIL | - |
| stops | gps_latency(latency_ms=100) | FAIL | - |
| stops | gps_latency(latency_ms=200) | FAIL | - |
| stops | gps_latency(ekf_latency_ms=0,latency_ms=200) | PASS | - |
| stops | gyro_bias(dps=0.2) | FAIL | - |
| stops | gyro_bias(dps=1.0) | FAIL | - |
| stops | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| stops | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| stops | accel_bias(axis=x,ms2=0.05) | PASS | - |
| stops | accel_bias(axis=x,ms2=0.2) | FAIL | - |
| stops | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| stops | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| stops | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| stops | accel_bias(axis=z,ms2=0.5) | FAIL | - |
| stops | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | - |
| stops | mag_bias(frac=0.05) | PASS | - |
| stops | mag_bias(frac=0.15) | FAIL | - |
| stops | mag_interference(amp=0.3) | FAIL | - |
| stops | mag_interference(amp=1.0) | FAIL | - |
| stops | imu_noise | FAIL | - |
| stops | imu_spikes | WARN | - |
| stops | imu_dropouts | FAIL | - |
| stops | imu_gap(gap_s=0.2) | FAIL | - |
| stops | imu_gap(gap_s=1.0) | FAIL | - |
| stops | combined_realistic | WARN | - |
| stops | cal_ideal | FAIL | - |
| yaw_steps | none | FAIL | - |
| yaw_steps | gps_dropout(duration_s=5) | FAIL | - |
| yaw_steps | gps_dropout(duration_s=15) | FAIL | - |
| yaw_steps | gps_dropout(duration_s=30) | FAIL | - |
| yaw_steps | gps_stale(duration_s=5) | FAIL | tilt_rms_deg 0.0211->0.0209; tilt_max_deg 0.0697->0.0707; yaw_rms_deg 0.0509->0.0499; pos_h_rms_m 0.0195->0.0195; vel_h_rms_ms 0.0081->0.00813; nees_att 0.00832->0.0083; nees_vel 0.017->0.0169; nees_pos 0.00173->0.00171; tilt_drift_deg_s 0.00528->0.000737 |
| yaw_steps | gps_stale(duration_s=15) | FAIL | tilt_rms_deg 0.0213->0.0263; tilt_max_deg 0.0715->0.153; yaw_rms_deg 0.0511->0.0684; pos_h_rms_m 0.0188->0.0392; vel_h_rms_ms 0.00808->0.0143; nees_att 0.00833->0.00825; nees_vel 0.017->0.0155; nees_pos 0.00164->0.000944; tilt_drift_deg_s 0.00207->0.0051 |
| yaw_steps | gps_stale(duration_s=30) | FAIL | tilt_rms_deg 0.0213->0.0644; tilt_max_deg 0.0715->0.253; yaw_rms_deg 0.0513->0.142; pos_h_rms_m 0.0173->0.924 **WORSE**; vel_h_rms_ms 0.00799->0.113 **WORSE**; nees_att 0.00834->0.00724; nees_vel 0.0166->0.04 **BETTER**; nees_pos 0.00142->0.0143 **BETTER**; tilt_drift_deg_s 0.000728->0.00594; recovery_s 0.0036->0.612 **WORSE** |
| yaw_steps | gps_noise | FAIL | - |
| yaw_steps | gps_outliers(frac=0.01) | FAIL | - |
| yaw_steps | gps_rate(hz=1.0) | FAIL | - |
| yaw_steps | gps_latency(latency_ms=100) | FAIL | - |
| yaw_steps | gps_latency(latency_ms=200) | FAIL | - |
| yaw_steps | gps_latency(ekf_latency_ms=0,latency_ms=200) | FAIL | - |
| yaw_steps | gyro_bias(dps=0.2) | FAIL | - |
| yaw_steps | gyro_bias(dps=1.0) | FAIL | - |
| yaw_steps | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| yaw_steps | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| yaw_steps | accel_bias(axis=x,ms2=0.05) | PASS | - |
| yaw_steps | accel_bias(axis=x,ms2=0.2) | FAIL | - |
| yaw_steps | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| yaw_steps | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| yaw_steps | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| yaw_steps | accel_bias(axis=z,ms2=0.5) | FAIL | - |
| yaw_steps | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | - |
| yaw_steps | mag_bias(frac=0.05) | WARN | - |
| yaw_steps | mag_bias(frac=0.15) | FAIL | - |
| yaw_steps | mag_interference(amp=0.3) | FAIL | - |
| yaw_steps | mag_interference(amp=1.0) | FAIL | - |
| yaw_steps | imu_noise | FAIL | - |
| yaw_steps | imu_spikes | WARN | - |
| yaw_steps | imu_dropouts | FAIL | - |
| yaw_steps | imu_gap(gap_s=0.2) | FAIL | - |
| yaw_steps | imu_gap(gap_s=1.0) | FAIL | - |
| yaw_steps | combined_realistic | PASS | - |
| yaw_steps | cal_ideal | FAIL | - |
| yaw_spin | none | FAIL | - |
| yaw_spin | gps_dropout(duration_s=5) | FAIL | - |
| yaw_spin | gps_dropout(duration_s=15) | FAIL | - |
| yaw_spin | gps_dropout(duration_s=30) | FAIL | - |
| yaw_spin | gps_stale(duration_s=5) | FAIL | tilt_rms_deg 0.0195->0.0199; tilt_max_deg 0.0414->0.0511; yaw_rms_deg 0.0528->0.0537; pos_h_rms_m 0.0249->0.0297; vel_h_rms_ms 0.0098->0.0109; nees_att 0.0115->0.0114; nees_vel 0.025->0.0265; nees_pos 0.00272->0.00305; tilt_drift_deg_s 0.00355->0.00554 |
| yaw_spin | gps_stale(duration_s=15) | FAIL | tilt_rms_deg 0.0196->0.0216; tilt_max_deg 0.0414->0.0543; yaw_rms_deg 0.0529->0.0579; pos_h_rms_m 0.0248->0.142 **WORSE**; vel_h_rms_ms 0.00979->0.0224; nees_att 0.0115->0.0113; nees_vel 0.025->0.0404; nees_pos 0.0027->0.0131 **BETTER**; tilt_drift_deg_s -0.000542->2.2e-05; recovery_s 0.00385->0.608 **WORSE** |
| yaw_spin | gps_stale(duration_s=30) | FAIL | tilt_rms_deg 0.0196->0.0237; tilt_max_deg 0.0414->0.0578; yaw_rms_deg 0.0528->0.0632; pos_h_rms_m 0.0246->0.648 **WORSE**; vel_h_rms_ms 0.00977->0.0541 **WORSE**; nees_att 0.0115->0.0111; nees_vel 0.0249->0.0845 **BETTER**; nees_pos 0.00265->0.0489 **BETTER**; tilt_drift_deg_s -2e-06->4e-05; recovery_s 1.9e-05->0.809 **WORSE** |
| yaw_spin | gps_noise | FAIL | - |
| yaw_spin | gps_outliers(frac=0.01) | FAIL | - |
| yaw_spin | gps_rate(hz=1.0) | FAIL | - |
| yaw_spin | gps_latency(latency_ms=100) | FAIL | - |
| yaw_spin | gps_latency(latency_ms=200) | FAIL | - |
| yaw_spin | gps_latency(ekf_latency_ms=0,latency_ms=200) | FAIL | - |
| yaw_spin | gyro_bias(dps=0.2) | FAIL | - |
| yaw_spin | gyro_bias(dps=1.0) | FAIL | - |
| yaw_spin | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| yaw_spin | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| yaw_spin | accel_bias(axis=x,ms2=0.05) | PASS | - |
| yaw_spin | accel_bias(axis=x,ms2=0.2) | WARN | - |
| yaw_spin | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| yaw_spin | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| yaw_spin | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| yaw_spin | accel_bias(axis=z,ms2=0.5) | FAIL | - |
| yaw_spin | accel_bias(axis=x,from_cal=False,ms2=0.2) | WARN | - |
| yaw_spin | mag_bias(frac=0.05) | WARN | - |
| yaw_spin | mag_bias(frac=0.15) | WARN | - |
| yaw_spin | mag_interference(amp=0.3) | FAIL | - |
| yaw_spin | mag_interference(amp=1.0) | FAIL | - |
| yaw_spin | imu_noise | FAIL | - |
| yaw_spin | imu_spikes | WARN | - |
| yaw_spin | imu_dropouts | FAIL | - |
| yaw_spin | imu_gap(gap_s=0.2) | FAIL | - |
| yaw_spin | imu_gap(gap_s=1.0) | FAIL | - |
| yaw_spin | combined_realistic | PASS | - |
| yaw_spin | cal_ideal | FAIL | - |
| patrol_long | none | FAIL | - |
| patrol_long | gps_dropout(duration_s=30) | FAIL | - |
| patrol_long | gps_noise | FAIL | - |
| patrol_long | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| patrol_long | accel_bias(axis=x,ms2=0.05) | PASS | - |
| patrol_long | mag_bias(frac=0.05) | PASS | - |
| patrol_long | imu_noise | FAIL | - |
| patrol_long | imu_spikes | WARN | - |
| patrol_long | combined_realistic | WARN | - |
| patrol_long | cal_ideal | FAIL | - |
| takeoff_land | none | FAIL | tilt_rms_deg 0.00466->0.00513; tilt_max_deg 0.0126->0.016; yaw_rms_deg 0.0202->0.0197; pos_h_rms_m 0.000433->0.00125; vel_h_rms_ms 0.00069->0.000976; nees_att 0.000122->0.000121; nees_vel 1.15->1.34; nees_pos 0.0594->0.156 **BETTER** |
| takeoff_land | gps_dropout(duration_s=5) | FAIL | tilt_rms_deg 0.00478->0.00524; tilt_max_deg 0.0143->0.016; yaw_rms_deg 0.0198->0.0193; pos_h_rms_m 0.00204->0.0023; vel_h_rms_ms 0.00089->0.00112; nees_att 0.000122->0.000121; nees_vel 1.07->1.24; nees_pos 0.0274->0.0916 **BETTER** |
| takeoff_land | gps_dropout(duration_s=15) | FAIL | tilt_rms_deg 0.00582->0.00612; yaw_rms_deg 0.0211->0.0207; pos_h_rms_m 0.0165->0.0169; vel_h_rms_ms 0.00376->0.00379; nees_att 0.00012->0.000119; nees_vel 0.888->0.99; nees_pos 0.023->0.0601 **BETTER** |
| takeoff_land | gps_dropout(duration_s=30) | FAIL | - |
| takeoff_land | gps_stale(duration_s=5) | FAIL | tilt_rms_deg 0.00471->0.00515; tilt_max_deg 0.0126->0.016; yaw_rms_deg 0.0203->0.0195; pos_h_rms_m 0.00114->0.00207; vel_h_rms_ms 0.000687->0.00106; nees_att 0.000123->0.000121; nees_vel 1.02->1.24; nees_pos 0.0399->0.0933 **BETTER**; tilt_drift_deg_s 0.000164->0.000191 |
| takeoff_land | gps_stale(duration_s=15) | FAIL | tilt_rms_deg 0.00468->0.00645; tilt_max_deg 0.0123->0.0326; yaw_rms_deg 0.0202->0.0229; pos_h_rms_m 0.00335->0.0206; vel_h_rms_ms 0.000784->0.0047; nees_att 0.000123->0.000119; nees_vel 0.531->0.999 **BETTER**; nees_pos 0.883->0.0623 **WORSE**; tilt_drift_deg_s 4.9e-05->0.000775 |
| takeoff_land | gps_stale(duration_s=30) | FAIL | tilt_rms_deg 0.00484->0.112 **WORSE**; tilt_max_deg 0.0123->0.413 **WORSE**; yaw_rms_deg 0.0204->0.231 **WORSE**; pos_h_rms_m 0.00602->1.75 **WORSE**; vel_h_rms_ms 0.000941->0.217 **WORSE**; nees_att 0.000123->0.000121; nees_vel 1.22->1.53; nees_pos 0.719->0.443 **WORSE**; tilt_drift_deg_s 4.1e-05->0.0118 |
| takeoff_land | gps_noise | FAIL | - |
| takeoff_land | gps_outliers(frac=0.01) | FAIL | tilt_rms_deg 0.00483->0.00483; yaw_rms_deg 0.0203->0.0202; pos_h_rms_m 0.0177->0.0178; vel_h_rms_ms 0.000703->0.000719; nees_vel 1.15->1.27; nees_pos 0.0701->0.135 **BETTER**; gps_rejected 1->6 **WORSE** |
| takeoff_land | gps_rate(hz=1.0) | FAIL | tilt_rms_deg 0.00479->0.00493; tilt_max_deg 0.0146->0.0143; yaw_rms_deg 0.0205->0.02; pos_h_rms_m 0.00116->0.00135; vel_h_rms_ms 0.000986->0.00113; nees_att 0.000117->0.000116; nees_vel 1.13->1.2; nees_pos 0.0709->0.103 |
| takeoff_land | gps_latency(latency_ms=100) | FAIL | tilt_rms_deg 0.00486->0.00529; tilt_max_deg 0.0134->0.0159; yaw_rms_deg 0.0204->0.0198; pos_h_rms_m 0.000517->0.00134; vel_h_rms_ms 0.000743->0.00103; nees_att 0.000123->0.000122; nees_vel 1.2->1.39; nees_pos 0.0721->0.173 **BETTER** |
| takeoff_land | gps_latency(latency_ms=200) | FAIL | tilt_rms_deg 0.005->0.00538; tilt_max_deg 0.0144->0.0154; yaw_rms_deg 0.0205->0.0199; pos_h_rms_m 0.000692->0.00144; vel_h_rms_ms 0.000793->0.00106; nees_att 0.000124->0.000123; nees_vel 1.2->1.39; nees_pos 0.0745->0.178 **BETTER** |
| takeoff_land | gps_latency(ekf_latency_ms=0,latency_ms=200) | FAIL | tilt_rms_deg 0.00467->0.00496; tilt_max_deg 0.0124->0.0144; yaw_rms_deg 0.0202->0.0197; pos_h_rms_m 0.000443->0.00109; vel_h_rms_ms 0.000704->0.000901; nees_att 0.000122->0.000121; nees_vel 1.2->1.39; nees_pos 0.0667->0.169 **BETTER** |
| takeoff_land | gyro_bias(dps=0.2) | FAIL | tilt_rms_deg 0.00466->0.00513; tilt_max_deg 0.0126->0.016; yaw_rms_deg 0.0202->0.0197; pos_h_rms_m 0.000433->0.00125; vel_h_rms_ms 0.00069->0.000976; nees_att 0.000122->0.000121; nees_vel 1.15->1.34; nees_pos 0.0594->0.156 **BETTER** |
| takeoff_land | gyro_bias(dps=1.0) | FAIL | tilt_rms_deg 0.00466->0.00513; tilt_max_deg 0.0126->0.016; yaw_rms_deg 0.0202->0.0197; pos_h_rms_m 0.000433->0.00125; vel_h_rms_ms 0.00069->0.000976; nees_att 0.000122->0.000121; nees_vel 1.15->1.34; nees_pos 0.0594->0.156 **BETTER** |
| takeoff_land | gyro_bias(dps=1.0,from_cal=False) | FAIL | tilt_rms_deg 0.0653->0.0653; yaw_rms_deg 0.251->0.252; pos_h_rms_m 0.00171->0.00211; vel_h_rms_ms 0.00935->0.00937; nees_att 0.00545->0.00545; nees_vel 1.16->1.34; nees_pos 0.0594->0.156 **BETTER** |
| takeoff_land | gyro_drift(dps=0.5,over_s=60) | FAIL | tilt_rms_deg 0.00543->0.00914; tilt_max_deg 0.0145->0.0441; yaw_rms_deg 0.014->0.0167; pos_h_rms_m 0.000431->0.00342; vel_h_rms_ms 0.000733->0.00244; nees_vel 1.15->1.34; nees_pos 0.0594->0.156 **BETTER** |
| takeoff_land | accel_bias(axis=x,ms2=0.05) | PASS | tilt_rms_deg 0.292->0.292; tilt_max_deg 0.303->0.303; yaw_rms_deg 0.0462->0.0455; pos_h_rms_m 0.000807->0.002; vel_h_rms_ms 0.00122->0.00141; nees_att 1.15->1.15; nees_vel 1.16->1.34; nees_pos 0.0595->0.156 **BETTER** |
| takeoff_land | accel_bias(axis=x,ms2=0.2) | FAIL | tilt_rms_deg 1.17->1.17; tilt_max_deg 1.18->1.18; yaw_rms_deg 0.129->0.129; pos_h_rms_m 0.00227->0.00683; vel_h_rms_ms 0.00401->0.00404; nees_att 18.4->18.4; nees_vel 1.16->1.34; nees_pos 0.0593->0.155 **BETTER** |
| takeoff_land | accel_bias(axis=x,ms2=0.5) | FAIL | tilt_rms_deg 2.92->2.92; tilt_max_deg 2.93->2.93; yaw_rms_deg 0.317->0.316; pos_h_rms_m 0.00529->0.017; vel_h_rms_ms 0.00984->0.00976; nees_att 115->115; nees_vel 1.16->1.34; nees_pos 0.0581->0.154 **BETTER** |
| takeoff_land | accel_bias(axis=z,ms2=0.05) | FAIL | tilt_rms_deg 0.00466->0.00512; tilt_max_deg 0.0126->0.016; yaw_rms_deg 0.0202->0.0197; pos_h_rms_m 0.000433->0.00125; vel_h_rms_ms 0.00069->0.000976; nees_att 0.000122->0.000121; nees_vel 1.1->1.28; nees_pos 0.0537->0.145 **BETTER** |
| takeoff_land | accel_bias(axis=z,ms2=0.2) | FAIL | tilt_rms_deg 0.00466->0.00512; tilt_max_deg 0.0127->0.016; yaw_rms_deg 0.0203->0.0198; pos_h_rms_m 0.000433->0.00125; vel_h_rms_ms 0.00069->0.000977; nees_att 0.000122->0.000121; nees_vel 0.974->1.12; nees_pos 0.0386->0.115 **BETTER** |
| takeoff_land | accel_bias(axis=z,ms2=0.5) | FAIL | tilt_rms_deg 0.00466->0.00511; tilt_max_deg 0.0127->0.0159; yaw_rms_deg 0.0204->0.0199; pos_h_rms_m 0.000433->0.00125; vel_h_rms_ms 0.00069->0.000977; nees_att 0.000123->0.000122; nees_vel 0.809->0.896; nees_pos 0.0177->0.0672 **BETTER** |
| takeoff_land | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | tilt_rms_deg 0.803->0.8; tilt_max_deg 0.973->0.974; yaw_rms_deg 8.69->8.67; pos_h_rms_m 0.393->0.558 **WORSE**; vel_h_rms_ms 0.246->0.262; nees_att 99.7->98.6; nees_vel 13.6->14.4; nees_pos 0.691->1.1 **BETTER** |
| takeoff_land | mag_bias(frac=0.05) | WARN | tilt_rms_deg 0.00466->0.0051; tilt_max_deg 0.0125->0.0156; yaw_rms_deg 3.51->3.51; pos_h_rms_m 0.000406->0.00123; vel_h_rms_ms 0.000693->0.000966; nees_att 0.626->0.626; nees_vel 1.15->1.34; nees_pos 0.0594->0.156 **BETTER** |
| takeoff_land | mag_bias(frac=0.15) | FAIL | tilt_rms_deg 0.00471->0.00508; tilt_max_deg 0.0123->0.015; yaw_rms_deg 10->10; pos_h_rms_m 0.000445->0.00122; vel_h_rms_ms 0.00072->0.000958; nees_att 5.1->5.1; nees_vel 1.15->1.34; nees_pos 0.0594->0.156 **BETTER** |
| takeoff_land | mag_interference(amp=0.3) | FAIL | tilt_rms_deg 0.00531->0.00572; yaw_rms_deg 0.0242->0.0238; pos_h_rms_m 0.000304->0.0012; vel_h_rms_ms 0.000643->0.000936; nees_att 0.000119->0.000118; nees_vel 1.15->1.34; nees_pos 0.0594->0.156 **BETTER** |
| takeoff_land | mag_interference(amp=1.0) | FAIL | tilt_rms_deg 0.00531->0.00572; yaw_rms_deg 0.0242->0.0238; pos_h_rms_m 0.000304->0.0012; vel_h_rms_ms 0.000643->0.000936; nees_att 0.000119->0.000118; nees_vel 1.15->1.34; nees_pos 0.0594->0.156 **BETTER** |
| takeoff_land | imu_noise | FAIL | tilt_rms_deg 0.0252->0.0277; tilt_max_deg 0.0621->0.0789; yaw_rms_deg 0.0495->0.0585; pos_h_rms_m 0.0022->0.00992; vel_h_rms_ms 0.00404->0.00714; nees_att 0.00108->0.00105; nees_vel 1.24->1.44; nees_pos 0.0694->0.169 **BETTER** |
| takeoff_land | imu_spikes | FAIL | tilt_rms_deg 0.275->0.3; tilt_max_deg 1.18->1.21; yaw_rms_deg 0.533->0.597; pos_h_rms_m 0.0188->0.124 **WORSE**; vel_h_rms_ms 0.0317->0.0556 **WORSE**; nees_att 0.0889->0.0853; nees_vel 4.51->5.26; nees_pos 0.287->0.661 **BETTER** |
| takeoff_land | imu_dropouts | FAIL | tilt_rms_deg 0.00461->0.00514; tilt_max_deg 0.0131->0.0162; yaw_rms_deg 0.0204->0.0198; pos_h_rms_m 0.00039->0.00141; vel_h_rms_ms 0.000691->0.00105; nees_vel 1.17->1.37; nees_pos 0.0585->0.159 **BETTER** |
| takeoff_land | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.00466->0.00519; tilt_max_deg 0.0123->0.0164; yaw_rms_deg 0.02->0.0195; pos_h_rms_m 0.00032->0.00122; vel_h_rms_ms 0.000607->0.000944; nees_att 0.000122->0.000121; nees_vel 1.03->1.19; nees_pos 0.0444->0.125 **BETTER** |
| takeoff_land | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.00791->0.00824; yaw_rms_deg 0.0249->0.0245; pos_h_rms_m 0.000666->0.00145; vel_h_rms_ms 0.000659->0.000985; nees_att 0.000125->0.000124; nees_vel 0.961->1.12; nees_pos 0.0451->0.118 **BETTER** |
| takeoff_land | combined_realistic | WARN | - |
| takeoff_land | cal_ideal | FAIL | tilt_rms_deg 0.00466->0.00513; tilt_max_deg 0.0126->0.016; yaw_rms_deg 0.0202->0.0197; pos_h_rms_m 0.000434->0.00125; vel_h_rms_ms 0.00069->0.000976; nees_att 0.000122->0.000121; nees_vel 1.15->1.33; nees_pos 0.0586->0.154 **BETTER** |
| hover_ct | none | FAIL | - |
| hover_ct | gps_dropout(duration_s=5) | FAIL | - |
| hover_ct | gps_dropout(duration_s=15) | FAIL | - |
| hover_ct | gps_dropout(duration_s=30) | FAIL | - |
| hover_ct | gps_stale(duration_s=5) | FAIL | tilt_rms_deg 0.0068->0.00681; yaw_rms_deg 0.0281->0.028; pos_h_rms_m 0.00253->0.00305; vel_h_rms_ms 0.00136->0.00142; nees_att 0.00207->0.00207; nees_vel 0.00247->0.00258; nees_pos 0.000389->0.000456; tilt_drift_deg_s 0.000112->0.000511 |
| hover_ct | gps_stale(duration_s=15) | FAIL | tilt_rms_deg 0.00686->0.0072; yaw_rms_deg 0.0282->0.0288; pos_h_rms_m 0.00319->0.0115; vel_h_rms_ms 0.00138->0.00263; nees_att 0.00207->0.00206; nees_vel 0.00247->0.00291; nees_pos 0.000407->0.000712; tilt_drift_deg_s -5.3e-05->0.0005 |
| hover_ct | gps_stale(duration_s=30) | FAIL | tilt_rms_deg 0.00687->0.0326; tilt_max_deg 0.0334->0.208; yaw_rms_deg 0.0283->0.074; pos_h_rms_m 0.00344->0.428 **WORSE**; vel_h_rms_ms 0.00139->0.0568 **WORSE**; nees_att 0.00207->0.00206; nees_vel 0.00247->0.00397; nees_pos 0.000416->0.00161; tilt_drift_deg_s 4e-06->0.0034 |
| hover_ct | gps_noise | FAIL | - |
| hover_ct | gps_outliers(frac=0.01) | FAIL | - |
| hover_ct | gps_rate(hz=1.0) | FAIL | - |
| hover_ct | gps_latency(latency_ms=100) | FAIL | - |
| hover_ct | gps_latency(latency_ms=200) | FAIL | - |
| hover_ct | gps_latency(ekf_latency_ms=0,latency_ms=200) | FAIL | - |
| hover_ct | gyro_bias(dps=0.2) | FAIL | - |
| hover_ct | gyro_bias(dps=1.0) | FAIL | - |
| hover_ct | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| hover_ct | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| hover_ct | accel_bias(axis=x,ms2=0.05) | PASS | - |
| hover_ct | accel_bias(axis=x,ms2=0.2) | FAIL | - |
| hover_ct | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| hover_ct | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| hover_ct | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| hover_ct | accel_bias(axis=z,ms2=0.5) | FAIL | - |
| hover_ct | accel_bias(axis=x,from_cal=False,ms2=0.2) | WARN | - |
| hover_ct | mag_bias(frac=0.05) | FAIL | - |
| hover_ct | mag_bias(frac=0.15) | WARN | - |
| hover_ct | mag_interference(amp=0.3) | FAIL | - |
| hover_ct | mag_interference(amp=1.0) | FAIL | - |
| hover_ct | imu_noise | FAIL | - |
| hover_ct | imu_spikes | WARN | - |
| hover_ct | imu_dropouts | FAIL | - |
| hover_ct | imu_gap(gap_s=0.2) | FAIL | - |
| hover_ct | imu_gap(gap_s=1.0) | FAIL | - |
| hover_ct | combined_realistic | PASS | - |
| hover_ct | cal_ideal | FAIL | - |
| box_ct | none | FAIL | - |
| box_ct | gps_dropout(duration_s=5) | FAIL | - |
| box_ct | gps_dropout(duration_s=15) | FAIL | - |
| box_ct | gps_dropout(duration_s=30) | FAIL | - |
| box_ct | gps_stale(duration_s=5) | FAIL | tilt_rms_deg 0.16->0.0168 **BETTER**; tilt_max_deg 1.12->0.0894 **BETTER**; yaw_rms_deg 0.308->0.045 **BETTER**; pos_h_rms_m 0.719->0.015 **BETTER**; vel_h_rms_ms 0.0895->0.0166 **BETTER**; nees_att 0.0187->0.00242 **WORSE**; nees_vel 1.64->0.0374 **WORSE**; nees_pos 2.23->0.000695 **WORSE**; tilt_drift_deg_s 0.00177->0.0126; recovery_s never->0.00347 **BETTER** |
| box_ct | gps_stale(duration_s=15) | FAIL | tilt_rms_deg 0.372->0.0551 **BETTER**; tilt_max_deg 2->0.241 **BETTER**; yaw_rms_deg 0.745->0.121 **BETTER**; pos_h_rms_m 1.89->0.289 **BETTER**; vel_h_rms_ms 0.223->0.0635 **BETTER**; nees_att 0.0743->0.00242 **WORSE**; nees_vel 8.54->0.0341 **WORSE**; nees_pos 15.3->0.00096 **WORSE**; tilt_drift_deg_s 0.0544->0.0131; recovery_s 5.22->0.00306 **BETTER**; gps_resets 1->0 **BETTER**; gps_rejected 5->0 **BETTER** |
| box_ct | gps_stale(duration_s=30) | WARN -> FAIL | tilt_rms_deg 0.444->0.131 **BETTER**; tilt_max_deg 2.12->0.388 **BETTER**; yaw_rms_deg 0.88->0.277 **BETTER**; pos_h_rms_m 3.86->2.44 **BETTER**; vel_h_rms_ms 0.318->0.275; nees_att 0.136->0.00241 **WORSE**; nees_vel 12.8->0.025 **WORSE**; nees_pos 63.7->0.0016 **WORSE**; tilt_drift_deg_s 0.00918->0.0108; recovery_s 1.55->0.00229 **BETTER**; gps_resets 1->0 **BETTER**; gps_rejected 5->0 **BETTER** |
| box_ct | gps_noise | FAIL | - |
| box_ct | gps_outliers(frac=0.01) | FAIL | - |
| box_ct | gps_rate(hz=1.0) | FAIL | - |
| box_ct | gps_latency(latency_ms=100) | FAIL | - |
| box_ct | gps_latency(latency_ms=200) | FAIL | - |
| box_ct | gps_latency(ekf_latency_ms=0,latency_ms=200) | FAIL | - |
| box_ct | gyro_bias(dps=0.2) | FAIL | - |
| box_ct | gyro_bias(dps=1.0) | FAIL | - |
| box_ct | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| box_ct | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| box_ct | accel_bias(axis=x,ms2=0.05) | PASS | - |
| box_ct | accel_bias(axis=x,ms2=0.2) | FAIL | - |
| box_ct | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| box_ct | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| box_ct | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| box_ct | accel_bias(axis=z,ms2=0.5) | FAIL | - |
| box_ct | accel_bias(axis=x,from_cal=False,ms2=0.2) | WARN | - |
| box_ct | mag_bias(frac=0.05) | FAIL | - |
| box_ct | mag_bias(frac=0.15) | WARN | - |
| box_ct | mag_interference(amp=0.3) | FAIL | - |
| box_ct | mag_interference(amp=1.0) | FAIL | - |
| box_ct | imu_noise | FAIL | - |
| box_ct | imu_spikes | WARN | - |
| box_ct | imu_dropouts | FAIL | - |
| box_ct | imu_gap(gap_s=0.2) | FAIL | - |
| box_ct | imu_gap(gap_s=1.0) | FAIL | - |
| box_ct | combined_realistic | PASS | - |
| box_ct | cal_ideal | FAIL | - |
| ref_live1 | none | FAIL | - |
