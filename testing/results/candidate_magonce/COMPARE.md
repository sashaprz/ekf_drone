# EKF suite comparison

baseline `6074c75` dirty (filter {'state estimation/FINAL_gps.py': 'c14058582dcd', 'state estimation/calibration.py': 'f891872e7efd'}) vs candidate `6f70846` dirty (filter {'state estimation/FINAL_gps.py': 'b12ef7484f1b', 'state estimation/calibration.py': 'f891872e7efd'})

**120 metric improvements, 216 metric regressions, 12 overall-grade changes** across 361 matched runs.


## Regressions

- **hover / accel_bias(axis=x,from_cal=False,ms2=0.2)** `yaw_rms_deg`: 6.53 -> 8.56 (FAIL -> FAIL)
- **hover / accel_bias(axis=x,from_cal=False,ms2=0.2)** `nees_pos`: 0.804 -> 0.57 (None -> None)
- **box / gps_dropout(duration_s=30)** `pos_h_rms_m`: 0.96 -> 1.46 (None -> None)
- **box / gps_dropout(duration_s=30)** `vel_h_rms_ms`: 0.125 -> 0.162 (None -> None)
- **box / gps_stale(duration_s=5)** `yaw_rms_deg`: 1.29 -> 2.04 (PASS -> WARN)
- **box / gps_stale(duration_s=15)** `yaw_rms_deg`: 1.41 -> 1.91 (PASS -> PASS)
- **box / gps_stale(duration_s=15)** `nees_att`: 0.996 -> 1.38 (PASS -> PASS)
- **box / gps_stale(duration_s=30)** `yaw_rms_deg`: 2.37 -> 3.12 (WARN -> WARN)
- **box / gps_stale(duration_s=30)** `nees_att`: 3.13 -> 4.74 (WARN -> WARN)
- **box / accel_bias(axis=x,from_cal=False,ms2=0.2)** `yaw_rms_deg`: 6.51 -> 8.58 (FAIL -> FAIL)
- **box / accel_bias(axis=x,from_cal=False,ms2=0.2)** `nees_pos`: 0.784 -> 0.548 (None -> None)
- **box / mag_bias(frac=0.05)** `nees_att`: 0.997 -> 0.707 (PASS -> PASS)
- **circle_slow / gps_latency(ekf_latency_ms=0,latency_ms=200)** `yaw_rms_deg`: 0.614 -> 1.05 (PASS -> PASS)
- **circle_slow / accel_bias(axis=x,from_cal=False,ms2=0.2)** `yaw_rms_deg`: 5.41 -> 7.85 (FAIL -> FAIL)
- **circle_slow / accel_bias(axis=x,from_cal=False,ms2=0.2)** `nees_pos`: 0.937 -> 0.672 (None -> None)
- **circle_slow / mag_bias(frac=0.05)** `nees_att`: 0.701 -> 0.54 (PASS -> PASS)
- **circle_fast / gps_dropout(duration_s=5)** `nees_att`: 0.08 -> 0.0482 (FAIL -> FAIL)
- **circle_fast / gps_dropout(duration_s=15)** `nees_att`: 0.0747 -> 0.0359 (FAIL -> FAIL)
- **circle_fast / gps_dropout(duration_s=30)** `nees_att`: 0.0727 -> 0.0281 (FAIL -> FAIL)
- **circle_fast / gps_stale(duration_s=5)** `tilt_rms_deg`: 0.735 -> 4.75 (PASS -> FAIL)
- **circle_fast / gps_stale(duration_s=5)** `tilt_max_deg`: 5.79 -> 25.2 (WARN -> FAIL)
- **circle_fast / gps_stale(duration_s=5)** `yaw_rms_deg`: 1.6 -> 7.02 (PASS -> FAIL)
- **circle_fast / gps_stale(duration_s=5)** `tilt_drift_deg_s`: 0.725 -> 1.37 (WARN -> WARN)
- **circle_fast / gps_stale(duration_s=5)** `recovery_s`: 3.58 -> 8.46 (WARN -> WARN)
- **circle_fast / gps_stale(duration_s=5)** `gps_resets`: 4 -> 6 (None -> None)
- **circle_fast / gps_stale(duration_s=5)** `gps_rejected`: 20 -> 30 (None -> None)
- **circle_fast / gps_stale(duration_s=15)** `tilt_rms_deg`: 3.14 -> 6.28 (FAIL -> FAIL)
- **circle_fast / gps_stale(duration_s=15)** `tilt_max_deg`: 12 -> 23.1 (FAIL -> FAIL)
- **circle_fast / gps_stale(duration_s=15)** `yaw_rms_deg`: 5.88 -> 10.8 (FAIL -> FAIL)
- **circle_fast / gps_stale(duration_s=15)** `nees_att`: 0.972 -> 6.44 (PASS -> WARN)
- **circle_fast / gps_stale(duration_s=15)** `tilt_drift_deg_s`: 0.677 -> 1.05 (WARN -> WARN)
- **circle_fast / gps_stale(duration_s=30)** `tilt_rms_deg`: 5.54 -> 8.62 (FAIL -> FAIL)
- **circle_fast / gps_stale(duration_s=30)** `tilt_max_deg`: 15.6 -> 23.1 (FAIL -> FAIL)
- **circle_fast / gps_stale(duration_s=30)** `yaw_rms_deg`: 10.9 -> 14.6 (FAIL -> FAIL)
- **circle_fast / gps_stale(duration_s=30)** `nees_att`: 5.25 -> 11.4 (WARN -> FAIL)
- **circle_fast / gps_stale(duration_s=30)** `recovery_s`: 4.18 -> 5.07 (WARN -> WARN)
- **circle_fast / gps_rate(hz=1.0)** `nees_att`: 0.0665 -> 0.0175 (FAIL -> FAIL)
- **circle_fast / gps_latency(latency_ms=200)** `nees_att`: 0.102 -> 0.0846 (WARN -> FAIL)
- **circle_fast / accel_bias(axis=x,ms2=0.2)** `nees_pos`: 0.181 -> 0.0398 (None -> None)
- **circle_fast / accel_bias(axis=x,ms2=0.5)** `nees_pos`: 1.05 -> 0.18 (None -> None)
- **circle_fast / accel_bias(axis=x,from_cal=False,ms2=0.2)** `yaw_rms_deg`: 2.6 -> 3.56 (WARN -> WARN)
- **circle_fast / imu_spikes** `nees_att`: 0.308 -> 0.245 (PASS -> WARN)
- **stops / none** `nees_att`: 0.0734 -> 0.0124 (FAIL -> FAIL)
- **stops / gps_dropout(duration_s=5)** `nees_att`: 0.0739 -> 0.0123 (FAIL -> FAIL)
- **stops / gps_dropout(duration_s=15)** `nees_att`: 0.0743 -> 0.0119 (FAIL -> FAIL)
- **stops / gps_dropout(duration_s=30)** `nees_att`: 0.0749 -> 0.0116 (FAIL -> FAIL)
- **stops / gps_stale(duration_s=5)** `nees_att`: 0.0766 -> 0.0163 (FAIL -> FAIL)
- **stops / gps_stale(duration_s=15)** `nees_att`: 0.0861 -> 0.0252 (FAIL -> FAIL)
- **stops / gps_stale(duration_s=30)** `nees_att`: 0.104 -> 0.041 (WARN -> FAIL)
- **stops / gps_noise** `nees_att`: 0.161 -> 0.101 (WARN -> WARN)
- **stops / gps_outliers(frac=0.01)** `nees_att`: 0.0735 -> 0.0124 (FAIL -> FAIL)
- **stops / gps_rate(hz=1.0)** `nees_att`: 0.0739 -> 0.0109 (FAIL -> FAIL)
- **stops / gps_latency(latency_ms=100)** `nees_att`: 0.073 -> 0.0125 (FAIL -> FAIL)
- **stops / gps_latency(latency_ms=200)** `nees_att`: 0.0715 -> 0.0113 (FAIL -> FAIL)
- **stops / gyro_bias(dps=0.2)** `nees_att`: 0.0734 -> 0.0124 (FAIL -> FAIL)
- **stops / gyro_bias(dps=1.0)** `nees_att`: 0.0734 -> 0.0124 (FAIL -> FAIL)
- **stops / gyro_bias(dps=1.0,from_cal=False)** `nees_att`: 0.0743 -> 0.0139 (FAIL -> FAIL)
- **stops / gyro_drift(dps=0.5,over_s=60)** `nees_att`: 0.0733 -> 0.0122 (FAIL -> FAIL)
- **stops / accel_bias(axis=x,ms2=0.2)** `nees_pos`: 0.13 -> 0.0339 (None -> None)
- **stops / accel_bias(axis=x,ms2=0.5)** `nees_pos`: 0.609 -> 0.121 (None -> None)
- **stops / accel_bias(axis=z,ms2=0.05)** `nees_att`: 0.0734 -> 0.0124 (FAIL -> FAIL)
- **stops / accel_bias(axis=z,ms2=0.2)** `nees_att`: 0.0734 -> 0.0124 (FAIL -> FAIL)
- **stops / accel_bias(axis=z,ms2=0.5)** `nees_att`: 0.0733 -> 0.0123 (FAIL -> FAIL)
- **stops / accel_bias(axis=x,from_cal=False,ms2=0.2)** `tilt_max_deg`: 4.67 -> 5.91 (WARN -> WARN)
- **stops / mag_bias(frac=0.05)** `nees_att`: 0.962 -> 0.522 (PASS -> PASS)
- **stops / mag_interference(amp=0.3)** `nees_att`: 0.0677 -> 0.0118 (FAIL -> FAIL)
- **stops / mag_interference(amp=1.0)** `nees_att`: 0.0677 -> 0.0118 (FAIL -> FAIL)
- **stops / imu_noise** `nees_att`: 0.0793 -> 0.0181 (FAIL -> FAIL)
- **stops / imu_spikes** `nees_att`: 0.243 -> 0.179 (WARN -> WARN)
- **stops / imu_dropouts** `nees_att`: 0.0745 -> 0.0129 (FAIL -> FAIL)
- **stops / imu_gap(gap_s=0.2)** `nees_att`: 0.0737 -> 0.0129 (FAIL -> FAIL)
- **stops / imu_gap(gap_s=1.0)** `nees_att`: 0.0743 -> 0.0126 (FAIL -> FAIL)
- **stops / cal_ideal** `nees_att`: 0.0735 -> 0.0124 (FAIL -> FAIL)
- **yaw_steps / none** `nees_att`: 0.0357 -> 0.00832 (FAIL -> FAIL)
- **yaw_steps / gps_dropout(duration_s=5)** `nees_att`: 0.0354 -> 0.0083 (FAIL -> FAIL)
- **yaw_steps / gps_dropout(duration_s=15)** `nees_att`: 0.0347 -> 0.00825 (FAIL -> FAIL)
- **yaw_steps / gps_dropout(duration_s=30)** `nees_att`: 0.0329 -> 0.00723 (FAIL -> FAIL)
- **yaw_steps / gps_stale(duration_s=5)** `nees_att`: 0.0357 -> 0.00832 (FAIL -> FAIL)
- **yaw_steps / gps_stale(duration_s=15)** `nees_att`: 0.0357 -> 0.00833 (FAIL -> FAIL)
- **yaw_steps / gps_stale(duration_s=30)** `nees_att`: 0.0357 -> 0.00834 (FAIL -> FAIL)
- **yaw_steps / gps_noise** `nees_att`: 0.11 -> 0.0813 (WARN -> FAIL)
- **yaw_steps / gps_outliers(frac=0.01)** `nees_att`: 0.0357 -> 0.00832 (FAIL -> FAIL)
- **yaw_steps / gps_rate(hz=1.0)** `nees_att`: 0.0347 -> 0.00821 (FAIL -> FAIL)
- **yaw_steps / gps_latency(latency_ms=100)** `nees_att`: 0.0358 -> 0.00834 (FAIL -> FAIL)
- **yaw_steps / gps_latency(latency_ms=200)** `nees_att`: 0.0361 -> 0.00836 (FAIL -> FAIL)
- **yaw_steps / gps_latency(ekf_latency_ms=0,latency_ms=200)** `nees_att`: 0.0357 -> 0.00832 (FAIL -> FAIL)
- **yaw_steps / gyro_bias(dps=0.2)** `nees_att`: 0.0357 -> 0.00832 (FAIL -> FAIL)
- **yaw_steps / gyro_bias(dps=1.0)** `nees_att`: 0.0357 -> 0.00832 (FAIL -> FAIL)
- **yaw_steps / gyro_bias(dps=1.0,from_cal=False)** `nees_att`: 0.0385 -> 0.011 (FAIL -> FAIL)
- **yaw_steps / gyro_drift(dps=0.5,over_s=60)** `nees_att`: 0.0356 -> 0.00834 (FAIL -> FAIL)
- **yaw_steps / accel_bias(axis=x,ms2=0.05)** `nees_pos`: 0.0346 -> 0.0149 (None -> None)
- **yaw_steps / accel_bias(axis=x,ms2=0.2)** `nees_pos`: 0.499 -> 0.25 (None -> None)
- **yaw_steps / accel_bias(axis=z,ms2=0.05)** `nees_att`: 0.0357 -> 0.00832 (FAIL -> FAIL)
- **yaw_steps / accel_bias(axis=z,ms2=0.2)** `nees_att`: 0.0357 -> 0.00832 (FAIL -> FAIL)
- **yaw_steps / accel_bias(axis=z,ms2=0.5)** `nees_att`: 0.0357 -> 0.00833 (FAIL -> FAIL)
- **yaw_steps / accel_bias(axis=x,from_cal=False,ms2=0.2)** `tilt_max_deg`: 1.43 -> 1.97 (PASS -> PASS)
- **yaw_steps / accel_bias(axis=x,from_cal=False,ms2=0.2)** `yaw_rms_deg`: 3.05 -> 4.03 (WARN -> WARN)
- **yaw_steps / accel_bias(axis=x,from_cal=False,ms2=0.2)** `nees_pos`: 1.04 -> 0.714 (None -> None)
- **yaw_steps / mag_interference(amp=0.3)** `nees_att`: 0.0302 -> 0.00779 (FAIL -> FAIL)
- **yaw_steps / mag_interference(amp=1.0)** `nees_att`: 0.0302 -> 0.00779 (FAIL -> FAIL)
- **yaw_steps / imu_noise** `nees_att`: 0.0361 -> 0.00872 (FAIL -> FAIL)
- **yaw_steps / imu_spikes** `nees_att`: 0.282 -> 0.212 (WARN -> WARN)
- **yaw_steps / imu_dropouts** `nees_att`: 0.0362 -> 0.0085 (FAIL -> FAIL)
- **yaw_steps / imu_gap(gap_s=0.2)** `nees_att`: 0.0356 -> 0.00829 (FAIL -> FAIL)
- **yaw_steps / imu_gap(gap_s=1.0)** `nees_att`: 0.0358 -> 0.00836 (FAIL -> FAIL)
- **yaw_steps / cal_ideal** `nees_att`: 0.0357 -> 0.00832 (FAIL -> FAIL)
- **yaw_spin / none** `nees_att`: 0.0713 -> 0.0115 (FAIL -> FAIL)
- **yaw_spin / gps_dropout(duration_s=5)** `nees_att`: 0.071 -> 0.0114 (FAIL -> FAIL)
- **yaw_spin / gps_dropout(duration_s=15)** `nees_att`: 0.0706 -> 0.0113 (FAIL -> FAIL)
- **yaw_spin / gps_dropout(duration_s=30)** `nees_att`: 0.0701 -> 0.0111 (FAIL -> FAIL)
- **yaw_spin / gps_stale(duration_s=5)** `nees_att`: 0.0713 -> 0.0115 (FAIL -> FAIL)
- **yaw_spin / gps_stale(duration_s=15)** `nees_att`: 0.0713 -> 0.0115 (FAIL -> FAIL)
- **yaw_spin / gps_stale(duration_s=30)** `nees_att`: 0.0713 -> 0.0115 (FAIL -> FAIL)
- **yaw_spin / gps_noise** `nees_att`: 0.135 -> 0.075 (WARN -> FAIL)
- **yaw_spin / gps_outliers(frac=0.01)** `nees_att`: 0.0713 -> 0.0115 (FAIL -> FAIL)
- **yaw_spin / gps_rate(hz=1.0)** `nees_att`: 0.0696 -> 0.011 (FAIL -> FAIL)
- **yaw_spin / gps_latency(latency_ms=100)** `nees_att`: 0.0713 -> 0.0115 (FAIL -> FAIL)
- **yaw_spin / gps_latency(latency_ms=200)** `nees_att`: 0.0713 -> 0.0115 (FAIL -> FAIL)
- **yaw_spin / gps_latency(ekf_latency_ms=0,latency_ms=200)** `nees_att`: 0.0713 -> 0.0115 (FAIL -> FAIL)
- **yaw_spin / gyro_bias(dps=0.2)** `nees_att`: 0.0713 -> 0.0115 (FAIL -> FAIL)
- **yaw_spin / gyro_bias(dps=1.0)** `nees_att`: 0.0713 -> 0.0115 (FAIL -> FAIL)
- **yaw_spin / gyro_bias(dps=1.0,from_cal=False)** `nees_att`: 0.0736 -> 0.0137 (FAIL -> FAIL)
- **yaw_spin / gyro_drift(dps=0.5,over_s=60)** `nees_att`: 0.0708 -> 0.0115 (FAIL -> FAIL)
- **yaw_spin / accel_bias(axis=x,ms2=0.05)** `nees_att`: 0.681 -> 0.443 (PASS -> PASS)
- **yaw_spin / accel_bias(axis=x,ms2=0.05)** `nees_vel`: 0.628 -> 0.383 (None -> None)
- **yaw_spin / accel_bias(axis=x,ms2=0.05)** `nees_pos`: 0.00816 -> 0.00111 (None -> None)
- **yaw_spin / accel_bias(axis=x,ms2=0.2)** `nees_pos`: 0.0506 -> 0.017 (None -> None)
- **yaw_spin / accel_bias(axis=x,ms2=0.5)** `tilt_rms_deg`: 2.55 -> 3.07 (WARN -> FAIL)
- **yaw_spin / accel_bias(axis=x,ms2=0.5)** `tilt_max_deg`: 3.93 -> 5.4 (WARN -> WARN)
- **yaw_spin / accel_bias(axis=x,ms2=0.5)** `yaw_rms_deg`: 3.24 -> 4.76 (WARN -> WARN)
- **yaw_spin / accel_bias(axis=x,ms2=0.5)** `mag_rejected`: 0 -> 1061 (None -> None)
- **yaw_spin / accel_bias(axis=z,ms2=0.05)** `nees_att`: 0.0713 -> 0.0115 (FAIL -> FAIL)
- **yaw_spin / accel_bias(axis=z,ms2=0.2)** `nees_att`: 0.0713 -> 0.0115 (FAIL -> FAIL)
- **yaw_spin / accel_bias(axis=z,ms2=0.5)** `nees_att`: 0.0713 -> 0.0115 (FAIL -> FAIL)
- **yaw_spin / accel_bias(axis=x,from_cal=False,ms2=0.2)** `nees_pos`: 0.332 -> 0.238 (None -> None)
- **yaw_spin / mag_bias(frac=0.05)** `nees_att`: 0.179 -> 0.11 (WARN -> WARN)
- **yaw_spin / mag_interference(amp=0.3)** `nees_att`: 0.0651 -> 0.0104 (FAIL -> FAIL)
- **yaw_spin / mag_interference(amp=1.0)** `nees_att`: 0.0651 -> 0.0104 (FAIL -> FAIL)
- **yaw_spin / imu_noise** `nees_att`: 0.0726 -> 0.0124 (FAIL -> FAIL)
- **yaw_spin / imu_spikes** `nees_att`: 0.319 -> 0.216 (PASS -> WARN)
- **yaw_spin / imu_dropouts** `nees_att`: 0.071 -> 0.0118 (FAIL -> FAIL)
- **yaw_spin / imu_gap(gap_s=0.2)** `nees_att`: 0.071 -> 0.0114 (FAIL -> FAIL)
- **yaw_spin / imu_gap(gap_s=1.0)** `nees_att`: 0.0708 -> 0.0114 (FAIL -> FAIL)
- **yaw_spin / combined_realistic** `nees_att`: 0.894 -> 0.648 (PASS -> PASS)
- **yaw_spin / combined_realistic** `nees_vel`: 0.613 -> 0.431 (PASS -> PASS)
- **yaw_spin / cal_ideal** `nees_att`: 0.0713 -> 0.0115 (FAIL -> FAIL)
- **patrol_long / imu_spikes** `yaw_rms_deg`: 0.625 -> 0.835 (PASS -> PASS)
- **takeoff_land / accel_bias(axis=x,from_cal=False,ms2=0.2)** `yaw_rms_deg`: 6.55 -> 8.69 (FAIL -> FAIL)
- **takeoff_land / accel_bias(axis=x,from_cal=False,ms2=0.2)** `nees_pos`: 0.972 -> 0.691 (None -> None)
- **hover_ct / none** `nees_att`: 0.0134 -> 0.00207 (FAIL -> FAIL)
- **hover_ct / gps_dropout(duration_s=5)** `nees_att`: 0.0134 -> 0.00207 (FAIL -> FAIL)
- **hover_ct / gps_dropout(duration_s=15)** `nees_att`: 0.0134 -> 0.00206 (FAIL -> FAIL)
- **hover_ct / gps_dropout(duration_s=30)** `nees_att`: 0.0134 -> 0.00206 (FAIL -> FAIL)
- **hover_ct / gps_stale(duration_s=5)** `nees_att`: 0.0134 -> 0.00207 (FAIL -> FAIL)
- **hover_ct / gps_stale(duration_s=15)** `nees_att`: 0.0134 -> 0.00207 (FAIL -> FAIL)
- **hover_ct / gps_stale(duration_s=30)** `nees_att`: 0.0134 -> 0.00207 (FAIL -> FAIL)
- **hover_ct / gps_outliers(frac=0.01)** `nees_att`: 0.0134 -> 0.00207 (FAIL -> FAIL)
- **hover_ct / gps_rate(hz=1.0)** `nees_att`: 0.0132 -> 0.00197 (FAIL -> FAIL)
- **hover_ct / gps_latency(latency_ms=100)** `nees_att`: 0.0134 -> 0.00209 (FAIL -> FAIL)
- **hover_ct / gps_latency(latency_ms=200)** `nees_att`: 0.0134 -> 0.0021 (FAIL -> FAIL)
- **hover_ct / gps_latency(ekf_latency_ms=0,latency_ms=200)** `nees_att`: 0.0134 -> 0.00206 (FAIL -> FAIL)
- **hover_ct / gyro_bias(dps=0.2)** `nees_att`: 0.0134 -> 0.00207 (FAIL -> FAIL)
- **hover_ct / gyro_bias(dps=1.0)** `nees_att`: 0.0134 -> 0.00207 (FAIL -> FAIL)
- **hover_ct / gyro_bias(dps=1.0,from_cal=False)** `nees_att`: 0.0153 -> 0.00393 (FAIL -> FAIL)
- **hover_ct / gyro_drift(dps=0.5,over_s=60)** `nees_att`: 0.0132 -> 0.00201 (FAIL -> FAIL)
- **hover_ct / accel_bias(axis=x,ms2=0.05)** `nees_vel`: 0.522 -> 0.152 (None -> None)
- **hover_ct / accel_bias(axis=x,ms2=0.05)** `nees_pos`: 0.0419 -> 0.00805 (None -> None)
- **hover_ct / accel_bias(axis=x,ms2=0.2)** `tilt_rms_deg`: 0.944 -> 1.04 (PASS -> WARN)
- **hover_ct / accel_bias(axis=x,ms2=0.2)** `nees_pos`: 0.632 -> 0.118 (None -> None)
- **hover_ct / accel_bias(axis=x,ms2=0.5)** `nees_pos`: 0.277 -> 0.0349 (None -> None)
- **hover_ct / accel_bias(axis=z,ms2=0.05)** `nees_att`: 0.0134 -> 0.00207 (FAIL -> FAIL)
- **hover_ct / accel_bias(axis=z,ms2=0.2)** `nees_att`: 0.0134 -> 0.00207 (FAIL -> FAIL)
- **hover_ct / accel_bias(axis=z,ms2=0.5)** `nees_att`: 0.0134 -> 0.00207 (FAIL -> FAIL)
- **hover_ct / accel_bias(axis=x,from_cal=False,ms2=0.2)** `yaw_rms_deg`: 1.52 -> 2.02 (PASS -> WARN)
- **hover_ct / mag_bias(frac=0.05)** `nees_att`: 0.113 -> 0.0944 (WARN -> FAIL)
- **hover_ct / mag_interference(amp=0.3)** `nees_att`: 0.0133 -> 0.00202 (FAIL -> FAIL)
- **hover_ct / mag_interference(amp=1.0)** `nees_att`: 0.0133 -> 0.00202 (FAIL -> FAIL)
- **hover_ct / imu_noise** `nees_att`: 0.0144 -> 0.00337 (FAIL -> FAIL)
- **hover_ct / imu_dropouts** `nees_att`: 0.0134 -> 0.0021 (FAIL -> FAIL)
- **hover_ct / imu_gap(gap_s=0.2)** `nees_att`: 0.0134 -> 0.00207 (FAIL -> FAIL)
- **hover_ct / imu_gap(gap_s=1.0)** `nees_att`: 0.0135 -> 0.00209 (FAIL -> FAIL)
- **hover_ct / combined_realistic** `nees_vel`: 0.893 -> 0.507 (PASS -> PASS)
- **hover_ct / cal_ideal** `nees_att`: 0.0134 -> 0.00207 (FAIL -> FAIL)
- **box_ct / none** `nees_att`: 0.0141 -> 0.0024 (FAIL -> FAIL)
- **box_ct / gps_dropout(duration_s=5)** `nees_att`: 0.0141 -> 0.00241 (FAIL -> FAIL)
- **box_ct / gps_dropout(duration_s=15)** `nees_att`: 0.0141 -> 0.00239 (FAIL -> FAIL)
- **box_ct / gps_dropout(duration_s=30)** `nees_att`: 0.0141 -> 0.00238 (FAIL -> FAIL)
- **box_ct / gps_outliers(frac=0.01)** `nees_att`: 0.0141 -> 0.0024 (FAIL -> FAIL)
- **box_ct / gps_rate(hz=1.0)** `nees_att`: 0.0139 -> 0.00237 (FAIL -> FAIL)
- **box_ct / gps_latency(latency_ms=100)** `nees_att`: 0.0141 -> 0.00244 (FAIL -> FAIL)
- **box_ct / gps_latency(latency_ms=200)** `nees_att`: 0.0141 -> 0.00249 (FAIL -> FAIL)
- **box_ct / gps_latency(ekf_latency_ms=0,latency_ms=200)** `nees_att`: 0.0196 -> 0.008 (FAIL -> FAIL)
- **box_ct / gyro_bias(dps=0.2)** `nees_att`: 0.0141 -> 0.0024 (FAIL -> FAIL)
- **box_ct / gyro_bias(dps=1.0)** `nees_att`: 0.0141 -> 0.0024 (FAIL -> FAIL)
- **box_ct / gyro_bias(dps=1.0,from_cal=False)** `nees_att`: 0.0162 -> 0.00429 (FAIL -> FAIL)
- **box_ct / gyro_drift(dps=0.5,over_s=60)** `nees_att`: 0.0139 -> 0.00233 (FAIL -> FAIL)
- **box_ct / accel_bias(axis=x,ms2=0.05)** `nees_vel`: 0.529 -> 0.176 (None -> None)
- **box_ct / accel_bias(axis=x,ms2=0.05)** `nees_pos`: 0.0383 -> 0.00679 (None -> None)
- **box_ct / accel_bias(axis=x,ms2=0.2)** `tilt_rms_deg`: 0.944 -> 1.04 (PASS -> WARN)
- **box_ct / accel_bias(axis=x,ms2=0.2)** `nees_pos`: 0.62 -> 0.113 (None -> None)
- **box_ct / accel_bias(axis=x,ms2=0.5)** `nees_pos`: 0.294 -> 0.0368 (None -> None)
- **box_ct / accel_bias(axis=z,ms2=0.05)** `nees_att`: 0.0141 -> 0.0024 (FAIL -> FAIL)
- **box_ct / accel_bias(axis=z,ms2=0.2)** `nees_att`: 0.0141 -> 0.0024 (FAIL -> FAIL)
- **box_ct / accel_bias(axis=z,ms2=0.5)** `nees_att`: 0.0141 -> 0.0024 (FAIL -> FAIL)
- **box_ct / accel_bias(axis=x,from_cal=False,ms2=0.2)** `yaw_rms_deg`: 1.55 -> 2.05 (PASS -> WARN)
- **box_ct / mag_bias(frac=0.05)** `nees_att`: 0.115 -> 0.0954 (WARN -> FAIL)
- **box_ct / mag_interference(amp=0.3)** `nees_att`: 0.0139 -> 0.00233 (FAIL -> FAIL)
- **box_ct / mag_interference(amp=1.0)** `nees_att`: 0.0139 -> 0.00233 (FAIL -> FAIL)
- **box_ct / imu_noise** `nees_att`: 0.0152 -> 0.00387 (FAIL -> FAIL)
- **box_ct / imu_spikes** `nees_att`: 0.329 -> 0.275 (PASS -> WARN)
- **box_ct / imu_dropouts** `nees_att`: 0.0141 -> 0.00246 (FAIL -> FAIL)
- **box_ct / imu_gap(gap_s=0.2)** `nees_att`: 0.0141 -> 0.00242 (FAIL -> FAIL)
- **box_ct / imu_gap(gap_s=1.0)** `nees_att`: 0.0142 -> 0.00242 (FAIL -> FAIL)
- **box_ct / combined_realistic** `nees_vel`: 0.843 -> 0.583 (PASS -> PASS)
- **box_ct / cal_ideal** `nees_att`: 0.0141 -> 0.0024 (FAIL -> FAIL)
- **ref_live1 / none** `nees_att`: 0.184 -> 0.0873 (WARN -> FAIL)

## Overall grade changes

- circle_slow / combined_realistic: WARN -> PASS
- circle_fast / gps_stale(duration_s=5): WARN -> FAIL
- circle_fast / gps_latency(latency_ms=200): WARN -> FAIL
- circle_fast / imu_spikes: PASS -> WARN
- stops / gps_stale(duration_s=30): WARN -> FAIL
- yaw_steps / gps_noise: WARN -> FAIL
- yaw_spin / gps_noise: WARN -> FAIL
- yaw_spin / imu_spikes: PASS -> WARN
- hover_ct / mag_bias(frac=0.05): WARN -> FAIL
- box_ct / mag_bias(frac=0.05): WARN -> FAIL
- box_ct / imu_spikes: PASS -> WARN
- ref_live1 / none: WARN -> FAIL

## Improvements

- hover / gps_dropout(duration_s=30) `tilt_rms_deg`: 0.105 -> 0.0355 (PASS -> PASS)
- hover / gps_dropout(duration_s=30) `tilt_max_deg`: 0.719 -> 0.183 (PASS -> PASS)
- hover / gps_dropout(duration_s=30) `yaw_rms_deg`: 0.213 -> 0.0782 (PASS -> PASS)
- hover / gps_dropout(duration_s=30) `pos_h_rms_m`: 1.63 -> 0.553 (None -> None)
- hover / gps_dropout(duration_s=30) `vel_h_rms_ms`: 0.202 -> 0.0682 (None -> None)
- hover / accel_bias(axis=x,ms2=0.05) `nees_att`: 2.05 -> 1.16 (PASS -> PASS)
- hover / combined_realistic `nees_att`: 2.73 -> 1.86 (PASS -> PASS)
- box / accel_bias(axis=x,ms2=0.05) `nees_att`: 2.02 -> 1.15 (PASS -> PASS)
- box / accel_bias(axis=x,ms2=0.2) `yaw_rms_deg`: 0.406 -> 0.221 (PASS -> PASS)
- box / accel_bias(axis=x,ms2=0.5) `yaw_rms_deg`: 2.12 -> 0.655 (WARN -> PASS)
- box / combined_realistic `nees_att`: 3.58 -> 2.19 (WARN -> PASS)
- circle_slow / gps_latency(ekf_latency_ms=0,latency_ms=200) `nees_att`: 0.374 -> 0.659 (PASS -> PASS)
- circle_slow / accel_bias(axis=x,ms2=0.05) `nees_att`: 2 -> 1.14 (PASS -> PASS)
- circle_slow / accel_bias(axis=x,ms2=0.5) `yaw_rms_deg`: 1.04 -> 0.655 (PASS -> PASS)
- circle_slow / combined_realistic `nees_att`: 3.1 -> 1.97 (WARN -> PASS)
- circle_fast / gps_dropout(duration_s=15) `tilt_max_deg`: 0.825 -> 0.449 (PASS -> PASS)
- circle_fast / gps_dropout(duration_s=15) `pos_h_rms_m`: 0.568 -> 0.316 (None -> None)
- circle_fast / gps_dropout(duration_s=15) `vel_h_rms_ms`: 0.176 -> 0.136 (None -> None)
- circle_fast / gps_dropout(duration_s=30) `tilt_rms_deg`: 0.498 -> 0.219 (PASS -> PASS)
- circle_fast / gps_dropout(duration_s=30) `tilt_max_deg`: 1.67 -> 0.554 (PASS -> PASS)
- circle_fast / gps_dropout(duration_s=30) `yaw_rms_deg`: 0.931 -> 0.431 (PASS -> PASS)
- circle_fast / gps_dropout(duration_s=30) `pos_h_rms_m`: 10.4 -> 3.43 (None -> None)
- circle_fast / gps_dropout(duration_s=30) `vel_h_rms_ms`: 1.12 -> 0.39 (None -> None)
- circle_fast / gps_stale(duration_s=5) `nees_att`: 0.131 -> 1.9 (WARN -> PASS)
- circle_fast / gps_stale(duration_s=15) `nees_pos`: 0.387 -> 0.509 (None -> None)
- circle_fast / gps_stale(duration_s=30) `tilt_drift_deg_s`: 0.26 -> 0.123 (PASS -> PASS)
- circle_fast / accel_bias(axis=x,ms2=0.05) `nees_att`: 1.57 -> 1.02 (PASS -> PASS)
- circle_fast / accel_bias(axis=x,ms2=0.2) `yaw_rms_deg`: 0.824 -> 0.585 (PASS -> PASS)
- circle_fast / accel_bias(axis=x,ms2=0.2) `pos_h_rms_m`: 0.192 -> 0.0949 (PASS -> PASS)
- circle_fast / accel_bias(axis=x,ms2=0.2) `nees_vel`: 3.15 -> 1.77 (None -> None)
- circle_fast / accel_bias(axis=x,ms2=0.5) `yaw_rms_deg`: 1.81 -> 1.22 (PASS -> PASS)
- circle_fast / accel_bias(axis=x,ms2=0.5) `pos_h_rms_m`: 0.458 -> 0.197 (WARN -> PASS)
- circle_fast / accel_bias(axis=x,ms2=0.5) `vel_h_rms_ms`: 0.23 -> 0.151 (None -> None)
- circle_fast / accel_bias(axis=x,ms2=0.5) `nees_vel`: 10.5 -> 3.19 (None -> None)
- circle_fast / accel_bias(axis=x,from_cal=False,ms2=0.2) `tilt_max_deg`: 2.82 -> 2.03 (PASS -> PASS)
- circle_fast / accel_bias(axis=x,from_cal=False,ms2=0.2) `nees_pos`: 1.5 -> 1.05 (None -> None)
- circle_fast / combined_realistic `nees_att`: 1.97 -> 1.41 (PASS -> PASS)
- stops / gps_dropout(duration_s=30) `pos_h_rms_m`: 1.95 -> 1.24 (None -> None)
- stops / gps_dropout(duration_s=30) `vel_h_rms_ms`: 0.196 -> 0.142 (None -> None)
- stops / accel_bias(axis=x,ms2=0.05) `tilt_max_deg`: 0.802 -> 0.488 (PASS -> PASS)
- stops / accel_bias(axis=x,ms2=0.05) `yaw_rms_deg`: 0.999 -> 0.453 (PASS -> PASS)
- stops / accel_bias(axis=x,ms2=0.05) `nees_att`: 2.14 -> 1.11 (PASS -> PASS)
- stops / accel_bias(axis=x,ms2=0.2) `tilt_max_deg`: 3.27 -> 2.09 (WARN -> PASS)
- stops / accel_bias(axis=x,ms2=0.2) `yaw_rms_deg`: 4.17 -> 2.03 (WARN -> WARN)
- stops / accel_bias(axis=x,ms2=0.2) `pos_h_rms_m`: 0.173 -> 0.0888 (PASS -> PASS)
- stops / accel_bias(axis=x,ms2=0.2) `vel_h_rms_ms`: 0.121 -> 0.0922 (None -> None)
- stops / accel_bias(axis=x,ms2=0.2) `nees_att`: 38.9 -> 17.8 (FAIL -> FAIL)
- stops / accel_bias(axis=x,ms2=0.2) `nees_vel`: 2.89 -> 1.88 (None -> None)
- stops / accel_bias(axis=x,ms2=0.5) `yaw_rms_deg`: 8.23 -> 5.77 (FAIL -> FAIL)
- stops / accel_bias(axis=x,ms2=0.5) `pos_h_rms_m`: 0.37 -> 0.165 (WARN -> PASS)
- stops / accel_bias(axis=x,ms2=0.5) `vel_h_rms_ms`: 0.202 -> 0.14 (None -> None)
- stops / accel_bias(axis=x,ms2=0.5) `nees_vel`: 6.86 -> 3 (None -> None)
- stops / accel_bias(axis=x,from_cal=False,ms2=0.2) `pos_h_rms_m`: 0.536 -> 0.415 (WARN -> WARN)
- stops / combined_realistic `yaw_rms_deg`: 2.89 -> 2.24 (WARN -> WARN)
- stops / combined_realistic `nees_att`: 4.78 -> 2.1 (WARN -> PASS)
- yaw_steps / gps_dropout(duration_s=15) `tilt_max_deg`: 0.359 -> 0.157 (PASS -> PASS)
- yaw_steps / gps_dropout(duration_s=15) `pos_h_rms_m`: 0.131 -> 0.0437 (None -> None)
- yaw_steps / gps_dropout(duration_s=15) `vel_h_rms_ms`: 0.0396 -> 0.0151 (None -> None)
- yaw_steps / gps_dropout(duration_s=30) `tilt_rms_deg`: 0.192 -> 0.0653 (PASS -> PASS)
- yaw_steps / gps_dropout(duration_s=30) `tilt_max_deg`: 0.706 -> 0.255 (PASS -> PASS)
- yaw_steps / gps_dropout(duration_s=30) `yaw_rms_deg`: 0.405 -> 0.144 (PASS -> PASS)
- yaw_steps / gps_dropout(duration_s=30) `pos_h_rms_m`: 3.09 -> 0.951 (None -> None)
- yaw_steps / gps_dropout(duration_s=30) `vel_h_rms_ms`: 0.375 -> 0.116 (None -> None)
- yaw_steps / accel_bias(axis=x,ms2=0.05) `vel_h_rms_ms`: 0.0828 -> 0.0612 (None -> None)
- yaw_steps / accel_bias(axis=x,ms2=0.05) `nees_vel`: 1.86 -> 0.897 (None -> None)
- yaw_steps / accel_bias(axis=x,ms2=0.2) `yaw_rms_deg`: 0.727 -> 0.566 (PASS -> PASS)
- yaw_steps / accel_bias(axis=x,ms2=0.2) `pos_h_rms_m`: 0.339 -> 0.24 (WARN -> PASS)
- yaw_steps / accel_bias(axis=x,ms2=0.2) `vel_h_rms_ms`: 0.329 -> 0.248 (None -> None)
- yaw_steps / accel_bias(axis=x,ms2=0.2) `nees_vel`: 29.4 -> 14.8 (None -> None)
- yaw_steps / accel_bias(axis=x,ms2=0.5) `mag_rejected`: 10886 -> 4361 (None -> None)
- yaw_steps / combined_realistic `nees_att`: 1.58 -> 1.07 (PASS -> PASS)
- yaw_steps / combined_realistic `nees_vel`: 1.6 -> 0.819 (PASS -> PASS)
- yaw_spin / accel_bias(axis=x,ms2=0.05) `yaw_rms_deg`: 0.373 -> 0.266 (PASS -> PASS)
- yaw_spin / accel_bias(axis=x,ms2=0.2) `yaw_rms_deg`: 1.39 -> 1.06 (PASS -> PASS)
- yaw_spin / accel_bias(axis=x,ms2=0.5) `gps_resets`: 9 -> 6 (FAIL -> FAIL)
- yaw_spin / accel_bias(axis=x,ms2=0.5) `gps_rejected`: 45 -> 30 (None -> None)
- yaw_spin / mag_bias(frac=0.15) `nees_att`: 2.31 -> 1.77 (PASS -> PASS)
- patrol_long / gps_dropout(duration_s=30) `pos_h_rms_m`: 0.711 -> 0.347 (None -> None)
- patrol_long / gps_dropout(duration_s=30) `vel_h_rms_ms`: 0.0966 -> 0.0568 (None -> None)
- patrol_long / accel_bias(axis=x,ms2=0.05) `nees_att`: 2.05 -> 1.16 (PASS -> PASS)
- patrol_long / imu_spikes `nees_att`: 0.33 -> 0.428 (PASS -> PASS)
- patrol_long / combined_realistic `nees_att`: 2.93 -> 1.9 (PASS -> PASS)
- takeoff_land / gps_dropout(duration_s=15) `pos_h_rms_m`: 0.0728 -> 0.0165 (None -> None)
- takeoff_land / accel_bias(axis=x,ms2=0.05) `nees_att`: 2.03 -> 1.15 (PASS -> PASS)
- takeoff_land / combined_realistic `nees_att`: 2.66 -> 1.8 (PASS -> PASS)
- hover_ct / gps_dropout(duration_s=30) `tilt_rms_deg`: 0.0885 -> 0.0353 (PASS -> PASS)
- hover_ct / gps_dropout(duration_s=30) `tilt_max_deg`: 0.495 -> 0.22 (PASS -> PASS)
- hover_ct / gps_dropout(duration_s=30) `yaw_rms_deg`: 0.19 -> 0.0792 (PASS -> PASS)
- hover_ct / gps_dropout(duration_s=30) `pos_h_rms_m`: 1.21 -> 0.483 (None -> None)
- hover_ct / gps_dropout(duration_s=30) `vel_h_rms_ms`: 0.158 -> 0.0628 (None -> None)
- hover_ct / accel_bias(axis=x,ms2=0.05) `pos_h_rms_m`: 0.098 -> 0.0422 (PASS -> PASS)
- hover_ct / accel_bias(axis=x,ms2=0.2) `pos_h_rms_m`: 0.382 -> 0.165 (WARN -> PASS)
- hover_ct / accel_bias(axis=x,ms2=0.2) `vel_h_rms_ms`: 0.173 -> 0.1 (None -> None)
- hover_ct / accel_bias(axis=x,ms2=0.2) `nees_vel`: 8.07 -> 2.38 (None -> None)
- hover_ct / accel_bias(axis=x,ms2=0.5) `yaw_rms_deg`: 5.03 -> 4.68 (FAIL -> WARN)
- hover_ct / accel_bias(axis=x,ms2=0.5) `pos_h_rms_m`: 0.258 -> 0.0919 (PASS -> PASS)
- hover_ct / accel_bias(axis=x,ms2=0.5) `vel_h_rms_ms`: 0.162 -> 0.116 (None -> None)
- hover_ct / accel_bias(axis=x,ms2=0.5) `nees_vel`: 6.03 -> 2.19 (None -> None)
- hover_ct / accel_bias(axis=x,ms2=0.5) `mag_rejected`: 1213 -> 475 (None -> None)
- hover_ct / accel_bias(axis=x,from_cal=False,ms2=0.2) `pos_h_rms_m`: 0.717 -> 0.547 (WARN -> WARN)
- hover_ct / accel_bias(axis=x,from_cal=False,ms2=0.2) `nees_pos`: 2.22 -> 1.29 (None -> None)
- hover_ct / mag_bias(frac=0.15) `nees_att`: 3.62 -> 2.98 (WARN -> PASS)
- hover_ct / combined_realistic `nees_att`: 1.44 -> 1.06 (PASS -> PASS)
- box_ct / gps_dropout(duration_s=30) `tilt_max_deg`: 0.605 -> 0.249 (PASS -> PASS)
- box_ct / gps_dropout(duration_s=30) `yaw_rms_deg`: 0.232 -> 0.122 (PASS -> PASS)
- box_ct / gps_dropout(duration_s=30) `pos_h_rms_m`: 2.07 -> 1.28 (None -> None)
- box_ct / gps_dropout(duration_s=30) `vel_h_rms_ms`: 0.24 -> 0.14 (None -> None)
- box_ct / accel_bias(axis=x,ms2=0.05) `pos_h_rms_m`: 0.0938 -> 0.0386 (PASS -> PASS)
- box_ct / accel_bias(axis=x,ms2=0.2) `pos_h_rms_m`: 0.379 -> 0.161 (WARN -> PASS)
- box_ct / accel_bias(axis=x,ms2=0.2) `vel_h_rms_ms`: 0.172 -> 0.1 (None -> None)
- box_ct / accel_bias(axis=x,ms2=0.2) `nees_vel`: 7.99 -> 2.37 (None -> None)
- box_ct / accel_bias(axis=x,ms2=0.5) `yaw_rms_deg`: 5.01 -> 4.67 (FAIL -> WARN)
- box_ct / accel_bias(axis=x,ms2=0.5) `pos_h_rms_m`: 0.265 -> 0.0932 (PASS -> PASS)
- box_ct / accel_bias(axis=x,ms2=0.5) `vel_h_rms_ms`: 0.166 -> 0.118 (None -> None)
- box_ct / accel_bias(axis=x,ms2=0.5) `nees_vel`: 6.32 -> 2.32 (None -> None)
- box_ct / accel_bias(axis=x,ms2=0.5) `mag_rejected`: 1200 -> 470 (None -> None)
- box_ct / accel_bias(axis=x,from_cal=False,ms2=0.2) `pos_h_rms_m`: 0.713 -> 0.543 (WARN -> WARN)
- box_ct / accel_bias(axis=x,from_cal=False,ms2=0.2) `nees_pos`: 2.19 -> 1.27 (None -> None)
- box_ct / mag_bias(frac=0.15) `nees_att`: 3.6 -> 2.96 (WARN -> PASS)
- box_ct / combined_realistic `nees_att`: 1.51 -> 1.11 (PASS -> PASS)

## All matched runs (metrics that changed at all)

| mission | fault | overall | changes |
|---|---|---|---|
| hover | none | FAIL | tilt_rms_deg 0.00525->0.00467; tilt_max_deg 0.0155->0.0136; yaw_rms_deg 0.0181->0.0189; pos_h_rms_m 0.00168->0.0016; vel_h_rms_ms 0.00112->0.0011; nees_att 0.000317->0.000155; nees_vel 0.00162->0.00159; nees_pos 0.000242->0.000241 |
| hover | gps_dropout(duration_s=5) | FAIL | tilt_rms_deg 0.00527->0.00469; tilt_max_deg 0.0155->0.0136; yaw_rms_deg 0.0181->0.0189; pos_h_rms_m 0.00243->0.00241; vel_h_rms_ms 0.0012->0.00117; nees_att 0.000318->0.000155; nees_vel 0.00174->0.0017; tilt_drift_deg_s -0.000108->-0.000115 |
| hover | gps_dropout(duration_s=15) | FAIL | tilt_rms_deg 0.00882->0.00548; tilt_max_deg 0.062->0.0211; yaw_rms_deg 0.0233->0.0215; pos_h_rms_m 0.0296->0.0135; vel_h_rms_ms 0.00705->0.00347; nees_att 0.000318->0.000155; nees_vel 0.00215->0.00211; nees_pos 0.000605->0.0006; tilt_drift_deg_s 0.0018->0.000428 |
| hover | gps_dropout(duration_s=30) | FAIL | tilt_rms_deg 0.105->0.0355 **BETTER**; tilt_max_deg 0.719->0.183 **BETTER**; yaw_rms_deg 0.213->0.0782 **BETTER**; pos_h_rms_m 1.63->0.553 **BETTER**; vel_h_rms_ms 0.202->0.0682 **BETTER**; nees_att 0.000324->0.000157; nees_vel 0.0032->0.00319; nees_pos 0.00158->0.00155; tilt_drift_deg_s 0.0119->0.00393 |
| hover | gps_stale(duration_s=5) | FAIL | tilt_rms_deg 0.00531->0.00473; tilt_max_deg 0.0155->0.0136; yaw_rms_deg 0.0181->0.0189; pos_h_rms_m 0.00172->0.00164; vel_h_rms_ms 0.00113->0.0011; nees_att 0.000318->0.000155; nees_vel 0.00162->0.00159; nees_pos 0.000244->0.000243; tilt_drift_deg_s -0.000178->-0.000253 |
| hover | gps_stale(duration_s=15) | FAIL | tilt_rms_deg 0.00531->0.00475; tilt_max_deg 0.0155->0.0136; yaw_rms_deg 0.0181->0.019; pos_h_rms_m 0.00242->0.00235; vel_h_rms_ms 0.00115->0.00113; nees_att 0.000318->0.000155; nees_vel 0.00164->0.0016; nees_pos 0.000257->0.000256; tilt_drift_deg_s -6e-06->3.1e-05 |
| hover | gps_stale(duration_s=30) | FAIL | tilt_rms_deg 0.00539->0.00483; tilt_max_deg 0.0156->0.0137; yaw_rms_deg 0.0182->0.019; pos_h_rms_m 0.0032->0.00312; vel_h_rms_ms 0.00121->0.00118; nees_att 0.000319->0.000156; nees_vel 0.00167->0.00163; nees_pos 0.000278->0.000276; tilt_drift_deg_s -3e-06->-1.4e-05 |
| hover | gps_noise | FAIL | tilt_rms_deg 0.344->0.344; tilt_max_deg 1.29->1.29; yaw_rms_deg 0.682->0.682; pos_h_rms_m 0.649->0.648; vel_h_rms_ms 0.0631->0.0632; nees_att 0.0762->0.0746; nees_vel 0.24->0.238; nees_pos 2.25->2.24 |
| hover | gps_outliers(frac=0.01) | FAIL | tilt_rms_deg 0.00525->0.00467; tilt_max_deg 0.0156->0.0136; yaw_rms_deg 0.0181->0.0189; pos_h_rms_m 0.00176->0.00166; vel_h_rms_ms 0.00113->0.0011; nees_att 0.000317->0.000155; nees_vel 0.00163->0.00159; nees_pos 0.000244->0.000243 |
| hover | gps_rate(hz=1.0) | FAIL | tilt_rms_deg 0.00551->0.00497; tilt_max_deg 0.0169->0.0157; yaw_rms_deg 0.0182->0.019; pos_h_rms_m 0.0063->0.00612; vel_h_rms_ms 0.0022->0.00218; nees_att 0.00033->0.000162; nees_vel 0.00266->0.00262; nees_pos 0.000366->0.000364 |
| hover | gps_latency(latency_ms=100) | FAIL | tilt_rms_deg 0.00537->0.0048; tilt_max_deg 0.0155->0.0137; yaw_rms_deg 0.0182->0.019; pos_h_rms_m 0.0021->0.00204; vel_h_rms_ms 0.00116->0.00114; nees_att 0.000318->0.000156; nees_vel 0.00159->0.00156; nees_pos 0.000214->0.000213 |
| hover | gps_latency(latency_ms=200) | FAIL | tilt_rms_deg 0.00548->0.00492; tilt_max_deg 0.0159->0.0142; yaw_rms_deg 0.0183->0.0191; pos_h_rms_m 0.00255->0.00252; vel_h_rms_ms 0.00121->0.00118; nees_att 0.000318->0.000156; nees_vel 0.0017->0.00167; nees_pos 0.000307->0.000306 |
| hover | gps_latency(ekf_latency_ms=0,latency_ms=200) | FAIL | tilt_rms_deg 0.00525->0.00467; tilt_max_deg 0.0154->0.0135; yaw_rms_deg 0.0181->0.0189; pos_h_rms_m 0.00168->0.0016; vel_h_rms_ms 0.00112->0.0011; nees_att 0.000317->0.000155; nees_vel 0.00141->0.00137; nees_pos 0.000313->0.000312 |
| hover | gyro_bias(dps=0.2) | FAIL | tilt_rms_deg 0.00525->0.00467; tilt_max_deg 0.0155->0.0136; yaw_rms_deg 0.0181->0.0189; pos_h_rms_m 0.00168->0.0016; vel_h_rms_ms 0.00112->0.0011; nees_att 0.000317->0.000155; nees_vel 0.00162->0.00159; nees_pos 0.000242->0.000241 |
| hover | gyro_bias(dps=1.0) | FAIL | tilt_rms_deg 0.00525->0.00467; tilt_max_deg 0.0155->0.0136; yaw_rms_deg 0.0181->0.0189; pos_h_rms_m 0.00168->0.0016; vel_h_rms_ms 0.00112->0.0011; nees_att 0.000317->0.000155; nees_vel 0.00162->0.00159; nees_pos 0.000242->0.000241 |
| hover | gyro_bias(dps=1.0,from_cal=False) | FAIL | tilt_rms_deg 0.0559->0.0558; tilt_max_deg 0.603->0.603; yaw_rms_deg 0.239->0.22; pos_h_rms_m 0.0022->0.00212; vel_h_rms_ms 0.00796->0.00795; nees_att 0.00502->0.00417; nees_vel 0.00401->0.00397; nees_pos 0.000246->0.000244 |
| hover | gyro_drift(dps=0.5,over_s=60) | FAIL | tilt_rms_deg 0.00577->0.00522; tilt_max_deg 0.0179->0.0161; yaw_rms_deg 0.0134->0.0141; pos_h_rms_m 0.00169->0.00161; vel_h_rms_ms 0.00114->0.00112; nees_att 0.000337->0.000173; nees_vel 0.00162->0.00159; nees_pos 0.000242->0.000241 |
| hover | accel_bias(axis=x,ms2=0.05) | PASS | tilt_rms_deg 0.294->0.294; tilt_max_deg 0.306->0.305; yaw_rms_deg 0.0438->0.0446; pos_h_rms_m 0.0018->0.00169; vel_h_rms_ms 0.00124->0.00121; nees_att 2.05->1.16 **BETTER**; nees_vel 0.00169->0.00164; nees_pos 0.000243->0.000241 |
| hover | accel_bias(axis=x,ms2=0.2) | FAIL | tilt_rms_deg 1.17->1.17; tilt_max_deg 1.18->1.18; yaw_rms_deg 0.126->0.127; pos_h_rms_m 0.00262->0.00242; vel_h_rms_ms 0.00248->0.00241; nees_att 32.6->18.5; nees_vel 0.00256->0.00241; nees_pos 0.000244->0.000239 |
| hover | accel_bias(axis=x,ms2=0.5) | FAIL | tilt_rms_deg 2.92->2.92; tilt_max_deg 2.94->2.93; yaw_rms_deg 0.312->0.312; pos_h_rms_m 0.00496->0.00461; vel_h_rms_ms 0.00568->0.00553; nees_att 203->115; nees_vel 0.00741->0.00675; nees_pos 0.000245->0.000231 |
| hover | accel_bias(axis=z,ms2=0.05) | FAIL | tilt_rms_deg 0.00525->0.00467; tilt_max_deg 0.0155->0.0136; yaw_rms_deg 0.0181->0.0189; pos_h_rms_m 0.00168->0.0016; vel_h_rms_ms 0.00112->0.0011; nees_att 0.000317->0.000155; nees_vel 0.00041->0.000374; nees_pos 4.3e-05->4.2e-05 |
| hover | accel_bias(axis=z,ms2=0.2) | FAIL | tilt_rms_deg 0.00525->0.00467; tilt_max_deg 0.0155->0.0136; yaw_rms_deg 0.0182->0.019; pos_h_rms_m 0.00168->0.0016; vel_h_rms_ms 0.00112->0.0011; nees_att 0.000317->0.000155; nees_vel 0.012->0.0119; nees_pos 0.00147->0.00146 |
| hover | accel_bias(axis=z,ms2=0.5) | FAIL | tilt_rms_deg 0.00525->0.00468; tilt_max_deg 0.0154->0.0135; yaw_rms_deg 0.0182->0.019; pos_h_rms_m 0.00168->0.0016; vel_h_rms_ms 0.00112->0.0011; nees_att 0.000317->0.000155; nees_vel 0.103->0.103; nees_pos 0.0134->0.0134 |
| hover | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | tilt_rms_deg 0.835->0.859; tilt_max_deg 1.02->1.02; yaw_rms_deg 6.53->8.56 **WORSE**; pos_h_rms_m 0.434->0.366; vel_h_rms_ms 0.245->0.215; nees_att 90.9->108; nees_vel 14.1->9.78; nees_pos 0.804->0.57 **WORSE** |
| hover | mag_bias(frac=0.05) | WARN | tilt_rms_deg 0.00519->0.0046; tilt_max_deg 0.0159->0.0139; yaw_rms_deg 3.51->3.51; pos_h_rms_m 0.00171->0.00162; vel_h_rms_ms 0.00113->0.00111; nees_att 0.63->0.625; nees_vel 0.00163->0.00159; nees_pos 0.000242->0.000241 |
| hover | mag_bias(frac=0.15) | FAIL | tilt_rms_deg 0.00518->0.00457; tilt_max_deg 0.0165->0.0144; yaw_rms_deg 10->10; pos_h_rms_m 0.00174->0.00166; vel_h_rms_ms 0.00115->0.00113; nees_att 5.14->5.1; nees_vel 0.00163->0.0016; nees_pos 0.000243->0.000242 |
| hover | mag_interference(amp=0.3) | FAIL | tilt_rms_deg 0.00533->0.00478; tilt_max_deg 0.0155->0.0136; yaw_rms_deg 0.0202->0.0205; pos_h_rms_m 0.00161->0.00153; vel_h_rms_ms 0.00112->0.0011; nees_att 0.000298->0.000149; nees_vel 0.00162->0.00158; nees_pos 0.000241->0.00024; tilt_drift_deg_s -0.000879->-0.000855 |
| hover | mag_interference(amp=1.0) | FAIL | tilt_rms_deg 0.00533->0.00478; tilt_max_deg 0.0155->0.0136; yaw_rms_deg 0.0202->0.0205; pos_h_rms_m 0.00161->0.00153; vel_h_rms_ms 0.00112->0.0011; nees_att 0.000298->0.000149; nees_vel 0.00162->0.00158; nees_pos 0.000241->0.00024; tilt_drift_deg_s -0.000879->-0.000855 |
| hover | imu_noise | FAIL | tilt_rms_deg 0.0239->0.024; tilt_max_deg 0.0658->0.0663; yaw_rms_deg 0.0469->0.0474; pos_h_rms_m 0.00184->0.00174; vel_h_rms_ms 0.00364->0.0036; nees_att 0.000957->0.000746; nees_vel 0.00696->0.0067; nees_pos 0.000371->0.000369 |
| hover | imu_spikes | WARN | tilt_rms_deg 0.246->0.255; tilt_max_deg 1.65->1.69; yaw_rms_deg 0.499->0.485; pos_h_rms_m 0.0419->0.0374; vel_h_rms_ms 0.0495->0.0478; nees_att 0.125->0.109; nees_vel 0.94->0.851; nees_pos 0.0139->0.0124 |
| hover | imu_dropouts | FAIL | tilt_rms_deg 0.00524->0.00468; tilt_max_deg 0.016->0.0142; yaw_rms_deg 0.0181->0.019; pos_h_rms_m 0.00171->0.00161; vel_h_rms_ms 0.00117->0.00113; nees_att 0.000315->0.000157; nees_vel 0.00175->0.00171; nees_pos 0.000259->0.000258 |
| hover | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.00539->0.00482; tilt_max_deg 0.02->0.0146; yaw_rms_deg 0.0183->0.019; pos_h_rms_m 0.00156->0.00148; vel_h_rms_ms 0.00112->0.00109; nees_att 0.000322->0.000157; nees_vel 0.00128->0.00125; nees_pos 0.000164->0.000163 |
| hover | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.00571->0.00517; tilt_max_deg 0.0257->0.0241; yaw_rms_deg 0.0181->0.019; pos_h_rms_m 0.00168->0.00158; vel_h_rms_ms 0.00112->0.0011; nees_att 0.00032->0.000157; nees_vel 0.00125->0.00122; nees_pos 0.000145->0.000144 |
| hover | combined_realistic | WARN | tilt_rms_deg 0.457->0.457; tilt_max_deg 1.35->1.35; yaw_rms_deg 3.63->3.63; pos_h_rms_m 0.648->0.648; vel_h_rms_ms 0.0635->0.0636; nees_att 2.73->1.86 **BETTER**; nees_vel 0.245->0.243; nees_pos 2.24->2.24 |
| hover | cal_ideal | FAIL | tilt_rms_deg 0.00525->0.00467; tilt_max_deg 0.0155->0.0136; yaw_rms_deg 0.0181->0.0189; pos_h_rms_m 0.00169->0.00161; vel_h_rms_ms 0.00112->0.0011; nees_att 0.000318->0.000155; nees_vel 0.00133->0.0013; nees_pos 0.000199->0.000198 |
| box | none | FAIL | tilt_rms_deg 0.00877->0.00862; tilt_max_deg 0.0377->0.0427; yaw_rms_deg 0.0198->0.021; pos_h_rms_m 0.00994->0.00996; vel_h_rms_ms 0.0302->0.0303; nees_att 0.000504->0.000189; nees_vel 0.156->0.146; nees_pos 0.00102->0.00102 |
| box | gps_dropout(duration_s=5) | FAIL | tilt_rms_deg 0.0134->0.0145; tilt_max_deg 0.0619->0.0693; yaw_rms_deg 0.0266->0.028; pos_h_rms_m 0.0131->0.014; vel_h_rms_ms 0.0304->0.0305; nees_att 0.000527->0.00019; nees_vel 0.151->0.142; nees_pos 0.00113->0.00117; tilt_drift_deg_s 0.00716->0.00924 |
| box | gps_dropout(duration_s=15) | FAIL | tilt_rms_deg 0.0357->0.0421; tilt_max_deg 0.148->0.151; yaw_rms_deg 0.0666->0.0798; pos_h_rms_m 0.173->0.21; vel_h_rms_ms 0.0475->0.0536; nees_att 0.00053->0.000187; nees_vel 0.148->0.139; nees_pos 0.00167->0.00168; tilt_drift_deg_s 0.00579->0.00777 |
| box | gps_dropout(duration_s=30) | FAIL | tilt_rms_deg 0.0724->0.0782; tilt_max_deg 0.315->0.23; yaw_rms_deg 0.145->0.148; pos_h_rms_m 0.96->1.46 **WORSE**; vel_h_rms_ms 0.125->0.162 **WORSE**; nees_att 0.000538->0.000185; nees_vel 0.0887->0.0855; nees_pos 0.00357->0.00355; tilt_drift_deg_s 0.00353->0.00359 |
| box | gps_stale(duration_s=5) | FAIL | tilt_rms_deg 0.187->0.189; tilt_max_deg 1.19->1.19; yaw_rms_deg 1.29->2.04 **WORSE**; pos_h_rms_m 0.814->0.811; vel_h_rms_ms 0.104->0.106; nees_att 0.606->1.32; nees_vel 2.29->2.17; nees_pos 2.85->2.83; tilt_drift_deg_s -0.00167->-0.000738 |
| box | gps_stale(duration_s=15) | WARN | tilt_rms_deg 0.411->0.415; tilt_max_deg 2.02->2.03; yaw_rms_deg 1.41->1.91 **WORSE**; pos_h_rms_m 2.11->2.11; vel_h_rms_ms 0.251->0.253; nees_att 0.996->1.38 **WORSE**; nees_vel 11.2->10.6; nees_pos 19->19.1; tilt_drift_deg_s 0.0545->0.0551; recovery_s 5.42->5.46 |
| box | gps_stale(duration_s=30) | WARN | tilt_rms_deg 0.495->0.498; tilt_max_deg 2.14->2.13; yaw_rms_deg 2.37->3.12 **WORSE**; pos_h_rms_m 4.3->4.29; vel_h_rms_ms 0.356->0.362; nees_att 3.13->4.74 **WORSE**; nees_vel 16.7->16.3; nees_pos 79.1->78.9; tilt_drift_deg_s 0.009->0.00989 |
| box | gps_noise | FAIL | tilt_rms_deg 0.335->0.335; tilt_max_deg 1.04->1.04; yaw_rms_deg 0.669->0.686; pos_h_rms_m 0.498->0.496; vel_h_rms_ms 0.0686->0.0688; nees_att 0.0716->0.0725; nees_vel 0.432->0.418; nees_pos 1.19->1.18 |
| box | gps_outliers(frac=0.01) | FAIL | tilt_rms_deg 0.00886->0.00873; tilt_max_deg 0.0377->0.0427; yaw_rms_deg 0.0199->0.0212; pos_h_rms_m 0.0099->0.00993; vel_h_rms_ms 0.0302->0.0303; nees_att 0.000505->0.00019; nees_vel 0.156->0.146; nees_pos 0.00102->0.00102 |
| box | gps_rate(hz=1.0) | FAIL | tilt_rms_deg 0.0111->0.0115; tilt_max_deg 0.0481->0.0525; yaw_rms_deg 0.0272->0.0264; pos_h_rms_m 0.0101->0.0102; vel_h_rms_ms 0.0303->0.0303; nees_att 0.00053->0.000177; nees_vel 0.0721->0.0682; nees_pos 0.00094->0.000942 |
| box | gps_latency(latency_ms=100) | FAIL | tilt_rms_deg 0.00934->0.00953; tilt_max_deg 0.0335->0.038; yaw_rms_deg 0.0211->0.023; pos_h_rms_m 0.0105->0.0105; vel_h_rms_ms 0.0303->0.0304; nees_att 0.000508->0.000203; nees_vel 0.157->0.147; nees_pos 0.00138->0.00138 |
| box | gps_latency(latency_ms=200) | FAIL | tilt_rms_deg 0.0115->0.0122; tilt_max_deg 0.0424->0.049; yaw_rms_deg 0.025->0.0269; pos_h_rms_m 0.0104->0.0105; vel_h_rms_ms 0.0304->0.0304; nees_att 0.000537->0.000233; nees_vel 0.156->0.147; nees_pos 0.00124->0.00125 |
| box | gps_latency(ekf_latency_ms=0,latency_ms=200) | FAIL | tilt_rms_deg 0.105->0.106; tilt_max_deg 0.392->0.396; yaw_rms_deg 0.241->0.322; pos_h_rms_m 0.111->0.111; vel_h_rms_ms 0.0445->0.0447; nees_att 0.0124->0.0177; nees_vel 0.284->0.271; nees_pos 0.0529->0.0531 |
| box | gyro_bias(dps=0.2) | FAIL | tilt_rms_deg 0.00877->0.00862; tilt_max_deg 0.0377->0.0427; yaw_rms_deg 0.0198->0.021; pos_h_rms_m 0.00994->0.00996; vel_h_rms_ms 0.0302->0.0303; nees_att 0.000504->0.000189; nees_vel 0.156->0.146; nees_pos 0.00102->0.00102 |
| box | gyro_bias(dps=1.0) | FAIL | tilt_rms_deg 0.00877->0.00862; tilt_max_deg 0.0377->0.0427; yaw_rms_deg 0.0198->0.021; pos_h_rms_m 0.00994->0.00996; vel_h_rms_ms 0.0302->0.0303; nees_att 0.000504->0.000189; nees_vel 0.156->0.146; nees_pos 0.00102->0.00102 |
| box | gyro_bias(dps=1.0,from_cal=False) | FAIL | tilt_rms_deg 0.0568->0.0568; tilt_max_deg 0.596->0.597; yaw_rms_deg 0.174->0.174; pos_h_rms_m 0.0101->0.0101; vel_h_rms_ms 0.0313->0.0313; nees_att 0.00398->0.00375; nees_vel 0.158->0.149; nees_pos 0.00102->0.00103 |
| box | gyro_drift(dps=0.5,over_s=60) | FAIL | tilt_rms_deg 0.00914->0.00898; tilt_max_deg 0.0364->0.04; yaw_rms_deg 0.0163->0.0197; pos_h_rms_m 0.00994->0.00996; vel_h_rms_ms 0.0302->0.0303; nees_att 0.000503->0.00019; nees_vel 0.156->0.146; nees_pos 0.00102->0.00102 |
| box | accel_bias(axis=x,ms2=0.05) | PASS | tilt_rms_deg 0.292->0.292; tilt_max_deg 0.316->0.305; yaw_rms_deg 0.0867->0.0617; pos_h_rms_m 0.00964->0.00987; vel_h_rms_ms 0.0303->0.0303; nees_att 2.02->1.15 **BETTER**; nees_vel 0.156->0.147; nees_pos 0.00101->0.00103 |
| box | accel_bias(axis=x,ms2=0.2) | FAIL | tilt_rms_deg 1.17->1.17; tilt_max_deg 1.19->1.18; yaw_rms_deg 0.406->0.221 **BETTER**; pos_h_rms_m 0.00903->0.00973; vel_h_rms_ms 0.0304->0.0304; nees_att 32.4->18.4; nees_vel 0.161->0.151; nees_pos 0.00106->0.00111 |
| box | accel_bias(axis=x,ms2=0.5) | FAIL | tilt_rms_deg 2.93->2.92; tilt_max_deg 2.99->2.95; yaw_rms_deg 2.12->0.655 **BETTER**; pos_h_rms_m 0.0122->0.0101; vel_h_rms_ms 0.033->0.0311; nees_att 203->115; nees_vel 0.213->0.175; nees_pos 0.00185->0.00165 |
| box | accel_bias(axis=z,ms2=0.05) | FAIL | tilt_rms_deg 0.00873->0.00858; tilt_max_deg 0.0377->0.0427; yaw_rms_deg 0.0198->0.021; pos_h_rms_m 0.00993->0.00995; vel_h_rms_ms 0.0302->0.0303; nees_att 0.000503->0.000188; nees_vel 0.153->0.143; nees_pos 0.000575->0.000577 |
| box | accel_bias(axis=z,ms2=0.2) | FAIL | tilt_rms_deg 0.00863->0.00846; tilt_max_deg 0.0376->0.0426; yaw_rms_deg 0.0196->0.0209; pos_h_rms_m 0.00991->0.00993; vel_h_rms_ms 0.0302->0.0303; nees_att 0.000501->0.000188; nees_vel 0.16->0.151; nees_pos 0.00148->0.00149 |
| box | accel_bias(axis=z,ms2=0.5) | FAIL | tilt_rms_deg 0.00844->0.00821; tilt_max_deg 0.0373->0.0423; yaw_rms_deg 0.0194->0.0209; pos_h_rms_m 0.00988->0.0099; vel_h_rms_ms 0.0302->0.0303; nees_att 0.000498->0.000187; nees_vel 0.248->0.239; nees_pos 0.0134->0.0134 |
| box | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | tilt_rms_deg 0.831->0.859; tilt_max_deg 1.12->1.19; yaw_rms_deg 6.51->8.58 **WORSE**; pos_h_rms_m 0.429->0.359; vel_h_rms_ms 0.245->0.216; nees_att 90.7->108; nees_vel 14->9.85; nees_pos 0.784->0.548 **WORSE** |
| box | mag_bias(frac=0.05) | WARN | tilt_rms_deg 0.029->0.0311; tilt_max_deg 0.106->0.105; yaw_rms_deg 2.23->2.38; pos_h_rms_m 0.016->0.0141; vel_h_rms_ms 0.0309->0.0309; nees_att 0.997->0.707 **WORSE**; nees_vel 0.163->0.153; nees_pos 0.0017->0.00145 |
| box | mag_bias(frac=0.15) | FAIL | tilt_rms_deg 0.0961->0.1; tilt_max_deg 0.308->0.308; yaw_rms_deg 7.39->7.73; pos_h_rms_m 0.0339->0.0265; vel_h_rms_ms 0.039->0.0379; nees_att 34.5->19.6; nees_vel 0.307->0.263; nees_pos 0.00553->0.0036 |
| box | mag_interference(amp=0.3) | FAIL | tilt_rms_deg 0.00996->0.00967; tilt_max_deg 0.0376->0.0428; yaw_rms_deg 0.0218->0.0204; pos_h_rms_m 0.00995->0.00997; vel_h_rms_ms 0.0303->0.0303; nees_att 0.000466->0.000177; nees_vel 0.15->0.141; nees_pos 0.00102->0.00102; tilt_drift_deg_s -0.00407->-0.00358 |
| box | mag_interference(amp=1.0) | FAIL | tilt_rms_deg 0.00996->0.00967; tilt_max_deg 0.0376->0.0428; yaw_rms_deg 0.0218->0.0204; pos_h_rms_m 0.00995->0.00997; vel_h_rms_ms 0.0303->0.0303; nees_att 0.000466->0.000177; nees_vel 0.15->0.141; nees_pos 0.00102->0.00102; tilt_drift_deg_s -0.00407->-0.00358 |
| box | imu_noise | FAIL | tilt_rms_deg 0.0264->0.0267; tilt_max_deg 0.0804->0.0818; yaw_rms_deg 0.0555->0.0612; pos_h_rms_m 0.0148->0.0146; vel_h_rms_ms 0.0309->0.0309; nees_att 0.00207->0.00148; nees_vel 0.179->0.169; nees_pos 0.00314->0.00312 |
| box | imu_spikes | WARN | tilt_rms_deg 0.219->0.224; tilt_max_deg 1.28->1.33; yaw_rms_deg 0.476->0.529; pos_h_rms_m 0.0182->0.0169; vel_h_rms_ms 0.0484->0.0477; nees_att 0.208->0.176; nees_vel 0.829->0.784; nees_pos 0.0115->0.0113 |
| box | imu_dropouts | FAIL | tilt_rms_deg 0.00912->0.00901; tilt_max_deg 0.0355->0.0374; yaw_rms_deg 0.0207->0.0214; pos_h_rms_m 0.00979->0.00982; vel_h_rms_ms 0.0304->0.0305; nees_att 0.000505->0.000189; nees_vel 0.158->0.149; nees_pos 0.00103->0.00104 |
| box | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.00909->0.00907; tilt_max_deg 0.0439->0.0439; yaw_rms_deg 0.0228->0.0221; pos_h_rms_m 0.00937->0.00939; vel_h_rms_ms 0.0303->0.0304; nees_att 0.000522->0.000186; nees_vel 0.154->0.145; nees_pos 0.000808->0.00081 |
| box | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.0166->0.017; tilt_max_deg 0.121->0.123; yaw_rms_deg 0.0333->0.0333; pos_h_rms_m 0.0174->0.0176; vel_h_rms_ms 0.0305->0.0306; nees_att 0.000503->0.00019; nees_vel 0.154->0.145; nees_pos 0.00116->0.00117 |
| box | combined_realistic | WARN | tilt_rms_deg 0.454->0.453; tilt_max_deg 1.07->1.07; yaw_rms_deg 2.54->2.67; pos_h_rms_m 0.508->0.505; vel_h_rms_ms 0.07->0.0701; nees_att 3.58->2.19 **BETTER**; nees_vel 0.468->0.449; nees_pos 1.19->1.18 |
| box | cal_ideal | FAIL | tilt_rms_deg 0.00877->0.00862; tilt_max_deg 0.0377->0.0427; yaw_rms_deg 0.0198->0.021; pos_h_rms_m 0.00993->0.00996; vel_h_rms_ms 0.0302->0.0303; nees_att 0.000504->0.000189; nees_vel 0.155->0.146; nees_pos 0.000947->0.00095 |
| circle_slow | none | FAIL | tilt_rms_deg 0.0127->0.0126; tilt_max_deg 0.0566->0.046; yaw_rms_deg 0.0474->0.0816; pos_h_rms_m 0.0153->0.0152; vel_h_rms_ms 0.0301->0.0301; nees_att 0.00268->0.00391; nees_vel 0.137->0.126; nees_pos 0.00115->0.00114 |
| circle_slow | gps_dropout(duration_s=5) | FAIL | tilt_rms_deg 0.019->0.0194; tilt_max_deg 0.0787->0.081; yaw_rms_deg 0.0487->0.0786; pos_h_rms_m 0.0191->0.0194; vel_h_rms_ms 0.0314->0.0315; nees_att 0.00261->0.00374; nees_vel 0.129->0.119; nees_pos 0.000966->0.000981; tilt_drift_deg_s 0.0102->0.0105 |
| circle_slow | gps_dropout(duration_s=15) | FAIL | tilt_rms_deg 0.0501->0.0536; tilt_max_deg 0.165->0.175; yaw_rms_deg 0.0883->0.0987; pos_h_rms_m 0.279->0.305; vel_h_rms_ms 0.0646->0.069; nees_att 0.00208->0.00256; nees_vel 0.108->0.0997; nees_pos 0.000811->0.000825; tilt_drift_deg_s 0.0089->0.0102 |
| circle_slow | gps_dropout(duration_s=30) | FAIL | tilt_rms_deg 0.148->0.116; tilt_max_deg 0.523->0.344; yaw_rms_deg 0.272->0.201; pos_h_rms_m 2.31->2.14; vel_h_rms_ms 0.283->0.241; nees_att 0.00154->0.00141; nees_vel 0.0741->0.0688; nees_pos 0.000719->0.000696; tilt_drift_deg_s 0.0116->0.00681 |
| circle_slow | gps_stale(duration_s=5) | WARN | tilt_rms_deg 0.65->0.65; tilt_max_deg 3.02->3.02; yaw_rms_deg 1.3->1.34; pos_h_rms_m 0.728->0.728; vel_h_rms_ms 0.348->0.348; nees_att 0.265->0.274; nees_vel 3.9->3.84; nees_pos 2.11->2.11; tilt_drift_deg_s 0.467->0.467 |
| circle_slow | gps_stale(duration_s=15) | WARN | tilt_rms_deg 0.926->0.925; tilt_max_deg 3.02->3.02; yaw_rms_deg 1.79->1.8; pos_h_rms_m 2.86->2.86; vel_h_rms_ms 0.778->0.781; nees_att 0.605->0.676; nees_vel 74.5->70.8; nees_pos 12.8->12.8; tilt_drift_deg_s 0.0326->0.0321 |
| circle_slow | gps_stale(duration_s=30) | WARN | tilt_rms_deg 1.27->1.27; tilt_max_deg 3.14->3.14; yaw_rms_deg 2.52->2.61; pos_h_rms_m 3.67->3.65; vel_h_rms_ms 1.04->1.04; nees_att 1->1.16; nees_vel 115->108; nees_pos 19->18.6; tilt_drift_deg_s -0.00764->-0.00768 |
| circle_slow | gps_noise | FAIL | tilt_rms_deg 0.283->0.283; tilt_max_deg 1.05->1.05; yaw_rms_deg 0.573->0.591; pos_h_rms_m 0.597->0.597; vel_h_rms_ms 0.0629->0.0633; nees_att 0.0585->0.062; nees_vel 0.354->0.345; nees_pos 1.87->1.87 |
| circle_slow | gps_outliers(frac=0.01) | FAIL | tilt_rms_deg 0.0126->0.0124; tilt_max_deg 0.0576->0.046; yaw_rms_deg 0.0478->0.0819; pos_h_rms_m 0.023->0.023; vel_h_rms_ms 0.0301->0.0301; nees_att 0.00267->0.00389; nees_vel 0.136->0.126; nees_pos 0.00263->0.00262 |
| circle_slow | gps_rate(hz=1.0) | FAIL | tilt_rms_deg 0.0152->0.0152; tilt_max_deg 0.0504->0.0395; yaw_rms_deg 0.0389->0.0529; pos_h_rms_m 0.0182->0.0179; vel_h_rms_ms 0.0302->0.0303; nees_att 0.00148->0.00129; nees_vel 0.0547->0.0511; nees_pos 0.000379->0.000367 |
| circle_slow | gps_latency(latency_ms=100) | FAIL | tilt_rms_deg 0.014->0.0138; tilt_max_deg 0.0578->0.0458; yaw_rms_deg 0.0556->0.0969; pos_h_rms_m 0.0181->0.018; vel_h_rms_ms 0.0303->0.0304; nees_att 0.00355->0.00545; nees_vel 0.14->0.128; nees_pos 0.00145->0.00144 |
| circle_slow | gps_latency(latency_ms=200) | FAIL | tilt_rms_deg 0.015->0.015; tilt_max_deg 0.0534->0.0521; yaw_rms_deg 0.0528->0.0893; pos_h_rms_m 0.0166->0.0166; vel_h_rms_ms 0.0302->0.0303; nees_att 0.0031->0.0046; nees_vel 0.138->0.127; nees_pos 0.00125->0.00124 |
| circle_slow | gps_latency(ekf_latency_ms=0,latency_ms=200) | PASS | tilt_rms_deg 0.151->0.15; tilt_max_deg 0.442->0.444; yaw_rms_deg 0.614->1.05 **WORSE**; pos_h_rms_m 0.205->0.206; vel_h_rms_ms 0.0739->0.0741; nees_att 0.374->0.659 **BETTER**; nees_vel 0.647->0.612; nees_pos 0.18->0.181 |
| circle_slow | gyro_bias(dps=0.2) | FAIL | tilt_rms_deg 0.0127->0.0126; tilt_max_deg 0.0566->0.046; yaw_rms_deg 0.0474->0.0816; pos_h_rms_m 0.0153->0.0152; vel_h_rms_ms 0.0301->0.0301; nees_att 0.00268->0.00391; nees_vel 0.137->0.126; nees_pos 0.00115->0.00114 |
| circle_slow | gyro_bias(dps=1.0) | FAIL | tilt_rms_deg 0.0127->0.0126; tilt_max_deg 0.0566->0.046; yaw_rms_deg 0.0474->0.0816; pos_h_rms_m 0.0153->0.0152; vel_h_rms_ms 0.0301->0.0301; nees_att 0.00268->0.00391; nees_vel 0.137->0.126; nees_pos 0.00115->0.00114 |
| circle_slow | gyro_bias(dps=1.0,from_cal=False) | FAIL | tilt_rms_deg 0.0572->0.0571; tilt_max_deg 0.588->0.589; yaw_rms_deg 0.144->0.15; pos_h_rms_m 0.0154->0.0154; vel_h_rms_ms 0.0312->0.0312; nees_att 0.00453->0.0051; nees_vel 0.14->0.13; nees_pos 0.00116->0.00115 |
| circle_slow | gyro_drift(dps=0.5,over_s=60) | FAIL | tilt_rms_deg 0.013->0.0127; tilt_max_deg 0.0566->0.0436; yaw_rms_deg 0.0434->0.0774; pos_h_rms_m 0.0153->0.0152; vel_h_rms_ms 0.0301->0.0301; nees_att 0.00261->0.00381; nees_vel 0.137->0.126; nees_pos 0.00115->0.00113 |
| circle_slow | accel_bias(axis=x,ms2=0.05) | PASS | tilt_rms_deg 0.29->0.291; tilt_max_deg 0.33->0.31; yaw_rms_deg 0.11->0.129; pos_h_rms_m 0.0162->0.0155; vel_h_rms_ms 0.0301->0.0301; nees_att 2->1.14 **BETTER**; nees_vel 0.137->0.126; nees_pos 0.00128->0.00119 |
| circle_slow | accel_bias(axis=x,ms2=0.2) | FAIL | tilt_rms_deg 1.16->1.17; tilt_max_deg 1.2->1.19; yaw_rms_deg 0.307->0.281; pos_h_rms_m 0.0194->0.0164; vel_h_rms_ms 0.0304->0.0302; nees_att 32.2->18.3; nees_vel 0.144->0.131; nees_pos 0.0019->0.00144 |
| circle_slow | accel_bias(axis=x,ms2=0.5) | FAIL | tilt_rms_deg 2.91->2.91; tilt_max_deg 2.95->2.94; yaw_rms_deg 1.04->0.655 **BETTER**; pos_h_rms_m 0.0278->0.0189; vel_h_rms_ms 0.0318->0.031; nees_att 201->114; nees_vel 0.186->0.163; nees_pos 0.00411->0.00232 |
| circle_slow | accel_bias(axis=z,ms2=0.05) | FAIL | tilt_rms_deg 0.0126->0.0125; tilt_max_deg 0.0565->0.0457; yaw_rms_deg 0.0473->0.0813; pos_h_rms_m 0.0153->0.0152; vel_h_rms_ms 0.0301->0.0301; nees_att 0.00267->0.00389; nees_vel 0.138->0.128; nees_pos 0.00152->0.0015 |
| circle_slow | accel_bias(axis=z,ms2=0.2) | FAIL | tilt_rms_deg 0.0124->0.0122; tilt_max_deg 0.0561->0.0449; yaw_rms_deg 0.0468->0.0806; pos_h_rms_m 0.0153->0.0152; vel_h_rms_ms 0.0301->0.0301; nees_att 0.00263->0.00383; nees_vel 0.167->0.156; nees_pos 0.00569->0.00566 |
| circle_slow | accel_bias(axis=z,ms2=0.5) | FAIL | tilt_rms_deg 0.0118->0.0116; tilt_max_deg 0.0553->0.0447; yaw_rms_deg 0.0459->0.0793; pos_h_rms_m 0.0152->0.0151; vel_h_rms_ms 0.03->0.03; nees_att 0.00256->0.00372; nees_vel 0.328->0.317; nees_pos 0.0278->0.0278 |
| circle_slow | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | tilt_rms_deg 0.832->0.868; tilt_max_deg 1.12->1.21; yaw_rms_deg 5.41->7.85 **WORSE**; pos_h_rms_m 0.468->0.396; vel_h_rms_ms 0.257->0.229; nees_att 63.8->89.5; nees_vel 15.5->11.3; nees_pos 0.937->0.672 **WORSE** |
| circle_slow | mag_bias(frac=0.05) | PASS | tilt_rms_deg 0.0266->0.0291; tilt_max_deg 0.0704->0.0767; yaw_rms_deg 1.47->1.59; pos_h_rms_m 0.0145->0.0139; vel_h_rms_ms 0.0298->0.0296; nees_att 0.701->0.54 **WORSE**; nees_vel 0.13->0.117; nees_pos 0.00104->0.000966 |
| circle_slow | mag_bias(frac=0.15) | FAIL | tilt_rms_deg 0.123->0.129; tilt_max_deg 0.247->0.263; yaw_rms_deg 5.77->6.08; pos_h_rms_m 0.0426->0.0358; vel_h_rms_ms 0.0472->0.0463; nees_att 35.6->21; nees_vel 0.48->0.421; nees_pos 0.00793->0.00565 |
| circle_slow | mag_interference(amp=0.3) | FAIL | tilt_rms_deg 0.0134->0.0133; tilt_max_deg 0.0566->0.046; yaw_rms_deg 0.0499->0.0842; pos_h_rms_m 0.0154->0.0153; vel_h_rms_ms 0.0301->0.0301; nees_att 0.00264->0.00366; nees_vel 0.123->0.114; nees_pos 0.00115->0.00114; tilt_drift_deg_s 0.00127->0.00166; mag_rejected 1246->1245 |
| circle_slow | mag_interference(amp=1.0) | FAIL | tilt_rms_deg 0.0134->0.0133; tilt_max_deg 0.0566->0.046; yaw_rms_deg 0.0499->0.0842; pos_h_rms_m 0.0154->0.0153; vel_h_rms_ms 0.0301->0.0301; nees_att 0.00264->0.00366; nees_vel 0.123->0.114; nees_pos 0.00115->0.00114; tilt_drift_deg_s 0.00127->0.00166; mag_rejected 1246->1245 |
| circle_slow | imu_noise | FAIL | tilt_rms_deg 0.0269->0.027; tilt_max_deg 0.0887->0.0874; yaw_rms_deg 0.0757->0.117; pos_h_rms_m 0.0144->0.0143; vel_h_rms_ms 0.0303->0.0303; nees_att 0.00496->0.00739; nees_vel 0.185->0.174; nees_pos 0.00776->0.00774 |
| circle_slow | imu_spikes | WARN | tilt_rms_deg 0.247->0.256; tilt_max_deg 1.67->1.72; yaw_rms_deg 0.488->0.522; pos_h_rms_m 0.0304->0.0283; vel_h_rms_ms 0.0568->0.0558; nees_att 0.185->0.165; nees_vel 0.696->0.617; nees_pos 0.00431->0.00378 |
| circle_slow | imu_dropouts | FAIL | tilt_rms_deg 0.0133->0.0132; tilt_max_deg 0.045->0.0446; yaw_rms_deg 0.0505->0.0867; pos_h_rms_m 0.0164->0.0163; vel_h_rms_ms 0.0302->0.0302; nees_att 0.00299->0.00445; nees_vel 0.138->0.127; nees_pos 0.00128->0.00127 |
| circle_slow | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.0143->0.0144; tilt_max_deg 0.0516->0.0569; yaw_rms_deg 0.0478->0.0805; pos_h_rms_m 0.0149->0.0148; vel_h_rms_ms 0.0301->0.0301; nees_att 0.00264->0.00376; nees_vel 0.13->0.12; nees_pos 0.00108->0.00107 |
| circle_slow | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.0191->0.0192; tilt_max_deg 0.116->0.117; yaw_rms_deg 0.0522->0.0815; pos_h_rms_m 0.0334->0.0334; vel_h_rms_ms 0.0301->0.0302; nees_att 0.00266->0.00369; nees_vel 0.128->0.119; nees_pos 0.00282->0.00282 |
| circle_slow | combined_realistic | WARN -> PASS | tilt_rms_deg 0.418->0.419; tilt_max_deg 1.16->1.17; yaw_rms_deg 1.63->1.76; pos_h_rms_m 0.607->0.606; vel_h_rms_ms 0.0637->0.0636; nees_att 3.1->1.97 **BETTER**; nees_vel 0.386->0.366; nees_pos 1.84->1.83 |
| circle_slow | cal_ideal | FAIL | tilt_rms_deg 0.0127->0.0126; tilt_max_deg 0.0566->0.0459; yaw_rms_deg 0.0474->0.0815; pos_h_rms_m 0.0153->0.0153; vel_h_rms_ms 0.0301->0.0301; nees_att 0.00268->0.00391; nees_vel 0.137->0.126; nees_pos 0.00117->0.00116 |
| circle_fast | none | FAIL | tilt_rms_deg 0.148->0.156; tilt_max_deg 0.416->0.475; yaw_rms_deg 0.261->0.303; pos_h_rms_m 0.0568->0.0567; vel_h_rms_ms 0.11->0.111; nees_att 0.0825->0.0551; nees_vel 1.41->1.37; nees_pos 0.0141->0.0143 |
| circle_fast | gps_dropout(duration_s=5) | FAIL | tilt_rms_deg 0.146->0.154; tilt_max_deg 0.52->0.577; yaw_rms_deg 0.268->0.307; pos_h_rms_m 0.0725->0.0754; vel_h_rms_ms 0.11->0.111; nees_att 0.08->0.0482 **WORSE**; nees_vel 1.32->1.28; nees_pos 0.016->0.017; tilt_drift_deg_s 0.00751->0.00911 |
| circle_fast | gps_dropout(duration_s=15) | FAIL | tilt_rms_deg 0.214->0.17; tilt_max_deg 0.825->0.449 **BETTER**; yaw_rms_deg 0.364->0.337; pos_h_rms_m 0.568->0.316 **BETTER**; vel_h_rms_ms 0.176->0.136 **BETTER**; nees_att 0.0747->0.0359 **WORSE**; nees_vel 1.08->1.04; nees_pos 0.0105->0.0116; tilt_drift_deg_s 0.0422->0.0194 |
| circle_fast | gps_dropout(duration_s=30) | FAIL | tilt_rms_deg 0.498->0.219 **BETTER**; tilt_max_deg 1.67->0.554 **BETTER**; yaw_rms_deg 0.931->0.431 **BETTER**; pos_h_rms_m 10.4->3.43 **BETTER**; vel_h_rms_ms 1.12->0.39 **BETTER**; nees_att 0.0727->0.0281 **WORSE**; nees_vel 0.75->0.728; nees_pos 0.00813->0.00766; tilt_drift_deg_s 0.0411->0.00706 |
| circle_fast | gps_stale(duration_s=5) | WARN -> FAIL | tilt_rms_deg 0.735->4.75 **WORSE**; tilt_max_deg 5.79->25.2 **WORSE**; yaw_rms_deg 1.6->7.02 **WORSE**; pos_h_rms_m 1.07->1.1; vel_h_rms_ms 0.92->0.991; nees_att 0.131->1.9 **BETTER**; nees_vel 24.1->27.3; nees_pos 0.185->0.241; tilt_drift_deg_s 0.725->1.37 **WORSE**; recovery_s 3.58->8.46 **WORSE**; gps_resets 4->6 **WORSE**; gps_rejected 20->30 **WORSE** |
| circle_fast | gps_stale(duration_s=15) | FAIL | tilt_rms_deg 3.14->6.28 **WORSE**; tilt_max_deg 12->23.1 **WORSE**; yaw_rms_deg 5.88->10.8 **WORSE**; pos_h_rms_m 1.56->1.58; vel_h_rms_ms 1.49->1.5; nees_att 0.972->6.44 **WORSE**; nees_vel 65.4->59.8; nees_pos 0.387->0.509 **BETTER**; tilt_drift_deg_s 0.677->1.05 **WORSE**; recovery_s 4.92->5.33; gps_resets 11->10; gps_rejected 55->50 |
| circle_fast | gps_stale(duration_s=30) | FAIL | tilt_rms_deg 5.54->8.62 **WORSE**; tilt_max_deg 15.6->23.1 **WORSE**; yaw_rms_deg 10.9->14.6 **WORSE**; pos_h_rms_m 2.34->2.23; vel_h_rms_ms 2.31->2.13; nees_att 5.25->11.4 **WORSE**; nees_vel 170->115; nees_pos 1.16->0.951; tilt_drift_deg_s 0.26->0.123 **BETTER**; recovery_s 4.18->5.07 **WORSE**; gps_rejected 104->100 |
| circle_fast | gps_noise | WARN | tilt_rms_deg 0.409->0.414; tilt_max_deg 1.4->1.47; yaw_rms_deg 0.741->0.759; pos_h_rms_m 0.622->0.624; vel_h_rms_ms 0.129->0.13; nees_att 0.179->0.165; nees_vel 1.69->1.64; nees_pos 1.69->1.7 |
| circle_fast | gps_outliers(frac=0.01) | FAIL | tilt_rms_deg 0.147->0.155; tilt_max_deg 0.416->0.475; yaw_rms_deg 0.26->0.302; pos_h_rms_m 0.0568->0.0568; vel_h_rms_ms 0.11->0.111; nees_att 0.0823->0.0547; nees_vel 1.4->1.36; nees_pos 0.0141->0.0143 |
| circle_fast | gps_rate(hz=1.0) | FAIL | tilt_rms_deg 0.127->0.143; tilt_max_deg 0.418->0.519; yaw_rms_deg 0.268->0.292; pos_h_rms_m 0.0661->0.0677; vel_h_rms_ms 0.113->0.115; nees_att 0.0665->0.0175 **WORSE**; nees_vel 0.487->0.476; nees_pos 0.00412->0.0046 |
| circle_fast | gps_latency(latency_ms=100) | FAIL | tilt_rms_deg 0.199->0.209; tilt_max_deg 0.572->0.637; yaw_rms_deg 0.352->0.391; pos_h_rms_m 0.0636->0.0645; vel_h_rms_ms 0.115->0.117; nees_att 0.0986->0.0818; nees_vel 1.48->1.43; nees_pos 0.0175->0.0181 |
| circle_fast | gps_latency(latency_ms=200) | WARN -> FAIL | tilt_rms_deg 0.216->0.23; tilt_max_deg 0.594->0.679; yaw_rms_deg 0.377->0.419; pos_h_rms_m 0.0675->0.0697; vel_h_rms_ms 0.117->0.119; nees_att 0.102->0.0846 **WORSE**; nees_vel 1.49->1.44; nees_pos 0.0196->0.0209 |
| circle_fast | gps_latency(ekf_latency_ms=0,latency_ms=200) | WARN | tilt_rms_deg 2.82->2.56; tilt_max_deg 5.25->5.06; yaw_rms_deg 4.77->4.39; pos_h_rms_m 0.512->0.505; vel_h_rms_ms 0.642->0.614; nees_att 4.69->6.29; nees_vel 18->16.4; nees_pos 0.299->0.288 |
| circle_fast | gyro_bias(dps=0.2) | FAIL | tilt_rms_deg 0.148->0.156; tilt_max_deg 0.416->0.475; yaw_rms_deg 0.261->0.303; pos_h_rms_m 0.0568->0.0567; vel_h_rms_ms 0.11->0.111; nees_att 0.0825->0.0551; nees_vel 1.41->1.37; nees_pos 0.0141->0.0143 |
| circle_fast | gyro_bias(dps=1.0) | FAIL | tilt_rms_deg 0.148->0.156; tilt_max_deg 0.416->0.475; yaw_rms_deg 0.261->0.303; pos_h_rms_m 0.0568->0.0567; vel_h_rms_ms 0.11->0.111; nees_att 0.0825->0.0551; nees_vel 1.41->1.37; nees_pos 0.0141->0.0143 |
| circle_fast | gyro_bias(dps=1.0,from_cal=False) | FAIL | tilt_rms_deg 0.158->0.166; tilt_max_deg 0.595->0.598; yaw_rms_deg 0.292->0.328; pos_h_rms_m 0.0569->0.0567; vel_h_rms_ms 0.11->0.111; nees_att 0.0847->0.0568; nees_vel 1.42->1.37; nees_pos 0.0142->0.0143 |
| circle_fast | gyro_drift(dps=0.5,over_s=60) | FAIL | tilt_rms_deg 0.149->0.157; tilt_max_deg 0.422->0.48; yaw_rms_deg 0.262->0.304; pos_h_rms_m 0.0568->0.0567; vel_h_rms_ms 0.11->0.111; nees_att 0.0824->0.0546; nees_vel 1.41->1.37; nees_pos 0.0141->0.0143 |
| circle_fast | accel_bias(axis=x,ms2=0.05) | PASS | tilt_rms_deg 0.339->0.343; tilt_max_deg 0.593->0.64; yaw_rms_deg 0.366->0.343; pos_h_rms_m 0.0741->0.0599; vel_h_rms_ms 0.115->0.113; nees_att 1.57->1.02 **BETTER**; nees_vel 1.61->1.42; nees_pos 0.0243->0.0155 |
| circle_fast | accel_bias(axis=x,ms2=0.2) | FAIL | tilt_rms_deg 1.14->1.15; tilt_max_deg 1.47->1.49; yaw_rms_deg 0.824->0.585 **BETTER**; pos_h_rms_m 0.192->0.0949 **BETTER**; vel_h_rms_ms 0.145->0.122; nees_att 24.3->15.8; nees_vel 3.15->1.77 **BETTER**; nees_pos 0.181->0.0398 **WORSE** |
| circle_fast | accel_bias(axis=x,ms2=0.5) | FAIL | tilt_rms_deg 2.78->2.81; tilt_max_deg 3.4->3.3; yaw_rms_deg 1.81->1.22 **BETTER**; pos_h_rms_m 0.458->0.197 **BETTER**; vel_h_rms_ms 0.23->0.151 **BETTER**; nees_att 152->98.7; nees_vel 10.5->3.19 **BETTER**; nees_pos 1.05->0.18 **WORSE** |
| circle_fast | accel_bias(axis=z,ms2=0.05) | FAIL | tilt_rms_deg 0.147->0.155; tilt_max_deg 0.415->0.473; yaw_rms_deg 0.259->0.301; pos_h_rms_m 0.0568->0.0567; vel_h_rms_ms 0.109->0.111; nees_att 0.0823->0.0548; nees_vel 1.41->1.36; nees_pos 0.0138->0.0138 |
| circle_fast | accel_bias(axis=z,ms2=0.2) | FAIL | tilt_rms_deg 0.145->0.152; tilt_max_deg 0.411->0.469; yaw_rms_deg 0.252->0.294; pos_h_rms_m 0.0568->0.0566; vel_h_rms_ms 0.109->0.11; nees_att 0.0815->0.0539; nees_vel 1.4->1.34; nees_pos 0.0151->0.0146 |
| circle_fast | accel_bias(axis=z,ms2=0.5) | FAIL | tilt_rms_deg 0.14->0.147; tilt_max_deg 0.402->0.461; yaw_rms_deg 0.24->0.282; pos_h_rms_m 0.0569->0.0566; vel_h_rms_ms 0.108->0.109; nees_att 0.0801->0.0523; nees_vel 1.45->1.39; nees_pos 0.0272->0.0256 |
| circle_fast | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | tilt_rms_deg 1.18->1.06; tilt_max_deg 2.82->2.03 **BETTER**; yaw_rms_deg 2.6->3.56 **WORSE**; pos_h_rms_m 0.552->0.465; vel_h_rms_ms 0.326->0.292; nees_att 10.4->11.2; nees_vel 22.8->16.6; nees_pos 1.5->1.05 **BETTER** |
| circle_fast | mag_bias(frac=0.05) | WARN | tilt_rms_deg 0.138->0.141; tilt_max_deg 0.384->0.433; yaw_rms_deg 1.2->1.23; pos_h_rms_m 0.0576->0.0578; vel_h_rms_ms 0.107->0.108; nees_att 0.206->0.188; nees_vel 1.38->1.33; nees_pos 0.0151->0.0155 |
| circle_fast | mag_bias(frac=0.15) | WARN | tilt_rms_deg 0.378->0.394; tilt_max_deg 1.19->1.15; yaw_rms_deg 3.56->3.65; pos_h_rms_m 0.0952->0.1; vel_h_rms_ms 0.103->0.103; nees_att 5.19->4.09; nees_vel 1.74->1.68; nees_pos 0.0491->0.0545 |
| circle_fast | mag_interference(amp=0.3) | FAIL | tilt_rms_deg 0.156->0.162; tilt_max_deg 0.417->0.477; yaw_rms_deg 0.287->0.333; pos_h_rms_m 0.0569->0.0568; vel_h_rms_ms 0.11->0.111; nees_att 0.0771->0.0542; nees_vel 1.31->1.27; nees_pos 0.0142->0.0144; tilt_drift_deg_s 0.0447->0.0445; mag_rejected 1249->1248 |
| circle_fast | mag_interference(amp=1.0) | FAIL | tilt_rms_deg 0.156->0.162; tilt_max_deg 0.417->0.477; yaw_rms_deg 0.287->0.333; pos_h_rms_m 0.0569->0.0568; vel_h_rms_ms 0.11->0.111; nees_att 0.0771->0.0542; nees_vel 1.31->1.27; nees_pos 0.0142->0.0144; tilt_drift_deg_s 0.0447->0.0445; mag_rejected 1249->1248 |
| circle_fast | imu_noise | FAIL | tilt_rms_deg 0.148->0.155; tilt_max_deg 0.421->0.463; yaw_rms_deg 0.252->0.293; pos_h_rms_m 0.0567->0.0567; vel_h_rms_ms 0.109->0.111; nees_att 0.0831->0.0567; nees_vel 1.4->1.35; nees_pos 0.0139->0.0138 |
| circle_fast | imu_spikes | PASS -> WARN | tilt_rms_deg 0.288->0.29; tilt_max_deg 1.82->1.79; yaw_rms_deg 0.438->0.458; pos_h_rms_m 0.0665->0.0653; vel_h_rms_ms 0.117->0.118; nees_att 0.308->0.245 **WORSE**; nees_vel 2.05->1.95; nees_pos 0.0232->0.0225 |
| circle_fast | imu_dropouts | FAIL | tilt_rms_deg 0.162->0.169; tilt_max_deg 0.515->0.568; yaw_rms_deg 0.286->0.325; pos_h_rms_m 0.0611->0.0613; vel_h_rms_ms 0.11->0.112; nees_att 0.086->0.0588; nees_vel 1.44->1.4; nees_pos 0.0163->0.0166 |
| circle_fast | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.163->0.173; tilt_max_deg 0.465->0.542; yaw_rms_deg 0.29->0.328; pos_h_rms_m 0.0579->0.0578; vel_h_rms_ms 0.111->0.113; nees_att 0.0847->0.0568; nees_vel 1.41->1.36; nees_pos 0.0146->0.0147 |
| circle_fast | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.195->0.199; tilt_max_deg 0.875->0.841; yaw_rms_deg 0.331->0.364; pos_h_rms_m 0.179->0.177; vel_h_rms_ms 0.111->0.113; nees_att 0.0863->0.0577; nees_vel 1.39->1.35; nees_pos 0.0741->0.0725 |
| circle_fast | combined_realistic | PASS | tilt_rms_deg 0.506->0.51; tilt_max_deg 1.45->1.47; yaw_rms_deg 1.44->1.46; pos_h_rms_m 0.586->0.615; vel_h_rms_ms 0.131->0.129; nees_att 1.97->1.41 **BETTER**; nees_vel 1.81->1.63; nees_pos 1.49->1.64 |
| circle_fast | cal_ideal | FAIL | tilt_rms_deg 0.148->0.156; tilt_max_deg 0.416->0.475; yaw_rms_deg 0.261->0.303; pos_h_rms_m 0.0568->0.0567; vel_h_rms_ms 0.11->0.111; nees_att 0.0825->0.0551; nees_vel 1.41->1.37; nees_pos 0.0141->0.0142 |
| stops | none | FAIL | tilt_rms_deg 0.057->0.0292; tilt_max_deg 0.27->0.192; yaw_rms_deg 0.112->0.0965; pos_h_rms_m 0.0408->0.041; vel_h_rms_ms 0.0798->0.0804; nees_att 0.0734->0.0124 **WORSE**; nees_vel 1.7->1.59; nees_pos 0.0072->0.00728 |
| stops | gps_dropout(duration_s=5) | FAIL | tilt_rms_deg 0.0578->0.0296; tilt_max_deg 0.27->0.192; yaw_rms_deg 0.123->0.102; pos_h_rms_m 0.0437->0.0444; vel_h_rms_ms 0.0798->0.0803; nees_att 0.0739->0.0123 **WORSE**; nees_vel 1.57->1.47; nees_pos 0.00672->0.00713; tilt_drift_deg_s -0.00389->-0.00103; recovery_s 0.16->0.00409 |
| stops | gps_dropout(duration_s=15) | FAIL | tilt_rms_deg 0.06->0.0338; tilt_max_deg 0.274->0.193; yaw_rms_deg 0.133->0.101; pos_h_rms_m 0.139->0.109; vel_h_rms_ms 0.0818->0.0816; nees_att 0.0743->0.0119 **WORSE**; nees_vel 1.27->1.18; nees_pos 0.0047->0.00481; tilt_drift_deg_s 0.000418->0.00154 |
| stops | gps_dropout(duration_s=30) | FAIL | tilt_rms_deg 0.0885->0.0589; tilt_max_deg 0.387->0.291; yaw_rms_deg 0.142->0.106; pos_h_rms_m 1.95->1.24 **BETTER**; vel_h_rms_ms 0.196->0.142 **BETTER**; nees_att 0.0749->0.0116 **WORSE**; nees_vel 0.742->0.689; nees_pos 0.00523->0.0046; tilt_drift_deg_s 0.0053->0.0038 |
| stops | gps_stale(duration_s=5) | FAIL | tilt_rms_deg 0.145->0.139; tilt_max_deg 0.736->0.757; yaw_rms_deg 0.272->0.265; pos_h_rms_m 0.875->0.876; vel_h_rms_ms 0.673->0.675; nees_att 0.0766->0.0163 **WORSE**; nees_vel 41.2->40.8; nees_pos 0.354->0.354; tilt_drift_deg_s 0.138->0.14 |
| stops | gps_stale(duration_s=15) | FAIL | tilt_rms_deg 0.224->0.227; tilt_max_deg 0.739->0.76; yaw_rms_deg 0.424->0.422; pos_h_rms_m 1.52->1.52; vel_h_rms_ms 1.16->1.16; nees_att 0.0861->0.0252 **WORSE**; nees_vel 120->119; nees_pos 0.999->0.999; tilt_drift_deg_s 0.0162->0.017; recovery_s 11.2->11.2 |
| stops | gps_stale(duration_s=30) | WARN -> FAIL | tilt_rms_deg 0.323->0.331; tilt_max_deg 0.819->0.839; yaw_rms_deg 0.601->0.599; pos_h_rms_m 2.13->2.13; vel_h_rms_ms 1.63->1.64; nees_att 0.104->0.041 **WORSE**; nees_vel 232->240; nees_pos 1.85->1.93; tilt_drift_deg_s 0.00238->0.00257; recovery_s 1.8->1.66; gps_rejected 64->62 |
| stops | gps_noise | WARN | tilt_rms_deg 0.352->0.352; tilt_max_deg 1.11->1.11; yaw_rms_deg 0.698->0.709; pos_h_rms_m 0.496->0.497; vel_h_rms_ms 0.0989->0.0996; nees_att 0.161->0.101 **WORSE**; nees_vel 1.82->1.72; nees_pos 1.17->1.18 |
| stops | gps_outliers(frac=0.01) | FAIL | tilt_rms_deg 0.057->0.0292; tilt_max_deg 0.27->0.192; yaw_rms_deg 0.112->0.0966; pos_h_rms_m 0.0488->0.0486; vel_h_rms_ms 0.0798->0.0804; nees_att 0.0735->0.0124 **WORSE**; nees_vel 1.69->1.59; nees_pos 0.0123->0.0122 |
| stops | gps_rate(hz=1.0) | FAIL | tilt_rms_deg 0.0648->0.0334; tilt_max_deg 0.272->0.19; yaw_rms_deg 0.157->0.107; pos_h_rms_m 0.0471->0.045; vel_h_rms_ms 0.0787->0.0788; nees_att 0.0739->0.0109 **WORSE**; nees_vel 0.681->0.635; nees_pos 0.00219->0.00202 |
| stops | gps_latency(latency_ms=100) | FAIL | tilt_rms_deg 0.0579->0.0305; tilt_max_deg 0.27->0.192; yaw_rms_deg 0.112->0.0994; pos_h_rms_m 0.0415->0.0418; vel_h_rms_ms 0.0804->0.0811; nees_att 0.073->0.0125 **WORSE**; nees_vel 1.72->1.62; nees_pos 0.00761->0.00779 |
| stops | gps_latency(latency_ms=200) | FAIL | tilt_rms_deg 0.0581->0.0305; tilt_max_deg 0.27->0.192; yaw_rms_deg 0.101->0.089; pos_h_rms_m 0.0445->0.0454; vel_h_rms_ms 0.0801->0.0808; nees_att 0.0715->0.0113 **WORSE**; nees_vel 1.71->1.61; nees_pos 0.00856->0.00896 |
| stops | gps_latency(ekf_latency_ms=0,latency_ms=200) | PASS | tilt_rms_deg 0.137->0.133; tilt_max_deg 0.556->0.539; yaw_rms_deg 1.02->1.04; pos_h_rms_m 0.232->0.234; vel_h_rms_ms 0.138->0.143; nees_att 0.56->0.462; nees_vel 5.02->5; nees_pos 0.229->0.233 |
| stops | gyro_bias(dps=0.2) | FAIL | tilt_rms_deg 0.057->0.0292; tilt_max_deg 0.27->0.192; yaw_rms_deg 0.112->0.0965; pos_h_rms_m 0.0408->0.041; vel_h_rms_ms 0.0798->0.0804; nees_att 0.0734->0.0124 **WORSE**; nees_vel 1.7->1.59; nees_pos 0.0072->0.00728 |
| stops | gyro_bias(dps=1.0) | FAIL | tilt_rms_deg 0.057->0.0292; tilt_max_deg 0.27->0.192; yaw_rms_deg 0.112->0.0965; pos_h_rms_m 0.0408->0.041; vel_h_rms_ms 0.0798->0.0804; nees_att 0.0734->0.0124 **WORSE**; nees_vel 1.7->1.59; nees_pos 0.0072->0.00728 |
| stops | gyro_bias(dps=1.0,from_cal=False) | FAIL | tilt_rms_deg 0.0836->0.0686; tilt_max_deg 0.59->0.591; yaw_rms_deg 0.171->0.163; pos_h_rms_m 0.0408->0.041; vel_h_rms_ms 0.0803->0.0809; nees_att 0.0743->0.0139 **WORSE**; nees_vel 1.7->1.6; nees_pos 0.00721->0.00729 |
| stops | gyro_drift(dps=0.5,over_s=60) | FAIL | tilt_rms_deg 0.0568->0.0289; tilt_max_deg 0.269->0.192; yaw_rms_deg 0.103->0.0884; pos_h_rms_m 0.0408->0.041; vel_h_rms_ms 0.0798->0.0804; nees_att 0.0733->0.0122 **WORSE**; nees_vel 1.7->1.59; nees_pos 0.0072->0.00728 |
| stops | accel_bias(axis=x,ms2=0.05) | PASS | tilt_rms_deg 0.349->0.294; tilt_max_deg 0.802->0.488 **BETTER**; yaw_rms_deg 0.999->0.453 **BETTER**; pos_h_rms_m 0.0642->0.0451; vel_h_rms_ms 0.0833->0.081; nees_att 2.14->1.11 **BETTER**; nees_vel 1.8->1.61; nees_pos 0.0178->0.00883 |
| stops | accel_bias(axis=x,ms2=0.2) | FAIL | tilt_rms_deg 1.43->1.2; tilt_max_deg 3.27->2.09 **BETTER**; yaw_rms_deg 4.17->2.03 **BETTER**; pos_h_rms_m 0.173->0.0888 **BETTER**; vel_h_rms_ms 0.121->0.0922 **BETTER**; nees_att 38.9->17.8 **BETTER**; nees_vel 2.89->1.88 **BETTER**; nees_pos 0.13->0.0339 **WORSE** |
| stops | accel_bias(axis=x,ms2=0.5) | FAIL | tilt_rms_deg 3.32->3.12; tilt_max_deg 6.35->5.64; yaw_rms_deg 8.23->5.77 **BETTER**; pos_h_rms_m 0.37->0.165 **BETTER**; vel_h_rms_ms 0.202->0.14 **BETTER**; nees_att 250->118; nees_vel 6.86->3 **BETTER**; nees_pos 0.609->0.121 **WORSE** |
| stops | accel_bias(axis=z,ms2=0.05) | FAIL | tilt_rms_deg 0.057->0.0292; tilt_max_deg 0.27->0.192; yaw_rms_deg 0.111->0.0964; pos_h_rms_m 0.0408->0.041; vel_h_rms_ms 0.0797->0.0804; nees_att 0.0734->0.0124 **WORSE**; nees_vel 1.7->1.59; nees_pos 0.00722->0.00722 |
| stops | accel_bias(axis=z,ms2=0.2) | FAIL | tilt_rms_deg 0.0571->0.0292; tilt_max_deg 0.27->0.192; yaw_rms_deg 0.111->0.0959; pos_h_rms_m 0.0408->0.0409; vel_h_rms_ms 0.0796->0.0802; nees_att 0.0734->0.0124 **WORSE**; nees_vel 1.72->1.61; nees_pos 0.00966->0.00941 |
| stops | accel_bias(axis=z,ms2=0.5) | FAIL | tilt_rms_deg 0.0571->0.0293; tilt_max_deg 0.27->0.192; yaw_rms_deg 0.11->0.0949; pos_h_rms_m 0.0407->0.0409; vel_h_rms_ms 0.0793->0.0799; nees_att 0.0733->0.0123 **WORSE**; nees_vel 1.86->1.74; nees_pos 0.0253->0.0244 |
| stops | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | tilt_rms_deg 1.72->2.02; tilt_max_deg 4.67->5.91 **WORSE**; yaw_rms_deg 7.29->8.58; pos_h_rms_m 0.536->0.415 **BETTER**; vel_h_rms_ms 0.32->0.293; nees_att 86->79.3; nees_vel 22.1->15.5; nees_pos 1.25->0.777 |
| stops | mag_bias(frac=0.05) | PASS | tilt_rms_deg 0.289->0.252; tilt_max_deg 1.05->1.13; yaw_rms_deg 1.84->1.76; pos_h_rms_m 0.0432->0.0437; vel_h_rms_ms 0.0827->0.0828; nees_att 0.962->0.522 **WORSE**; nees_vel 1.74->1.62; nees_pos 0.00801->0.00823 |
| stops | mag_bias(frac=0.15) | FAIL | tilt_rms_deg 1.24->1.14; tilt_max_deg 3.52->3.25; yaw_rms_deg 6.48->6.2; pos_h_rms_m 0.0889->0.0798; vel_h_rms_ms 0.128->0.123; nees_att 36.2->19.3; nees_vel 2.64->2.35; nees_pos 0.0364->0.029 |
| stops | mag_interference(amp=0.3) | FAIL | tilt_rms_deg 0.0779->0.0573; tilt_max_deg 0.557->0.505; yaw_rms_deg 0.105->0.0943; pos_h_rms_m 0.041->0.041; vel_h_rms_ms 0.0807->0.0813; nees_att 0.0677->0.0118 **WORSE**; nees_vel 1.5->1.41; nees_pos 0.00726->0.0073; tilt_drift_deg_s -0.00333->0.00183; recovery_s 0.151->0.00409 |
| stops | mag_interference(amp=1.0) | FAIL | tilt_rms_deg 0.0779->0.0573; tilt_max_deg 0.557->0.505; yaw_rms_deg 0.105->0.0943; pos_h_rms_m 0.041->0.041; vel_h_rms_ms 0.0807->0.0813; nees_att 0.0677->0.0118 **WORSE**; nees_vel 1.5->1.41; nees_pos 0.00726->0.0073; tilt_drift_deg_s -0.00333->0.00183; recovery_s 0.151->0.00409 |
| stops | imu_noise | FAIL | tilt_rms_deg 0.0612->0.0376; tilt_max_deg 0.278->0.2; yaw_rms_deg 0.144->0.138; pos_h_rms_m 0.0404->0.0405; vel_h_rms_ms 0.0805->0.0811; nees_att 0.0793->0.0181 **WORSE**; nees_vel 1.93->1.84; nees_pos 0.0319->0.0326 |
| stops | imu_spikes | WARN | tilt_rms_deg 0.301->0.306; tilt_max_deg 1.95->2.07; yaw_rms_deg 0.682->0.698; pos_h_rms_m 0.0434->0.043; vel_h_rms_ms 0.09->0.0902; nees_att 0.243->0.179 **WORSE**; nees_vel 2.26->2.12; nees_pos 0.0139->0.0141 |
| stops | imu_dropouts | FAIL | tilt_rms_deg 0.0574->0.0297; tilt_max_deg 0.27->0.197; yaw_rms_deg 0.111->0.0973; pos_h_rms_m 0.04->0.0405; vel_h_rms_ms 0.0802->0.0808; nees_att 0.0745->0.0129 **WORSE**; nees_vel 1.73->1.62; nees_pos 0.00694->0.00713 |
| stops | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.0576->0.0306; tilt_max_deg 0.394->0.374; yaw_rms_deg 0.117->0.102; pos_h_rms_m 0.0401->0.0402; vel_h_rms_ms 0.081->0.0817; nees_att 0.0737->0.0129 **WORSE**; nees_vel 1.66->1.56; nees_pos 0.0069->0.00692; recovery_s 0.0116->0.0197 |
| stops | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.0583->0.0305; tilt_max_deg 0.27->0.192; yaw_rms_deg 0.12->0.104; pos_h_rms_m 0.207->0.206; vel_h_rms_ms 0.0805->0.0812; nees_att 0.0743->0.0126 **WORSE**; nees_vel 1.63->1.53; nees_pos 0.0901->0.0894 |
| stops | combined_realistic | WARN | tilt_rms_deg 0.745->0.604; tilt_max_deg 2.69->2.17; yaw_rms_deg 2.89->2.24 **BETTER**; pos_h_rms_m 0.452->0.48; vel_h_rms_ms 0.112->0.106; nees_att 4.78->2.1 **BETTER**; nees_vel 2.26->2.03; nees_pos 1.08->1.2 |
| stops | cal_ideal | FAIL | tilt_rms_deg 0.057->0.0292; tilt_max_deg 0.27->0.192; yaw_rms_deg 0.112->0.0965; pos_h_rms_m 0.0408->0.041; vel_h_rms_ms 0.0798->0.0804; nees_att 0.0735->0.0124 **WORSE**; nees_vel 1.7->1.59; nees_pos 0.00718->0.00725 |
| yaw_steps | none | FAIL | tilt_rms_deg 0.0428->0.0212; tilt_max_deg 0.147->0.0697; yaw_rms_deg 0.09->0.0509; pos_h_rms_m 0.0247->0.0195; vel_h_rms_ms 0.00939->0.0081; nees_att 0.0357->0.00832 **WORSE**; nees_vel 0.0236->0.017; nees_pos 0.00275->0.00174 |
| yaw_steps | gps_dropout(duration_s=5) | FAIL | tilt_rms_deg 0.0413->0.0209; tilt_max_deg 0.151->0.0707; yaw_rms_deg 0.0844->0.0501; pos_h_rms_m 0.026->0.0196; vel_h_rms_ms 0.0102->0.00811; nees_att 0.0354->0.0083 **WORSE**; nees_vel 0.0235->0.0169; nees_pos 0.00274->0.00172; tilt_drift_deg_s 0.000651->0.00106 |
| yaw_steps | gps_dropout(duration_s=15) | FAIL | tilt_rms_deg 0.0586->0.0268; tilt_max_deg 0.359->0.157 **BETTER**; yaw_rms_deg 0.142->0.0699; pos_h_rms_m 0.131->0.0437 **BETTER**; vel_h_rms_ms 0.0396->0.0151 **BETTER**; nees_att 0.0347->0.00825 **WORSE**; nees_vel 0.0225->0.0155; nees_pos 0.00194->0.000946; tilt_drift_deg_s 0.0125->0.00529 |
| yaw_steps | gps_dropout(duration_s=30) | FAIL | tilt_rms_deg 0.192->0.0653 **BETTER**; tilt_max_deg 0.706->0.255 **BETTER**; yaw_rms_deg 0.405->0.144 **BETTER**; pos_h_rms_m 3.09->0.951 **BETTER**; vel_h_rms_ms 0.375->0.116 **BETTER**; nees_att 0.0329->0.00723 **WORSE**; nees_vel 0.0502->0.0401; nees_pos 0.0243->0.0144; tilt_drift_deg_s 0.0198->0.00598; recovery_s 0.816->0.612 |
| yaw_steps | gps_stale(duration_s=5) | FAIL | tilt_rms_deg 0.0428->0.0211; tilt_max_deg 0.147->0.0697; yaw_rms_deg 0.0899->0.0509; pos_h_rms_m 0.0247->0.0195; vel_h_rms_ms 0.00938->0.0081; nees_att 0.0357->0.00832 **WORSE**; nees_vel 0.0237->0.017; nees_pos 0.00274->0.00173; tilt_drift_deg_s 0.0134->0.00528 |
| yaw_steps | gps_stale(duration_s=15) | FAIL | tilt_rms_deg 0.043->0.0213; tilt_max_deg 0.149->0.0715; yaw_rms_deg 0.0902->0.0511; pos_h_rms_m 0.024->0.0188; vel_h_rms_ms 0.00937->0.00808; nees_att 0.0357->0.00833 **WORSE**; nees_vel 0.0236->0.017; nees_pos 0.00261->0.00164; tilt_drift_deg_s 0.00228->0.00207 |
| yaw_steps | gps_stale(duration_s=30) | FAIL | tilt_rms_deg 0.0429->0.0213; tilt_max_deg 0.149->0.0715; yaw_rms_deg 0.0902->0.0513; pos_h_rms_m 0.0225->0.0173; vel_h_rms_ms 0.00929->0.00799; nees_att 0.0357->0.00834 **WORSE**; nees_vel 0.0231->0.0166; nees_pos 0.00232->0.00142; tilt_drift_deg_s 0.000146->0.000728 |
| yaw_steps | gps_noise | WARN -> FAIL | tilt_rms_deg 0.341->0.334; tilt_max_deg 1.05->1.05; yaw_rms_deg 0.665->0.662; pos_h_rms_m 0.616->0.621; vel_h_rms_ms 0.0617->0.0617; nees_att 0.11->0.0813 **WORSE**; nees_vel 0.245->0.244; nees_pos 1.96->1.98 |
| yaw_steps | gps_outliers(frac=0.01) | FAIL | tilt_rms_deg 0.0428->0.0212; tilt_max_deg 0.147->0.0697; yaw_rms_deg 0.09->0.0509; pos_h_rms_m 0.0247->0.0195; vel_h_rms_ms 0.00939->0.0081; nees_att 0.0357->0.00832 **WORSE**; nees_vel 0.0236->0.017; nees_pos 0.00275->0.00174 |
| yaw_steps | gps_rate(hz=1.0) | FAIL | tilt_rms_deg 0.0457->0.0223; tilt_max_deg 0.164->0.0718; yaw_rms_deg 0.0897->0.0523; pos_h_rms_m 0.0945->0.0756; vel_h_rms_ms 0.0195->0.0168; nees_att 0.0347->0.00821 **WORSE**; nees_vel 0.0478->0.0327; nees_pos 0.00846->0.00541 |
| yaw_steps | gps_latency(latency_ms=100) | FAIL | tilt_rms_deg 0.047->0.0222; tilt_max_deg 0.158->0.0735; yaw_rms_deg 0.0949->0.0524; pos_h_rms_m 0.0284->0.023; vel_h_rms_ms 0.00988->0.00835; nees_att 0.0358->0.00834 **WORSE**; nees_vel 0.0249->0.0179; nees_pos 0.00365->0.00244 |
| yaw_steps | gps_latency(latency_ms=200) | FAIL | tilt_rms_deg 0.052->0.0235; tilt_max_deg 0.168->0.0772; yaw_rms_deg 0.102->0.0545; pos_h_rms_m 0.0321->0.0265; vel_h_rms_ms 0.0105->0.00862; nees_att 0.0361->0.00836 **WORSE**; nees_vel 0.0263->0.0188; nees_pos 0.00458->0.00315 |
| yaw_steps | gps_latency(ekf_latency_ms=0,latency_ms=200) | FAIL | tilt_rms_deg 0.0428->0.0211; tilt_max_deg 0.147->0.0697; yaw_rms_deg 0.0899->0.0508; pos_h_rms_m 0.0247->0.0195; vel_h_rms_ms 0.00938->0.0081; nees_att 0.0357->0.00832 **WORSE**; nees_vel 0.0236->0.0169; nees_pos 0.00282->0.00181 |
| yaw_steps | gyro_bias(dps=0.2) | FAIL | tilt_rms_deg 0.0428->0.0212; tilt_max_deg 0.147->0.0697; yaw_rms_deg 0.09->0.0509; pos_h_rms_m 0.0247->0.0195; vel_h_rms_ms 0.00939->0.0081; nees_att 0.0357->0.00832 **WORSE**; nees_vel 0.0236->0.017; nees_pos 0.00275->0.00174 |
| yaw_steps | gyro_bias(dps=1.0) | FAIL | tilt_rms_deg 0.0428->0.0212; tilt_max_deg 0.147->0.0697; yaw_rms_deg 0.09->0.0509; pos_h_rms_m 0.0247->0.0195; vel_h_rms_ms 0.00939->0.0081; nees_att 0.0357->0.00832 **WORSE**; nees_vel 0.0236->0.017; nees_pos 0.00275->0.00174 |
| yaw_steps | gyro_bias(dps=1.0,from_cal=False) | FAIL | tilt_rms_deg 0.0704->0.0596; tilt_max_deg 0.601->0.603; yaw_rms_deg 0.184->0.16; pos_h_rms_m 0.0248->0.0196; vel_h_rms_ms 0.0123->0.0113; nees_att 0.0385->0.011 **WORSE**; nees_vel 0.0261->0.0195; nees_pos 0.00276->0.00175 |
| yaw_steps | gyro_drift(dps=0.5,over_s=60) | FAIL | tilt_rms_deg 0.0432->0.0219; tilt_max_deg 0.156->0.0728; yaw_rms_deg 0.0847->0.0448; pos_h_rms_m 0.0247->0.0195; vel_h_rms_ms 0.0094->0.00811; nees_att 0.0356->0.00834 **WORSE**; nees_vel 0.0236->0.017; nees_pos 0.00275->0.00174 |
| yaw_steps | accel_bias(axis=x,ms2=0.05) | PASS | tilt_rms_deg 0.244->0.249; tilt_max_deg 0.495->0.418; yaw_rms_deg 0.215->0.158; pos_h_rms_m 0.0892->0.0586; vel_h_rms_ms 0.0828->0.0612 **BETTER**; nees_att 1.16->0.707; nees_vel 1.86->0.897 **BETTER**; nees_pos 0.0346->0.0149 **WORSE** |
| yaw_steps | accel_bias(axis=x,ms2=0.2) | FAIL | tilt_rms_deg 0.965->0.994; tilt_max_deg 1.61->1.63; yaw_rms_deg 0.727->0.566 **BETTER**; pos_h_rms_m 0.339->0.24 **BETTER**; vel_h_rms_ms 0.329->0.248 **BETTER**; nees_att 17.2->10.9; nees_vel 29.4->14.8 **BETTER**; nees_pos 0.499->0.25 **WORSE** |
| yaw_steps | accel_bias(axis=x,ms2=0.5) | FAIL | tilt_rms_deg 3.16->3.15; tilt_max_deg 6.25->5.99; yaw_rms_deg 43.8->43.4; pos_h_rms_m 0.127->0.125; vel_h_rms_ms 0.208->0.206; nees_att 98.4->55.9; nees_vel 6.83->6.09; nees_pos 0.0205->0.0212; mag_rejected 10886->4361 **BETTER** |
| yaw_steps | accel_bias(axis=z,ms2=0.05) | FAIL | tilt_rms_deg 0.0428->0.0212; tilt_max_deg 0.147->0.0697; yaw_rms_deg 0.09->0.0509; pos_h_rms_m 0.0247->0.0195; vel_h_rms_ms 0.00939->0.0081; nees_att 0.0357->0.00832 **WORSE**; nees_vel 0.0235->0.0169; nees_pos 0.00272->0.00171 |
| yaw_steps | accel_bias(axis=z,ms2=0.2) | FAIL | tilt_rms_deg 0.0428->0.0212; tilt_max_deg 0.147->0.0697; yaw_rms_deg 0.09->0.0509; pos_h_rms_m 0.0247->0.0195; vel_h_rms_ms 0.00939->0.0081; nees_att 0.0357->0.00832 **WORSE**; nees_vel 0.036->0.0293; nees_pos 0.00435->0.00334 |
| yaw_steps | accel_bias(axis=z,ms2=0.5) | FAIL | tilt_rms_deg 0.0429->0.0212; tilt_max_deg 0.147->0.0697; yaw_rms_deg 0.09->0.0509; pos_h_rms_m 0.0247->0.0195; vel_h_rms_ms 0.00939->0.00809; nees_att 0.0357->0.00833 **WORSE**; nees_vel 0.119->0.112; nees_pos 0.0153->0.0143 |
| yaw_steps | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | tilt_rms_deg 0.712->0.764; tilt_max_deg 1.43->1.97 **WORSE**; yaw_rms_deg 3.05->4.03 **WORSE**; pos_h_rms_m 0.492->0.409; vel_h_rms_ms 0.389->0.327; nees_att 11->13.1; nees_vel 39.9->25.1; nees_pos 1.04->0.714 **WORSE** |
| yaw_steps | mag_bias(frac=0.05) | WARN | tilt_rms_deg 0.049->0.0353; tilt_max_deg 0.181->0.134; yaw_rms_deg 1.75->1.75; pos_h_rms_m 0.0369->0.0295; vel_h_rms_ms 0.0146->0.0144; nees_att 0.225->0.185; nees_vel 0.0593->0.0536; nees_pos 0.006->0.00386 |
| yaw_steps | mag_bias(frac=0.15) | FAIL | tilt_rms_deg 0.174->0.191; tilt_max_deg 0.523->0.505; yaw_rms_deg 5.01->5.02; pos_h_rms_m 0.165->0.138; vel_h_rms_ms 0.0852->0.0878; nees_att 2.81->2.24; nees_vel 2.06->1.98; nees_pos 0.118->0.0829 |
| yaw_steps | mag_interference(amp=0.3) | FAIL | tilt_rms_deg 0.0388->0.0201; tilt_max_deg 0.147->0.0699; yaw_rms_deg 0.0811->0.0501; pos_h_rms_m 0.0217->0.0173; vel_h_rms_ms 0.009->0.00771; nees_att 0.0302->0.00779 **WORSE**; nees_vel 0.0218->0.0153; nees_pos 0.00213->0.00138; tilt_drift_deg_s -6e-05->0.000145; mag_rejected 1250->1249 |
| yaw_steps | mag_interference(amp=1.0) | FAIL | tilt_rms_deg 0.0388->0.0201; tilt_max_deg 0.147->0.0699; yaw_rms_deg 0.0811->0.0501; pos_h_rms_m 0.0217->0.0173; vel_h_rms_ms 0.009->0.00771; nees_att 0.0302->0.00779 **WORSE**; nees_vel 0.0218->0.0153; nees_pos 0.00213->0.00138; tilt_drift_deg_s -6e-05->0.000145; mag_rejected 1250->1249 |
| yaw_steps | imu_noise | FAIL | tilt_rms_deg 0.0469->0.029; tilt_max_deg 0.174->0.0947; yaw_rms_deg 0.091->0.0601; pos_h_rms_m 0.0237->0.0187; vel_h_rms_ms 0.0101->0.00877; nees_att 0.0361->0.00872 **WORSE**; nees_vel 0.0373->0.0296; nees_pos 0.00364->0.00271 |
| yaw_steps | imu_spikes | WARN | tilt_rms_deg 0.243->0.248; tilt_max_deg 1.2->1.26; yaw_rms_deg 0.441->0.434; pos_h_rms_m 0.063->0.0514; vel_h_rms_ms 0.0464->0.0439; nees_att 0.282->0.212 **WORSE**; nees_vel 1.72->1.61; nees_pos 0.108->0.102 |
| yaw_steps | imu_dropouts | FAIL | tilt_rms_deg 0.0437->0.022; tilt_max_deg 0.15->0.0726; yaw_rms_deg 0.0912->0.0521; pos_h_rms_m 0.025->0.0197; vel_h_rms_ms 0.00945->0.00814; nees_att 0.0362->0.0085 **WORSE**; nees_vel 0.024->0.0172; nees_pos 0.00281->0.00177 |
| yaw_steps | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.0423->0.021; tilt_max_deg 0.148->0.0697; yaw_rms_deg 0.09->0.0507; pos_h_rms_m 0.0236->0.0189; vel_h_rms_ms 0.00937->0.00808; nees_att 0.0356->0.00829 **WORSE**; nees_vel 0.0236->0.0169; nees_pos 0.00249->0.00162 |
| yaw_steps | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.0421->0.0209; tilt_max_deg 0.148->0.0697; yaw_rms_deg 0.0896->0.0507; pos_h_rms_m 0.0242->0.0196; vel_h_rms_ms 0.00936->0.00808; nees_att 0.0358->0.00836 **WORSE**; nees_vel 0.0236->0.0169; nees_pos 0.00256->0.0017 |
| yaw_steps | combined_realistic | PASS | tilt_rms_deg 0.426->0.425; tilt_max_deg 1.26->1.17; yaw_rms_deg 1.91->1.91; pos_h_rms_m 0.596->0.625; vel_h_rms_ms 0.0942->0.0791; nees_att 1.58->1.07 **BETTER**; nees_vel 1.6->0.819 **BETTER**; nees_pos 1.87->2.02 |
| yaw_steps | cal_ideal | FAIL | tilt_rms_deg 0.0428->0.0212; tilt_max_deg 0.147->0.0697; yaw_rms_deg 0.09->0.0509; pos_h_rms_m 0.0247->0.0195; vel_h_rms_ms 0.00939->0.0081; nees_att 0.0357->0.00832 **WORSE**; nees_vel 0.0235->0.0169; nees_pos 0.00273->0.00172 |
| yaw_spin | none | FAIL | tilt_rms_deg 0.0214->0.0195; tilt_max_deg 0.0512->0.0414; yaw_rms_deg 0.117->0.0528; pos_h_rms_m 0.0307->0.0247; vel_h_rms_ms 0.0111->0.00979; nees_att 0.0713->0.0115 **WORSE**; nees_vel 0.0358->0.025; nees_pos 0.00411->0.00267 |
| yaw_spin | gps_dropout(duration_s=5) | FAIL | tilt_rms_deg 0.0216->0.0199; tilt_max_deg 0.0512->0.0511; yaw_rms_deg 0.118->0.0536; pos_h_rms_m 0.0379->0.0302; vel_h_rms_ms 0.0124->0.011; nees_att 0.071->0.0114 **WORSE**; nees_vel 0.0385->0.0266; nees_pos 0.00497->0.0031; tilt_drift_deg_s 0.00376->0.00542 |
| yaw_spin | gps_dropout(duration_s=15) | FAIL | tilt_rms_deg 0.0225->0.0216; tilt_max_deg 0.0558->0.0542; yaw_rms_deg 0.121->0.0578; pos_h_rms_m 0.15->0.144; vel_h_rms_ms 0.0223->0.0226; nees_att 0.0706->0.0113 **WORSE**; nees_vel 0.0575->0.0407; nees_pos 0.0184->0.0133; tilt_drift_deg_s -8e-05->2.5e-05; recovery_s 1.02->0.608 |
| yaw_spin | gps_dropout(duration_s=30) | FAIL | tilt_rms_deg 0.024->0.0237; tilt_max_deg 0.0622->0.0579; yaw_rms_deg 0.125->0.0632; pos_h_rms_m 0.636->0.655; vel_h_rms_ms 0.0504->0.0544; nees_att 0.0701->0.0111 **WORSE**; nees_vel 0.116->0.0851; nees_pos 0.0657->0.0494; tilt_drift_deg_s 0.000135->4.5e-05; recovery_s 1.21->0.809 |
| yaw_spin | gps_stale(duration_s=5) | FAIL | tilt_rms_deg 0.0214->0.0195; tilt_max_deg 0.0512->0.0414; yaw_rms_deg 0.117->0.0528; pos_h_rms_m 0.0309->0.0249; vel_h_rms_ms 0.0111->0.0098; nees_att 0.0713->0.0115 **WORSE**; nees_vel 0.0359->0.025; nees_pos 0.00416->0.00272; tilt_drift_deg_s 0.00256->0.00355 |
| yaw_spin | gps_stale(duration_s=15) | FAIL | tilt_rms_deg 0.0215->0.0196; tilt_max_deg 0.0512->0.0414; yaw_rms_deg 0.117->0.0529; pos_h_rms_m 0.0308->0.0248; vel_h_rms_ms 0.0111->0.00979; nees_att 0.0713->0.0115 **WORSE**; nees_vel 0.0358->0.025; nees_pos 0.00415->0.0027; tilt_drift_deg_s -0.000393->-0.000542 |
| yaw_spin | gps_stale(duration_s=30) | FAIL | tilt_rms_deg 0.0215->0.0196; tilt_max_deg 0.0512->0.0414; yaw_rms_deg 0.117->0.0528; pos_h_rms_m 0.0305->0.0246; vel_h_rms_ms 0.0111->0.00977; nees_att 0.0713->0.0115 **WORSE**; nees_vel 0.0357->0.0249; nees_pos 0.00407->0.00265; tilt_drift_deg_s -1.8e-05->-2e-06 |
| yaw_spin | gps_noise | WARN -> FAIL | tilt_rms_deg 0.307->0.306; tilt_max_deg 0.942->0.945; yaw_rms_deg 0.615->0.609; pos_h_rms_m 0.666->0.671; vel_h_rms_ms 0.0577->0.058; nees_att 0.135->0.075 **WORSE**; nees_vel 0.196->0.198; nees_pos 2.13->2.15 |
| yaw_spin | gps_outliers(frac=0.01) | FAIL | tilt_rms_deg 0.0214->0.0195; tilt_max_deg 0.0512->0.0414; yaw_rms_deg 0.117->0.0528; pos_h_rms_m 0.0308->0.0248; vel_h_rms_ms 0.0112->0.00982; nees_att 0.0713->0.0115 **WORSE**; nees_vel 0.0359->0.025; nees_pos 0.00413->0.00269 |
| yaw_spin | gps_rate(hz=1.0) | FAIL | tilt_rms_deg 0.0222->0.0198; tilt_max_deg 0.0565->0.0417; yaw_rms_deg 0.116->0.0523; pos_h_rms_m 0.111->0.0913; vel_h_rms_ms 0.0237->0.0208; nees_att 0.0696->0.011 **WORSE**; nees_vel 0.0764->0.0528; nees_pos 0.0116->0.00792 |
| yaw_spin | gps_latency(latency_ms=100) | FAIL | tilt_rms_deg 0.022->0.0197; tilt_max_deg 0.0544->0.041; yaw_rms_deg 0.117->0.0532; pos_h_rms_m 0.0356->0.0294; vel_h_rms_ms 0.0114->0.01; nees_att 0.0713->0.0115 **WORSE**; nees_vel 0.0375->0.0263; nees_pos 0.0055->0.00375 |
| yaw_spin | gps_latency(latency_ms=200) | FAIL | tilt_rms_deg 0.0225->0.0199; tilt_max_deg 0.0571->0.0405; yaw_rms_deg 0.117->0.0534; pos_h_rms_m 0.0402->0.0338; vel_h_rms_ms 0.0117->0.0103; nees_att 0.0713->0.0115 **WORSE**; nees_vel 0.0391->0.0275; nees_pos 0.00705->0.00497 |
| yaw_spin | gps_latency(ekf_latency_ms=0,latency_ms=200) | FAIL | tilt_rms_deg 0.0215->0.0196; tilt_max_deg 0.0512->0.0411; yaw_rms_deg 0.117->0.0528; pos_h_rms_m 0.0307->0.0247; vel_h_rms_ms 0.0111->0.00979; nees_att 0.0713->0.0115 **WORSE**; nees_vel 0.0358->0.025; nees_pos 0.0041->0.00266 |
| yaw_spin | gyro_bias(dps=0.2) | FAIL | tilt_rms_deg 0.0214->0.0195; tilt_max_deg 0.0512->0.0414; yaw_rms_deg 0.117->0.0528; pos_h_rms_m 0.0307->0.0247; vel_h_rms_ms 0.0111->0.00979; nees_att 0.0713->0.0115 **WORSE**; nees_vel 0.0358->0.025; nees_pos 0.00411->0.00267 |
| yaw_spin | gyro_bias(dps=1.0) | FAIL | tilt_rms_deg 0.0214->0.0195; tilt_max_deg 0.0512->0.0414; yaw_rms_deg 0.117->0.0528; pos_h_rms_m 0.0307->0.0247; vel_h_rms_ms 0.0111->0.00979; nees_att 0.0713->0.0115 **WORSE**; nees_vel 0.0358->0.025; nees_pos 0.00411->0.00267 |
| yaw_spin | gyro_bias(dps=1.0,from_cal=False) | FAIL | tilt_rms_deg 0.0587->0.058; tilt_max_deg 0.593->0.593; yaw_rms_deg 0.176->0.138; pos_h_rms_m 0.0307->0.0247; vel_h_rms_ms 0.0136->0.0125; nees_att 0.0736->0.0137 **WORSE**; nees_vel 0.0382->0.0274; nees_pos 0.00411->0.00268 |
| yaw_spin | gyro_drift(dps=0.5,over_s=60) | FAIL | tilt_rms_deg 0.0219->0.0201; tilt_max_deg 0.0548->0.0402; yaw_rms_deg 0.107->0.0486; pos_h_rms_m 0.0306->0.0247; vel_h_rms_ms 0.0111->0.00979; nees_att 0.0708->0.0115 **WORSE**; nees_vel 0.0358->0.025; nees_pos 0.00411->0.00267 |
| yaw_spin | accel_bias(axis=x,ms2=0.05) | PASS | tilt_rms_deg 0.262->0.261; tilt_max_deg 0.391->0.398; yaw_rms_deg 0.373->0.266 **BETTER**; pos_h_rms_m 0.0434->0.0158; vel_h_rms_ms 0.0474->0.0393; nees_att 0.681->0.443 **WORSE**; nees_vel 0.628->0.383 **WORSE**; nees_pos 0.00816->0.00111 **WORSE** |
| yaw_spin | accel_bias(axis=x,ms2=0.2) | WARN | tilt_rms_deg 1.05->1.05; tilt_max_deg 1.58->1.57; yaw_rms_deg 1.39->1.06 **BETTER**; pos_h_rms_m 0.108->0.0625; vel_h_rms_ms 0.192->0.166; nees_att 9.21->6.75; nees_vel 10.3->6.85; nees_pos 0.0506->0.017 **WORSE** |
| yaw_spin | accel_bias(axis=x,ms2=0.5) | FAIL | tilt_rms_deg 2.55->3.07 **WORSE**; tilt_max_deg 3.93->5.4 **WORSE**; yaw_rms_deg 3.24->4.76 **WORSE**; pos_h_rms_m 0.248->0.225; vel_h_rms_ms 0.374->0.333; nees_att 56.7->63; nees_vel 30->22.3; nees_pos 0.0924->0.107; gps_resets 9->6 **BETTER**; gps_rejected 45->30 **BETTER**; mag_rejected 0->1061 **WORSE** |
| yaw_spin | accel_bias(axis=z,ms2=0.05) | FAIL | tilt_rms_deg 0.0214->0.0195; tilt_max_deg 0.0513->0.0414; yaw_rms_deg 0.117->0.0528; pos_h_rms_m 0.0307->0.0247; vel_h_rms_ms 0.0111->0.00979; nees_att 0.0713->0.0115 **WORSE**; nees_vel 0.0371->0.0263; nees_pos 0.0043->0.00286 |
| yaw_spin | accel_bias(axis=z,ms2=0.2) | FAIL | tilt_rms_deg 0.0214->0.0195; tilt_max_deg 0.0513->0.0415; yaw_rms_deg 0.117->0.0528; pos_h_rms_m 0.0307->0.0247; vel_h_rms_ms 0.0111->0.00978; nees_att 0.0713->0.0115 **WORSE**; nees_vel 0.058->0.0471; nees_pos 0.00712->0.00568 |
| yaw_spin | accel_bias(axis=z,ms2=0.5) | FAIL | tilt_rms_deg 0.0214->0.0195; tilt_max_deg 0.0514->0.0416; yaw_rms_deg 0.117->0.0528; pos_h_rms_m 0.0307->0.0247; vel_h_rms_ms 0.0111->0.00978; nees_att 0.0713->0.0115 **WORSE**; nees_vel 0.176->0.165; nees_pos 0.0229->0.0215 |
| yaw_spin | accel_bias(axis=x,from_cal=False,ms2=0.2) | WARN | tilt_rms_deg 0.97->0.966; tilt_max_deg 1.58->1.56; yaw_rms_deg 2.12->2.34; pos_h_rms_m 0.28->0.238; vel_h_rms_ms 0.274->0.258; nees_att 1.84->1.75; nees_vel 19.4->15.5; nees_pos 0.332->0.238 **WORSE** |
| yaw_spin | mag_bias(frac=0.05) | WARN | tilt_rms_deg 0.0366->0.0405; tilt_max_deg 0.113->0.144; yaw_rms_deg 1.2->1.2; pos_h_rms_m 0.0366->0.0294; vel_h_rms_ms 0.0143->0.0134; nees_att 0.179->0.11 **WORSE**; nees_vel 0.0582->0.0463; nees_pos 0.00584->0.00377 |
| yaw_spin | mag_bias(frac=0.15) | WARN | tilt_rms_deg 0.2->0.227; tilt_max_deg 0.423->0.441; yaw_rms_deg 3.45->3.48; pos_h_rms_m 0.0839->0.0658; vel_h_rms_ms 0.055->0.058; nees_att 2.31->1.77 **BETTER**; nees_vel 0.852->0.855; nees_pos 0.0305->0.0187 |
| yaw_spin | mag_interference(amp=0.3) | FAIL | tilt_rms_deg 0.0227->0.0199; tilt_max_deg 0.0812->0.0616; yaw_rms_deg 0.123->0.0568; pos_h_rms_m 0.0242->0.0196; vel_h_rms_ms 0.00985->0.00866; nees_att 0.0651->0.0104 **WORSE**; nees_vel 0.0276->0.0193; nees_pos 0.00256->0.00168; tilt_drift_deg_s -0.00362->-0.00343 |
| yaw_spin | mag_interference(amp=1.0) | FAIL | tilt_rms_deg 0.0227->0.0199; tilt_max_deg 0.0812->0.0616; yaw_rms_deg 0.123->0.0568; pos_h_rms_m 0.0242->0.0196; vel_h_rms_ms 0.00985->0.00866; nees_att 0.0651->0.0104 **WORSE**; nees_vel 0.0276->0.0193; nees_pos 0.00256->0.00168; tilt_drift_deg_s -0.00362->-0.00343 |
| yaw_spin | imu_noise | FAIL | tilt_rms_deg 0.0296->0.0291; tilt_max_deg 0.0753->0.0724; yaw_rms_deg 0.13->0.0688; pos_h_rms_m 0.0367->0.0299; vel_h_rms_ms 0.0138->0.0123; nees_att 0.0726->0.0124 **WORSE**; nees_vel 0.0579->0.0422; nees_pos 0.00611->0.00414 |
| yaw_spin | imu_spikes | PASS -> WARN | tilt_rms_deg 0.271->0.278; tilt_max_deg 1.42->1.42; yaw_rms_deg 0.516->0.511; pos_h_rms_m 0.0436->0.0381; vel_h_rms_ms 0.0484->0.0471; nees_att 0.319->0.216 **WORSE**; nees_vel 1.48->1.4; nees_pos 0.0776->0.0757 |
| yaw_spin | imu_dropouts | FAIL | tilt_rms_deg 0.0215->0.0196; tilt_max_deg 0.0511->0.041; yaw_rms_deg 0.117->0.0535; pos_h_rms_m 0.0309->0.0249; vel_h_rms_ms 0.0112->0.00981; nees_att 0.071->0.0118 **WORSE**; nees_vel 0.0362->0.0253; nees_pos 0.00417->0.00271 |
| yaw_spin | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.0215->0.0195; tilt_max_deg 0.0534->0.0414; yaw_rms_deg 0.117->0.0527; pos_h_rms_m 0.0274->0.0225; vel_h_rms_ms 0.0104->0.00928; nees_att 0.071->0.0114 **WORSE**; nees_vel 0.0312->0.0222; nees_pos 0.0033->0.00222 |
| yaw_spin | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.022->0.0201; tilt_max_deg 0.0958->0.0599; yaw_rms_deg 0.118->0.0541; pos_h_rms_m 0.0277->0.0228; vel_h_rms_ms 0.0103->0.00915; nees_att 0.0708->0.0114 **WORSE**; nees_vel 0.0304->0.0216; nees_pos 0.00305->0.00207 |
| yaw_spin | combined_realistic | PASS | tilt_rms_deg 0.397->0.397; tilt_max_deg 1.01->0.979; yaw_rms_deg 1.38->1.37; pos_h_rms_m 0.651->0.672; vel_h_rms_ms 0.0691->0.0652; nees_att 0.894->0.648 **WORSE**; nees_vel 0.613->0.431 **WORSE**; nees_pos 2.05->2.17 |
| yaw_spin | cal_ideal | FAIL | tilt_rms_deg 0.0214->0.0195; tilt_max_deg 0.0512->0.0414; yaw_rms_deg 0.117->0.0528; pos_h_rms_m 0.0307->0.0247; vel_h_rms_ms 0.0111->0.00979; nees_att 0.0713->0.0115 **WORSE**; nees_vel 0.0358->0.025; nees_pos 0.00411->0.00268 |
| patrol_long | none | FAIL | tilt_rms_deg 0.016->0.0191; tilt_max_deg 0.39->0.457; yaw_rms_deg 0.0319->0.0394; pos_h_rms_m 0.0132->0.0132; vel_h_rms_ms 0.0405->0.0405; nees_att 0.000769->0.000442; nees_vel 0.238->0.222; nees_pos 0.000813->0.000815 |
| patrol_long | gps_dropout(duration_s=30) | FAIL | tilt_rms_deg 0.0487->0.0269; tilt_max_deg 0.582->0.457; yaw_rms_deg 0.0938->0.0541; pos_h_rms_m 0.711->0.347 **BETTER**; vel_h_rms_ms 0.0966->0.0568 **BETTER**; nees_att 0.000771->0.000398; nees_vel 0.232->0.217; nees_pos 0.00087->0.000882; tilt_drift_deg_s 0.0149->0.00558 |
| patrol_long | gps_noise | FAIL | tilt_rms_deg 0.351->0.35; tilt_max_deg 2.38->2.3; yaw_rms_deg 0.703->0.714; pos_h_rms_m 1.12->1.12; vel_h_rms_ms 0.0746->0.0748; nees_att 0.0752->0.0803; nees_vel 0.478->0.461; nees_pos 5.71->5.71 |
| patrol_long | gyro_drift(dps=0.5,over_s=60) | FAIL | tilt_rms_deg 0.015->0.018; tilt_max_deg 0.359->0.426; yaw_rms_deg 0.0296->0.0372; pos_h_rms_m 0.0131->0.0132; vel_h_rms_ms 0.0405->0.0405; nees_att 0.00077->0.000445; nees_vel 0.238->0.222; nees_pos 0.000812->0.000814 |
| patrol_long | accel_bias(axis=x,ms2=0.05) | PASS | tilt_rms_deg 0.292->0.293; tilt_max_deg 0.525->0.572; yaw_rms_deg 0.0679->0.0518; pos_h_rms_m 0.0126->0.013; vel_h_rms_ms 0.0405->0.0405; nees_att 2.05->1.16 **BETTER**; nees_vel 0.238->0.222; nees_pos 0.000767->0.000804 |
| patrol_long | mag_bias(frac=0.05) | PASS | tilt_rms_deg 0.0236->0.0282; tilt_max_deg 0.49->0.565; yaw_rms_deg 0.969->1.13; pos_h_rms_m 0.0165->0.0158; vel_h_rms_ms 0.0405->0.0406; nees_att 0.657->0.59; nees_vel 0.238->0.223; nees_pos 0.00121->0.00112 |
| patrol_long | imu_noise | FAIL | tilt_rms_deg 0.0258->0.0277; tilt_max_deg 0.316->0.379; yaw_rms_deg 0.051->0.0682; pos_h_rms_m 0.0168->0.0167; vel_h_rms_ms 0.0409->0.041; nees_att 0.00269->0.00284; nees_vel 0.288->0.272; nees_pos 0.00944->0.00942 |
| patrol_long | imu_spikes | WARN | tilt_rms_deg 0.294->0.297; tilt_max_deg 3.61->3.52; yaw_rms_deg 0.625->0.835 **WORSE**; pos_h_rms_m 0.0512->0.046; vel_h_rms_ms 0.0674->0.0659; nees_att 0.33->0.428 **BETTER**; nees_vel 2.35->2.22; nees_pos 0.0967->0.0946; mag_rejected 28->12 |
| patrol_long | combined_realistic | WARN | tilt_rms_deg 0.459->0.458; tilt_max_deg 2.31->2.17; yaw_rms_deg 1.24->1.36; pos_h_rms_m 1.12->1.12; vel_h_rms_ms 0.0749->0.075; nees_att 2.93->1.9 **BETTER**; nees_vel 0.515->0.497; nees_pos 5.81->5.8 |
| patrol_long | cal_ideal | FAIL | tilt_rms_deg 0.016->0.0191; tilt_max_deg 0.39->0.457; yaw_rms_deg 0.0319->0.0394; pos_h_rms_m 0.0132->0.0132; vel_h_rms_ms 0.0405->0.0405; nees_att 0.000769->0.000442; nees_vel 0.237->0.222; nees_pos 0.000785->0.000787 |
| takeoff_land | none | FAIL | tilt_rms_deg 0.00515->0.00466; tilt_max_deg 0.0141->0.0126; yaw_rms_deg 0.0208->0.0202; pos_h_rms_m 0.000533->0.000433; vel_h_rms_ms 0.000767->0.00069; nees_att 0.00026->0.000122; nees_vel 1.15->1.15 |
| takeoff_land | gps_dropout(duration_s=5) | FAIL | tilt_rms_deg 0.00524->0.00478; tilt_max_deg 0.0164->0.0143; yaw_rms_deg 0.0205->0.0198; pos_h_rms_m 0.00224->0.00204; vel_h_rms_ms 0.000943->0.00089; nees_att 0.000259->0.000122; nees_vel 1.07->1.07; nees_pos 0.0274->0.0274; tilt_drift_deg_s 0.000273->0.000394 |
| takeoff_land | gps_dropout(duration_s=15) | FAIL | tilt_rms_deg 0.0172->0.00582; tilt_max_deg 0.109->0.0281; yaw_rms_deg 0.0432->0.0211; pos_h_rms_m 0.0728->0.0165 **BETTER**; vel_h_rms_ms 0.0173->0.00376; nees_att 0.000255->0.00012; nees_vel 0.888->0.888; nees_pos 0.023->0.023; tilt_drift_deg_s 0.00433->0.000405 |
| takeoff_land | gps_dropout(duration_s=30) | FAIL | tilt_rms_deg 0.14->0.111; tilt_max_deg 0.417->0.412; yaw_rms_deg 0.288->0.229; pos_h_rms_m 2.1->1.73; vel_h_rms_ms 0.265->0.214; nees_att 0.000258->0.000121; nees_vel 1.5->1.51; nees_pos 0.432->0.434; tilt_drift_deg_s 0.014->0.0116 |
| takeoff_land | gps_stale(duration_s=5) | FAIL | tilt_rms_deg 0.00521->0.00471; tilt_max_deg 0.0152->0.0126; yaw_rms_deg 0.0208->0.0203; pos_h_rms_m 0.00118->0.00114; vel_h_rms_ms 0.000762->0.000687; nees_att 0.00026->0.000123; nees_vel 1.02->1.02; nees_pos 0.0399->0.0399; tilt_drift_deg_s 0.000258->0.000164 |
| takeoff_land | gps_stale(duration_s=15) | FAIL | tilt_rms_deg 0.00518->0.00468; tilt_max_deg 0.0152->0.0123; yaw_rms_deg 0.0208->0.0202; pos_h_rms_m 0.0034->0.00335; vel_h_rms_ms 0.000871->0.000784; nees_att 0.000261->0.000123; nees_vel 0.531->0.531; nees_pos 0.883->0.883; tilt_drift_deg_s 8.9e-05->4.9e-05 |
| takeoff_land | gps_stale(duration_s=30) | FAIL | tilt_rms_deg 0.00531->0.00484; tilt_max_deg 0.0152->0.0123; yaw_rms_deg 0.0209->0.0204; pos_h_rms_m 0.00609->0.00602; vel_h_rms_ms 0.00103->0.000941; nees_att 0.000261->0.000123; nees_vel 1.22->1.22; nees_pos 0.719->0.719; tilt_drift_deg_s 2.8e-05->4.1e-05 |
| takeoff_land | gps_noise | FAIL | tilt_rms_deg 0.306->0.306; tilt_max_deg 1.02->1.02; yaw_rms_deg 0.607->0.607; pos_h_rms_m 0.414->0.414; vel_h_rms_ms 0.0557->0.0558; nees_att 0.0607->0.0594; nees_vel 1.21->1.21; nees_pos 0.956->0.954 |
| takeoff_land | gps_outliers(frac=0.01) | FAIL | tilt_rms_deg 0.00531->0.00483; tilt_max_deg 0.0143->0.0134; yaw_rms_deg 0.0209->0.0203; pos_h_rms_m 0.0177->0.0177; vel_h_rms_ms 0.000779->0.000703; nees_att 0.000261->0.000123; nees_vel 1.15->1.15 |
| takeoff_land | gps_rate(hz=1.0) | FAIL | tilt_rms_deg 0.00523->0.00479; tilt_max_deg 0.0156->0.0146; yaw_rms_deg 0.0211->0.0205; pos_h_rms_m 0.00142->0.00116; vel_h_rms_ms 0.0011->0.000986; nees_att 0.000252->0.000117; nees_vel 1.13->1.13; nees_pos 0.0709->0.0709 |
| takeoff_land | gps_latency(latency_ms=100) | FAIL | tilt_rms_deg 0.00532->0.00486; tilt_max_deg 0.0141->0.0134; yaw_rms_deg 0.0209->0.0204; pos_h_rms_m 0.00064->0.000517; vel_h_rms_ms 0.000818->0.000743; nees_att 0.000261->0.000123; nees_vel 1.2->1.2 |
| takeoff_land | gps_latency(latency_ms=200) | FAIL | tilt_rms_deg 0.00544->0.005; tilt_max_deg 0.0143->0.0144; yaw_rms_deg 0.021->0.0205; pos_h_rms_m 0.000822->0.000692; vel_h_rms_ms 0.000868->0.000793; nees_att 0.000262->0.000124; nees_vel 1.2->1.2 |
| takeoff_land | gps_latency(ekf_latency_ms=0,latency_ms=200) | FAIL | tilt_rms_deg 0.00516->0.00467; tilt_max_deg 0.0143->0.0124; yaw_rms_deg 0.0208->0.0202; pos_h_rms_m 0.000542->0.000443; vel_h_rms_ms 0.000779->0.000704; nees_att 0.00026->0.000122; nees_vel 1.2->1.2 |
| takeoff_land | gyro_bias(dps=0.2) | FAIL | tilt_rms_deg 0.00515->0.00466; tilt_max_deg 0.0141->0.0126; yaw_rms_deg 0.0208->0.0202; pos_h_rms_m 0.000533->0.000433; vel_h_rms_ms 0.000767->0.00069; nees_att 0.00026->0.000122; nees_vel 1.15->1.15 |
| takeoff_land | gyro_bias(dps=1.0) | FAIL | tilt_rms_deg 0.00515->0.00466; tilt_max_deg 0.0141->0.0126; yaw_rms_deg 0.0208->0.0202; pos_h_rms_m 0.000533->0.000433; vel_h_rms_ms 0.000767->0.00069; nees_att 0.00026->0.000122; nees_vel 1.15->1.15 |
| takeoff_land | gyro_bias(dps=1.0,from_cal=False) | FAIL | tilt_rms_deg 0.0654->0.0653; tilt_max_deg 0.598->0.598; yaw_rms_deg 0.269->0.251; pos_h_rms_m 0.00173->0.00171; vel_h_rms_ms 0.00936->0.00935; nees_att 0.00635->0.00545; nees_vel 1.16->1.16 |
| takeoff_land | gyro_drift(dps=0.5,over_s=60) | FAIL | tilt_rms_deg 0.00598->0.00543; tilt_max_deg 0.0169->0.0145; yaw_rms_deg 0.0147->0.014; pos_h_rms_m 0.000532->0.000431; vel_h_rms_ms 0.000807->0.000733; nees_att 0.000264->0.000125; nees_vel 1.15->1.15 |
| takeoff_land | accel_bias(axis=x,ms2=0.05) | PASS | tilt_rms_deg 0.293->0.292; tilt_max_deg 0.304->0.303; yaw_rms_deg 0.0468->0.0462; pos_h_rms_m 0.000922->0.000807; vel_h_rms_ms 0.00129->0.00122; nees_att 2.03->1.15 **BETTER**; nees_vel 1.16->1.16; nees_pos 0.0595->0.0595 |
| takeoff_land | accel_bias(axis=x,ms2=0.2) | FAIL | tilt_rms_deg 1.17->1.17; tilt_max_deg 1.18->1.18; yaw_rms_deg 0.13->0.129; pos_h_rms_m 0.00245->0.00227; vel_h_rms_ms 0.0041->0.00401; nees_att 32.4->18.4; nees_vel 1.16->1.16; nees_pos 0.0593->0.0593 |
| takeoff_land | accel_bias(axis=x,ms2=0.5) | FAIL | tilt_rms_deg 2.92->2.92; tilt_max_deg 2.93->2.93; yaw_rms_deg 0.317->0.317; pos_h_rms_m 0.00562->0.00529; vel_h_rms_ms 0.01->0.00984; nees_att 202->115; nees_vel 1.17->1.16; nees_pos 0.0581->0.0581 |
| takeoff_land | accel_bias(axis=z,ms2=0.05) | FAIL | tilt_rms_deg 0.00515->0.00466; tilt_max_deg 0.0141->0.0126; yaw_rms_deg 0.0208->0.0202; pos_h_rms_m 0.000533->0.000433; vel_h_rms_ms 0.000767->0.00069; nees_att 0.00026->0.000122; nees_vel 1.1->1.1 |
| takeoff_land | accel_bias(axis=z,ms2=0.2) | FAIL | tilt_rms_deg 0.00515->0.00466; tilt_max_deg 0.0141->0.0127; yaw_rms_deg 0.0209->0.0203; pos_h_rms_m 0.000533->0.000433; vel_h_rms_ms 0.000767->0.00069; nees_att 0.00026->0.000122; nees_vel 0.974->0.974; nees_pos 0.0387->0.0386 |
| takeoff_land | accel_bias(axis=z,ms2=0.5) | FAIL | tilt_rms_deg 0.00515->0.00466; tilt_max_deg 0.014->0.0127; yaw_rms_deg 0.021->0.0204; pos_h_rms_m 0.000533->0.000433; vel_h_rms_ms 0.000767->0.00069; nees_att 0.00026->0.000123; nees_vel 0.809->0.809; nees_pos 0.0177->0.0177 |
| takeoff_land | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | tilt_rms_deg 0.768->0.803; tilt_max_deg 0.968->0.973; yaw_rms_deg 6.55->8.69 **WORSE**; pos_h_rms_m 0.469->0.393; vel_h_rms_ms 0.282->0.246; nees_att 80.5->99.7; nees_vel 19.4->13.6; nees_pos 0.972->0.691 **WORSE** |
| takeoff_land | mag_bias(frac=0.05) | WARN | tilt_rms_deg 0.00515->0.00466; tilt_max_deg 0.014->0.0125; yaw_rms_deg 3.51->3.51; pos_h_rms_m 0.000513->0.000406; vel_h_rms_ms 0.000767->0.000693; nees_att 0.631->0.626; nees_vel 1.15->1.15 |
| takeoff_land | mag_bias(frac=0.15) | FAIL | tilt_rms_deg 0.00519->0.00471; tilt_max_deg 0.014->0.0123; yaw_rms_deg 10->10; pos_h_rms_m 0.000551->0.000445; vel_h_rms_ms 0.000784->0.00072; nees_att 5.14->5.1; nees_vel 1.15->1.15 |
| takeoff_land | mag_interference(amp=0.3) | FAIL | tilt_rms_deg 0.00574->0.00531; tilt_max_deg 0.0276->0.0212; yaw_rms_deg 0.023->0.0242; pos_h_rms_m 0.000392->0.000304; vel_h_rms_ms 0.000708->0.000643; nees_att 0.000241->0.000119; nees_vel 1.15->1.15; nees_pos 0.0594->0.0594; tilt_drift_deg_s -0.00103->-0.00153 |
| takeoff_land | mag_interference(amp=1.0) | FAIL | tilt_rms_deg 0.00574->0.00531; tilt_max_deg 0.0276->0.0212; yaw_rms_deg 0.023->0.0242; pos_h_rms_m 0.000392->0.000304; vel_h_rms_ms 0.000708->0.000643; nees_att 0.000241->0.000119; nees_vel 1.15->1.15; nees_pos 0.0594->0.0594; tilt_drift_deg_s -0.00103->-0.00153 |
| takeoff_land | imu_noise | FAIL | tilt_rms_deg 0.0251->0.0252; tilt_max_deg 0.0629->0.0621; yaw_rms_deg 0.051->0.0495; pos_h_rms_m 0.00232->0.0022; vel_h_rms_ms 0.00408->0.00404; nees_att 0.00151->0.00108; nees_vel 1.24->1.24; nees_pos 0.0694->0.0694 |
| takeoff_land | imu_spikes | FAIL | tilt_rms_deg 0.272->0.275; tilt_max_deg 1.15->1.18; yaw_rms_deg 0.547->0.533; pos_h_rms_m 0.02->0.0188; vel_h_rms_ms 0.0322->0.0317; nees_att 0.0982->0.0889; nees_vel 4.53->4.51; nees_pos 0.287->0.287 |
| takeoff_land | imu_dropouts | FAIL | tilt_rms_deg 0.00507->0.00461; tilt_max_deg 0.0135->0.0131; yaw_rms_deg 0.0208->0.0204; pos_h_rms_m 0.000499->0.00039; vel_h_rms_ms 0.000765->0.000691; nees_att 0.000254->0.000121; nees_vel 1.17->1.17; nees_pos 0.0585->0.0585 |
| takeoff_land | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.00516->0.00466; tilt_max_deg 0.0139->0.0123; yaw_rms_deg 0.0206->0.02; pos_h_rms_m 0.000387->0.00032; vel_h_rms_ms 0.000656->0.000607; nees_att 0.000259->0.000122; nees_vel 1.03->1.03; nees_pos 0.0444->0.0444 |
| takeoff_land | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.00805->0.00791; tilt_max_deg 0.0475->0.0483; yaw_rms_deg 0.0249->0.0249; pos_h_rms_m 0.000735->0.000666; vel_h_rms_ms 0.000698->0.000659; nees_att 0.000262->0.000125; nees_vel 0.961->0.961 |
| takeoff_land | combined_realistic | WARN | tilt_rms_deg 0.425->0.424; tilt_max_deg 1.08->1.08; yaw_rms_deg 3.61->3.61; pos_h_rms_m 0.413->0.413; vel_h_rms_ms 0.0557->0.0559; nees_att 2.66->1.8 **BETTER**; nees_vel 1.3->1.29; nees_pos 0.971->0.969 |
| takeoff_land | cal_ideal | FAIL | tilt_rms_deg 0.00515->0.00466; tilt_max_deg 0.0141->0.0126; yaw_rms_deg 0.0208->0.0202; pos_h_rms_m 0.000534->0.000434; vel_h_rms_ms 0.000767->0.00069; nees_att 0.00026->0.000122; nees_vel 1.15->1.15; nees_pos 0.0587->0.0586 |
| hover_ct | none | FAIL | tilt_rms_deg 0.00906->0.00677; tilt_max_deg 0.0419->0.0334; yaw_rms_deg 0.0579->0.0281; pos_h_rms_m 0.00436->0.00221; vel_h_rms_ms 0.00195->0.00134; nees_att 0.0134->0.00207 **WORSE**; nees_vel 0.00301->0.00246; nees_pos 0.000442->0.000381 |
| hover_ct | gps_dropout(duration_s=5) | FAIL | tilt_rms_deg 0.00907->0.0068; tilt_max_deg 0.0419->0.0334; yaw_rms_deg 0.0579->0.028; pos_h_rms_m 0.00598->0.00308; vel_h_rms_ms 0.0021->0.00141; nees_att 0.0134->0.00207 **WORSE**; nees_vel 0.00319->0.00258; nees_pos 0.000548->0.000459; tilt_drift_deg_s -0.000205->0.000328 |
| hover_ct | gps_dropout(duration_s=15) | FAIL | tilt_rms_deg 0.0128->0.00743; tilt_max_deg 0.0616->0.0334; yaw_rms_deg 0.0619->0.0294; pos_h_rms_m 0.0411->0.0141; vel_h_rms_ms 0.00931->0.00324; nees_att 0.0134->0.00206 **WORSE**; nees_vel 0.00371->0.00292; nees_pos 0.000976->0.000722; tilt_drift_deg_s 0.00293->0.00079 |
| hover_ct | gps_dropout(duration_s=30) | FAIL | tilt_rms_deg 0.0885->0.0353 **BETTER**; tilt_max_deg 0.495->0.22 **BETTER**; yaw_rms_deg 0.19->0.0792 **BETTER**; pos_h_rms_m 1.21->0.483 **BETTER**; vel_h_rms_ms 0.158->0.0628 **BETTER**; nees_att 0.0134->0.00206 **WORSE**; nees_vel 0.00483->0.00399; nees_pos 0.00219->0.00163; tilt_drift_deg_s 0.00938->0.00377 |
| hover_ct | gps_stale(duration_s=5) | FAIL | tilt_rms_deg 0.00908->0.0068; tilt_max_deg 0.0419->0.0334; yaw_rms_deg 0.0579->0.0281; pos_h_rms_m 0.00469->0.00253; vel_h_rms_ms 0.00196->0.00136; nees_att 0.0134->0.00207 **WORSE**; nees_vel 0.00303->0.00247; nees_pos 0.000456->0.000389; tilt_drift_deg_s -0.000165->0.000112 |
| hover_ct | gps_stale(duration_s=15) | FAIL | tilt_rms_deg 0.00911->0.00686; tilt_max_deg 0.0419->0.0334; yaw_rms_deg 0.0579->0.0282; pos_h_rms_m 0.00526->0.00319; vel_h_rms_ms 0.00198->0.00138; nees_att 0.0134->0.00207 **WORSE**; nees_vel 0.00304->0.00247; nees_pos 0.000482->0.000407; tilt_drift_deg_s -6.9e-05->-5.3e-05 |
| hover_ct | gps_stale(duration_s=30) | FAIL | tilt_rms_deg 0.00913->0.00687; tilt_max_deg 0.0419->0.0334; yaw_rms_deg 0.058->0.0283; pos_h_rms_m 0.00547->0.00344; vel_h_rms_ms 0.00198->0.00139; nees_att 0.0134->0.00207 **WORSE**; nees_vel 0.00303->0.00247; nees_pos 0.000495->0.000416; tilt_drift_deg_s -1.1e-05->4e-06 |
| hover_ct | gps_noise | FAIL | tilt_rms_deg 0.305->0.305; tilt_max_deg 0.829->0.838; yaw_rms_deg 0.606->0.605; pos_h_rms_m 0.489->0.49; vel_h_rms_ms 0.0576->0.0578; nees_att 0.0733->0.0622; nees_vel 0.202->0.2; nees_pos 1.27->1.27 |
| hover_ct | gps_outliers(frac=0.01) | FAIL | tilt_rms_deg 0.00906->0.00677; tilt_max_deg 0.0419->0.0334; yaw_rms_deg 0.0579->0.0281; pos_h_rms_m 0.00436->0.00221; vel_h_rms_ms 0.00195->0.00135; nees_att 0.0134->0.00207 **WORSE**; nees_vel 0.00302->0.00246; nees_pos 0.000444->0.000383 |
| hover_ct | gps_rate(hz=1.0) | FAIL | tilt_rms_deg 0.0114->0.00754; tilt_max_deg 0.051->0.0393; yaw_rms_deg 0.0593->0.0282; pos_h_rms_m 0.0152->0.00705; vel_h_rms_ms 0.00406->0.00239; nees_att 0.0132->0.00197 **WORSE**; nees_vel 0.00556->0.00429; nees_pos 0.000888->0.00072 |
| hover_ct | gps_latency(latency_ms=100) | FAIL | tilt_rms_deg 0.00943->0.00705; tilt_max_deg 0.043->0.0338; yaw_rms_deg 0.0583->0.0285; pos_h_rms_m 0.00513->0.00262; vel_h_rms_ms 0.00204->0.00141; nees_att 0.0134->0.00209 **WORSE**; nees_vel 0.0031->0.00251; nees_pos 0.000507->0.000423 |
| hover_ct | gps_latency(latency_ms=200) | FAIL | tilt_rms_deg 0.00975->0.00727; tilt_max_deg 0.0462->0.0336; yaw_rms_deg 0.0587->0.0289; pos_h_rms_m 0.00566->0.00284; vel_h_rms_ms 0.00213->0.00147; nees_att 0.0134->0.0021 **WORSE**; nees_vel 0.00322->0.0026; nees_pos 0.000601->0.000498 |
| hover_ct | gps_latency(ekf_latency_ms=0,latency_ms=200) | FAIL | tilt_rms_deg 0.00906->0.00679; tilt_max_deg 0.0417->0.0333; yaw_rms_deg 0.0579->0.028; pos_h_rms_m 0.00437->0.00222; vel_h_rms_ms 0.00195->0.00136; nees_att 0.0134->0.00206 **WORSE**; nees_vel 0.00278->0.00222; nees_pos 0.000495->0.000434 |
| hover_ct | gyro_bias(dps=0.2) | FAIL | tilt_rms_deg 0.00906->0.00677; tilt_max_deg 0.0419->0.0334; yaw_rms_deg 0.0579->0.0281; pos_h_rms_m 0.00436->0.00221; vel_h_rms_ms 0.00195->0.00134; nees_att 0.0134->0.00207 **WORSE**; nees_vel 0.00301->0.00246; nees_pos 0.000442->0.000381 |
| hover_ct | gyro_bias(dps=1.0) | FAIL | tilt_rms_deg 0.00906->0.00677; tilt_max_deg 0.0419->0.0334; yaw_rms_deg 0.0579->0.0281; pos_h_rms_m 0.00436->0.00221; vel_h_rms_ms 0.00195->0.00134; nees_att 0.0134->0.00207 **WORSE**; nees_vel 0.00301->0.00246; nees_pos 0.000442->0.000381 |
| hover_ct | gyro_bias(dps=1.0,from_cal=False) | FAIL | tilt_rms_deg 0.0509->0.0505; tilt_max_deg 0.6->0.598; yaw_rms_deg 0.135->0.12; pos_h_rms_m 0.0046->0.00263; vel_h_rms_ms 0.0074->0.00726; nees_att 0.0153->0.00393 **WORSE**; nees_vel 0.00502->0.00445; nees_pos 0.000447->0.000386 |
| hover_ct | gyro_drift(dps=0.5,over_s=60) | FAIL | tilt_rms_deg 0.00949->0.00743; tilt_max_deg 0.0396->0.0368; yaw_rms_deg 0.0536->0.0253; pos_h_rms_m 0.00435->0.0022; vel_h_rms_ms 0.00196->0.00137; nees_att 0.0132->0.00201 **WORSE**; nees_vel 0.00302->0.00246; nees_pos 0.000441->0.000381 |
| hover_ct | accel_bias(axis=x,ms2=0.05) | PASS | tilt_rms_deg 0.236->0.261; tilt_max_deg 0.404->0.398; yaw_rms_deg 0.129->0.0811; pos_h_rms_m 0.098->0.0422 **BETTER**; vel_h_rms_ms 0.0438->0.0252; nees_att 1.27->0.882; nees_vel 0.522->0.152 **WORSE**; nees_pos 0.0419->0.00805 **WORSE** |
| hover_ct | accel_bias(axis=x,ms2=0.2) | FAIL | tilt_rms_deg 0.944->1.04 **WORSE**; tilt_max_deg 1.61->1.57; yaw_rms_deg 0.385->0.294; pos_h_rms_m 0.382->0.165 **BETTER**; vel_h_rms_ms 0.173->0.1 **BETTER**; nees_att 19.9->14.1; nees_vel 8.07->2.38 **BETTER**; nees_pos 0.632->0.118 **WORSE** |
| hover_ct | accel_bias(axis=x,ms2=0.5) | FAIL | tilt_rms_deg 2.81->2.93; tilt_max_deg 4.84->4.74; yaw_rms_deg 5.03->4.68 **BETTER**; pos_h_rms_m 0.258->0.0919 **BETTER**; vel_h_rms_ms 0.162->0.116 **BETTER**; nees_att 247->142; nees_vel 6.03->2.19 **BETTER**; nees_pos 0.277->0.0349 **WORSE**; mag_rejected 1213->475 **BETTER** |
| hover_ct | accel_bias(axis=z,ms2=0.05) | FAIL | tilt_rms_deg 0.00906->0.00677; tilt_max_deg 0.0419->0.0334; yaw_rms_deg 0.0579->0.0281; pos_h_rms_m 0.00436->0.00221; vel_h_rms_ms 0.00195->0.00134; nees_att 0.0134->0.00207 **WORSE**; nees_vel 0.00107->0.000515; nees_pos 0.000117->5.6e-05 |
| hover_ct | accel_bias(axis=z,ms2=0.2) | FAIL | tilt_rms_deg 0.00907->0.00678; tilt_max_deg 0.042->0.0335; yaw_rms_deg 0.0579->0.028; pos_h_rms_m 0.00436->0.00221; vel_h_rms_ms 0.00195->0.00135; nees_att 0.0134->0.00207 **WORSE**; nees_vel 0.0127->0.0122; nees_pos 0.00178->0.00171 |
| hover_ct | accel_bias(axis=z,ms2=0.5) | FAIL | tilt_rms_deg 0.0091->0.00679; tilt_max_deg 0.0421->0.0335; yaw_rms_deg 0.0578->0.028; pos_h_rms_m 0.00435->0.00221; vel_h_rms_ms 0.00195->0.00135; nees_att 0.0134->0.00207 **WORSE**; nees_vel 0.115->0.114; nees_pos 0.0169->0.0169 |
| hover_ct | accel_bias(axis=x,from_cal=False,ms2=0.2) | WARN | tilt_rms_deg 0.68->0.739; tilt_max_deg 1.61->1.58; yaw_rms_deg 1.52->2.02 **WORSE**; pos_h_rms_m 0.717->0.547 **BETTER**; vel_h_rms_ms 0.33->0.282; nees_att 9.08->6.77; nees_vel 28.4->18.6; nees_pos 2.22->1.29 **BETTER** |
| hover_ct | mag_bias(frac=0.05) | WARN -> FAIL | tilt_rms_deg 0.0235->0.0275; tilt_max_deg 0.0899->0.126; yaw_rms_deg 1.13->1.14; pos_h_rms_m 0.0166->0.0168; vel_h_rms_ms 0.00929->0.01; nees_att 0.113->0.0944 **WORSE**; nees_vel 0.0256->0.0268; nees_pos 0.00155->0.00158 |
| hover_ct | mag_bias(frac=0.15) | WARN | tilt_rms_deg 0.164->0.179; tilt_max_deg 0.482->0.502; yaw_rms_deg 3.31->3.38; pos_h_rms_m 0.136->0.124; vel_h_rms_ms 0.0754->0.0753; nees_att 3.62->2.98 **BETTER**; nees_vel 1.55->1.4; nees_pos 0.0805->0.0671 |
| hover_ct | mag_interference(amp=0.3) | FAIL | tilt_rms_deg 0.00932->0.00689; tilt_max_deg 0.0419->0.0334; yaw_rms_deg 0.0578->0.0282; pos_h_rms_m 0.00417->0.00216; vel_h_rms_ms 0.00193->0.00134; nees_att 0.0133->0.00202 **WORSE**; nees_vel 0.00297->0.00245; nees_pos 0.000435->0.00038; tilt_drift_deg_s -0.00255->-0.00183 |
| hover_ct | mag_interference(amp=1.0) | FAIL | tilt_rms_deg 0.00932->0.00689; tilt_max_deg 0.0419->0.0334; yaw_rms_deg 0.0578->0.0282; pos_h_rms_m 0.00417->0.00216; vel_h_rms_ms 0.00193->0.00134; nees_att 0.0133->0.00202 **WORSE**; nees_vel 0.00297->0.00245; nees_pos 0.000435->0.00038; tilt_drift_deg_s -0.00255->-0.00183 |
| hover_ct | imu_noise | FAIL | tilt_rms_deg 0.0238->0.0232; tilt_max_deg 0.093->0.0842; yaw_rms_deg 0.0726->0.051; pos_h_rms_m 0.00811->0.00611; vel_h_rms_ms 0.0055->0.00499; nees_att 0.0144->0.00337 **WORSE**; nees_vel 0.162->0.161; nees_pos 0.0235->0.0234 |
| hover_ct | imu_spikes | WARN | tilt_rms_deg 0.262->0.267; tilt_max_deg 1.9->1.85; yaw_rms_deg 0.502->0.51; pos_h_rms_m 0.0396->0.034; vel_h_rms_ms 0.0555->0.053; nees_att 0.276->0.236; nees_vel 1.18->1.04; nees_pos 0.0144->0.0126 |
| hover_ct | imu_dropouts | FAIL | tilt_rms_deg 0.0092->0.00692; tilt_max_deg 0.042->0.0335; yaw_rms_deg 0.0583->0.0286; pos_h_rms_m 0.00429->0.00216; vel_h_rms_ms 0.00193->0.00134; nees_att 0.0134->0.0021 **WORSE**; nees_vel 0.00301->0.00247; nees_pos 0.000444->0.000385 |
| hover_ct | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.00911->0.00681; tilt_max_deg 0.0419->0.0334; yaw_rms_deg 0.058->0.0282; pos_h_rms_m 0.00426->0.00218; vel_h_rms_ms 0.00192->0.00133; nees_att 0.0134->0.00207 **WORSE**; nees_vel 0.00267->0.00215; nees_pos 0.00035->0.000292 |
| hover_ct | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.00915->0.00683; tilt_max_deg 0.0419->0.0334; yaw_rms_deg 0.0582->0.0282; pos_h_rms_m 0.0043->0.00226; vel_h_rms_ms 0.00192->0.00134; nees_att 0.0135->0.00209 **WORSE**; nees_vel 0.00266->0.00213; nees_pos 0.000316->0.000265 |
| hover_ct | combined_realistic | PASS | tilt_rms_deg 0.382->0.4; tilt_max_deg 0.806->0.812; yaw_rms_deg 1.29->1.3; pos_h_rms_m 0.414->0.461; vel_h_rms_ms 0.0723->0.0625; nees_att 1.44->1.06 **BETTER**; nees_vel 0.893->0.507 **WORSE**; nees_pos 1.1->1.28 |
| hover_ct | cal_ideal | FAIL | tilt_rms_deg 0.00906->0.00677; tilt_max_deg 0.0419->0.0334; yaw_rms_deg 0.0579->0.0281; pos_h_rms_m 0.00436->0.00221; vel_h_rms_ms 0.00194->0.00134; nees_att 0.0134->0.00207 **WORSE**; nees_vel 0.00261->0.00206; nees_pos 0.000377->0.000317 |
| box_ct | none | FAIL | tilt_rms_deg 0.0108->0.00966; tilt_max_deg 0.0518->0.0435; yaw_rms_deg 0.0597->0.0334; pos_h_rms_m 0.00633->0.00678; vel_h_rms_ms 0.0151->0.0152; nees_att 0.0141->0.0024 **WORSE**; nees_vel 0.0435->0.0407; nees_pos 0.000517->0.000542 |
| box_ct | gps_dropout(duration_s=5) | FAIL | tilt_rms_deg 0.0125->0.012; tilt_max_deg 0.0518->0.0457; yaw_rms_deg 0.0601->0.0345; pos_h_rms_m 0.00788->0.00911; vel_h_rms_ms 0.0153->0.0153; nees_att 0.0141->0.00241 **WORSE**; nees_vel 0.0399->0.0373; nees_pos 0.000585->0.000637; tilt_drift_deg_s 0.00443->0.00578 |
| box_ct | gps_dropout(duration_s=15) | FAIL | tilt_rms_deg 0.0206->0.0236; tilt_max_deg 0.1->0.0959; yaw_rms_deg 0.0652->0.0477; pos_h_rms_m 0.0954->0.119; vel_h_rms_ms 0.024->0.0279; nees_att 0.0141->0.00239 **WORSE**; nees_vel 0.0366->0.034; nees_pos 0.000875->0.000926; tilt_drift_deg_s 0.00236->0.00395 |
| box_ct | gps_dropout(duration_s=30) | FAIL | tilt_rms_deg 0.113->0.0635; tilt_max_deg 0.605->0.249 **BETTER**; yaw_rms_deg 0.232->0.122 **BETTER**; pos_h_rms_m 2.07->1.28 **BETTER**; vel_h_rms_ms 0.24->0.14 **BETTER**; nees_att 0.0141->0.00238 **WORSE**; nees_vel 0.0268->0.025; nees_pos 0.00154->0.00161; tilt_drift_deg_s 0.0129->0.00578 |
| box_ct | gps_stale(duration_s=5) | FAIL | tilt_rms_deg 0.16->0.16; tilt_max_deg 1.12->1.12; yaw_rms_deg 0.313->0.308; pos_h_rms_m 0.718->0.719; vel_h_rms_ms 0.0874->0.0895; nees_att 0.0309->0.0187; nees_vel 1.69->1.64; nees_pos 2.23->2.23; tilt_drift_deg_s 0.00116->0.00177 |
| box_ct | gps_stale(duration_s=15) | FAIL | tilt_rms_deg 0.371->0.372; tilt_max_deg 2->2; yaw_rms_deg 0.746->0.745; pos_h_rms_m 1.89->1.89; vel_h_rms_ms 0.22->0.223; nees_att 0.0886->0.0743; nees_vel 8.93->8.54; nees_pos 15.2->15.3; tilt_drift_deg_s 0.0542->0.0544; recovery_s 5.41->5.22 |
| box_ct | gps_stale(duration_s=30) | WARN | tilt_rms_deg 0.444->0.444; tilt_max_deg 2.12->2.12; yaw_rms_deg 0.882->0.88; pos_h_rms_m 3.86->3.86; vel_h_rms_ms 0.314->0.318; nees_att 0.152->0.136; nees_vel 13.2->12.8; nees_pos 63.7->63.7; tilt_drift_deg_s 0.00908->0.00918 |
| box_ct | gps_noise | FAIL | tilt_rms_deg 0.337->0.338; tilt_max_deg 1.17->1.17; yaw_rms_deg 0.673->0.672; pos_h_rms_m 0.53->0.531; vel_h_rms_ms 0.0659->0.0662; nees_att 0.0901->0.0778; nees_vel 0.292->0.29; nees_pos 1.55->1.55 |
| box_ct | gps_outliers(frac=0.01) | FAIL | tilt_rms_deg 0.0108->0.00965; tilt_max_deg 0.0518->0.0435; yaw_rms_deg 0.0597->0.0334; pos_h_rms_m 0.00629->0.00675; vel_h_rms_ms 0.0151->0.0152; nees_att 0.0141->0.0024 **WORSE**; nees_vel 0.0435->0.0406; nees_pos 0.000513->0.000539 |
| box_ct | gps_rate(hz=1.0) | FAIL | tilt_rms_deg 0.012->0.0097; tilt_max_deg 0.0556->0.0384; yaw_rms_deg 0.0608->0.0334; pos_h_rms_m 0.00942->0.00657; vel_h_rms_ms 0.0153->0.0152; nees_att 0.0139->0.00237 **WORSE**; nees_vel 0.0217->0.0201; nees_pos 0.000847->0.000807 |
| box_ct | gps_latency(latency_ms=100) | FAIL | tilt_rms_deg 0.0119->0.0111; tilt_max_deg 0.0505->0.045; yaw_rms_deg 0.0608->0.0354; pos_h_rms_m 0.00669->0.00732; vel_h_rms_ms 0.0152->0.0153; nees_att 0.0141->0.00244 **WORSE**; nees_vel 0.0441->0.0412; nees_pos 0.000628->0.000666 |
| box_ct | gps_latency(latency_ms=200) | FAIL | tilt_rms_deg 0.0135->0.0131; tilt_max_deg 0.0501->0.0529; yaw_rms_deg 0.0623->0.0379; pos_h_rms_m 0.00664->0.0071; vel_h_rms_ms 0.0153->0.0154; nees_att 0.0141->0.00249 **WORSE**; nees_vel 0.044->0.0411; nees_pos 0.000628->0.000655 |
| box_ct | gps_latency(ekf_latency_ms=0,latency_ms=200) | FAIL | tilt_rms_deg 0.0903->0.0911; tilt_max_deg 0.357->0.366; yaw_rms_deg 0.188->0.183; pos_h_rms_m 0.0982->0.0988; vel_h_rms_ms 0.0316->0.032; nees_att 0.0196->0.008 **WORSE**; nees_vel 0.136->0.131; nees_pos 0.0419->0.0423 |
| box_ct | gyro_bias(dps=0.2) | FAIL | tilt_rms_deg 0.0108->0.00966; tilt_max_deg 0.0518->0.0435; yaw_rms_deg 0.0597->0.0334; pos_h_rms_m 0.00633->0.00678; vel_h_rms_ms 0.0151->0.0152; nees_att 0.0141->0.0024 **WORSE**; nees_vel 0.0435->0.0407; nees_pos 0.000517->0.000542 |
| box_ct | gyro_bias(dps=1.0) | FAIL | tilt_rms_deg 0.0108->0.00966; tilt_max_deg 0.0518->0.0435; yaw_rms_deg 0.0597->0.0334; pos_h_rms_m 0.00633->0.00678; vel_h_rms_ms 0.0151->0.0152; nees_att 0.0141->0.0024 **WORSE**; nees_vel 0.0435->0.0407; nees_pos 0.000517->0.000542 |
| box_ct | gyro_bias(dps=1.0,from_cal=False) | FAIL | tilt_rms_deg 0.0514->0.0511; tilt_max_deg 0.763->0.759; yaw_rms_deg 0.145->0.123; pos_h_rms_m 0.00652->0.00693; vel_h_rms_ms 0.0168->0.0168; nees_att 0.0162->0.00429 **WORSE**; nees_vel 0.0455->0.0426; nees_pos 0.000524->0.000548 |
| box_ct | gyro_drift(dps=0.5,over_s=60) | FAIL | tilt_rms_deg 0.0111->0.01; tilt_max_deg 0.045->0.0466; yaw_rms_deg 0.0553->0.0306; pos_h_rms_m 0.00634->0.00679; vel_h_rms_ms 0.0152->0.0152; nees_att 0.0139->0.00233 **WORSE**; nees_vel 0.0435->0.0407; nees_pos 0.000517->0.000543 |
| box_ct | accel_bias(axis=x,ms2=0.05) | PASS | tilt_rms_deg 0.237->0.262; tilt_max_deg 0.405->0.397; yaw_rms_deg 0.13->0.0806; pos_h_rms_m 0.0938->0.0386 **BETTER**; vel_h_rms_ms 0.045->0.0283; nees_att 1.27->0.885; nees_vel 0.529->0.176 **WORSE**; nees_pos 0.0383->0.00679 **WORSE** |
| box_ct | accel_bias(axis=x,ms2=0.2) | FAIL | tilt_rms_deg 0.944->1.04 **WORSE**; tilt_max_deg 1.6->1.58; yaw_rms_deg 0.388->0.293; pos_h_rms_m 0.379->0.161 **BETTER**; vel_h_rms_ms 0.172->0.1 **BETTER**; nees_att 19.9->14.1; nees_vel 7.99->2.37 **BETTER**; nees_pos 0.62->0.113 **WORSE** |
| box_ct | accel_bias(axis=x,ms2=0.5) | FAIL | tilt_rms_deg 2.81->2.93; tilt_max_deg 4.85->4.74; yaw_rms_deg 5.01->4.67 **BETTER**; pos_h_rms_m 0.265->0.0932 **BETTER**; vel_h_rms_ms 0.166->0.118 **BETTER**; nees_att 246->142; nees_vel 6.32->2.32 **BETTER**; nees_pos 0.294->0.0368 **WORSE**; mag_rejected 1200->470 **BETTER** |
| box_ct | accel_bias(axis=z,ms2=0.05) | FAIL | tilt_rms_deg 0.0108->0.00963; tilt_max_deg 0.0518->0.0434; yaw_rms_deg 0.0597->0.0334; pos_h_rms_m 0.00633->0.00677; vel_h_rms_ms 0.0151->0.0152; nees_att 0.0141->0.0024 **WORSE**; nees_vel 0.0411->0.0383; nees_pos 0.000198->0.000223 |
| box_ct | accel_bias(axis=z,ms2=0.2) | FAIL | tilt_rms_deg 0.0107->0.00955; tilt_max_deg 0.0518->0.0432; yaw_rms_deg 0.0596->0.0333; pos_h_rms_m 0.00632->0.00676; vel_h_rms_ms 0.0151->0.0152; nees_att 0.0141->0.0024 **WORSE**; nees_vel 0.0493->0.0465; nees_pos 0.00155->0.00157 |
| box_ct | accel_bias(axis=z,ms2=0.5) | FAIL | tilt_rms_deg 0.0106->0.00939; tilt_max_deg 0.0518->0.0428; yaw_rms_deg 0.0595->0.0331; pos_h_rms_m 0.00629->0.00674; vel_h_rms_ms 0.0151->0.0152; nees_att 0.0141->0.0024 **WORSE**; nees_vel 0.135->0.132; nees_pos 0.0146->0.0147 |
| box_ct | accel_bias(axis=x,from_cal=False,ms2=0.2) | WARN | tilt_rms_deg 0.682->0.741; tilt_max_deg 1.58->1.55; yaw_rms_deg 1.55->2.05 **WORSE**; pos_h_rms_m 0.713->0.543 **BETTER**; vel_h_rms_ms 0.329->0.281; nees_att 9.14->6.82; nees_vel 28.2->18.4; nees_pos 2.19->1.27 **BETTER** |
| box_ct | mag_bias(frac=0.05) | WARN -> FAIL | tilt_rms_deg 0.0245->0.0286; tilt_max_deg 0.0848->0.127; yaw_rms_deg 1.13->1.14; pos_h_rms_m 0.0218->0.0219; vel_h_rms_ms 0.0187->0.0191; nees_att 0.115->0.0954 **WORSE**; nees_vel 0.0758->0.0734; nees_pos 0.0024->0.00241 |
| box_ct | mag_bias(frac=0.15) | WARN | tilt_rms_deg 0.166->0.18; tilt_max_deg 0.475->0.492; yaw_rms_deg 3.31->3.37; pos_h_rms_m 0.142->0.13; vel_h_rms_ms 0.0791->0.0788; nees_att 3.6->2.96 **BETTER**; nees_vel 1.68->1.52; nees_pos 0.0871->0.0727 |
| box_ct | mag_interference(amp=0.3) | FAIL | tilt_rms_deg 0.0113->0.0104; tilt_max_deg 0.0518->0.0435; yaw_rms_deg 0.0597->0.0337; pos_h_rms_m 0.00634->0.00677; vel_h_rms_ms 0.0151->0.0152; nees_att 0.0139->0.00233 **WORSE**; nees_vel 0.0375->0.0358; nees_pos 0.000517->0.000541; tilt_drift_deg_s -0.00101->-0.0015 |
| box_ct | mag_interference(amp=1.0) | FAIL | tilt_rms_deg 0.0113->0.0104; tilt_max_deg 0.0518->0.0435; yaw_rms_deg 0.0597->0.0337; pos_h_rms_m 0.00634->0.00677; vel_h_rms_ms 0.0151->0.0152; nees_att 0.0139->0.00233 **WORSE**; nees_vel 0.0375->0.0358; nees_pos 0.000517->0.000541; tilt_drift_deg_s -0.00101->-0.0015 |
| box_ct | imu_noise | FAIL | tilt_rms_deg 0.025->0.025; tilt_max_deg 0.0785->0.0691; yaw_rms_deg 0.0761->0.0576; pos_h_rms_m 0.00812->0.00765; vel_h_rms_ms 0.0162->0.0161; nees_att 0.0152->0.00387 **WORSE**; nees_vel 0.234->0.23; nees_pos 0.0271->0.0271 |
| box_ct | imu_spikes | PASS -> WARN | tilt_rms_deg 0.269->0.279; tilt_max_deg 1.7->1.73; yaw_rms_deg 0.506->0.503; pos_h_rms_m 0.0277->0.0245; vel_h_rms_ms 0.0464->0.0454; nees_att 0.329->0.275 **WORSE**; nees_vel 0.653->0.59; nees_pos 0.00756->0.00682 |
| box_ct | imu_dropouts | FAIL | tilt_rms_deg 0.0109->0.00971; tilt_max_deg 0.0531->0.0444; yaw_rms_deg 0.0598->0.0337; pos_h_rms_m 0.00616->0.00648; vel_h_rms_ms 0.0151->0.0152; nees_att 0.0141->0.00246 **WORSE**; nees_vel 0.0438->0.0409; nees_pos 0.000511->0.000529 |
| box_ct | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.0113->0.0104; tilt_max_deg 0.0686->0.076; yaw_rms_deg 0.0601->0.0342; pos_h_rms_m 0.00662->0.00707; vel_h_rms_ms 0.0152->0.0153; nees_att 0.0141->0.00242 **WORSE**; nees_vel 0.0418->0.0395; nees_pos 0.00045->0.000477 |
| box_ct | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.0143->0.0136; tilt_max_deg 0.089->0.0905; yaw_rms_deg 0.0619->0.0371; pos_h_rms_m 0.0165->0.0158; vel_h_rms_ms 0.0153->0.0154; nees_att 0.0142->0.00242 **WORSE**; nees_vel 0.0414->0.0393; nees_pos 0.000918->0.000867 |
| box_ct | combined_realistic | PASS | tilt_rms_deg 0.421->0.436; tilt_max_deg 1.22->1.23; yaw_rms_deg 1.32->1.33; pos_h_rms_m 0.462->0.505; vel_h_rms_ms 0.0759->0.0699; nees_att 1.51->1.11 **BETTER**; nees_vel 0.843->0.583 **WORSE**; nees_pos 1.42->1.6 |
| box_ct | cal_ideal | FAIL | tilt_rms_deg 0.0108->0.00965; tilt_max_deg 0.0518->0.0435; yaw_rms_deg 0.0597->0.0334; pos_h_rms_m 0.00633->0.00678; vel_h_rms_ms 0.0151->0.0152; nees_att 0.0141->0.0024 **WORSE**; nees_vel 0.0431->0.0403; nees_pos 0.000457->0.000483 |
| ref_live1 | none | WARN -> FAIL | tilt_rms_deg 0.187->0.181; tilt_max_deg 0.379->0.36; yaw_rms_deg 0.232->0.268; pos_h_rms_m 0.068->0.0672; vel_h_rms_ms 0.204->0.204; nees_att 0.184->0.0873 **WORSE**; nees_vel 7.99->7.31; nees_pos 0.0188->0.0183 |
