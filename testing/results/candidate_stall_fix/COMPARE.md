# EKF suite comparison

baseline `86193e6` dirty (filter {'state estimation/FINAL_gps.py': 'faff4dbb1e3b', 'state estimation/calibration.py': '1709e77d2a42'}) vs candidate `0893e46` dirty (filter {'state estimation/FINAL_gps.py': 'f30ff01319ed', 'state estimation/calibration.py': '1709e77d2a42'})

**77 metric improvements, 57 metric regressions, 2 overall-grade changes** across 267 matched runs.


## Regressions

- **box / none** `nees_vel`: 0.235 -> 0.156 (None -> None)
- **box / none** `nees_pos`: 0.00225 -> 0.000593 (None -> None)
- **box / gps_dropout(duration_s=5)** `nees_vel`: 0.229 -> 0.152 (None -> None)
- **box / gps_dropout(duration_s=5)** `nees_pos`: 0.00201 -> 0.000508 (None -> None)
- **box / gps_dropout(duration_s=15)** `nees_vel`: 0.227 -> 0.149 (None -> None)
- **box / gps_outliers(frac=0.01)** `nees_vel`: 0.235 -> 0.156 (None -> None)
- **box / gps_outliers(frac=0.01)** `nees_pos`: 0.00223 -> 0.000579 (None -> None)
- **box / gyro_bias(dps=0.2)** `nees_vel`: 0.235 -> 0.156 (None -> None)
- **box / gyro_bias(dps=0.2)** `nees_pos`: 0.00225 -> 0.000592 (None -> None)
- **box / gyro_bias(dps=1.0)** `nees_vel`: 0.234 -> 0.155 (None -> None)
- **box / gyro_bias(dps=1.0)** `nees_pos`: 0.00224 -> 0.00059 (None -> None)
- **box / gyro_bias(dps=1.0,from_cal=False)** `nees_vel`: 0.235 -> 0.156 (None -> None)
- **box / gyro_bias(dps=1.0,from_cal=False)** `nees_pos`: 0.00225 -> 0.000593 (None -> None)
- **box / gyro_drift(dps=0.5,over_s=60)** `nees_vel`: 0.235 -> 0.156 (None -> None)
- **box / gyro_drift(dps=0.5,over_s=60)** `nees_pos`: 0.00225 -> 0.000593 (None -> None)
- **box / accel_bias(axis=x,ms2=0.05)** `nees_vel`: 0.262 -> 0.186 (None -> None)
- **box / accel_bias(axis=x,ms2=0.5)** `nees_vel`: 0.294 -> 0.215 (None -> None)
- **box / accel_bias(axis=z,ms2=0.05)** `nees_vel`: 0.235 -> 0.156 (None -> None)
- **box / accel_bias(axis=z,ms2=0.05)** `nees_pos`: 0.00225 -> 0.000593 (None -> None)
- **box / accel_bias(axis=z,ms2=0.2)** `nees_vel`: 0.235 -> 0.156 (None -> None)
- **box / accel_bias(axis=z,ms2=0.2)** `nees_pos`: 0.00225 -> 0.000592 (None -> None)
- **box / accel_bias(axis=z,ms2=0.5)** `nees_vel`: 0.234 -> 0.155 (None -> None)
- **box / accel_bias(axis=z,ms2=0.5)** `nees_pos`: 0.00224 -> 0.000591 (None -> None)
- **box / mag_bias(frac=0.05)** `nees_vel`: 0.319 -> 0.229 (None -> None)
- **box / mag_bias(frac=0.05)** `nees_pos`: 0.00253 -> 0.000694 (None -> None)
- **box / mag_interference(amp=0.3)** `nees_vel`: 0.225 -> 0.148 (None -> None)
- **box / mag_interference(amp=0.3)** `nees_pos`: 0.00225 -> 0.000606 (None -> None)
- **box / mag_interference(amp=1.0)** `nees_vel`: 0.225 -> 0.148 (None -> None)
- **box / mag_interference(amp=1.0)** `nees_pos`: 0.00225 -> 0.000606 (None -> None)
- **box / imu_dropouts** `nees_vel`: 0.236 -> 0.159 (None -> None)
- **box / imu_dropouts** `nees_pos`: 0.00223 -> 0.000594 (None -> None)
- **box / imu_gap(gap_s=0.2)** `nees_vel`: 0.234 -> 0.152 (None -> None)
- **box / imu_gap(gap_s=0.2)** `nees_pos`: 0.00216 -> 0.000543 (None -> None)
- **box / imu_gap(gap_s=1.0)** `nees_vel`: 0.596 -> 0.153 (None -> None)
- **box / imu_gap(gap_s=1.0)** `nees_pos`: 0.00792 -> 0.00115 (None -> None)
- **box / cal_ideal** `nees_vel`: 0.231 -> 0.153 (None -> None)
- **circle_slow / imu_gap(gap_s=0.2)** `nees_pos`: 0.0221 -> 0.004 (None -> None)
- **circle_slow / imu_gap(gap_s=1.0)** `nees_vel`: 0.565 -> 0.294 (None -> None)
- **circle_slow / imu_gap(gap_s=1.0)** `nees_pos`: 0.593 -> 0.00406 (None -> None)
- **circle_fast / imu_gap(gap_s=0.2)** `nees_pos`: 0.0688 -> 0.0275 (None -> None)
- **circle_fast / imu_gap(gap_s=1.0)** `nees_pos`: 0.234 -> 0.0266 (None -> None)
- **patrol_long / none** `nees_vel`: 0.531 -> 0.24 (None -> None)
- **patrol_long / none** `nees_pos`: 0.0326 -> 0.000806 (None -> None)
- **patrol_long / gps_dropout(duration_s=30)** `nees_vel`: 0.526 -> 0.234 (None -> None)
- **patrol_long / gps_dropout(duration_s=30)** `nees_pos`: 0.0341 -> 0.000914 (None -> None)
- **patrol_long / gps_noise** `nees_vel`: 0.813 -> 0.476 (PASS -> PASS)
- **patrol_long / gyro_drift(dps=0.5,over_s=60)** `nees_vel`: 0.531 -> 0.24 (None -> None)
- **patrol_long / gyro_drift(dps=0.5,over_s=60)** `nees_pos`: 0.0326 -> 0.000805 (None -> None)
- **patrol_long / accel_bias(axis=x,ms2=0.05)** `nees_vel`: 0.835 -> 0.504 (None -> None)
- **patrol_long / accel_bias(axis=x,ms2=0.05)** `nees_pos`: 0.0318 -> 0.00356 (None -> None)
- **patrol_long / mag_bias(frac=0.05)** `nees_vel`: 0.69 -> 0.372 (None -> None)
- **patrol_long / mag_bias(frac=0.05)** `nees_pos`: 0.0301 -> 0.00233 (None -> None)
- **patrol_long / imu_noise** `nees_pos`: 0.0656 -> 0.0366 (None -> None)
- **patrol_long / cal_ideal** `nees_att`: 0.072 -> 0.00164 (FAIL -> FAIL)
- **patrol_long / cal_ideal** `nees_vel`: 0.535 -> 0.237 (None -> None)
- **patrol_long / cal_ideal** `nees_pos`: 0.0281 -> 0.00077 (None -> None)
- **yaw_steps / imu_gap(gap_s=1.0)** `nees_pos`: 0.226 -> 0.156 (None -> None)

## Overall grade changes

- circle_fast / imu_gap(gap_s=1.0): FAIL -> WARN
- patrol_long / imu_spikes: FAIL -> WARN

## Improvements

- box / imu_gap(gap_s=1.0) `tilt_max_deg`: 1.43 -> 0.739 (PASS -> PASS)
- box / imu_gap(gap_s=1.0) `recovery_s`: 1.22 -> 0.00411 (PASS -> PASS)
- box / combined_realistic `nees_vel`: 3.26 -> 2.94 (WARN -> PASS)
- circle_slow / imu_gap(gap_s=1.0) `tilt_max_deg`: 1.76 -> 0.771 (PASS -> PASS)
- circle_slow / imu_gap(gap_s=1.0) `pos_h_rms_m`: 0.371 -> 0.02 (WARN -> PASS)
- circle_slow / imu_gap(gap_s=1.0) `recovery_s`: 2.03 -> 4.1e-05 (PASS -> PASS)
- circle_fast / imu_gap(gap_s=0.2) `tilt_max_deg`: 2.33 -> 1.08 (PASS -> PASS)
- circle_fast / imu_gap(gap_s=0.2) `nees_vel`: 2.53 -> 1.63 (None -> None)
- circle_fast / imu_gap(gap_s=0.2) `recovery_s`: 2.42 -> 0.000776 (PASS -> PASS)
- circle_fast / imu_gap(gap_s=1.0) `tilt_rms_deg`: 2.64 -> 0.679 (WARN -> PASS)
- circle_fast / imu_gap(gap_s=1.0) `tilt_max_deg`: 15.6 -> 2.1 (FAIL -> PASS)
- circle_fast / imu_gap(gap_s=1.0) `pos_h_rms_m`: 0.259 -> 0.0849 (PASS -> PASS)
- circle_fast / imu_gap(gap_s=1.0) `vel_h_rms_ms`: 0.315 -> 0.116 (None -> None)
- circle_fast / imu_gap(gap_s=1.0) `nees_att`: 14.4 -> 6.55 (FAIL -> WARN)
- circle_fast / imu_gap(gap_s=1.0) `nees_vel`: 6.56 -> 1.61 (None -> None)
- circle_fast / imu_gap(gap_s=1.0) `recovery_s`: 2.63 -> 0.489 (PASS -> PASS)
- circle_fast / imu_gap(gap_s=1.0) `gps_resets`: 2 -> 0 (FAIL -> PASS)
- circle_fast / imu_gap(gap_s=1.0) `gps_rejected`: 10 -> 0 (None -> None)
- circle_fast / imu_gap(gap_s=1.0) `mag_rejected`: 656 -> 0 (None -> None)
- stops / imu_gap(gap_s=0.2) `nees_vel`: 3.45 -> 2.18 (None -> None)
- stops / imu_gap(gap_s=1.0) `vel_h_rms_ms`: 0.283 -> 0.109 (None -> None)
- stops / imu_gap(gap_s=1.0) `nees_vel`: 16.2 -> 2.13 (None -> None)
- stops / imu_gap(gap_s=1.0) `gps_resets`: 1 -> 0 (WARN -> PASS)
- stops / imu_gap(gap_s=1.0) `gps_rejected`: 5 -> 0 (None -> None)
- patrol_long / none `tilt_rms_deg`: 0.326 -> 0.194 (PASS -> PASS)
- patrol_long / none `tilt_max_deg`: 6.28 -> 0.397 (WARN -> PASS)
- patrol_long / none `pos_h_rms_m`: 0.089 -> 0.0133 (PASS -> PASS)
- patrol_long / none `gps_resets`: 1 -> 0 (WARN -> PASS)
- patrol_long / none `gps_rejected`: 5 -> 0 (None -> None)
- patrol_long / none `mag_rejected`: 430 -> 0 (None -> None)
- patrol_long / gps_dropout(duration_s=30) `tilt_rms_deg`: 0.329 -> 0.198 (PASS -> PASS)
- patrol_long / gps_dropout(duration_s=30) `tilt_max_deg`: 6.28 -> 0.594 (WARN -> PASS)
- patrol_long / gps_dropout(duration_s=30) `gps_resets`: 1 -> 0 (None -> None)
- patrol_long / gps_dropout(duration_s=30) `gps_rejected`: 5 -> 0 (None -> None)
- patrol_long / gps_dropout(duration_s=30) `mag_rejected`: 430 -> 0 (None -> None)
- patrol_long / gps_noise `tilt_max_deg`: 4.79 -> 2.87 (WARN -> PASS)
- patrol_long / gps_noise `mag_rejected`: 300 -> 0 (None -> None)
- patrol_long / gyro_drift(dps=0.5,over_s=60) `tilt_rms_deg`: 0.326 -> 0.194 (PASS -> PASS)
- patrol_long / gyro_drift(dps=0.5,over_s=60) `tilt_max_deg`: 6.28 -> 0.393 (WARN -> PASS)
- patrol_long / gyro_drift(dps=0.5,over_s=60) `pos_h_rms_m`: 0.089 -> 0.0133 (PASS -> PASS)
- patrol_long / gyro_drift(dps=0.5,over_s=60) `gps_resets`: 1 -> 0 (WARN -> PASS)
- patrol_long / gyro_drift(dps=0.5,over_s=60) `gps_rejected`: 5 -> 0 (None -> None)
- patrol_long / gyro_drift(dps=0.5,over_s=60) `mag_rejected`: 430 -> 0 (None -> None)
- patrol_long / accel_bias(axis=x,ms2=0.05) `tilt_rms_deg`: 0.501 -> 0.389 (PASS -> PASS)
- patrol_long / accel_bias(axis=x,ms2=0.05) `tilt_max_deg`: 7.91 -> 0.724 (WARN -> PASS)
- patrol_long / accel_bias(axis=x,ms2=0.05) `pos_h_rms_m`: 0.0879 -> 0.0288 (PASS -> PASS)
- patrol_long / accel_bias(axis=x,ms2=0.05) `gps_resets`: 1 -> 0 (WARN -> PASS)
- patrol_long / accel_bias(axis=x,ms2=0.05) `gps_rejected`: 5 -> 0 (None -> None)
- patrol_long / accel_bias(axis=x,ms2=0.05) `mag_rejected`: 440 -> 0 (None -> None)
- patrol_long / mag_bias(frac=0.05) `tilt_rms_deg`: 0.391 -> 0.263 (PASS -> PASS)
- patrol_long / mag_bias(frac=0.05) `tilt_max_deg`: 7.21 -> 0.583 (WARN -> PASS)
- patrol_long / mag_bias(frac=0.05) `pos_h_rms_m`: 0.0854 -> 0.0229 (PASS -> PASS)
- patrol_long / mag_bias(frac=0.05) `gps_resets`: 1 -> 0 (WARN -> PASS)
- patrol_long / mag_bias(frac=0.05) `gps_rejected`: 5 -> 0 (None -> None)
- patrol_long / mag_bias(frac=0.05) `mag_rejected`: 440 -> 0 (None -> None)
- patrol_long / imu_noise `tilt_max_deg`: 9.84 -> 3.76 (FAIL -> WARN)
- patrol_long / imu_noise `gps_resets`: 1 -> 0 (WARN -> PASS)
- patrol_long / imu_noise `gps_rejected`: 5 -> 0 (None -> None)
- patrol_long / imu_noise `mag_rejected`: 440 -> 0 (None -> None)
- patrol_long / imu_spikes `tilt_rms_deg`: 0.527 -> 0.331 (PASS -> PASS)
- patrol_long / imu_spikes `tilt_max_deg`: 10.2 -> 3.09 (FAIL -> WARN)
- patrol_long / imu_spikes `gps_resets`: 1 -> 0 (WARN -> PASS)
- patrol_long / imu_spikes `gps_rejected`: 5 -> 0 (None -> None)
- patrol_long / imu_spikes `mag_rejected`: 484 -> 28 (None -> None)
- patrol_long / combined_realistic `mag_rejected`: 294 -> 0 (None -> None)
- patrol_long / cal_ideal `tilt_rms_deg`: 0.305 -> 0.0149 (PASS -> PASS)
- patrol_long / cal_ideal `tilt_max_deg`: 7.53 -> 0.338 (WARN -> PASS)
- patrol_long / cal_ideal `pos_h_rms_m`: 0.0827 -> 0.013 (PASS -> PASS)
- patrol_long / cal_ideal `gps_resets`: 1 -> 0 (WARN -> PASS)
- patrol_long / cal_ideal `gps_rejected`: 5 -> 0 (None -> None)
- patrol_long / cal_ideal `mag_rejected`: 440 -> 0 (None -> None)
- yaw_spin / imu_gap(gap_s=0.2) `tilt_max_deg`: 1.74 -> 1.22 (PASS -> PASS)
- yaw_spin / imu_gap(gap_s=0.2) `recovery_s`: 0.608 -> 0.000164 (PASS -> PASS)
- yaw_spin / imu_gap(gap_s=1.0) `tilt_max_deg`: 2.97 -> 1.22 (PASS -> PASS)
- yaw_spin / imu_gap(gap_s=1.0) `yaw_rms_deg`: 7.91 -> 2.27 (FAIL -> WARN)
- yaw_spin / imu_gap(gap_s=1.0) `recovery_s`: 5.6 -> 0.000353 (WARN -> PASS)
- yaw_spin / imu_gap(gap_s=1.0) `mag_rejected`: 1400 -> 0 (None -> None)

## All matched runs (metrics that changed at all)

| mission | fault | overall | changes |
|---|---|---|---|
| hover | none | FAIL | - |
| hover | gps_dropout(duration_s=5) | FAIL | - |
| hover | gps_dropout(duration_s=15) | FAIL | - |
| hover | gps_dropout(duration_s=30) | FAIL | - |
| hover | gps_stale(duration_s=5) | FAIL | - |
| hover | gps_stale(duration_s=15) | FAIL | - |
| hover | gps_stale(duration_s=30) | FAIL | - |
| hover | gps_noise | FAIL | - |
| hover | gps_outliers(frac=0.01) | FAIL | - |
| hover | gps_rate(hz=1.0) | FAIL | - |
| hover | gyro_bias(dps=0.2) | WARN | - |
| hover | gyro_bias(dps=1.0) | FAIL | - |
| hover | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| hover | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| hover | accel_bias(axis=x,ms2=0.05) | FAIL | - |
| hover | accel_bias(axis=x,ms2=0.2) | FAIL | - |
| hover | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| hover | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| hover | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| hover | accel_bias(axis=z,ms2=0.5) | WARN | - |
| hover | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | - |
| hover | mag_bias(frac=0.05) | FAIL | - |
| hover | mag_bias(frac=0.15) | FAIL | - |
| hover | mag_interference(amp=0.3) | FAIL | - |
| hover | mag_interference(amp=1.0) | FAIL | - |
| hover | imu_noise | FAIL | - |
| hover | imu_spikes | FAIL | - |
| hover | imu_dropouts | FAIL | - |
| hover | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.755->0.755; yaw_rms_deg 2.58->2.58; pos_h_rms_m 0.00154->0.00152; vel_h_rms_ms 0.00152->0.00153; nees_att 11.7->11.6; nees_vel 0.00123->0.00102; nees_pos 0.000165->0.000116 |
| hover | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.755->0.755; yaw_rms_deg 2.58->2.58; pos_h_rms_m 0.00152->0.00172; nees_att 11.7->11.6; nees_vel 0.00123->0.001; nees_pos 0.000163->9.9e-05 |
| hover | combined_realistic | FAIL | - |
| hover | cal_ideal | FAIL | - |
| box | none | FAIL | tilt_rms_deg 0.697->0.697; tilt_max_deg 0.74->0.74; yaw_rms_deg 0.998->0.998; pos_h_rms_m 0.022->0.0102; vel_h_rms_ms 0.0306->0.0306; nees_att 19.8->19.8; nees_vel 0.235->0.156 **WORSE**; nees_pos 0.00225->0.000593 **WORSE** |
| box | gps_dropout(duration_s=5) | FAIL | tilt_rms_deg 0.695->0.695; tilt_max_deg 0.739->0.739; yaw_rms_deg 1->1; pos_h_rms_m 0.024->0.0148; vel_h_rms_ms 0.0312->0.0312; nees_att 19.8->19.8; nees_vel 0.229->0.152 **WORSE**; nees_pos 0.00201->0.000508 **WORSE**; tilt_drift_deg_s 0.000413->0.000425 |
| box | gps_dropout(duration_s=15) | FAIL | tilt_rms_deg 0.69->0.691; tilt_max_deg 0.74->0.74; yaw_rms_deg 1.03->1.03; pos_h_rms_m 0.197->0.195; vel_h_rms_ms 0.0479->0.0478; nees_att 19.8->19.8; nees_vel 0.227->0.149 **WORSE**; nees_pos 0.00217->0.000812; tilt_drift_deg_s -0.00194->-0.00193 |
| box | gps_dropout(duration_s=30) | FAIL | tilt_rms_deg 0.692->0.692; tilt_max_deg 0.924->0.924; yaw_rms_deg 1.04->1.04; pos_h_rms_m 0.954->0.95; vel_h_rms_ms 0.124->0.123; nees_att 19.8->19.8; nees_vel 0.113->0.0852; nees_pos 0.00136->0.00133; tilt_drift_deg_s 0.00278->0.0028 |
| box | gps_stale(duration_s=5) | FAIL | tilt_rms_deg 0.72->0.72; tilt_max_deg 1.08->1.08; yaw_rms_deg 1.06->1.06; pos_h_rms_m 0.817->0.819; vel_h_rms_ms 0.106->0.106; nees_att 19.8->19.8; nees_vel 2.44->2.36; nees_pos 2.87->2.88; tilt_drift_deg_s -0.00963->-0.00968 |
| box | gps_stale(duration_s=15) | FAIL | tilt_rms_deg 0.835->0.835; tilt_max_deg 2.45->2.45; yaw_rms_deg 1.16->1.16; pos_h_rms_m 2.12->2.12; vel_h_rms_ms 0.253->0.253; nees_att 19.6->19.6; nees_vel 11.7->11.6; nees_pos 19.2->19.2; tilt_drift_deg_s 0.044->0.044 |
| box | gps_stale(duration_s=30) | FAIL | tilt_rms_deg 0.857->0.857; tilt_max_deg 2.45->2.45; yaw_rms_deg 1.4->1.4; pos_h_rms_m 4.31->4.31; vel_h_rms_ms 0.355->0.355; nees_att 20.1->20.1; nees_vel 16.8->16.8; nees_pos 79.4->79.4; tilt_drift_deg_s 0.00602->0.00604 |
| box | gps_noise | FAIL | tilt_rms_deg 0.768->0.769; tilt_max_deg 1.31->1.31; yaw_rms_deg 1.19->1.18; pos_h_rms_m 0.502->0.496; vel_h_rms_ms 0.0677->0.069; nees_att 20->20; nees_vel 0.487->0.424; nees_pos 1.2->1.17 |
| box | gps_outliers(frac=0.01) | FAIL | tilt_max_deg 0.74->0.74; yaw_rms_deg 0.998->0.998; pos_h_rms_m 0.0219->0.0101; vel_h_rms_ms 0.0306->0.0306; nees_att 19.8->19.8; nees_vel 0.235->0.156 **WORSE**; nees_pos 0.00223->0.000579 **WORSE** |
| box | gps_rate(hz=1.0) | FAIL | tilt_rms_deg 0.696->0.696; tilt_max_deg 0.738->0.738; yaw_rms_deg 0.999->0.999; pos_h_rms_m 0.0238->0.0113; vel_h_rms_ms 0.0311->0.031; nees_att 19.7->19.7; nees_vel 0.0988->0.0689; nees_pos 0.000565->0.000157 |
| box | gyro_bias(dps=0.2) | FAIL | tilt_max_deg 0.741->0.74; yaw_rms_deg 0.989->0.989; pos_h_rms_m 0.022->0.0102; vel_h_rms_ms 0.0306->0.0306; nees_att 19.8->19.7; nees_vel 0.235->0.156 **WORSE**; nees_pos 0.00225->0.000592 **WORSE** |
| box | gyro_bias(dps=1.0) | FAIL | tilt_max_deg 0.744->0.743; yaw_rms_deg 0.933->0.933; pos_h_rms_m 0.022->0.0102; vel_h_rms_ms 0.0306->0.0306; nees_att 19.2->19.1; nees_vel 0.234->0.155 **WORSE**; nees_pos 0.00224->0.00059 **WORSE** |
| box | gyro_bias(dps=1.0,from_cal=False) | FAIL | tilt_max_deg 0.74->0.74; yaw_rms_deg 0.998->0.998; pos_h_rms_m 0.022->0.0102; vel_h_rms_ms 0.0306->0.0306; nees_att 19.8->19.8; nees_vel 0.235->0.156 **WORSE**; nees_pos 0.00225->0.000593 **WORSE** |
| box | gyro_drift(dps=0.5,over_s=60) | FAIL | tilt_rms_deg 0.696->0.696; tilt_max_deg 0.738->0.738; yaw_rms_deg 1->1; pos_h_rms_m 0.022->0.0102; vel_h_rms_ms 0.0306->0.0306; nees_att 19.9->19.9; nees_vel 0.235->0.156 **WORSE**; nees_pos 0.00225->0.000593 **WORSE** |
| box | accel_bias(axis=x,ms2=0.05) | FAIL | tilt_rms_deg 1.03->1.03; yaw_rms_deg 3.82->3.82; pos_h_rms_m 0.0204->0.0117; vel_h_rms_ms 0.0323->0.0323; nees_att 25.6->25.6; nees_vel 0.262->0.186 **WORSE**; nees_pos 0.00198->0.000758 |
| box | accel_bias(axis=x,ms2=0.2) | FAIL | tilt_rms_deg 1.73->1.73; yaw_rms_deg 9.88->9.88; pos_h_rms_m 0.0316->0.021; vel_h_rms_ms 0.0449->0.0448; nees_att 405->405; nees_vel 0.516->0.428; nees_pos 0.00479->0.00233 |
| box | accel_bias(axis=x,ms2=0.5) | FAIL | tilt_rms_deg 2.51->2.51; tilt_max_deg 2.64->2.64; yaw_rms_deg 4.67->4.67; pos_h_rms_m 0.0188->0.0111; vel_h_rms_ms 0.0334->0.0334; nees_att 201->201; nees_vel 0.294->0.215 **WORSE**; nees_pos 0.00227->0.00121 |
| box | accel_bias(axis=z,ms2=0.05) | FAIL | tilt_max_deg 0.74->0.74; yaw_rms_deg 0.993->0.993; pos_h_rms_m 0.022->0.0102; vel_h_rms_ms 0.0306->0.0306; nees_att 19.8->19.8; nees_vel 0.235->0.156 **WORSE**; nees_pos 0.00225->0.000593 **WORSE** |
| box | accel_bias(axis=z,ms2=0.2) | FAIL | tilt_max_deg 0.739->0.739; yaw_rms_deg 0.979->0.979; pos_h_rms_m 0.022->0.0102; vel_h_rms_ms 0.0306->0.0306; nees_att 19.6->19.6; nees_vel 0.235->0.156 **WORSE**; nees_pos 0.00225->0.000592 **WORSE** |
| box | accel_bias(axis=z,ms2=0.5) | FAIL | tilt_max_deg 0.739->0.739; yaw_rms_deg 0.952->0.952; pos_h_rms_m 0.022->0.0102; vel_h_rms_ms 0.0306->0.0306; nees_att 19.3->19.2; nees_vel 0.234->0.155 **WORSE**; nees_pos 0.00224->0.000591 **WORSE** |
| box | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | tilt_rms_deg 1.62->1.62; tilt_max_deg 1.78->1.78; yaw_rms_deg 0.929->0.929; pos_h_rms_m 0.186->0.188; vel_h_rms_ms 0.156->0.156; nees_att 69.8->69.7; nees_vel 5.19->5.1; nees_pos 0.162->0.164 |
| box | mag_bias(frac=0.05) | WARN | tilt_rms_deg 0.642->0.642; tilt_max_deg 0.752->0.752; yaw_rms_deg 1.67->1.67; pos_h_rms_m 0.0236->0.0116; vel_h_rms_ms 0.0355->0.0353; nees_att 8.24->8.23; nees_vel 0.319->0.229 **WORSE**; nees_pos 0.00253->0.000694 **WORSE** |
| box | mag_bias(frac=0.15) | FAIL | tilt_rms_deg 0.428->0.427; yaw_rms_deg 1.34->1.34; pos_h_rms_m 0.0341->0.0232; vel_h_rms_ms 0.0506->0.0502; nees_att 11.8->11.8; nees_vel 0.656->0.537; nees_pos 0.00511->0.0024 |
| box | mag_interference(amp=0.3) | FAIL | tilt_rms_deg 0.697->0.697; tilt_max_deg 0.739->0.739; yaw_rms_deg 0.996->0.996; pos_h_rms_m 0.022->0.0104; vel_h_rms_ms 0.0305->0.0305; nees_att 18.5->18.5; nees_vel 0.225->0.148 **WORSE**; nees_pos 0.00225->0.000606 **WORSE**; tilt_drift_deg_s 0.00384->0.00383 |
| box | mag_interference(amp=1.0) | FAIL | tilt_rms_deg 0.697->0.697; tilt_max_deg 0.739->0.739; yaw_rms_deg 0.996->0.996; pos_h_rms_m 0.022->0.0104; vel_h_rms_ms 0.0305->0.0305; nees_att 18.5->18.5; nees_vel 0.225->0.148 **WORSE**; nees_pos 0.00225->0.000606 **WORSE**; tilt_drift_deg_s 0.00384->0.00383 |
| box | imu_noise | FAIL | tilt_rms_deg 5.57->5.57; yaw_rms_deg 3.96->3.96; pos_h_rms_m 0.0206->0.0127; vel_h_rms_ms 0.0312->0.031; nees_att 903->903; nees_vel 3.04->2.65; nees_pos 0.349->0.328 |
| box | imu_spikes | FAIL | tilt_rms_deg 0.742->0.742; tilt_max_deg 1.96->1.96; yaw_rms_deg 1.09->1.09; pos_h_rms_m 0.0251->0.0167; vel_h_rms_ms 0.0489->0.0487; nees_att 20.7->20.7; nees_vel 0.958->0.839; nees_pos 0.0163->0.0143 |
| box | imu_dropouts | FAIL | tilt_max_deg 0.742->0.742; yaw_rms_deg 0.998->0.998; pos_h_rms_m 0.022->0.0102; vel_h_rms_ms 0.0308->0.0308; nees_att 19.8->19.8; nees_vel 0.236->0.159 **WORSE**; nees_pos 0.00223->0.000594 **WORSE** |
| box | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.696->0.697; tilt_max_deg 0.739->0.747; yaw_rms_deg 1->0.998; pos_h_rms_m 0.0216->0.00954; vel_h_rms_ms 0.0307->0.0306; nees_att 19.9->19.8; nees_vel 0.234->0.152 **WORSE**; nees_pos 0.00216->0.000543 **WORSE** |
| box | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.71->0.697; tilt_max_deg 1.43->0.739 **BETTER**; yaw_rms_deg 1.01->1; pos_h_rms_m 0.0425->0.0202; vel_h_rms_ms 0.0482->0.0308; nees_att 20.2->19.8; nees_vel 0.596->0.153 **WORSE**; nees_pos 0.00792->0.00115 **WORSE**; recovery_s 1.22->0.00411 **BETTER**; mag_rejected 19->0 |
| box | combined_realistic | FAIL | tilt_rms_deg 5.07->5.07; tilt_max_deg 5.96->5.96; yaw_rms_deg 7.02->7.02; pos_h_rms_m 0.494->0.488; vel_h_rms_ms 0.0739->0.075; nees_att 107->106; nees_vel 3.26->2.94 **BETTER**; nees_pos 1.15->1.12 |
| box | cal_ideal | FAIL | tilt_rms_deg 0.00903->0.00892; tilt_max_deg 0.0377->0.0377; yaw_rms_deg 0.0167->0.0164; pos_h_rms_m 0.0215->0.00999; vel_h_rms_ms 0.0303->0.0303; nees_att 0.000951->0.00095; nees_vel 0.231->0.153 **WORSE**; nees_pos 0.00254->0.00095 |
| circle_slow | none | FAIL | - |
| circle_slow | gps_dropout(duration_s=5) | FAIL | - |
| circle_slow | gps_dropout(duration_s=15) | FAIL | - |
| circle_slow | gps_dropout(duration_s=30) | FAIL | - |
| circle_slow | gps_stale(duration_s=5) | FAIL | - |
| circle_slow | gps_stale(duration_s=15) | FAIL | - |
| circle_slow | gps_stale(duration_s=30) | FAIL | - |
| circle_slow | gps_noise | FAIL | - |
| circle_slow | gps_outliers(frac=0.01) | FAIL | - |
| circle_slow | gps_rate(hz=1.0) | FAIL | - |
| circle_slow | gyro_bias(dps=0.2) | FAIL | - |
| circle_slow | gyro_bias(dps=1.0) | FAIL | - |
| circle_slow | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| circle_slow | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| circle_slow | accel_bias(axis=x,ms2=0.05) | FAIL | - |
| circle_slow | accel_bias(axis=x,ms2=0.2) | FAIL | - |
| circle_slow | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| circle_slow | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| circle_slow | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| circle_slow | accel_bias(axis=z,ms2=0.5) | FAIL | - |
| circle_slow | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | - |
| circle_slow | mag_bias(frac=0.05) | FAIL | - |
| circle_slow | mag_bias(frac=0.15) | FAIL | - |
| circle_slow | mag_interference(amp=0.3) | FAIL | - |
| circle_slow | mag_interference(amp=1.0) | FAIL | - |
| circle_slow | imu_noise | FAIL | - |
| circle_slow | imu_spikes | FAIL | - |
| circle_slow | imu_dropouts | FAIL | - |
| circle_slow | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.692->0.685; yaw_rms_deg 2.29->2.3; pos_h_rms_m 0.0642->0.0166; vel_h_rms_ms 0.0404->0.0384; nees_att 32.4->32.3; nees_vel 0.358->0.3; nees_pos 0.0221->0.004 **WORSE** |
| circle_slow | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.738->0.684; tilt_max_deg 1.76->0.771 **BETTER**; yaw_rms_deg 2.28->2.31; pos_h_rms_m 0.371->0.02 **BETTER**; vel_h_rms_ms 0.0526->0.0382; nees_att 32.7->32.3; nees_vel 0.565->0.294 **WORSE**; nees_pos 0.593->0.00406 **WORSE**; recovery_s 2.03->4.1e-05 **BETTER** |
| circle_slow | combined_realistic | FAIL | - |
| circle_slow | cal_ideal | FAIL | - |
| circle_fast | none | WARN | - |
| circle_fast | gps_dropout(duration_s=5) | WARN | - |
| circle_fast | gps_dropout(duration_s=15) | WARN | - |
| circle_fast | gps_dropout(duration_s=30) | WARN | - |
| circle_fast | gps_stale(duration_s=5) | WARN | - |
| circle_fast | gps_stale(duration_s=15) | FAIL | - |
| circle_fast | gps_stale(duration_s=30) | FAIL | - |
| circle_fast | gps_noise | WARN | - |
| circle_fast | gps_outliers(frac=0.01) | WARN | - |
| circle_fast | gps_rate(hz=1.0) | WARN | - |
| circle_fast | gyro_bias(dps=0.2) | WARN | - |
| circle_fast | gyro_bias(dps=1.0) | WARN | - |
| circle_fast | gyro_bias(dps=1.0,from_cal=False) | WARN | - |
| circle_fast | gyro_drift(dps=0.5,over_s=60) | WARN | - |
| circle_fast | accel_bias(axis=x,ms2=0.05) | FAIL | - |
| circle_fast | accel_bias(axis=x,ms2=0.2) | FAIL | - |
| circle_fast | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| circle_fast | accel_bias(axis=z,ms2=0.05) | WARN | - |
| circle_fast | accel_bias(axis=z,ms2=0.2) | WARN | - |
| circle_fast | accel_bias(axis=z,ms2=0.5) | WARN | - |
| circle_fast | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | - |
| circle_fast | mag_bias(frac=0.05) | FAIL | - |
| circle_fast | mag_bias(frac=0.15) | FAIL | - |
| circle_fast | mag_interference(amp=0.3) | WARN | - |
| circle_fast | mag_interference(amp=1.0) | WARN | - |
| circle_fast | imu_noise | FAIL | - |
| circle_fast | imu_spikes | WARN | - |
| circle_fast | imu_dropouts | WARN | - |
| circle_fast | imu_gap(gap_s=0.2) | WARN | tilt_rms_deg 0.747->0.673; tilt_max_deg 2.33->1.08 **BETTER**; yaw_rms_deg 0.764->0.667; pos_h_rms_m 0.126->0.0796; vel_h_rms_ms 0.132->0.116; nees_att 6.94->6.56; nees_vel 2.53->1.63 **BETTER**; nees_pos 0.0688->0.0275 **WORSE**; recovery_s 2.42->0.000776 **BETTER** |
| circle_fast | imu_gap(gap_s=1.0) | FAIL -> WARN | tilt_rms_deg 2.64->0.679 **BETTER**; tilt_max_deg 15.6->2.1 **BETTER**; yaw_rms_deg 0.698->0.716; pos_h_rms_m 0.259->0.0849 **BETTER**; vel_h_rms_ms 0.315->0.116 **BETTER**; nees_att 14.4->6.55 **BETTER**; nees_vel 6.56->1.61 **BETTER**; nees_pos 0.234->0.0266 **WORSE**; recovery_s 2.63->0.489 **BETTER**; gps_resets 2->0 **BETTER**; gps_rejected 10->0 **BETTER**; mag_rejected 656->0 **BETTER** |
| circle_fast | combined_realistic | FAIL | - |
| circle_fast | cal_ideal | FAIL | - |
| stops | none | FAIL | - |
| stops | gps_dropout(duration_s=5) | FAIL | - |
| stops | gps_dropout(duration_s=15) | FAIL | - |
| stops | gps_dropout(duration_s=30) | FAIL | - |
| stops | gps_stale(duration_s=5) | FAIL | - |
| stops | gps_stale(duration_s=15) | FAIL | - |
| stops | gps_stale(duration_s=30) | FAIL | - |
| stops | gps_noise | FAIL | - |
| stops | gps_outliers(frac=0.01) | FAIL | - |
| stops | gps_rate(hz=1.0) | FAIL | - |
| stops | gyro_bias(dps=0.2) | FAIL | - |
| stops | gyro_bias(dps=1.0) | FAIL | - |
| stops | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| stops | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| stops | accel_bias(axis=x,ms2=0.05) | FAIL | - |
| stops | accel_bias(axis=x,ms2=0.2) | FAIL | - |
| stops | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| stops | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| stops | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| stops | accel_bias(axis=z,ms2=0.5) | FAIL | - |
| stops | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | - |
| stops | mag_bias(frac=0.05) | FAIL | - |
| stops | mag_bias(frac=0.15) | FAIL | - |
| stops | mag_interference(amp=0.3) | FAIL | - |
| stops | mag_interference(amp=1.0) | FAIL | - |
| stops | imu_noise | FAIL | - |
| stops | imu_spikes | FAIL | - |
| stops | imu_dropouts | FAIL | - |
| stops | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 1.13->1.12; yaw_rms_deg 4.27->4.28; pos_h_rms_m 0.0605->0.0931; vel_h_rms_ms 0.131->0.111; nees_att 37.2->37.2; nees_vel 3.45->2.18 **BETTER**; nees_pos 0.0253->0.0406 |
| stops | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 1.09->1.09; tilt_max_deg 3.12->3.1; yaw_rms_deg 4.27->4.26; pos_h_rms_m 0.124->0.133; vel_h_rms_ms 0.283->0.109 **BETTER**; nees_att 37.6->37.4; nees_vel 16.2->2.13 **BETTER**; nees_pos 0.0607->0.0594; gps_resets 1->0 **BETTER**; gps_rejected 5->0 **BETTER** |
| stops | combined_realistic | FAIL | - |
| stops | cal_ideal | FAIL | - |
| patrol_long | none | WARN | tilt_rms_deg 0.326->0.194 **BETTER**; tilt_max_deg 6.28->0.397 **BETTER**; yaw_rms_deg 1.37->1.36; pos_h_rms_m 0.089->0.0133 **BETTER**; vel_h_rms_ms 0.0578->0.0405; nees_att 6.57->6.44; nees_vel 0.531->0.24 **WORSE**; nees_pos 0.0326->0.000806 **WORSE**; gps_resets 1->0 **BETTER**; gps_rejected 5->0 **BETTER**; mag_rejected 430->0 **BETTER** |
| patrol_long | gps_dropout(duration_s=30) | WARN | tilt_rms_deg 0.329->0.198 **BETTER**; tilt_max_deg 6.28->0.594 **BETTER**; yaw_rms_deg 1.36->1.35; pos_h_rms_m 0.694->0.669; vel_h_rms_ms 0.109->0.0995; nees_att 6.57->6.44; nees_vel 0.526->0.234 **WORSE**; nees_pos 0.0341->0.000914 **WORSE**; tilt_drift_deg_s 0.0065->0.00643; recovery_s 0.405->0.0012; gps_resets 1->0 **BETTER**; gps_rejected 5->0 **BETTER**; mag_rejected 430->0 **BETTER** |
| patrol_long | gps_noise | WARN | tilt_rms_deg 0.426->0.399; tilt_max_deg 4.79->2.87 **BETTER**; yaw_rms_deg 1.51->1.51; pos_h_rms_m 1.19->1.12; vel_h_rms_ms 0.0825->0.0745; nees_att 6.66->6.49; nees_vel 0.813->0.476 **WORSE**; nees_pos 6.44->5.69; gps_rejected 2->0; mag_rejected 300->0 **BETTER** |
| patrol_long | gyro_drift(dps=0.5,over_s=60) | WARN | tilt_rms_deg 0.326->0.194 **BETTER**; tilt_max_deg 6.28->0.393 **BETTER**; yaw_rms_deg 1.37->1.36; pos_h_rms_m 0.089->0.0133 **BETTER**; vel_h_rms_ms 0.0578->0.0405; nees_att 6.57->6.44; nees_vel 0.531->0.24 **WORSE**; nees_pos 0.0326->0.000805 **WORSE**; gps_resets 1->0 **BETTER**; gps_rejected 5->0 **BETTER**; mag_rejected 430->0 **BETTER** |
| patrol_long | accel_bias(axis=x,ms2=0.05) | FAIL | tilt_rms_deg 0.501->0.389 **BETTER**; tilt_max_deg 7.91->0.724 **BETTER**; yaw_rms_deg 8.23->8.23; pos_h_rms_m 0.0879->0.0288 **BETTER**; vel_h_rms_ms 0.066->0.0512; nees_att 274->274; nees_vel 0.835->0.504 **WORSE**; nees_pos 0.0318->0.00356 **WORSE**; gps_resets 1->0 **BETTER**; gps_rejected 5->0 **BETTER**; mag_rejected 440->0 **BETTER** |
| patrol_long | mag_bias(frac=0.05) | FAIL | tilt_rms_deg 0.391->0.263 **BETTER**; tilt_max_deg 7.21->0.583 **BETTER**; yaw_rms_deg 2.93->2.93; pos_h_rms_m 0.0854->0.0229 **BETTER**; vel_h_rms_ms 0.0616->0.0461; nees_att 24.3->24.3; nees_vel 0.69->0.372 **WORSE**; nees_pos 0.0301->0.00233 **WORSE**; gps_resets 1->0 **BETTER**; gps_rejected 5->0 **BETTER**; mag_rejected 440->0 **BETTER** |
| patrol_long | imu_noise | FAIL | tilt_rms_deg 3.18->3.15; tilt_max_deg 9.84->3.76 **BETTER**; yaw_rms_deg 17.6->17.6; pos_h_rms_m 0.102->0.0563; vel_h_rms_ms 0.0904->0.0789; nees_att 958->960; nees_vel 2->1.62; nees_pos 0.0656->0.0366 **WORSE**; gps_resets 1->0 **BETTER**; gps_rejected 5->0 **BETTER**; mag_rejected 440->0 **BETTER** |
| patrol_long | imu_spikes | FAIL -> WARN | tilt_rms_deg 0.527->0.331 **BETTER**; tilt_max_deg 10.2->3.09 **BETTER**; yaw_rms_deg 1.47->1.46; pos_h_rms_m 0.0941->0.0554; vel_h_rms_ms 0.081->0.0673; nees_att 6.94->6.8; nees_vel 2.66->2.34; nees_pos 0.122->0.0977; gps_resets 1->0 **BETTER**; gps_rejected 5->0 **BETTER**; mag_rejected 484->28 **BETTER** |
| patrol_long | combined_realistic | FAIL | tilt_rms_deg 2.53->2.53; tilt_max_deg 6.39->5.15; yaw_rms_deg 16.7->16.7; pos_h_rms_m 1.19->1.12; vel_h_rms_ms 0.109->0.102; nees_att 799->801; nees_vel 2.34->1.9; nees_pos 6.62->5.87; gps_rejected 2->0; mag_rejected 294->0 **BETTER** |
| patrol_long | cal_ideal | FAIL | tilt_rms_deg 0.305->0.0149 **BETTER**; tilt_max_deg 7.53->0.338 **BETTER**; yaw_rms_deg 0.0609->0.0304; pos_h_rms_m 0.0827->0.013 **BETTER**; vel_h_rms_ms 0.0571->0.0405; nees_att 0.072->0.00164 **WORSE**; nees_vel 0.535->0.237 **WORSE**; nees_pos 0.0281->0.00077 **WORSE**; gps_resets 1->0 **BETTER**; gps_rejected 5->0 **BETTER**; mag_rejected 440->0 **BETTER** |
| takeoff_land | none | FAIL | - |
| takeoff_land | gps_dropout(duration_s=5) | FAIL | - |
| takeoff_land | gps_dropout(duration_s=15) | FAIL | - |
| takeoff_land | gps_dropout(duration_s=30) | FAIL | - |
| takeoff_land | gps_stale(duration_s=5) | FAIL | - |
| takeoff_land | gps_stale(duration_s=15) | FAIL | - |
| takeoff_land | gps_stale(duration_s=30) | FAIL | - |
| takeoff_land | gps_noise | FAIL | - |
| takeoff_land | gps_outliers(frac=0.01) | FAIL | - |
| takeoff_land | gps_rate(hz=1.0) | FAIL | - |
| takeoff_land | gyro_bias(dps=0.2) | FAIL | - |
| takeoff_land | gyro_bias(dps=1.0) | FAIL | - |
| takeoff_land | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| takeoff_land | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| takeoff_land | accel_bias(axis=x,ms2=0.05) | FAIL | - |
| takeoff_land | accel_bias(axis=x,ms2=0.2) | FAIL | - |
| takeoff_land | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| takeoff_land | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| takeoff_land | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| takeoff_land | accel_bias(axis=z,ms2=0.5) | FAIL | - |
| takeoff_land | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | - |
| takeoff_land | mag_bias(frac=0.05) | FAIL | - |
| takeoff_land | mag_bias(frac=0.15) | FAIL | - |
| takeoff_land | mag_interference(amp=0.3) | FAIL | - |
| takeoff_land | mag_interference(amp=1.0) | FAIL | - |
| takeoff_land | imu_noise | FAIL | - |
| takeoff_land | imu_spikes | FAIL | - |
| takeoff_land | imu_dropouts | FAIL | - |
| takeoff_land | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.725->0.725; yaw_rms_deg 1.93->1.93; pos_h_rms_m 0.00146->0.00138; vel_h_rms_ms 0.00261->0.00258; nees_att 31.1->31; nees_vel 1.08->0.956; nees_pos 0.0517->0.0377 |
| takeoff_land | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.725->0.725; tilt_max_deg 0.739->0.752; yaw_rms_deg 1.93->1.93; pos_h_rms_m 0.00143->0.00243; vel_h_rms_ms 0.00263->0.00259; nees_att 31.1->30.9; nees_vel 1.1->0.9; nees_pos 0.0522->0.0386 |
| takeoff_land | combined_realistic | FAIL | - |
| takeoff_land | cal_ideal | FAIL | - |
| ref_live1 | none | WARN | - |
| yaw_steps | none | FAIL | - |
| yaw_steps | gps_dropout(duration_s=5) | FAIL | - |
| yaw_steps | gps_dropout(duration_s=15) | FAIL | - |
| yaw_steps | gps_dropout(duration_s=30) | FAIL | - |
| yaw_steps | gps_stale(duration_s=5) | FAIL | - |
| yaw_steps | gps_stale(duration_s=15) | FAIL | - |
| yaw_steps | gps_stale(duration_s=30) | FAIL | - |
| yaw_steps | gps_noise | FAIL | - |
| yaw_steps | gps_outliers(frac=0.01) | FAIL | - |
| yaw_steps | gps_rate(hz=1.0) | FAIL | - |
| yaw_steps | gyro_bias(dps=0.2) | FAIL | - |
| yaw_steps | gyro_bias(dps=1.0) | FAIL | - |
| yaw_steps | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| yaw_steps | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| yaw_steps | accel_bias(axis=x,ms2=0.05) | FAIL | - |
| yaw_steps | accel_bias(axis=x,ms2=0.2) | FAIL | - |
| yaw_steps | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| yaw_steps | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| yaw_steps | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| yaw_steps | accel_bias(axis=z,ms2=0.5) | FAIL | - |
| yaw_steps | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | - |
| yaw_steps | mag_bias(frac=0.05) | FAIL | - |
| yaw_steps | mag_bias(frac=0.15) | FAIL | - |
| yaw_steps | mag_interference(amp=0.3) | FAIL | - |
| yaw_steps | mag_interference(amp=1.0) | FAIL | - |
| yaw_steps | imu_noise | FAIL | - |
| yaw_steps | imu_spikes | FAIL | - |
| yaw_steps | imu_dropouts | FAIL | - |
| yaw_steps | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.444->0.461; tilt_max_deg 2.31->2.3; yaw_rms_deg 2.91->2.92; pos_h_rms_m 0.227->0.204; vel_h_rms_ms 0.156->0.149; nees_att 38.8->39.2; nees_vel 5.82->5.15; nees_pos 0.223->0.18 |
| yaw_steps | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.448->0.467; tilt_max_deg 2.31->2.26; yaw_rms_deg 2.87->2.87; pos_h_rms_m 0.229->0.219; vel_h_rms_ms 0.158->0.145; nees_att 37.8->38.2; nees_vel 5.97->4.76; nees_pos 0.226->0.156 **WORSE** |
| yaw_steps | combined_realistic | FAIL | - |
| yaw_steps | cal_ideal | FAIL | - |
| yaw_spin | none | FAIL | - |
| yaw_spin | gps_dropout(duration_s=5) | FAIL | - |
| yaw_spin | gps_dropout(duration_s=15) | FAIL | - |
| yaw_spin | gps_dropout(duration_s=30) | FAIL | - |
| yaw_spin | gps_stale(duration_s=5) | FAIL | - |
| yaw_spin | gps_stale(duration_s=15) | FAIL | - |
| yaw_spin | gps_stale(duration_s=30) | FAIL | - |
| yaw_spin | gps_noise | FAIL | - |
| yaw_spin | gps_outliers(frac=0.01) | FAIL | - |
| yaw_spin | gps_rate(hz=1.0) | FAIL | - |
| yaw_spin | gyro_bias(dps=0.2) | FAIL | - |
| yaw_spin | gyro_bias(dps=1.0) | FAIL | - |
| yaw_spin | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| yaw_spin | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| yaw_spin | accel_bias(axis=x,ms2=0.05) | FAIL | - |
| yaw_spin | accel_bias(axis=x,ms2=0.2) | FAIL | - |
| yaw_spin | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| yaw_spin | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| yaw_spin | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| yaw_spin | accel_bias(axis=z,ms2=0.5) | FAIL | - |
| yaw_spin | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | - |
| yaw_spin | mag_bias(frac=0.05) | FAIL | - |
| yaw_spin | mag_bias(frac=0.15) | FAIL | - |
| yaw_spin | mag_interference(amp=0.3) | FAIL | - |
| yaw_spin | mag_interference(amp=1.0) | FAIL | - |
| yaw_spin | imu_noise | FAIL | - |
| yaw_spin | imu_spikes | FAIL | - |
| yaw_spin | imu_dropouts | FAIL | - |
| yaw_spin | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.586->0.535; tilt_max_deg 1.74->1.22 **BETTER**; yaw_rms_deg 2.33->2.26; pos_h_rms_m 0.157->0.147; vel_h_rms_ms 0.149->0.146; nees_att 14.6->14.4; nees_vel 5.63->5.38; nees_pos 0.105->0.0915; recovery_s 0.608->0.000164 **BETTER**; mag_rejected 25->0 |
| yaw_spin | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.628->0.531; tilt_max_deg 2.97->1.22 **BETTER**; yaw_rms_deg 7.91->2.27 **BETTER**; pos_h_rms_m 0.155->0.15; vel_h_rms_ms 0.147->0.147; nees_att 24.3->14.6; nees_vel 5.21->5.48; nees_pos 0.102->0.0833; recovery_s 5.6->0.000353 **BETTER**; mag_rejected 1400->0 **BETTER** |
| yaw_spin | combined_realistic | FAIL | - |
| yaw_spin | cal_ideal | FAIL | - |
