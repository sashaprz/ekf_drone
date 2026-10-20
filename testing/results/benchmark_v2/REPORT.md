# Filter benchmark v2 - your EKF vs PX4 EKF2 vs Madgwick vs Mahony

All flights flown by **PX4 itself** in Gazebo SITL (none of this project's code in the loop); every filter sees the identical sensor stream (EKF2 via PX4's own replay of the same - possibly fault-injected - log). Truth = Gazebo ground truth. Method details: `testing/benchmark/score.py` docstring; setup: `testing/benchmark/README.md`.

**180 runs** from **36 flights** (box, circle_fast, figure8, hover, stops, yaw_spin; trials: [1, 2, 3]). Values: mean ± 95% CI over trials.

CV-tuned gains chosen (per held-out trial): madgwick t1: `{"beta": 0.05, "zeta": 0.002}`; madgwick t2: `{"beta": 0.02, "zeta": 0.002}`; madgwick t3: `{"beta": 0.05, "zeta": 0.002}`; mahony t1: `{"ki": 0.01, "kp": 0.5}`; mahony t2: `{"ki": 0.05, "kp": 0.5}`; mahony t3: `{"ki": 0.01, "kp": 0.5}`

## 1. Headline

### Stock x500 IMU (near-ideal), no fault  (18 runs)

| filter | tilt rms ° | tilt max ° | heading rms ° | heading rms, offset removed ° | pos h rms m | vel rms m/s | lost |
|---|---|---|---|---|---|---|---|
| **your EKF** | 1.00 ± 0.28 | 3.00 ± 0.90 | 4.86 ± 1.32 | 4.19 ± 1.23 | 0.073 ± 0.012 | 0.100 ± 0.029 | 0/18 |
| your EKF, detected field | 1.01 ± 0.32 | 3.44 ± 1.38 | 4.91 ± 1.31 | 3.80 ± 1.28 | 0.073 ± 0.012 | 0.102 ± 0.032 | 0/18 |
| your EKF, detected field + decl. learning | 0.73 ± 0.19 | 2.46 ± 0.92 | 3.88 ± 1.03 | 3.48 ± 0.91 | 0.074 ± 0.012 | 0.089 ± 0.024 | 0/18 |
| PX4 EKF2 | 0.13 ± 0.04 | 0.90 ± 0.35 | 2.06 ± 0.82 | 1.05 ± 0.42 | 0.061 ± 0.014 | 0.078 ± 0.020 | 0/18 |
| Madgwick (default) | 4.35 ± 1.53 | 9.92 ± 3.04 | 6.03 ± 2.46 | 4.67 ± 1.69 | - | - | 0/18 |
| Madgwick (CV-tuned) | 3.49 ± 1.44 | 7.58 ± 2.82 | 4.57 ± 1.91 | 3.10 ± 1.12 | - | - | 0/18 |
| Mahony (default) | 3.86 ± 1.13 | 7.87 ± 2.31 | 3.94 ± 1.80 | 2.84 ± 0.81 | - | - | 0/18 |
| Mahony (CV-tuned) | 4.36 ± 1.48 | 8.79 ± 2.86 | 4.21 ± 1.21 | 3.46 ± 0.88 | - | - | 0/18 |

(mean ± CI here is over all runs in the group, not over trials)

### Realistic IMU, no fault  (18 runs)

| filter | tilt rms ° | tilt max ° | heading rms ° | heading rms, offset removed ° | pos h rms m | vel rms m/s | lost |
|---|---|---|---|---|---|---|---|
| **your EKF** | 1.51 ± 0.30 | 4.30 ± 1.37 | 4.78 ± 1.41 | 3.96 ± 1.36 | 0.085 ± 0.015 | 0.239 ± 0.086 | 0/18 |
| your EKF, detected field | 1.57 ± 0.36 | 5.16 ± 1.94 | 5.59 ± 1.67 | 4.37 ± 1.67 | 0.088 ± 0.017 | 0.247 ± 0.087 | 0/18 |
| your EKF, detected field + decl. learning | 1.39 ± 0.29 | 4.38 ± 1.98 | 4.35 ± 1.20 | 3.55 ± 1.22 | 0.085 ± 0.016 | 0.234 ± 0.087 | 0/18 |
| PX4 EKF2 | 1.54 ± 0.19 | 3.81 ± 1.01 | 4.14 ± 1.47 | 2.97 ± 0.62 | 0.184 ± 0.018 | 0.347 ± 0.066 | 0/18 |
| Madgwick (default) | 5.12 ± 1.62 | 12.28 ± 3.66 | 7.06 ± 3.48 | 5.11 ± 1.99 | - | - | 0/18 |
| Madgwick (CV-tuned) | 3.53 ± 1.14 | 8.20 ± 2.62 | 4.66 ± 1.73 | 3.13 ± 0.91 | - | - | 0/18 |
| Mahony (default) | 5.19 ± 1.50 | 12.24 ± 5.33 | 8.99 ± 3.31 | 4.13 ± 1.41 | - | - | 2/18 |
| Mahony (CV-tuned) | 5.05 ± 1.55 | 11.61 ± 4.94 | 6.35 ± 2.07 | 3.99 ± 0.97 | - | - | 1/18 |

(mean ± CI here is over all runs in the group, not over trials)

### Realistic IMU, all 8 faults pooled  (144 runs)

| filter | tilt rms ° | tilt max ° | heading rms ° | heading rms, offset removed ° | pos h rms m | vel rms m/s | lost |
|---|---|---|---|---|---|---|---|
| **your EKF** | 2.02 ± 0.28 | 7.41 ± 1.40 | 5.86 ± 0.81 | 5.00 ± 0.76 | 1.751 ± 0.843 | 0.558 ± 0.162 | 0/144 |
| your EKF, detected field | 2.31 ± 0.36 | 9.38 ± 1.85 | 9.08 ± 2.16 | 7.42 ± 1.86 | 1.759 ± 0.811 | 0.578 ± 0.157 | 10/144 |
| your EKF, detected field + decl. learning | 1.93 ± 0.24 | 7.68 ± 1.56 | 7.31 ± 1.85 | 6.10 ± 1.57 | 1.588 ± 0.752 | 0.527 ± 0.141 | 4/144 |
| PX4 EKF2 | 1.92 ± 0.12 | 5.45 ± 0.66 | 6.31 ± 1.02 | 4.29 ± 0.50 | 1.288 ± 0.590 | 0.568 ± 0.104 | 2/144 |
| Madgwick (default) | 5.33 ± 0.56 | 13.24 ± 1.33 | 8.39 ± 1.28 | 6.03 ± 0.76 | - | - | 1/144 |
| Madgwick (CV-tuned) | 3.93 ± 0.45 | 9.30 ± 1.06 | 6.30 ± 0.93 | 4.04 ± 0.53 | - | - | 2/144 |
| Mahony (default) | 5.64 ± 0.53 | 13.49 ± 1.93 | 10.71 ± 1.31 | 4.94 ± 0.56 | - | - | 19/144 |
| Mahony (CV-tuned) | 5.44 ± 0.55 | 12.68 ± 1.78 | 8.08 ± 0.94 | 4.89 ± 0.50 | - | - | 8/144 |

(mean ± CI here is over all runs in the group, not over trials)

## 2. Paired comparison against your EKF (realistic IMU, all conditions)

Ratio = other filter's error / your EKF's error on the *same run*, geometric mean, 95% CI by resampling flights. > 1 means your EKF is better. 'EKF better' = share of runs where yours had the lower error. All runs with numbers included.

| vs | metric | ratio [95% CI] | EKF better | runs |
|---|---|---|---|---|
| your EKF, detected field | tilt rms | 1.07 [1.00, 1.17] | 53% | 162 |
| your EKF, detected field | heading rms | 1.26 [0.98, 1.68] | 60% | 162 |
| your EKF, detected field | heading (offset removed) | 1.11 [0.91, 1.37] | 48% | 162 |
| your EKF, detected field | pos h rms | 1.06 [1.02, 1.10] | 53% | 162 |
| your EKF, detected field | vel rms | 1.06 [1.01, 1.12] | 45% | 162 |
| your EKF, detected field + decl. learning | tilt rms | 0.95 [0.88, 1.03] | 33% | 162 |
| your EKF, detected field + decl. learning | heading rms | 1.06 [0.83, 1.40] | 50% | 162 |
| your EKF, detected field + decl. learning | heading (offset removed) | 1.01 [0.84, 1.23] | 49% | 162 |
| your EKF, detected field + decl. learning | pos h rms | 1.02 [0.98, 1.06] | 56% | 162 |
| your EKF, detected field + decl. learning | vel rms | 1.00 [0.95, 1.05] | 31% | 162 |
| PX4 EKF2 | tilt rms | 1.07 [0.86, 1.33] | 56% | 162 |
| PX4 EKF2 | heading rms | 1.06 [0.72, 1.58] | 52% | 162 |
| PX4 EKF2 | heading (offset removed) | 0.99 [0.69, 1.39] | 57% | 162 |
| PX4 EKF2 | pos h rms | 2.21 [2.00, 2.45] | 91% | 162 |
| PX4 EKF2 | vel rms | 1.54 [1.34, 1.81] | 87% | 162 |
| Madgwick (default) | tilt rms | 2.40 [1.83, 3.14] | 87% | 162 |
| Madgwick (default) | heading rms | 1.28 [0.96, 1.81] | 62% | 162 |
| Madgwick (default) | heading (offset removed) | 1.25 [0.98, 1.67] | 54% | 162 |
| Madgwick (CV-tuned) | tilt rms | 1.83 [1.51, 2.21] | 85% | 162 |
| Madgwick (CV-tuned) | heading rms | 0.99 [0.80, 1.25] | 52% | 162 |
| Madgwick (CV-tuned) | heading (offset removed) | 0.87 [0.72, 1.08] | 36% | 162 |
| Mahony (default) | tilt rms | 2.77 [2.23, 3.44] | 91% | 162 |
| Mahony (default) | heading rms | 1.81 [1.26, 2.58] | 73% | 162 |
| Mahony (default) | heading (offset removed) | 1.08 [0.81, 1.48] | 48% | 162 |
| Mahony (CV-tuned) | tilt rms | 2.60 [2.08, 3.22] | 90% | 162 |
| Mahony (CV-tuned) | heading rms | 1.42 [1.07, 1.91] | 62% | 162 |
| Mahony (CV-tuned) | heading (offset removed) | 1.13 [0.87, 1.49] | 52% | 162 |

## 3. Per scenario (realistic IMU, no fault) - mean ± 95% CI over trials

**tilt rms °**

| filter | box | circle_fast | figure8 | hover | stops | yaw_spin |
|---|---|---|---|---|---|---|
| **your EKF** | 1.10 ± 0.94 | 2.04 ± 1.11 | 2.05 ± 0.98 | 0.66 ± 0.46 | 1.72 ± 0.89 | 1.50 ± 1.88 |
| your EKF, detected field | 1.15 ± 0.78 | 1.99 ± 1.24 | 2.66 ± 1.37 | 0.65 ± 0.46 | 1.46 ± 1.10 | 1.50 ± 1.88 |
| your EKF, detected field + decl. learning | 0.92 ± 0.64 | 1.64 ± 0.64 | 2.07 ± 0.72 | 0.65 ± 0.47 | 1.52 ± 1.70 | 1.50 ± 1.88 |
| PX4 EKF2 | 1.55 ± 0.67 | 1.80 ± 0.32 | 1.37 ± 1.22 | 1.47 ± 0.39 | 1.99 ± 1.25 | 1.06 ± 0.63 |
| Madgwick (default) | 3.79 ± 0.25 | 10.08 ± 1.51 | 8.56 ± 0.88 | 0.82 ± 0.24 | 5.72 ± 2.53 | 1.77 ± 2.02 |
| Madgwick (CV-tuned) | 2.22 ± 1.86 | 6.19 ± 5.20 | 6.64 ± 2.14 | 0.82 ± 0.27 | 3.44 ± 3.49 | 1.86 ± 2.12 |
| Mahony (default) | 4.31 ± 1.85 | 7.78 ± 1.21 | 6.78 ± 2.32 | 1.18 ± 0.98 | 8.96 ± 8.98 | 2.13 ± 1.91 |
| Mahony (CV-tuned) | 3.39 ± 0.45 | 8.35 ± 6.56 | 7.48 ± 3.18 | 0.96 ± 0.69 | 7.88 ± 7.12 | 2.24 ± 2.14 |

**heading rms °**

| filter | box | circle_fast | figure8 | hover | stops | yaw_spin |
|---|---|---|---|---|---|---|
| **your EKF** | 3.77 ± 5.81 | 7.14 ± 12.37 | 7.24 ± 9.33 | 3.08 ± 4.38 | 4.88 ± 1.45 | 2.56 ± 2.53 |
| your EKF, detected field | 4.08 ± 3.80 | 6.45 ± 10.26 | 11.17 ± 5.18 | 3.86 ± 2.36 | 5.17 ± 9.71 | 2.84 ± 4.75 |
| your EKF, detected field + decl. learning | 2.23 ± 1.61 | 4.81 ± 7.85 | 7.83 ± 5.62 | 2.78 ± 3.19 | 3.15 ± 3.33 | 5.29 ± 5.98 |
| PX4 EKF2 | 3.19 ± 2.52 | 2.76 ± 2.51 | 3.90 ± 7.10 | 2.99 ± 1.57 | 3.38 ± 2.94 | 8.60 ± 15.19 |
| Madgwick (default) | 2.69 ± 1.17 | 21.98 ± 14.71 | 8.26 ± 4.19 | 1.92 ± 1.54 | 4.34 ± 6.44 | 3.19 ± 1.48 |
| Madgwick (CV-tuned) | 1.91 ± 1.73 | 8.56 ± 7.21 | 9.56 ± 9.46 | 1.87 ± 1.20 | 3.57 ± 4.98 | 2.48 ± 0.80 |
| Mahony (default) | 6.67 ± 15.02 | 16.86 ± 17.13 | 5.67 ± 7.56 | 3.24 ± 3.38 | 15.61 ± 23.26 | 5.90 ± 6.15 |
| Mahony (CV-tuned) | 2.47 ± 2.40 | 12.02 ± 7.49 | 6.60 ± 3.41 | 3.14 ± 3.25 | 10.05 ± 15.74 | 3.82 ± 1.75 |

**pos h rms m**

| filter | box | circle_fast | figure8 | hover | stops | yaw_spin |
|---|---|---|---|---|---|---|
| **your EKF** | 0.072 ± 0.018 | 0.111 ± 0.019 | 0.094 ± 0.014 | 0.060 ± 0.014 | 0.120 ± 0.137 | 0.050 ± 0.019 |
| your EKF, detected field | 0.072 ± 0.018 | 0.111 ± 0.021 | 0.115 ± 0.033 | 0.060 ± 0.014 | 0.122 ± 0.146 | 0.050 ± 0.019 |
| your EKF, detected field + decl. learning | 0.072 ± 0.018 | 0.111 ± 0.021 | 0.094 ± 0.017 | 0.060 ± 0.014 | 0.122 ± 0.150 | 0.050 ± 0.019 |
| PX4 EKF2 | 0.200 ± 0.067 | 0.220 ± 0.030 | 0.195 ± 0.093 | 0.172 ± 0.041 | 0.193 ± 0.076 | 0.125 ± 0.104 |

## 4. Per fault (realistic IMU, all scenarios pooled) - mean ± 95% CI over the 18 flights

**tilt rms °**

| filter | none | accel_bias_step | gps_dropout | gps_outliers | gyro_bias_step | gyro_drift | imu_spikes | mag_bias | mag_interference |
|---|---|---|---|---|---|---|---|---|---|
| **your EKF** | 1.51 ± 0.30 | 1.88 ± 0.30 | 4.81 ± 1.57 | 1.51 ± 0.29 | 1.50 ± 0.29 | 1.51 ± 0.30 | 1.82 ± 0.24 | 1.63 ± 0.43 | 1.49 ± 0.30 |
| your EKF, detected field | 1.57 ± 0.36 | 2.02 ± 0.47 (1 lost) | 5.16 ± 1.75 (3 lost) | 1.56 ± 0.34 | 1.62 ± 0.39 (2 lost) | 1.60 ± 0.37 (1 lost) | 1.83 ± 0.26 | 3.10 ± 1.53 (2 lost) | 1.57 ± 0.39 (1 lost) |
| your EKF, detected field + decl. learning | 1.39 ± 0.29 | 1.76 ± 0.32 | 3.58 ± 0.48 | 1.48 ± 0.41 (1 lost) | 1.36 ± 0.27 | 1.36 ± 0.27 | 1.75 ± 0.26 (1 lost) | 2.81 ± 1.42 (2 lost) | 1.34 ± 0.27 |
| PX4 EKF2 | 1.54 ± 0.19 | 1.58 ± 0.18 | 2.32 ± 0.49 | 1.54 ± 0.19 | 2.93 ± 0.37 (2 lost) | 1.70 ± 0.25 | 2.14 ± 0.16 | 1.56 ± 0.20 | 1.58 ± 0.21 |
| Madgwick (default) | 5.12 ± 1.62 | 5.35 ± 1.50 | 5.12 ± 1.62 | 5.13 ± 1.62 | 5.13 ± 1.61 | 5.12 ± 1.62 | 5.11 ± 1.60 | 5.55 ± 1.74 (1 lost) | 6.17 ± 1.48 |
| Madgwick (CV-tuned) | 3.53 ± 1.14 | 3.81 ± 1.09 | 3.53 ± 1.14 | 3.53 ± 1.14 | 3.53 ± 1.11 | 3.52 ± 1.15 | 3.75 ± 1.17 | 4.86 ± 1.92 (1 lost) | 4.92 ± 1.21 (1 lost) |
| Mahony (default) | 5.19 ± 1.50 (2 lost) | 5.30 ± 1.40 (2 lost) | 5.22 ± 1.50 (2 lost) | 5.20 ± 1.49 (2 lost) | 6.44 ± 1.65 (3 lost) | 5.63 ± 1.55 (3 lost) | 5.32 ± 1.49 (2 lost) | 5.65 ± 1.65 (3 lost) | 6.36 ± 1.54 (2 lost) |
| Mahony (CV-tuned) | 5.05 ± 1.55 (1 lost) | 5.21 ± 1.42 | 5.06 ± 1.55 (1 lost) | 5.06 ± 1.55 (1 lost) | 5.56 ± 1.52 (1 lost) | 5.19 ± 1.58 (1 lost) | 5.18 ± 1.52 (1 lost) | 5.70 ± 1.84 (3 lost) | 6.53 ± 1.55 |

**tilt max °**

| filter | none | accel_bias_step | gps_dropout | gps_outliers | gyro_bias_step | gyro_drift | imu_spikes | mag_bias | mag_interference |
|---|---|---|---|---|---|---|---|---|---|
| **your EKF** | 4.30 ± 1.37 | 5.65 ± 2.29 | 20.56 ± 7.11 | 5.15 ± 2.32 | 5.01 ± 2.00 | 5.14 ± 2.20 | 6.51 ± 1.69 | 6.76 ± 3.33 | 4.51 ± 1.59 |
| your EKF, detected field | 5.16 ± 1.94 | 6.72 ± 2.86 (1 lost) | 22.40 ± 7.56 (3 lost) | 5.75 ± 2.44 | 5.83 ± 2.53 (2 lost) | 5.82 ± 2.41 (1 lost) | 6.83 ± 1.73 | 15.94 ± 8.68 (2 lost) | 5.73 ± 2.42 (1 lost) |
| your EKF, detected field + decl. learning | 4.38 ± 1.98 | 5.33 ± 2.67 | 15.78 ± 3.16 | 5.58 ± 3.42 (1 lost) | 4.52 ± 2.23 | 4.62 ± 2.54 | 6.53 ± 1.97 (1 lost) | 15.20 ± 8.89 (2 lost) | 3.86 ± 1.40 |
| PX4 EKF2 | 3.81 ± 1.01 | 4.01 ± 1.11 | 5.90 ± 1.53 | 3.82 ± 1.01 | 11.43 ± 3.08 (2 lost) | 4.49 ± 1.22 | 5.72 ± 0.85 | 4.27 ± 1.11 | 3.94 ± 1.10 |
| Madgwick (default) | 12.28 ± 3.66 | 12.36 ± 3.44 | 12.26 ± 3.63 | 12.30 ± 3.65 | 12.59 ± 3.89 | 12.44 ± 3.75 | 12.46 ± 3.55 | 13.22 ± 4.29 (1 lost) | 18.28 ± 3.61 |
| Madgwick (CV-tuned) | 8.20 ± 2.62 | 8.48 ± 2.52 | 8.17 ± 2.58 | 8.22 ± 2.63 | 8.30 ± 2.67 | 8.39 ± 2.79 | 9.04 ± 2.52 | 11.20 ± 4.42 (1 lost) | 12.58 ± 3.24 (1 lost) |
| Mahony (default) | 12.24 ± 5.33 (2 lost) | 12.28 ± 5.16 (2 lost) | 12.31 ± 5.35 (2 lost) | 12.27 ± 5.33 (2 lost) | 14.81 ± 6.15 (3 lost) | 13.40 ± 5.63 (3 lost) | 12.82 ± 5.37 (2 lost) | 13.50 ± 5.90 (3 lost) | 16.53 ± 5.49 (2 lost) |
| Mahony (CV-tuned) | 11.61 ± 4.94 (1 lost) | 11.96 ± 4.76 | 11.63 ± 4.96 (1 lost) | 11.65 ± 4.98 (1 lost) | 12.54 ± 5.22 (1 lost) | 11.99 ± 5.12 (1 lost) | 12.21 ± 4.92 (1 lost) | 13.17 ± 5.85 (3 lost) | 16.26 ± 5.11 |

**heading rms °**

| filter | none | accel_bias_step | gps_dropout | gps_outliers | gyro_bias_step | gyro_drift | imu_spikes | mag_bias | mag_interference |
|---|---|---|---|---|---|---|---|---|---|
| **your EKF** | 4.78 ± 1.41 | 4.50 ± 1.18 | 9.80 ± 3.39 | 4.68 ± 1.32 | 4.68 ± 1.32 | 4.77 ± 1.37 | 5.04 ± 1.37 | 8.76 ± 3.79 | 4.61 ± 1.37 |
| your EKF, detected field | 5.59 ± 1.67 | 5.76 ± 2.20 (1 lost) | 16.27 ± 8.14 (3 lost) | 5.03 ± 1.35 | 5.94 ± 2.35 (2 lost) | 5.98 ± 1.97 (1 lost) | 5.30 ± 1.34 | 22.95 ± 12.32 (2 lost) | 5.44 ± 2.36 (1 lost) |
| your EKF, detected field + decl. learning | 4.35 ± 1.20 | 4.17 ± 1.08 | 7.90 ± 2.44 | 5.19 ± 2.31 (1 lost) | 4.20 ± 1.24 | 4.37 ± 1.21 | 5.61 ± 1.68 (1 lost) | 22.76 ± 12.10 (2 lost) | 4.28 ± 1.17 |
| PX4 EKF2 | 4.14 ± 1.47 | 4.10 ± 1.24 | 4.95 ± 2.03 | 4.13 ± 1.47 | 15.25 ± 5.05 (2 lost) | 5.06 ± 1.91 | 6.45 ± 2.58 | 5.19 ± 1.38 | 5.36 ± 1.81 |
| Madgwick (default) | 7.06 ± 3.48 | 6.70 ± 3.43 | 7.05 ± 3.47 | 7.05 ± 3.48 | 7.38 ± 3.70 | 7.07 ± 3.47 | 6.98 ± 3.39 | 12.22 ± 3.82 (1 lost) | 12.66 ± 3.45 |
| Madgwick (CV-tuned) | 4.66 ± 1.73 | 4.45 ± 1.72 | 4.66 ± 1.73 | 4.66 ± 1.72 | 4.86 ± 1.66 | 4.79 ± 1.94 | 4.95 ± 1.91 | 11.79 ± 3.94 (1 lost) | 10.27 ± 3.16 (1 lost) |
| Mahony (default) | 8.99 ± 3.31 (2 lost) | 9.03 ± 3.13 (2 lost) | 9.17 ± 3.29 (2 lost) | 9.00 ± 3.26 (2 lost) | 13.94 ± 4.63 (3 lost) | 11.22 ± 3.48 (3 lost) | 9.37 ± 3.27 (2 lost) | 11.86 ± 3.95 (3 lost) | 12.13 ± 4.35 (2 lost) |
| Mahony (CV-tuned) | 6.35 ± 2.07 (1 lost) | 6.42 ± 2.00 | 6.44 ± 2.06 (1 lost) | 6.37 ± 2.05 (1 lost) | 8.53 ± 2.57 (1 lost) | 7.51 ± 2.09 (1 lost) | 6.72 ± 2.04 (1 lost) | 11.12 ± 3.16 (3 lost) | 11.53 ± 3.76 |

**pos h rms m**

| filter | none | accel_bias_step | gps_dropout | gps_outliers | gyro_bias_step | gyro_drift | imu_spikes | mag_bias | mag_interference |
|---|---|---|---|---|---|---|---|---|---|
| **your EKF** | 0.085 ± 0.015 | 0.079 ± 0.012 | 13.447 ± 3.529 | 0.077 ± 0.011 | 0.079 ± 0.011 | 0.080 ± 0.013 | 0.081 ± 0.010 | 0.085 ± 0.014 | 0.081 ± 0.014 |
| your EKF, detected field | 0.088 ± 0.017 | 0.088 ± 0.017 (1 lost) | 13.442 ± 2.995 (3 lost) | 0.079 ± 0.012 | 0.083 ± 0.013 (2 lost) | 0.084 ± 0.015 (1 lost) | 0.083 ± 0.011 | 0.128 ± 0.055 (2 lost) | 0.086 ± 0.016 (1 lost) |
| your EKF, detected field + decl. learning | 0.085 ± 0.016 | 0.079 ± 0.012 | 12.097 ± 3.087 | 0.081 ± 0.015 (1 lost) | 0.079 ± 0.011 | 0.080 ± 0.013 | 0.083 ± 0.012 (1 lost) | 0.126 ± 0.053 (2 lost) | 0.082 ± 0.015 |
| PX4 EKF2 | 0.184 ± 0.018 | 0.186 ± 0.017 | 8.822 ± 2.960 | 0.184 ± 0.017 | 0.288 ± 0.023 (2 lost) | 0.204 ± 0.025 | 0.248 ± 0.013 | 0.184 ± 0.017 | 0.188 ± 0.018 |

## 5. During the fault (+10 s recovery)

| filter | GPS dropout: max pos h err m | GPS dropout: max tilt ° | mag interference: max heading err ° | mag interference: max tilt ° |
|---|---|---|---|---|
| **your EKF** | 60.14 ± 15.29 | 20.31 ± 7.14 | 9.29 ± 3.03 | 2.75 ± 0.67 |
| your EKF, detected field | 60.64 ± 13.83 | 21.19 ± 7.82 | 9.02 ± 2.58 | 2.69 ± 0.59 |
| your EKF, detected field + decl. learning | 56.30 ± 13.96 | 15.35 ± 3.09 | 8.69 ± 3.37 | 2.56 ± 0.68 |
| PX4 EKF2 | 35.61 ± 12.25 | 5.46 ± 1.65 | 8.48 ± 2.51 | 3.28 ± 0.71 |
| Madgwick (default) | - | 10.85 ± 3.21 | 31.39 ± 7.94 | 18.16 ± 3.59 |
| Madgwick (CV-tuned) | - | 6.82 ± 2.06 | 20.82 ± 6.87 | 11.75 ± 2.66 |
| Mahony (default) | - | 10.45 ± 3.77 | 20.03 ± 6.69 | 14.76 ± 3.19 |
| Mahony (CV-tuned) | - | 9.48 ± 3.01 | 21.22 ± 6.79 | 14.53 ± 3.06 |

## 6. Field detection (no magnetic model: declination assumed 0)

Detected inclination from the 6 s dwell vs the true field, and the declination error that the missing magnetic model leaves (true declination of the aligned field). 'learned' = FINAL_gps.py's declination state at the end of the flight - it should approach the declination error.

| fault | inclination error ° | declination error ° | learned declination ° | heading rms: true field | heading rms: detected | heading rms: detected + learning |
|---|---|---|---|---|---|---|
| none | -0.68 ± 0.63 | 3.11 ± 0.35 | 2.48 ± 1.31 | 4.78 ± 1.41 | 5.59 ± 1.67 | 4.35 ± 1.20 |
| accel_bias_step | -0.68 ± 0.63 | 3.11 ± 0.35 | 2.05 ± 1.36 | 4.50 ± 1.18 | 5.76 ± 2.20 | 4.17 ± 1.08 |
| gps_dropout | -0.67 ± 0.63 | 3.11 ± 0.35 | 3.59 ± 1.94 | 9.80 ± 3.39 | 16.27 ± 8.14 | 7.90 ± 2.44 |
| gps_outliers | -0.68 ± 0.63 | 3.11 ± 0.35 | 2.44 ± 1.35 | 4.68 ± 1.32 | 5.03 ± 1.35 | 5.19 ± 2.31 |
| gyro_bias_step | -0.67 ± 0.63 | 3.11 ± 0.35 | 2.84 ± 1.29 | 4.68 ± 1.32 | 5.94 ± 2.35 | 4.20 ± 1.24 |
| gyro_drift | -0.67 ± 0.63 | 3.11 ± 0.35 | 2.53 ± 1.33 | 4.77 ± 1.37 | 5.98 ± 1.97 | 4.37 ± 1.21 |
| imu_spikes | -0.68 ± 0.63 | 3.11 ± 0.35 | 2.67 ± 1.19 | 5.04 ± 1.37 | 5.30 ± 1.34 | 5.61 ± 1.68 |
| mag_bias | -2.46 ± 2.27 | 3.11 ± 0.35 | 3.32 ± 4.65 | 8.76 ± 3.79 | 22.95 ± 12.32 | 22.76 ± 12.10 |
| mag_interference | -0.67 ± 0.63 | 3.11 ± 0.35 | 2.57 ± 1.37 | 4.61 ± 1.37 | 5.44 ± 2.36 | 4.28 ± 1.17 |

## Caveats

- Simulation only. The realistic IMU model (white noise 0.29 °/s, random turn-on gyro bias σ 0.5 °/s, Gauss-Markov bias wander, accel 0.1 m/s²) is a guess at hobby hardware; there's no vibration spectrum, no motor-induced mag interference, no temperature drift. Real flights are the real test.
- PX4 flew every flight on its own EKF2 (live). The replayed EKF2 matches the live one to ≤0.06° after its first 2 s, so its numbers are not inflated by replay; but EKF2 had the trajectory 'it caused', the others didn't - this matters only for closed-loop effects, which a replay can't capture for anyone.
- Magnetic field: Gazebo's field is rotated onto the World Magnetic Model field EKF2 uses (EKF2 ignores the declination parameter when GPS is valid), so all filters share one north. The replayed filters get the field direction measured on the clean log against truth - slightly more information than EKF2's WMM lookup, which here equals the true field by construction.
- Your EKF gets a 6 s ground dwell for calibration (gyro bias, attitude); Madgwick/Mahony get the same dwell's mean gyro (bias) and accel/mag (TRIAD attitude). EKF2 uses its own on-ground alignment from the same data.
- Your EKF was developed on Gazebo recordings from the stock IMU, flown by your controller - none of these PX4 flights or the realistic-IMU model were used to tune it (held-out data). Madgwick/Mahony CV-tuned gains are chosen on other trials of these same scenarios - a mild advantage your EKF and EKF2 don't get.
- 3 trials per condition: confidence intervals are wide; differences inside them are not established.
