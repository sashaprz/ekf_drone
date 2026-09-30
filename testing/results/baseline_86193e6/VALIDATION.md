# Suite tooling validation (EKF_TEST_PLAN.md s3.8)

## 1. Reference replay

`ref_sensors_live1.csv` (git show 33be40e), MAG_REMAPPED=1, first 12 s, all rows, raw truth column (replay_ekf.py's convention): attitude RMS **0.14 / 0.12 / 0.20 deg** (expected 0.14 / 0.12 / 0.20) -> **PASS**

## 2. Fault sanity

`gyro_bias(dps=0)` vs `none`: estimate and covariance traces bit-identical -> **PASS**

Channels each fault changes (flight rows / calibration rows), vs `none`:

| fault | flight channels changed | calibration rows changed | rows kept | gps_ok=0 rows |
|---|---|---|---|---|
| none | - | - | 18948/18948 | 0 |
| gps_dropout(duration_s=5) | gps_ok | - | 18948/18948 | 1250 |
| gps_dropout(duration_s=15) | gps_ok | - | 18948/18948 | 3751 |
| gps_dropout(duration_s=30) | gps_ok | - | 18948/18948 | 7501 |
| gps_stale(duration_s=5) | px, py, pz, vx, vy, vz | - | 18948/18948 | 0 |
| gps_stale(duration_s=15) | px, py, pz, vx, vy, vz | - | 18948/18948 | 0 |
| gps_stale(duration_s=30) | px, py, pz, vx, vy, vz | - | 18948/18948 | 0 |
| gps_noise | px, py, pz, vx, vy, vz | px, py, pz, vx, vy, vz | 18948/18948 | 0 |
| gps_outliers(frac=0.01) | px, py, pz | px, py, pz | 18948/18948 | 0 |
| gps_rate(hz=1.0) | gps_ok | - | 18948/18948 | 18673 |
| gyro_bias(dps=0.2) | gx, gy, gz | gx, gy, gz | 18948/18948 | 0 |
| gyro_bias(dps=1.0) | gx, gy, gz | gx, gy, gz | 18948/18948 | 0 |
| gyro_bias(dps=1.0,from_cal=False) | gx, gy, gz | - | 18948/18948 | 0 |
| gyro_drift(dps=0.5,over_s=60) | gx, gy, gz | - | 18948/18948 | 0 |
| accel_bias(axis=x,ms2=0.05) | ax | ax | 18948/18948 | 0 |
| accel_bias(axis=x,ms2=0.2) | ax | ax | 18948/18948 | 0 |
| accel_bias(axis=x,ms2=0.5) | ax | ax | 18948/18948 | 0 |
| accel_bias(axis=z,ms2=0.05) | az | az | 18948/18948 | 0 |
| accel_bias(axis=z,ms2=0.2) | az | az | 18948/18948 | 0 |
| accel_bias(axis=z,ms2=0.5) | az | az | 18948/18948 | 0 |
| accel_bias(axis=x,from_cal=False,ms2=0.2) | ax | - | 18948/18948 | 0 |
| mag_bias(frac=0.05) | mx, my, mz | mx, my, mz | 18948/18948 | 0 |
| mag_bias(frac=0.15) | mx, my, mz | mx, my, mz | 18948/18948 | 0 |
| mag_interference(amp=0.3) | mx, my, mz | - | 18948/18948 | 0 |
| mag_interference(amp=1.0) | mx, my, mz | - | 18948/18948 | 0 |
| imu_noise | ax, ay, az, gx, gy, gz | ax, ay, az, gx, gy, gz | 18948/18948 | 0 |
| imu_spikes | ax, ay, az, gx, gy, gz | - | 18948/18948 | 0 |
| imu_dropouts | - | - | 18589/18948 | 0 |
| imu_gap(gap_s=0.2) | - | - | 18898/18948 | 0 |
| imu_gap(gap_s=1.0) | - | - | 18697/18948 | 0 |
| combined_realistic | ax, ay, az, gx, gy, gz, mx, my, mz, px, py, pz, vx, vy, vz | ax, ay, az, gx, gy, gz, mx, my, mz, px, py, pz, vx, vy, vz | 18948/18948 | 0 |
| cal_ideal | - | ax, ay, az, gx, gy, gz, mx, my, mz | 18948/18948 | 0 |

Per-fault channel deltas (faulted - clean, recording `hover`): `fault_channels.png`

## 3. NEES metric sanity

10k errors drawn from N(0, P) for a random SPD 3x3 P: mean NEES/dof = **0.994** (need 1 +/- 0.05), fraction outside chi2(3) 95% bounds = 0.047 (expect ~0.05) -> **PASS**

## 4. Determinism

See `compare.py` of two full runs (recorded in the report).
