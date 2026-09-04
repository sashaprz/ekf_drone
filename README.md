# python_drone

Attitude estimation for a drone from gyro + accelerometer + magnetometer — a complementary filter (`complementary.py`) vs. an EKF (`ekf.py`), tested with a simulated 10° roll tilt and a constant 5°/s gyro bias.

## Results

| | Complementary filter | EKF |
|---|---|---|
| True roll | 10° | 10° |
| Converged roll | ~12.6–12.7° | ~10.14° |
| Why | Fixed blend weight (0.98) can't fully reject a sustained gyro bias — settles at an offset instead of the true value | Kalman gain computed from `Q`/`R` (~0.27 trust in the measurement, vs. the complementary filter's fixed 2%) tracks the true value much more closely |
| Coupled pitch/yaw (gyro_y ≠ 0) | Not supported (decoupled per-axis integration) | pitch/yaw pick up realistic nonzero values, since the process-model Jacobian correctly couples gyro_y into both |

Neither filter converges to *exactly* 10° — that residual is a genuinely constant gyro bias, which would need a bias state in the filter to fully cancel (not yet implemented).
