# python_drone

Attitude estimation for a drone from gyro + accelerometer + magnetometer — three filters (`complementary.py`, `ekf.py`), tested with a simulated 10° roll tilt and a constant 5°/s gyro bias.

## Results

| | Complementary filter | EKF (angle-based) | EKF (raw-vector + bias) |
|---|---|---|---|
| True roll | 10° | 10° | 10° |
| Converged roll | ~12.6–12.7° | ~10.14° | ~10.03° |
| Converged pitch | n/a (0°, no coupling) | ~4.8° | ~0.017° |
| Converged yaw | n/a (0°, no coupling) | ~0.86° | ~0.003° |

Each upgrade closes a different gap: the fixed-weight complementary filter can't reject a sustained gyro bias, the angle-based EKF fixes that via a computed gain but still leaves a coupling-driven offset in pitch/yaw, and adding raw-vector measurements plus a gyro-bias state pulls all three angles close to their true values.
