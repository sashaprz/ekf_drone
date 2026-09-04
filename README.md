# python_drone

Attitude estimation for a drone from gyro + accelerometer + magnetometer — three filters (`complementary.py`, `ekf.py`), tested with a simulated 10° roll tilt and a constant 5°/s gyro bias.

## Results

| | Complementary filter | EKF (angle-based) | EKF (raw-vector + bias) |
|---|---|---|---|
| What it does | Blends gyro integration with accel/mag-derived angles using a fixed 0.98 weight | Same idea, but the gain is computed dynamically from `Q`/`R`, and the process model uses a real coupled Jacobian (`F`) instead of independent per-axis integration | Adds a gyro-bias state (subtracted from the raw gyro before integration) and compares the raw accel vector directly against a predicted vector via a real `H` Jacobian, instead of pre-converting to angles |
| Converged roll (true: 10°) | ~12.6–12.7° | ~10.14° | ~10.03° |
| Converged pitch (true: 0°) | n/a (no coupling) | ~4.8° | ~0.017° |
| Converged yaw (true: 0°) | n/a (no coupling) | ~0.86° | ~0.003° |
