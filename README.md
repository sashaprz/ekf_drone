# python_drone

Attitude estimation for a drone from gyro + accelerometer + magnetometer — three filters (`complementary.py`, `ekf.py`), tested with a simulated 10° roll tilt and a constant 5°/s gyro bias.

## Results

| | Complementary filter | EKF (angle-based) | EKF (raw-vector + bias) |
|---|---|---|---|
| What it does | Blends gyro integration with accel/mag-derived angles using a fixed 0.98 weight | Same idea, but the gain is computed dynamically from `Q`/`R`, and the process model uses a real coupled Jacobian (`F`) instead of independent per-axis integration | Adds a gyro-bias state (subtracted from the raw gyro before integration) and compares the raw accel vector directly against a predicted vector via a real `H` Jacobian, instead of pre-converting to angles |
| Converged roll (true: 10°) | ~12.6–12.7° | ~10.14° | ~10.03° |
| Converged pitch (true: 0°) | n/a (no coupling) | ~4.8° | ~0.017° |
| Converged yaw (true: 0°) | n/a (no coupling) | ~0.86° | ~0.003° |

## Dynamic trajectory test

The results above all use a **static** test: a fixed 10° tilt and a constant gyro rate. That's enough to check whether a filter settles near the right number, but it can't test whether a filter *tracks a moving target* — a static pose has no time-varying truth to compare against, so a filter that's actually broken (e.g. correcting toward the wrong sign) can still look fine if it happens to sit near the right value.

`ekf_gyro_bias.py` generates a moving ground-truth trajectory instead: `roll`/`pitch`/`yaw` each follow a sine wave (different amplitude, frequency, and phase per axis), with known analytic derivatives. Gyro, accel, and mag readings are synthesized from that true trajectory each iteration — gyro via the inverse of the body-rate/world-rate kinematics (true rate → simulated body rate), accel and mag via the same rotation-based formulas used elsewhere in the filter — each with realistic bias (gyro only) and Gaussian noise added on top. The true value is printed alongside the estimate every iteration, so tracking error is measured directly instead of eyeballed. We'll keep iterating on this test as the filter evolves; each iteration gets a row below.

### Performance by iteration

| Iteration | Change | Roll error | Pitch error | Yaw error |
|---|---|---|---|---|
| 1 | Trajectory + gyro synthesis wired up; accel/mag still static, disconnected from the true motion | Doesn't track — pinned near the old static reading regardless of the true (swinging ±15°) trajectory | Doesn't track (pinned near 0) | Doesn't track (pinned near 0) |
| 2 | Accel + mag synthesized from true trajectory too, but `get_mag`'s yaw sign convention was backwards | Roughly bounded, noisy (~±1°) | Roughly bounded, noisy (~±1–2°) | Grows without bound (past +1.5° and still climbing) |
| 3 | Fixed the mag yaw sign bug | ~±0.3–0.5° | ~±0.3–0.5° | ~±0.3–0.5° |
