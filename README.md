# python_drone

Attitude estimation for a drone from gyro + accelerometer + magnetometer — complementary filter vs. two EKF designs.

## Static test (fixed 10° tilt, constant 5°/s gyro bias)

| | Complementary filter | EKF (angle-based) | EKF (raw-vector + bias) |
|---|---|---|---|
| Converged roll (true: 10°) | ~12.6–12.7° | ~10.14° | ~10.03° |
| Converged pitch (true: 0°) | n/a (no coupling) | ~4.8° | ~0.017° |
| Converged yaw (true: 0°) | n/a (no coupling) | ~0.86° | ~0.003° |

## Dynamic trajectory test (`ekf_gyro_bias.py`)

Sine-wave ground truth per axis, sensors synthesized from it each step (bias + noise), true value compared against the estimate directly instead of eyeballed.

| Iteration | Change | Roll error | Pitch error | Yaw error |
|---|---|---|---|---|
| 1 | Gyro synthesized; accel/mag still static | Doesn't track | Doesn't track | Doesn't track |
| 2 | Accel + mag synthesized too, but mag yaw sign was backwards | ~±1° | ~±1–2° | Grows unbounded |
| 3 | Fixed the mag yaw sign bug | ~±0.3–0.5° | ~±0.3–0.5° | ~±0.3–0.5° |

## RMS comparison (`ekf_rms.py`)

Both EKF designs run against identical trajectory + sensor stream, fixed `dt`, 20s.

| Gyro bias | Filter | Roll RMS | Pitch RMS | Yaw RMS |
|---|---|---|---|---|
| Original (2, -1, 0.5°/s) | Angle-based | 0.474° | 0.427° | 0.238° |
| Original | Raw-vector + bias | 0.469° | 0.296° | 0.240° |
| 10x larger | Angle-based | 0.723° | 0.510° | 0.268° |
| 10x larger | Raw-vector + bias | 0.483° | 0.299° | 0.246° |

At 10x bias the angle-based EKF degrades on every axis while raw-vector+bias barely moves — the bias state is actively canceling the bias, not just tolerating it.
