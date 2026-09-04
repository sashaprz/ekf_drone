# python_drone

Attitude estimation for a drone from gyro + accelerometer + magnetometer — complementary filter vs. two EKF designs.

**Gyro bias**: a real gyro doesn't read exactly zero at rest — manufacturing imperfections, temperature drift, and vibration give it a small persistent offset that isn't true rotation. Left uncorrected it integrates into steadily growing angle error, which is why the better filters below estimate and subtract it.

## Static test (fixed 10° tilt, constant 5°/s gyro bias)

![Static test comparison](static_comparison.png)

| | Complementary filter | EKF (angle-based) | EKF (raw-vector + bias) |
|---|---|---|---|
| Converged roll (true: 10°) | ~12.6–12.7° | ~10.14° | ~10.03° |
| Converged pitch (true: 0°, isolated single-axis test) | ~12.55° | ~4.8° | ~0.017° |
| Converged yaw (true: 0°, isolated single-axis test) | ~12.54° | ~0.86° | ~0.003° |

(Complementary filter has no cross-axis coupling, so pitch/yaw are tested with their own isolated 5°/s bias, not the same combined run as the EKFs.)

## Dynamic trajectory test (`ekf_gyro_bias.py`)

Sine-wave ground truth per axis, sensors synthesized from it each step (bias + noise), true value compared against the estimate directly instead of eyeballed.

| Iteration | Change | Roll error | Pitch error | Yaw error |
|---|---|---|---|---|
| 1 | Gyro synthesized; accel/mag still static | Doesn't track | Doesn't track | Doesn't track |
| 2 | Accel + mag synthesized too, but mag yaw sign was backwards | ~±1° | ~±1–2° | Grows unbounded |
| 3 | Fixed the mag yaw sign bug | ~±0.3–0.5° | ~±0.3–0.5° | ~±0.3–0.5° |

## RMS comparison (`ekf_rms.py`)

RMS (root-mean-square) error: square each iteration's error, average over the run, square-root back to degrees. One number per filter that summarizes accuracy over a whole run and penalizes big spikes more than a plain average would — the standard metric for comparing estimators.

All three filters run against identical trajectory + sensor stream (same RNG seed), fixed `dt`, 20s.

![RMS error comparison](rms_comparison.png)

| Gyro bias | Filter | Roll RMS | Pitch RMS | Yaw RMS |
|---|---|---|---|---|
| Original (2, -1, 0.5°/s) | Complementary | 0.954° | 1.249° | 0.327° |
| Original | Angle-based | 0.445° | 0.460° | 0.230° |
| Original | Raw-vector + bias | 0.443° | 0.305° | 0.229° |
| 10x larger | Complementary | 9.603° | 5.029° | 2.534° |
| 10x larger | Angle-based | 0.681° | 0.521° | 0.268° |
| 10x larger | Raw-vector + bias | 0.449° | 0.308° | 0.229° |

Complementary is already visibly worse at original bias (fixed weight, no computed gain), and falls apart at 10x bias (roll RMS 9.6°) while both EKFs stay well-behaved — raw-vector+bias barely moves at all, since it's actively canceling the bias rather than just tolerating it.
