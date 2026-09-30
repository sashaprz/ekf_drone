# python_drone

Attitude and position estimation for a drone from gyro + accelerometer + magnetometer + GPS — a complementary filter vs. a progression of EKF designs, from Euler-angle to quaternion attitude, adaptive noise + gating, and GPS-fused position/velocity tracking.

**Gyro bias**: a real gyro doesn't read exactly zero at rest — manufacturing imperfections, temperature drift, and vibration give it a small persistent offset that isn't true rotation. Left uncorrected it integrates into steadily growing angle error, which is why the better filters below estimate and subtract it.

## Static test (fixed 10° tilt, constant 5°/s gyro bias)

![Static test comparison](state%20estimation/testing/static_comparison.png)

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

![RMS error comparison](state%20estimation/testing/rms_comparison.png)

| Gyro bias | Filter | Roll RMS | Pitch RMS | Yaw RMS |
|---|---|---|---|---|
| Original (2, -1, 0.5°/s) | Complementary | 0.954° | 1.249° | 0.327° |
| Original | Angle-based | 0.445° | 0.460° | 0.230° |
| Original | Raw-vector + bias | 0.443° | 0.305° | 0.229° |
| 10x larger | Complementary | 9.603° | 5.029° | 2.534° |
| 10x larger | Angle-based | 0.681° | 0.521° | 0.268° |
| 10x larger | Raw-vector + bias | 0.449° | 0.308° | 0.229° |

Complementary is already visibly worse at original bias (fixed weight, no computed gain), and falls apart at 10x bias (roll RMS 9.6°) while both EKFs stay well-behaved — raw-vector+bias barely moves at all, since it's actively canceling the bias rather than just tolerating it.

## From EKF to MEKF: quaternion attitude (`quaternarions.py`)

Switched the attitude state from Euler angles to a quaternion, which turns this into a multiplicative EKF (MEKF): a quaternion has 4 numbers for only 3 rotational degrees of freedom, so it can't be corrected by plain vector addition the way Euler angles can. Instead, the Kalman math (`P`, `K`) runs on a 3-dimensional small-angle error state, which then gets folded onto the quaternion multiplicatively and renormalized — this avoids gimbal lock (no singular orientation, unlike Euler angles) and keeps the covariance math well-posed instead of operating on a redundant, constrained state. It performed essentially identically on the RMS test (~0.445°/0.305°/0.230° vs. 0.443/0.305/0.229 for the Euler version), which is expected — same underlying model, just a numerically better-behaved state representation.

## Motion robustness: adaptive R + gating (`ekf.py`, `test_gating.py`)

`ekf.py` now inflates accel's measurement noise (`R_accel`) when the raw accel magnitude deviates from 1g (real acceleration, not just tilt), and gates out (fully rejects) any accel reading too implausible to trust at all — a chi-squared test on the residual, 3 DOF.

`test_gating.py` injects a physically-absurd accel spike (~8.7g) for 10 iterations and compares roll with gating on vs. off:

| | Roll before spike | Max roll during spike | Recovery |
|---|---|---|---|
| Without gating | 10.06° | 32.07° | ~30+ iterations |
| With gating | 10.06° | 10.28° | Immediate |

Gating only works because it checks the residual against the *base* noise, not the already-inflated adaptive `R` — gating against the inflated value let the spike pass every time (adaptive `R` grows in lockstep with the outlier, so the Mahalanobis distance never crosses the threshold no matter how extreme the reading is).

## GPS position/velocity tracking (`gps.py`, `test_gps_tracking.py`)

Extends the quaternion MEKF's error state to 12 dimensions (adds velocity + position), predicting from gravity-compensated world-frame acceleration and correcting with GPS position + velocity gated to a realistic ~5Hz, instead of every IMU tick.

![GPS tracking trajectory](state%20estimation/testing/gps_tracking_trajectory.png)
![GPS tracking RMS comparison](state%20estimation/testing/gps_tracking_rms.png)

GPS cuts position error 8-10x across the board; a 10°-tilted version of each scenario matches these numbers exactly, confirming `update_F`'s attitude/acceleration coupling handles a non-identity attitude correctly.

## Gating all three corrections (`gps.py`, `test_gates.py`)

`gps.py` gates every correction source, not just accel: accel and mag each get a 3-DOF chi-squared test on their residual, and GPS gets its own 6-DOF version (it corrects position + velocity jointly). Any correction that fails its gate is skipped entirely for that tick — state and `P` are left exactly as the predict step produced them, rather than clamped or partially applied.

`test_gates.py` checks two things: that the gates stay quiet on clean data (a gate that fires on legitimate readings is worse than no gate at all), and that they actually catch a bad reading the way the original accel gate did. It runs a stationary ground truth so any deviation is unambiguous, injects a +50m GPS multipath-style spike and a 90°-wrong magnetometer reading, and compares gates on vs. off:

| | GPS rejected | Mag rejected | Peak position error | Peak attitude error |
|---|---|---|---|---|
| Clean run (no outliers) | 0/0 | 0/0 | — | — |
| Gates ON | 1/1 | 1/1 | 0.00 m | 0.00° |
| Gates OFF | 0/1 | 0/1 | 9.82 m | 15.53° |

With gates on, both outliers are caught and the state never moves. Without them, the GPS spike alone injects ~9.4m of position error that takes many correction cycles to bleed off, and the mag spike's ~15.5° of attitude error keeps doing damage afterward — the corrupted attitude estimate misreads subsequent accelerometer readings, so position keeps drifting for several ticks even though the bad reading was magnetometer, not GPS.

## Pooled vs. sequential corrections (`gps.py`, `calibration.py`, `test_gate_monte_carlo.py`)

GPS, accel, and mag corrections used to be pooled against one stale attitude estimate per tick and injected together, which is only safe for a linear filter; now each one is applied immediately and the next re-linearizes against the result. That alone cut a real ~30% chance of permanently locking the accel gate on at startup down to 0/20 in Monte Carlo testing (`test_gate_monte_carlo.py`), independent of how well `P0` happens to be tuned.

## Magnetometer bias (`gps.py`, `calibration.py`, `test_accel_bias_convergence.py`)

Added `mag_bias` as a full state (mirrors `accel_bias`: same tilt/bias ambiguity, same fix - a yaw wiggle added to the pre-flight calibration breaks mag_bias from heading error the way the roll/pitch wiggle already broke accel_bias from tilt). Converges to within ~0.0004 of the true injected bias under a rotating trajectory.

## Robustness scenarios (`test_robustness_scenarios.py`)

Three untested-but-plausible failure modes, checked directly: a sustained (not single-spike) magnetometer interference burst is rejected 500/500 by the gate, but heading still drifts ~26° during a 5s outage and recovers slowly afterward - yaw has no backup reference once mag is unavailable. A 30s GPS dropout degrades boundedly (~0.7m to ~1.7m) and recovers cleanly once GPS returns. A sudden mid-flight accel_bias step is tracked correctly but slowly (~40s to mostly converge), consistent with `gyro_bias`'s already-known slow settling rather than a new issue.

## Flying it: PID cascade in Gazebo (`run_sim.py`, `PID/`)

The EKF (`FINAL_gps.py`, the evolved `gps.py`) now drives a cascaded PID controller (position → velocity → attitude → rate → motor mixer) flying PX4's x500 quadrotor in Gazebo Harmonic — real rigid-body physics and simulated sensors, but PX4's own estimator and controller are bypassed entirely; `gz_bridge.py` reads Gazebo's IMU/mag/GPS topics and publishes motor speeds directly.

For a long time it tumbled within ~4-6s of enabling position hold, which looked like a controller resonance. Two diagnostic tools found the real cause: Gazebo's **ground truth** printed next to the estimate, and an **oracle mode** where the controller flies on ground truth instead of the EKF. On truth, with identical gains, it hovered dead level — the controller was fine; the EKF was feeding it a wrong state. Bugs found, all in how Gazebo's data met the EKF:

| Bug | Symptom | Fix |
|---|---|---|
| GPS fed as (north, east) but EKF's x axis is east | A swap is a mirror, not a rotation — position feedback pushed the wrong way on one diagonal | Bridge outputs ENU, matching Gazebo's world frame |
| Mag reference hardcoded horizontal `(1,0,0)` | Gazebo's field is steeply inclined; calibration hid it in `mag_bias_z ≈ 0.89`, and every tilt leaked into fake heading corrections | Reference measured from flight data vs. ground truth: `(0.450, 0.020, 0.893)` |
| Attitude process noise 0.01 rad²/step | ~10⁹x Gazebo's real gyro noise — single GPS updates rotated attitude by tens of degrees | 1e-7 in live mode |
| Accel gravity correction in flight | A multirotor's accel measures *thrust* (body z), not gravity, so it pulled attitude toward level while tilted — and \|a\| ≈ 1g, so the gate couldn't catch it | Off in live mode; GPS velocity observes tilt through `F`'s attitude→velocity block instead |
| Accel/mag bias random walk too loose | Heading error absorbed into `mag_bias` (an injected 10° yaw error stuck at 6.7°), tilt into `accel_bias` (wandered to -3 m/s²) | Pinned near calibrated values in live mode |
| GPS gate lockout | After one hard event (a ground touch), GPS was rejected forever and altitude ran away to -115 m | Reset position/velocity to GPS after 5 consecutive rejections |

Attitude error on a replay of the flight that crashed (first 12s, same recorded sensor data each time — see `testing/replay_ekf.py` below):

| EKF config | Roll RMS | Pitch RMS | Yaw RMS |
|---|---|---|---|
| Correct mag reference, accel correction on, loose bias noise | 16.9° | 8.1° | 12.2° |
| + bias noise pinned | 4.7° | 1.5° | 9.0° |
| + accel gravity correction off | **0.14°** | **0.12°** | **0.20°** |

One lesson worth keeping: the magnetometer was first "fixed" by copying PX4's remap for Gazebo's supposedly left-handed mag frame. Checked against ground truth under rotation, that made estimated yaw turn *opposite* to true yaw — the raw reading was already correct, only the reference vector was wrong. Frame claims get verified against truth now, not taken from comments.

Two controller fixes, confirmed in oracle mode: altitude gains re-derived from the actual thrust sensitivity (0.026 m/s² per rad/s of motor trim — the old `vel_z` loop was slower than the `pos_z` loop wrapped around it, giving a 0↔4 m oscillation every ~6s), and the attitude D-term moved onto the gyro rate so tilt-setpoint changes no longer cause derivative kicks.

**Result: 3/3 fresh 60s flights on the EKF held true roll/pitch at 0.0°, altitude at 2.00 ± 0.01 m, and horizontal position within a few cm, with position hold on.**

## File structure

```
run_sim.py                  flies the full cascade in Gazebo; GAINS/LIMITS live here (single source of truth)
PID/
  cascade.py                position -> velocity -> attitude -> rate loops, plus the motor mixer
  pid.py                    PID class (anti-windup, filtered derivative-on-measurement)
  gz_bridge.py              Gazebo <-> Python: sensor topics in, motor speeds out, ground truth for logging
state estimation/
  FINAL_gps.py              the EKF in use: 18-state quaternion MEKF (attitude, gyro bias, velocity,
                            position, accel bias, mag bias) - the evolved gps.py from the sections above
  calibration.py            pre-flight calibration filter that seeds FINAL_gps.py
  complementary.py, ekf_angle_based.py, ekf_gyro_bias.py, ekf.py, quaternarions.py
                            the earlier filters from the progression above, kept for comparison
  testing/                  offline EKF tests + the graphs in this README (test_*.py, plot_*.py, *.png)
testing/                    Gazebo test harnesses + the replay tool
HANDOFF.md                  running engineering log: every bug, theory, and result, in detail
```

### Running it (needs WSL + PX4/Gazebo, see `HANDOFF.md`)

Start Gazebo with `make px4_sitl gz_x500` in `~/PX4-Autopilot`, kill only the PX4 process (`pkill -9 -f bin/px4`) so the motor topic is free, then:

| Command | What it does |
|---|---|
| `python3 run_sim.py` | Full cascade: climb to 2 m and hold. Prints the estimate next to Gazebo's true attitude/position |
| `ORACLE=1 python3 run_sim.py` | Controller flies on ground truth instead of the EKF — separates "controller problem" from "estimator problem" |
| `SENSOR_LOG=flight.csv python3 run_sim.py` | Also records every tick's raw sensors + ground truth, for replay |
| `bash testing/run_rate_test.sh roll 0.3 5.0` | Innermost loop only: step one axis's rotation rate (rad/s) — args are axis, step, duration |
| `bash testing/run_attitude_test.sh pitch 0.2 3.0` | Attitude + rate loops: step a tilt/heading angle (rad) |
| `bash testing/run_velocity_test.sh vz 0.5 3.0` | Velocity + attitude + rate loops: step a velocity (m/s) |

The loop harnesses exist for inside-out tuning — the standard flight-controller practice of getting the rate loop right before trusting attitude, and attitude before velocity/position. They share takeoff/reset code in `testing/test_common.py` and write their logs into `testing/`.

### Offline replay (`testing/replay_ekf.py`)

`python testing/replay_ekf.py flight.csv` feeds a recorded flight back through `DroneEKF` exactly as if it were live, then reports the estimate's error against Gazebo's ground truth. No Gazebo needed, ~3 seconds per run, identical data every time, so two EKF versions can be compared fairly — that's how the estimator bugs above were found. Knobs: `T_END=12` scores only the first N seconds; `EXP="QATT=1e-6 R_ACC=5 ROLL0=5 ..."` overrides noise settings or injects a known attitude error without editing the EKF (full list in the file). Add `MAG_REMAPPED=0` for recordings made after 2026-09-30.
