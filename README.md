# python_drone

A from-scratch EKF and PID controller for a drone, built as the foundation for battery state-of-charge estimation and smarter return-to-home.

## Overview

The end goal is battery state-of-charge (SOC) estimation that lets the drone make better return-to-home decisions. Drones are power hungry, so they drain battery quickly and non-linearly. The drone should know not just how far it is from home, but also how confident it can be that it will make it home.

*Caveat: SOC estimation itself isn't built yet. This repo is the foundation for it.*

I built a state estimator (EKF) and a flight controller (PID) from scratch to learn them properly, and I'm applying them to SOC-aware return-to-home. These are the two core pieces an autopilot like PX4 normally gives you as a black box:

1. **State estimation**: an EKF that turns raw gyro, accelerometer, magnetometer, GPS and barometer readings into an estimate of the drone's orientation, velocity and position.
2. **Control**: a cascaded PID controller (position → velocity → attitude → rate → motors) that flies the drone (in simulation) using that estimate.

## Why it's built this way: design decisions and test results

The estimator is a quaternion multiplicative EKF (MEKF) that fuses gyro, accelerometer, magnetometer, GPS and barometer readings. It also estimates each sensor's bias (gyro, accel, mag, baro) as part of the state, and statistical gates reject any measurement too far from what the filter predicts.

Below I explain why it's built this way. I ran a bunch of tests on simulated data to determine which features were worth adding. The hard part about estimating the drone's orientation and position is that my sensors are noisy & biased, due to manufacturing imperfections and chassis jitter and electromagnetic interference, etc. So a lot of work went into minimizing the sensor bias and noise. 

### 1. An EKF that estimates gyro bias, not a complementary filter

First I wanted to see if it even made sense to use an EKF. The most basic "filter" is a complementary filter: it uses gyro and accel/mag readings blended with a constant weighting. You always trust gyro 40% and accel/mag the other 60%. It also has nothing that estimates the gyro's bias, so it can't remove it.

An EKF recomputes this weighting each timestep, based on its uncertainty. Every step it integrates the gyro, its uncertainty grows; every accel/mag correction shrinks it again. The weighting comes from comparing that uncertainty to how noisy each sensor is (noise levels I set). So when the filter is uncertain it leans on accel/mag, and when it's confident it leans on the gyro.

The weighting alone doesn't remove bias, though. That takes adding the gyro bias to the EKF's state, so the filter estimates the bias and subtracts it. I tested three filters: the complementary filter, an EKF without a bias state, and an EKF with one. First holding a fixed tilt, then following a moving trajectory.

#### Test: converging on a fixed 10° tilt with a constant 5°/s gyro bias

![Static test comparison](state%20estimation/testing/static_comparison.png)

| | Complementary filter | EKF (angle-based) | EKF (raw-vector + bias) |
|---|---|---|---|
| Converged roll (true: 10°) | ~12.6–12.7° | ~10.14° | ~10.03° |
| Converged pitch (true: 0°, isolated single-axis test) | ~12.55° | ~4.8° | ~0.017° |
| Converged yaw (true: 0°, isolated single-axis test) | ~12.54° | ~0.86° | ~0.003° |

(Complementary filter has no cross-axis coupling, so pitch/yaw are tested with their own isolated 5°/s bias, not the same combined run as the EKFs.)

#### Test: RMS error on identical data, at normal and 10x gyro bias (`ekf_rms.py`)

The trajectory is a sine wave on each axis, with sensor readings (bias + noise) generated from it each step.

RMS (root-mean-square) error: square each iteration's error, average over the run, square-root back to degrees. One number per filter that summarizes accuracy over a whole run and penalizes big spikes more than a plain average would.

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

Complementary is already visibly worse at original bias (fixed weight, no computed gain), and falls apart at 10x bias (roll RMS 9.6°) while both EKFs do way better, since since they're actively canceling the bias rather than just tolerating it.

### 2. Quaternion attitude (MEKF), not Euler angles (`quaternarions.py`)

**Decision:** store attitude as a quaternion. Euler angles hit gimbal lock at some orientations; a quaternion never does. The test below shows the switch costs no accuracy.

Switched the attitude state from Euler angles to a quaternion, which turns this into a multiplicative EKF (MEKF): a quaternion has 4 numbers for only 3 rotational degrees of freedom, so it can't be corrected by plain vector addition the way Euler angles can. Instead, the Kalman math (`P`, `K`) runs on a 3-dimensional small-angle error state, which then gets folded onto the quaternion multiplicatively and renormalized — this avoids gimbal lock (no singular orientation, unlike Euler angles) and keeps the covariance math well-posed instead of operating on a redundant, constrained state. It performed essentially identically on the RMS test (~0.445°/0.305°/0.230° vs. 0.443/0.305/0.229 for the Euler version), which is expected — same underlying model, just a numerically better-behaved state representation.

### 3. Distrust the accelerometer during motion, and reject impossible readings (`ekf.py`, `test_gating.py`)

**Decision:** the accelerometer only measures gravity (and so tilt) when the drone isn't accelerating. So the filter trusts it less during motion, and throws out readings that are too far off to be real.

`ekf.py` now inflates accel's measurement noise (`R_accel`) when the raw accel magnitude deviates from 1g (real acceleration, not just tilt), and gates out (fully rejects) any accel reading too implausible to trust at all — a chi-squared test on the residual, 3 DOF.

**How the chi-squared gate works.** Before each correction the filter predicts what the sensor should read; the *residual* is the actual reading minus that prediction. A fixed limit ("reject if off by more than X") doesn't work, because what counts as a big residual changes: right after startup the filter is uncertain and large residuals are normal, while after a long clean flight the same residual is suspicious, and each axis has its own noise level. So the residual is first scaled by how big it's *expected* to be, `S = H P Hᵀ + R` (the filter's own uncertainty mapped into sensor space, plus sensor noise):

```
d² = residualᵀ · S⁻¹ · residual
```

This is the squared Mahalanobis distance, the residual measured in standard deviations (in 1D it's just (residual / σ)²). If the filter's model is right, each scaled axis is a standard Gaussian, and a sum of k squared standard Gaussians follows a **chi-squared distribution with k degrees of freedom** (k = how many numbers are compared: 3 for accel x/y/z). That distribution is known, so the threshold can be looked up: for 3 DOF, `d²` stays under 11.34 99% of the time. Rejecting above that means a good reading is wrongly thrown out only 1% of the time, a chosen false-rejection rate instead of a guessed limit. The gate also adapts on its own: as `P` shrinks it tightens, and as `P` grows it loosens.

#### Test: an injected 8.7g accelerometer spike, gate on vs. off

`test_gating.py` injects a physically-absurd accel spike (~8.7g) for 10 iterations and compares roll with gating on vs. off:

| | Roll before spike | Max roll during spike | Recovery |
|---|---|---|---|
| Without gating | 10.06° | 32.07° | ~30+ iterations |
| With gating | 10.06° | 10.28° | Immediate |

Gating only works because it checks the residual against the *base* noise, not the already-inflated adaptive `R` — gating against the inflated value let the spike pass every time (adaptive `R` grows in lockstep with the outlier, so the Mahalanobis distance never crosses the threshold no matter how extreme the reading is).

### 4. Add GPS to estimate position and velocity (`gps.py`, `test_gps_tracking.py`)

**Decision:** integrating the IMU alone drifts without bound, so the filter also tracks position and velocity and corrects them with GPS. Return-to-home needs to know where home is.

Extends the quaternion MEKF's error state to 12 dimensions (adds velocity + position), predicting from gravity-compensated world-frame acceleration and correcting with GPS position + velocity gated to a realistic ~5Hz, instead of every IMU tick.

![GPS tracking trajectory](state%20estimation/testing/gps_tracking_trajectory.png)
![GPS tracking RMS comparison](state%20estimation/testing/gps_tracking_rms.png)

GPS cuts position error 8-10x across the board; a 10°-tilted version of each scenario matches these numbers exactly, confirming `update_F`'s attitude/acceleration coupling handles a non-identity attitude correctly.

### 5. Gate every sensor, not just the accelerometer (`gps.py`, `test_gates.py`)

**Decision:** GPS and the magnetometer fail too (multipath reflections, magnetic interference), so every correction gets its own gate.

`gps.py` gates every correction source, not just accel: accel and mag each get a 3-DOF chi-squared test on their residual, and GPS gets its own 6-DOF version (it corrects position + velocity jointly). Any correction that fails its gate is skipped entirely for that tick — state and `P` are left exactly as the predict step produced them, rather than clamped or partially applied.

#### Test: an injected 50 m GPS jump and a 90°-wrong magnetometer reading, gates on vs. off

`test_gates.py` checks two things: that the gates stay quiet on clean data (a gate that fires on legitimate readings is worse than no gate at all), and that they actually catch a bad reading the way the original accel gate did. It runs a stationary ground truth so any deviation is unambiguous, injects a +50m GPS multipath-style spike and a 90°-wrong magnetometer reading, and compares gates on vs. off:

| | GPS rejected | Mag rejected | Peak position error | Peak attitude error |
|---|---|---|---|---|
| Clean run (no outliers) | 0/0 | 0/0 | — | — |
| Gates ON | 1/1 | 1/1 | 0.00 m | 0.00° |
| Gates OFF | 0/1 | 0/1 | 9.82 m | 15.53° |

With gates on, both outliers are caught and the state never moves. Without them, the GPS spike alone injects ~9.4m of position error that takes many correction cycles to bleed off, and the mag spike's ~15.5° of attitude error keeps doing damage afterward — the corrupted attitude estimate misreads subsequent accelerometer readings, so position keeps drifting for several ticks even though the bad reading was magnetometer, not GPS.

### 6. Apply corrections one at a time, not pooled (`gps.py`, `calibration.py`, `test_gate_monte_carlo.py`)

**Decision:** apply each sensor's correction immediately, so the next one is computed against an up-to-date estimate. Tested with repeated randomized startups (Monte Carlo).

GPS, accel, and mag corrections used to be pooled against one stale attitude estimate per tick and injected together, which is only safe for a linear filter; now each one is applied immediately and the next re-linearizes against the result. That alone cut a real ~30% chance of permanently locking the accel gate on at startup down to 0/20 in Monte Carlo testing (`test_gate_monte_carlo.py`), independent of how well `P0` happens to be tuned.

### 7. Estimate magnetometer bias too (`gps.py`, `calibration.py`, `test_accel_bias_convergence.py`)

**Decision:** like the gyro and accelerometer, the magnetometer has its own offset, and left alone it shows up as heading error. So it gets a bias state as well.

Added `mag_bias` as a full state (mirrors `accel_bias`: same tilt/bias ambiguity, same fix - a yaw wiggle added to the pre-flight calibration breaks mag_bias from heading error the way the roll/pitch wiggle already broke accel_bias from tilt). Converges to within ~0.0004 of the true injected bias under a rotating trajectory.

### 8. Stress tests: sustained interference, GPS dropout, a sudden bias jump (`test_robustness_scenarios.py`)

Not a new design choice: these check how the finished design behaves under longer failures, and they show its known limits.

Three untested-but-plausible failure modes, checked directly: a sustained (not single-spike) magnetometer interference burst is rejected 500/500 by the gate, but heading still drifts ~26° during a 5s outage and recovers slowly afterward - yaw has no backup reference once mag is unavailable. A 30s GPS dropout degrades boundedly (~0.7m to ~1.7m) and recovers cleanly once GPS returns. A sudden mid-flight accel_bias step is tracked correctly but slowly (~40s to mostly converge), consistent with `gyro_bias`'s already-known slow settling rather than a new issue.

### 9. Flying it in Gazebo: what only real physics revealed (`run_sim.py`, `PID/`)

The final check: can a controller actually fly on this estimate? Synthetic tests can't catch mismatches between the filter's assumptions and a real physics engine's sensors. Several of the settings below (accel correction off in flight, pinned bias noise) were chosen because of what this showed.

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
