# ekf_drone

A from-scratch EKF and PID controller for a drone, built as the foundation for battery state-of-charge estimation and smarter return-to-home.

## Overview

The end goal is battery state-of-charge (SOC) estimation that lets the drone make better return-to-home decisions. Drones are power hungry, so they drain battery quickly and non-linearly. The drone should know not just how far it is from home, but also how confident it can be that it will make it home.

*Caveat: SOC estimation itself isn't built yet. This repo is the foundation for it.*

I built a state estimator (EKF) and a flight controller (PID) from scratch, and I'm applying them to SOC-aware return-to-home. These are the two core pieces an autopilot like PX4 normally gives you as a black box:

1. **State estimation**: an EKF that turns raw gyro, accelerometer, magnetometer, GPS and barometer readings into an estimate of the drone's orientation, velocity and position.
2. **Control**: a cascaded PID controller (position → velocity → attitude → rate → motors) that flies the drone (in simulation) using that estimate.

## Results at a glance

**Flying in the Gazebo physics simulator on my own EKF and PID controller, the drone held a level hover at 2.00 ± 0.01 m, with horizontal position within a few cm, in 3 out of 3 fresh 60 s flights.**

![Results summary](state%20estimation/testing/project_hero.png)

Left: with a gyro bias 10x larger than normal, the EKF's orientation error is 17x smaller than a basic complementary filter's ([section 1](#1-an-ekf-that-estimates-gyro-bias-not-a-complementary-filter)). Right: adding GPS keeps the position estimate on track, where the accelerometer alone drifts away ([section 4](#4-add-gps-so-the-drone-knows-where-it-is-gpspy-test_gps_trackingpy)).

## The estimator: design decisions and test results

The estimator takes readings from five sensors (gyroscope, accelerometer, magnetometer, GPS and barometer) and blends them into one best guess of which way the drone is pointing, how fast it's moving and where it is. It uses past data to learn how far off each sensor is (its bias) and subtracts that out, and it throws away any reading too far from what it expected to be real.

Below I explain why it's built this way. I ran a bunch of tests on simulated data to determine which features were worth adding. The hard part about estimating the drone's orientation and position is that my sensors are noisy & biased, due to manufacturing imperfections and chassis jitter and electromagnetic interference, etc. So a lot of work went into minimizing the sensor bias and noise.

Each section names the file where that idea was first built and the test that checks it. The filters build on each other, and everything ends up in `state estimation/FINAL_gps.py`, the EKF that actually flies the drone.

### 1. An EKF that estimates gyro bias, not a complementary filter

First I wanted to see if it even made sense to use an EKF. The most basic "filter" is a complementary filter: it uses gyro and accel/mag readings blended with a constant weighting. You always trust gyro 40% and accel/mag the other 60%. It also has nothing that estimates the gyro's bias, so it can't remove it.

An EKF recomputes this weighting each timestep, based on its uncertainty. Every step it integrates the gyro, its uncertainty grows; every accel/mag correction shrinks it again. The weighting comes from comparing that uncertainty to how noisy each sensor is (noise levels I set). So when the filter is uncertain it leans on accel/mag, and when it's confident it leans on the gyro.

The weighting alone doesn't remove bias, though. That takes adding the gyro bias to the EKF's state, so the filter estimates the bias and subtracts it. I tested three filters: the complementary filter, an EKF without a bias state, and an EKF with one.

#### Test: RMS error on identical data, at normal and 10x gyro bias (`ekf_rms.py`)

The trajectory is a sine wave on each axis, with sensor readings (bias + noise) generated from it each step.

RMS (root-mean-square) error: square each iteration's error, average over the run, square-root back to degrees. One number per filter that summarizes accuracy over a whole run and penalizes big spikes more than a plain average would.

All three filters run against identical trajectory + sensor stream (same RNG seed), fixed `dt`, 20s.

![RMS error comparison](state%20estimation/testing/rms_comparison.png)

| Gyro bias | Filter | Roll RMS | Pitch RMS | Yaw RMS |
|---|---|---|---|---|
| Original (2, -1, 0.5°/s) | Complementary | 0.954° | 1.249° | 0.327° |
| Original | EKF, no bias state | 0.445° | 0.460° | 0.230° |
| Original | EKF + bias state | 0.443° | 0.305° | 0.229° |
| 10x larger | Complementary | 9.603° | 5.029° | 2.534° |
| 10x larger | EKF, no bias state | 0.681° | 0.521° | 0.268° |
| 10x larger | EKF + bias state | 0.449° | 0.308° | 0.229° |

Complementary is already visibly worse at original bias (fixed weight, no computed gain), and falls apart at 10x bias (roll RMS 9.6°) while both EKFs do way better. The EKF with a bias state barely moves at all, since it's actively canceling the bias rather than just tolerating it.

### 2. Quaternion attitude (MEKF) instead of Euler angles (`quaternions.py`)

Standard Euler angles (roll, pitch, yaw) blow up at the axes. For example, yaw = (q·sin φ + r·cos φ) / cos θ. If cos θ is 0, then this is undefined. This is called gimbal lock. 

Every 3-number representation of a 3D rotation has a discontinuity somewhere. The best explanation is [3Blue1Brown's video on this topic](https://youtu.be/d4EgbgTm0Bg), but I will give a quick summary. 

Suppose you have a 1D rotation - a simple circle. Since this is 1D, it is effectively just a line, and you can picture opening up that circle to create a line. You can represent position on that circle (line) with one number, θ. 0 and 2π represent the same spot in the rotation. But on the line, they are very far apart. This is called a seam. It is where points that should be neighbours, are no longer neighbours. Since a filter looks at tiny changes, it is unnatural for it to say that 2π + some tiny increase in angle is 0. 

If we instead keep this 1D rotation, a circle, inside a 2D space (a plane), then we remove this discontinuity. Instead, we can represent every point on the circle as x = cos θ, y = sin θ. By increasing our number of descriptors, we can describe the circle as what it actually is: a loop in a 2D plane. 

However we also need to add a constraint. A circle is a 1D object, but our descriptors currently have 2 degrees of freedom (x and y can span the whole plane, but we want to only describe the circle with radius = 1). So we constrain x² + y² = 1, reducing the degrees of freedom back down to 1. 

Quaternions are the same idea, just in higher dimensions. To describe a rotation in 3D space, we use 4 numbers, plus the constraint x² + y² + z² + w² = 1. 

That constraint is also why the correction has to be multiplied in rather than added. Back on the circle: if the filter nudges (x, y) by adding a small vector, the result lands slightly off the circle, so it's no longer a valid rotation. The natural correction is to *rotate* the point by a small angle, which keeps it on the circle by construction. Quaternions work the same way: adding a correction vector to the 4 numbers breaks x² + y² + z² + w² = 1, while multiplying by a small rotation quaternion keeps it.

I switched the attitude state from Euler angles to a quaternion, which turns this into a multiplicative EKF (MEKF). Instead, the Kalman math (`P`, `K`) runs on a 3-dimensional small-angle error state, which then gets folded onto the quaternion multiplicatively and renormalized. It performed essentially identically on the RMS test (~0.445°/0.305°/0.230° vs. 0.443/0.305/0.229 for the Euler version), which is expected — same underlying model, just a numerically better-behaved state representation.

### 3. Trust the accelerometer less when the drone is moving, and throw out impossible readings (`ekf.py`, `test_gating.py`)

The accelerometer is how the filter knows which way is down. When the drone is sitting still, the only thing it feels is gravity, so it points straight at the ground and tells you exactly how tilted you are. But the moment the drone speeds up, slows down or turns, the accelerometer feels that too, and it can't tell the difference. To the accelerometer, "accelerating forward" looks the same as "tilted backward".

So the filter trusts the accelerometer less whenever it's clearly feeling more than just gravity. If the total reading is far from 1g, something other than gravity is going on (ie the drone is accelerating), and the filter trusts that measurement less.

Some readings are so far off they can't be real at all, like a glitch or a hard bump. Those get thrown out completely. The tricky part is deciding what counts as "too far off". A fixed limit ("reject anything more than X away from what I expected") doesn't work, because how surprising a reading is depends on how sure the filter is. Right after startup the filter barely knows anything, so big differences between what it expected and what it got are normal. After a long, clean flight, the same difference would be suspicious.

So instead of a fixed limit, the filter asks: given how uncertain I am right now, and how noisy this sensor is, how many standard deviations away is this reading? It's like grading on a curve. If the filter's model is right, that number follows a known distribution (the chi-squared distribution), so I can pick the cutoff by choosing how often I'm willing to throw out a good reading by mistake. I picked 1%, which for the accelerometer's 3 axes works out to a cutoff of 11.34. A nice side effect is that the gate tightens on its own as the filter gets more confident, and loosens when it's unsure.

<details>
<summary>The math</summary>

The *residual* is the actual reading minus the predicted one. It gets scaled by how big it's expected to be, `S = H P Hᵀ + R` (the filter's own uncertainty mapped into sensor space, plus sensor noise):

```
d² = residualᵀ · S⁻¹ · residual
```

This is the squared Mahalanobis distance (in 1D it's just (residual / σ)²). If the model is right, each scaled axis is a standard Gaussian, and a sum of k squared standard Gaussians follows a chi-squared distribution with k degrees of freedom. For k = 3, `d²` stays under 11.34 99% of the time.

</details>

#### Test: a fake 8.7g accelerometer spike, gate on vs. off

`test_gating.py` injects a physically impossible spike (about 8.7g, far more than a hovering drone could ever feel) for 10 steps, and compares roll with the gate on and off:

| | Roll before spike | Max roll during spike | Recovery |
|---|---|---|---|
| Without gating | 10.06° | 32.07° | ~30+ iterations |
| With gating | 10.06° | 10.28° | Immediate |

Without the gate, one bad burst threw roll off by 22° and it took 30+ steps to recover. With the gate, the filter barely noticed.

One gotcha I hit: the gate has to compare against the accelerometer's *normal* noise level, not the turned-down trust from earlier. Otherwise a huge spike makes the filter trust the accelerometer less, which makes the spike look less surprising, which lets it through. The spike excused itself every time.

### 4. Add GPS so the drone knows where it is (`gps.py`, `test_gps_tracking.py`)

Knowing which way it's pointing isn't enough for return-to-home. The drone needs to know where it is. In theory you can get that from the accelerometer alone: acceleration tells you how your speed changes, and speed tells you how your position changes. In practice, every tiny error in acceleration gets added up twice, so the position estimate drifts further and further from reality and never comes back. This is called dead reckoning.

GPS fixes that. It's noisy and slow (about 5 readings a second, vs. hundreds a second from the gyro and accelerometer), but it doesn't drift. So the filter does both. Between GPS readings it predicts position and velocity from the accelerometer, and every time a GPS reading arrives it pulls the estimate back toward it.

![GPS tracking trajectory](state%20estimation/testing/gps_tracking_trajectory.png)
![GPS tracking RMS comparison](state%20estimation/testing/gps_tracking_rms.png)

With GPS, position error dropped 8-10x in every scenario. I also reran each scenario with the drone tilted 10° and got the same numbers, which confirmed the filter correctly accounts for tilt when it turns the drone's own acceleration into movement across the ground.

### 5. Gate every sensor, not just the accelerometer (`gps.py`, `test_gates.py`)

The accelerometer isn't the only sensor that can lie. GPS signals can bounce off buildings before reaching the drone (multipath), making it look like the drone jumped tens of meters. The magnetometer, which works like a compass, gets thrown off by anything magnetic nearby, like motors (which are definitely nearby), power lines or steel. So every sensor gets the same kind of gate as the accelerometer. If a reading fails its gate, the filter skips it entirely for that step, rather than using part of it.

#### Test: a fake 50 m GPS jump and a 90°-wrong compass reading, gates on vs. off

`test_gates.py` checks two things. First, that the gates stay quiet on clean data, since a gate that throws out good readings is worse than no gate at all. Second, that they catch the bad ones. The drone sits still the whole time, so any movement in the estimate is pure error. Then it injects a GPS reading 50 m off and a magnetometer reading pointing 90° the wrong way:

| | GPS rejected | Mag rejected | Peak position error | Peak attitude error |
|---|---|---|---|---|
| Clean run (no outliers) | 0/0 | 0/0 | — | — |
| Gates ON | 1/1 | 1/1 | 0.00 m | 0.00° |
| Gates OFF | 0/1 | 0/1 | 9.82 m | 15.53° |

With gates on, both bad readings were caught and the estimate didn't move at all. Without them, the GPS jump alone pulled the position estimate ~9.4 m off, which took many GPS updates to undo. The compass error was worse in a sneakier way. It knocked the orientation estimate ~15.5° off, and since the filter uses orientation to interpret the accelerometer, the position kept drifting for several more steps, even though the GPS was fine.

Gating also exposed a second problem. Every step, the filter can get corrections from up to three sensors: GPS, accelerometer and magnetometer. Originally I computed all three against the same starting estimate, then applied them together. That's like three people each editing their own copy of the same draft, then pasting all the edits in at once. If one correction moves the estimate a lot, the other two were computed for an estimate that no longer exists. Now each correction is applied immediately, and the next sensor works from the updated estimate. I ran the filter's startup many times with random starting errors (a Monte Carlo test, `test_gate_monte_carlo.py`). Before the change, about 30% of startups got permanently stuck, with the accelerometer gate rejecting every reading forever. After it, 0 out of 20 did.

### 6. Estimate accelerometer and magnetometer bias too (`gps.py`, `calibration.py`, `test_accel_bias_convergence.py`)

The gyro isn't the only sensor with a built-in offset. The accelerometer and magnetometer have them too, and left alone they show up as fake tilt and fake heading error. So, like the gyro, each one gets a bias that the filter learns and subtracts.

But with the drone sitting still, an accelerometer bias looks exactly like the drone being slightly tilted. Both just mean "gravity reads a little off from straight down", and there's no way to tell which one it is. Same with the magnetometer: a bias looks just like the drone pointing in a slightly different direction.

So we have a calibration routine, which can separate these biases out. A sensor's bias is stuck to the sensor, so it stays the same no matter how the drone is turned. A real tilt doesn't: it changes as the drone rotates. Once the drone moves around, the two stop looking alike.

Under a rotating test trajectory, both biases settle onto their true values (the magnetometer bias to within ~0.0004 of the true value):

![Accelerometer bias convergence](state%20estimation/testing/accel_bias_convergence.png)

## Flying it in Gazebo: what only real physics revealed (`run_sim.py`, `PID/`)

The real test: can a controller actually fly a drone on this estimate? My own simulated sensor data only checks the filter against my own assumptions. A physics simulator checks it against something I didn't write. Some settings in the final filter are there because of what this showed.

Gazebo is a robotics simulator with real physics and simulated sensors. I flew PX4's x500 quadcopter model in it, but bypassed PX4's own estimator and controller entirely. `gz_bridge.py` reads Gazebo's sensor data and hands it to my EKF (`FINAL_gps.py`, the evolved `gps.py`). My PID controller then turns the estimate into motor speeds.

The controller is a chain of loops, each one feeding the next: where should I be → how fast should I move to get there → how much should I tilt to move at that speed → how fast should I rotate to reach that tilt → how hard should each motor spin.

### Tracking down the crash

For a long time, the drone flipped over within 4-6 seconds of switching on position hold. It looked like the controller was oscillating. To check, I added an **oracle mode**, where the controller flies on Gazebo's ground truth (the drone's true position and orientation) instead of the EKF. On ground truth, with exactly the same controller settings, it hovered perfectly level. So the controller was fine, and the EKF was feeding it the wrong state.

Turns out all the bugs were in my EKF's interface with Gazebo. The two most interesting:

- **In flight, the accelerometer feels thrust, not gravity.** On a hovering drone, thrust always points straight up through the drone's body, so using the accelerometer to find "down" pulled the estimate toward level even while the drone was tilted. Thrust is ≈ 1g in a hover, so the gate couldn't catch it. That correction is now off in flight, and GPS velocity reveals tilt instead, since a tilted drone drifts sideways.
- **The compass assumed Earth's magnetic field is horizontal.** Gazebo's field points steeply into the ground, like the real one at most latitudes. Calibration hid the mismatch as a fake bias, and every tilt leaked into fake heading corrections. My first "fix" copied a remap from PX4's code, which made the estimated heading turn the *opposite* way from the real one. The real fix was measuring the field's actual direction from flight data against ground truth. Lesson learned: check claims about axes against ground truth instead of trusting comments.

Replaying the crashed flight's sensor data through the EKF (see [Offline replay](#offline-replay-testingreplay_ekfpy) below), the final fixes took roll error from 16.9° to 0.14°. The full list of bugs, including swapped GPS axes and a GPS gate that locked out forever, is in `HANDOFF.md`.

### Controller fixes

Two controller fixes, both confirmed in oracle mode:

- **Altitude bobbing.** Altitude is controlled by two loops: an outer one that turns height error into a target climb speed, and an inner one that turns climb speed into throttle. The inner loop has to react faster than the outer one, but mine was slower, so the drone bobbed between 0 and 4 m every ~6 s. I recalculated the gains from how much lift each bit of extra motor speed actually produces (0.026 m/s² per rad/s).
- **Derivative kick.** The D term reacts to how fast the error is changing. When the target tilt jumped, the error jumped with it, so the D term gave the motors a sudden kick. Now it uses the gyro's measured rotation rate instead, which only changes when the drone actually moves.

**Result: 3/3 fresh 60 s flights on the EKF held true roll and pitch at 0.0°, altitude at 2.00 ± 0.01 m, and horizontal position within a few cm, with position hold on.**

## What's next: battery-aware return-to-home

Everything above answers "where is the drone, and which way is it pointing?" The next step is answering "how much flight does it have left?"

I plan to estimate the battery's state of charge the same way this EKF estimates position. From noisy measurements (voltage and current), with an honest measure of uncertainty. Battery voltage sags under load and recovers at rest, so the raw voltage alone isn't a reliable fuel gauge. Combined with the position estimate, the drone can then ask not just "how far is home?" but "how confident am I that I'll make it there?", and turn back early when that confidence drops.

The Gazebo simulation doesn't model battery sag yet, so that's part of the work too.

There's also one known limit in the estimator (from `test_robustness_scenarios.py`): when the compass is out, nothing else tells the filter which way is north, so heading drifts (~26° over 5 seconds of magnetic interference). GPS dropouts are handled much better: 30 seconds without GPS only grew position error from ~0.7 m to ~1.7 m, and it recovered cleanly once GPS came back.

## File structure

```
run_sim.py                  flies the full cascade in Gazebo; GAINS/LIMITS live here (single source of truth)
PID/
  cascade.py                position -> velocity -> attitude -> rate loops, plus the motor mixer
  pid.py                    PID class (anti-windup, filtered derivative-on-measurement)
  gz_bridge.py              Gazebo <-> Python: sensor topics in, motor speeds out, ground truth for logging
state estimation/
  FINAL_gps.py              the EKF in use: 19-state quaternion MEKF (attitude, gyro bias, velocity,
                            position, accel bias, mag bias, baro bias) - the evolved gps.py from the sections above
  calibration.py            pre-flight calibration filter that seeds FINAL_gps.py
  complementary.py, ekf_angle_based.py, ekf_gyro_bias.py, ekf.py, quaternions.py
                            the earlier filters from the progression above, kept for comparison
  testing/                  offline EKF tests + the graphs in this README (test_*.py, plot_*.py, *.png)
testing/                    Gazebo test harnesses + the replay tool
HANDOFF.md                  running engineering log: every bug, theory, and result, in detail
```

## Requirements

- **Offline tests and replay** (everything in `state estimation/` and `testing/replay_ekf.py`): Python 3 with `numpy` and `matplotlib`. No simulator needed.
- **Live flights in Gazebo**: Linux or WSL with [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) built for SITL and Gazebo Harmonic, which provides the `gz.transport13` and `gz.msgs10` Python bindings that `PID/gz_bridge.py` uses.
- **Optional**: `pyulog` (`testing/ulog_to_csv.py`) and `pymavlink` (`PID/px4_bridge.py`).

## Running it

`HANDOFF.md` is my running engineering log, with more detail on setup and every bug along the way.

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

## Offline replay (`testing/replay_ekf.py`)

`python testing/replay_ekf.py flight.csv` feeds a recorded flight back through `DroneEKF` exactly as if it were live, then reports the estimate's error against Gazebo's ground truth. No Gazebo needed, ~3 seconds per run, identical data every time, so two EKF versions can be compared fairly — that's how the estimator bugs above were found. Knobs: `T_END=12` scores only the first N seconds; `EXP="QATT=1e-6 R_ACC=5 ROLL0=5 ..."` overrides noise settings or injects a known attitude error without editing the EKF (full list in the file). Add `MAG_REMAPPED=0` for recordings made after 2026-09-30.
