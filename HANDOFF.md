# Handoff: PID cascade tuning against Gazebo/x500

## 2026-09-27 session: found and fixed the actual root cause of the run-to-run variance

**The control loop (`run_sim.py` and every `test_*_loop.py` harness) was an
unsynchronized busy loop, running at ~2000-3000Hz against sensor data that only
actually updates at Gazebo's real IMU rate (~260Hz).** `GazeboBridge`'s `_gyro`/
`_accel` fields are just "last value received" caches, mutated asynchronously by
gz-transport's own background thread; the main loop never waited for a new message,
it just spun and read whatever was cached `dt = now - last_t` measured pure wall-clock
loop time, completely decoupled from real sensor arrival. Consequences, confirmed via
the actual dt log data (`run_sim_full.log`, sampled every 30th tick, showed
0.27-7.57ms with a 0.46ms median = up to ~2000Hz): most iterations reprocessed the
exact same stale sensor reading, and the moment a genuinely new reading landed via the
background thread was a race against the main loop's own OS-scheduled timing -
non-deterministic by construction under WSL2 (already a noisier scheduler than bare
metal, further loaded by WSLg's Gazebo GUI rendering). This is a clean, direct
explanation for the run-to-run noise documented throughout this file (identical gains,
identical code, wildly different divergence times - e.g. the att_rp kd=1.5 trial that
diverged at t=5.3s once and t=2.9s on an immediate re-run) and for the intermittent
single-tick sensor "corruption" events: a real sensor value jump divided by whatever
near-random dt the busy loop happened to have at that instant produces an
effectively-random-magnitude derivative kick straight into the rate loop (the
innermost loop, with no sub-rate averaging to smooth it out - see
`attitude_loop`'s own comment about needing to be held between sub-rate recomputes,
which was a patch around this same symptom one level up).

**Fix** (`gz_bridge.py`, `FINAL_gps.py`, `run_sim.py`, `test_common.py`, all three
`test_*_loop.py` harnesses):
- `GazeboBridge` now has a `threading.Event` set by `_on_imu()` and a
  `wait_for_imu(timeout=0.1)` method - every control loop now blocks on this before
  calling `ekf.step()`, so the loop's rate (and therefore every PID's dt) is paced to
  real sensor arrival instead of busy-spinning. `timeout` is just a safety net against
  a stalled topic, not the normal path.
- `FINAL_gps.py` and `run_sim.py` both now clamp `dt` to `MAX_DT=0.05s` as a
  defensive backstop against a genuine stall (WSL2/WSLg hiccup, GC pause) dumping an
  unbounded integration/integral step in one tick - should essentially never bind now
  that the loop is properly paced, but cheap insurance.

**Verified via actual Gazebo trials, 2026-09-27 (see "What's confirmed fixed" #15 and
below for the full detail):**
- Loop dt is now consistently ~3.5-4.7ms (~250-260Hz), not the old 0.3-0.8ms chaos -
  confirmed directly from `rate_test_verify.log`.
- Rate-loop isolation test (`run_rate_test.sh roll 0.3 5.0`) tracked cleanly, no
  saturation/oscillation, max accel deviation 2.78 m/s² (genuine physics, nowhere near
  the 400+ m/s² corruption-era garbage values).
- **`run_sim.py` (full cascade) survived a full 20-second test with `pos_xy=0` -
  roll/pitch stayed under ~6 degrees almost the whole time, no tumble, no saturation
  lockup.** This is the first time the full cascade has ever stayed controlled anywhere
  near this long (previous best was t=5.3s before an unrecoverable yaw spike). The
  fix's value isn't just "less noise" - it appears to have actually removed a real
  destabilizing mechanism (the derivative-kick-on-stale-data effect above), not just
  made results more consistent.
- **Remaining problem, now cleanly characterized instead of noisy: with `pos_xy=0`
  nothing corrects horizontal position error at all (only velocity is held near zero),
  so the vehicle drifts away unboundedly (0->35m over 19s, accelerating in the back
  half).** Re-enabling `pos_xy` (tried kp=1.0/kd=0.5, then a much gentler kp=0.15/kd=0)
  reproducibly causes a violent tumble around t=4.3-5.1s regardless of gain magnitude -
  see the comment on `pos_xy` in `run_sim.py`'s `GAINS` and "What's NOT resolved" below
  for the full diagnostic detail. This is now understood to be a real resonance in the
  velocity->attitude->rate cascade (confirmed via the EKF_ACCEL diagnostic showing a
  smoothly growing/oscillating accel deviation before each tumble, not corrupted-sensor
  garbage), not a bad pos_xy gain choice - the cascade doesn't yet have enough damping
  margin to absorb ANY outer-loop position correction. `pos_xy` is back to 0 for now.

## Goal

Custom Python EKF (`state estimation/FINAL_gps.py`) + custom Python PID cascade
(`PID/cascade.py`, `PID/pid.py`) flying a simulated PX4 x500 quadrotor directly in
Gazebo (Harmonic, via `gz.transport13`), bypassing PX4's own estimator/controller
entirely. This is deliberate: the user is building their own EKF+PID stack from
scratch as a learning project, and wants to validate it against real physics before
ever touching real hardware or PX4/Betaflight integration (a separate, later step).

**Current status (updated late in the 2026-09-26 session): rate, attitude, and now
velocity loops have each been isolated, tested, and had real bugs found and fixed** -
see "What's confirmed fixed" below. This was a long chain: a ground-contact bug meant
early rate-loop tests were never airborne; a yaw-authority gap meant yaw was never
really tested; the attitude loop uncovered a genuine intermittent EKF corruption bug
(traced all the way to garbage IMU sensor readings from Gazebo, now filtered at the
bridge); the velocity loop then uncovered a *second*, different EKF bug (accel gating
using raw instead of bias-corrected deviation, causing a real sustained acceleration to
get silently absorbed into `accel_bias` and freezing `state['vel']`), a missing
feedforward on `vel_z` (real fall risk on cold start), and a genuine sign bug in
`velocity_loop()`'s `a_right` term (was driving `roll_sp` in the direction that made
horizontal drift *worse*, not better). With all of that fixed, a `vz` step test now
shows `vx`/`vy` staying bounded and settling instead of diverging - the first time any
loop above rate has stayed stable for a full multi-second test.

**Full-cascade flight WAS re-tested extensively (2026-09-26, after all the above
fixes) - real, measurable progress, still not flying.** `run_sim.py` climbs cleanly to
~1.8-2m every time now. The remaining problem is a divergence that happens later and
later as more real bugs get fixed, but hasn't been eliminated yet:
- Original post-velocity-fixes state: diverged ~t=1.85-1.90s.
- After finding and fixing a genuinely dead D-term in `attitude_loop()` (see fix #11) -
  still diverged, similar timing.
- After filtering that D-term + adding a gyro sensor sanity clamp (fixes #12/#13) -
  still diverged, similar timing, but roll/pitch stayed under ~5-9 degrees noticeably
  longer in some trials.
- After raising `att_rp`/`att_yaw`'s `kd` (fix #14) - **best result so far**: roll/pitch
  stayed under ~9 degrees through t=5.3s (nearly 3x longer than the original), before a
  sudden, violent yaw rate spike (14+ rad/s) that the vehicle never recovered from.
- Tried adding `kd` to `rate_yaw` (the one remaining completely undamped loop) - made
  things clearly *worse* (diverged by t=2.0s, permanent lockup). Reverted.
- **Re-ran the exact same gains (`att_rp` kd=1.5, `att_yaw` kd=0.3, `rate_yaw` kd=0.0)
  as a confirmation - diverged MUCH earlier this time (full tumble by t~2.9s, vs
  t=5.3s for the first trial with identical code).** This is clean, direct proof that
  the run-to-run noise already documented elsewhere in this file (see the attitude-loop
  kp=3-vs-6 comparison) is real, large, and affects the full cascade just as much as
  the isolated-loop tests - single full-cascade trials are NOT reliable evidence for
  judging a gain change here. The current gains are a reasonable, well-reasoned
  starting point (each individual change was independently justified), not a confirmed
  "best" configuration - don't treat the t=5.3s trial as proof they're better than
  what came before, and don't be discouraged by the t=2.9s trial either.

This is no longer "root cause unknown" - it's now understood to be a genuine tuning
gap (insufficient damping somewhere in the outer loops, most likely still yaw-related
given the failure signature), being closed incrementally, not a single bug. See
"What's confirmed fixed" #11-14 and "What's NOT resolved" for the full detail and
"Recommended next steps" for where to pick this up.

## How the user likes to work

Default to explaining *what* to change and *why* rather than silently editing files,
UNLESS they've given a clear go-ahead ("yes", "go ahead", "let's do X") - then just
implement directly and report back. They're hands-on and want to understand the
reasoning, not just receive a working result. Be honest and explicit when a theory
turns out wrong on closer inspection (this happened at least twice below) - they
value the correction more than a smooth narrative.

## Architecture map

```
state estimation/
  FINAL_gps.py    - DroneEKF class, 18-state MEKF (attitude, gyro_bias, velocity,
                     position, accel_bias, mag_bias). __init__(sensors=None) - pass
                     a sensors object (get_gyro/get_accel/get_mag/get_gps methods) to
                     drive it from something other than its own built-in simulated
                     ground truth, e.g. GazeboBridge.
  calibration.py  - pre-flight calibration (separate 12-state EKF), unchanged from
                     before this Gazebo integration work.
PID/
  pid.py          - PID class (kp/ki/kd, integral_limits anti-windup, d_filter_alpha
                     derivative filtering).
  cascade.py      - Cascade class: position_loop -> velocity_loop -> attitude_loop ->
                     rate_loop -> mix(). Cascade.step() runs the full chain at
                     sub-rates (position 30Hz/velocity 75Hz/attitude 200Hz by default,
                     rate loop runs every call). rate_loop() and the others are also
                     callable individually - that's what test_rate_loop.py uses.
  gz_bridge.py    - GazeboBridge: subscribes to the x500's real IMU/magnetometer/GPS
                     Gazebo topics, exposes get_gyro/get_accel/get_mag/get_gps in the
                     same shapes FINAL_gps.py's own simulated versions use, and
                     publish_motors(m1,m2,m3,m4) publishes real motor commands.
run_sim.py        - Full-cascade runner (repo root). GAINS/LIMITS live here - this is
                     the single source of truth, other scripts import from it.
test_rate_loop.py - NEW: isolated rate-loop-only test harness (see below).
run_rate_test.sh  - NEW: launcher for test_rate_loop.py (repo root).
```

`run_drone_sim.sh` (launcher for `run_sim.py`) lives in the WSL home directory
(`~/run_drone_sim.sh`), not the repo - inconsistent with `run_rate_test.sh`'s
location, just a historical accident, not meaningful.

## Environment

WSL2, distro `Ubuntu-24.04`, mirrored networking enabled (`%UserProfile%\.wslconfig`
has `networkingMode=mirrored` - needed so QGroundControl-on-Windows could reach
PX4-in-WSL over `localhost`; not actually needed for the current direct-Gazebo
approach, but don't remove it, nothing depends on removing it either).
`PX4-Autopilot` is cloned+built at `~/PX4-Autopilot` in WSL (`--no-nuttx`, SITL-only,
no real-hardware cross-compiler installed). Gazebo Harmonic (`gz-harmonic`) installed
alongside it. QGroundControl is installed on Windows but **not used** by the current
approach - it was for an earlier abandoned path (see "Roads not taken" below).

The repo is at `C:\Users\Sasha\repos\python_drone`, reachable from WSL at
`/mnt/c/Users/Sasha/repos/python_drone`.

## The standard test procedure

This took many iterations to get reliable - follow it exactly.

1. **Launch PX4+Gazebo together** (in its own terminal, stays running/foreground):
   ```
   wsl -d Ubuntu-24.04 -- bash -c "cd ~/PX4-Autopilot && make px4_sitl gz_x500"
   ```
2. **If the Gazebo GUI window doesn't render** (shows blank, or doesn't appear at
   all, despite the process running - check with `ps aux | grep -i 'gz sim'` in a
   second terminal) - this is a recurring WSLg display glitch, not a real failure.
   Fix: `wsl --shutdown` (from PowerShell), wait a few seconds, then redo step 1.
3. **Kill only PX4's process**, not Gazebo, to free the motor-command topic for our
   own script:
   ```
   ps aux | grep -iE 'bin/px4|gz sim'    # find PIDs
   kill -9 <bin/px4 pid> <its two parent wrapper pids>
   ```
4. **Verify the motor topic has real subscribers**:
   ```
   gz topic -i -t /x500_0/command/motor_speed
   ```
   Should show 4 subscriber entries (the real motor plugins). If it shows 0, or if
   you see a stray publisher already there, check for a leftover `run_sim.py` /
   `test_rate_loop.py` process (`ps aux | grep -i run_sim`) and kill it first.
5. **Run the test** in a *separate* terminal from the one running `make` (that
   terminal's shell is occupied by the foreground process tree and won't take input):
   ```
   wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Sasha/repos/python_drone/run_rate_test.sh roll 1.0 5.0
   ```
6. **If the vehicle diverges badly** (flies far away, flips repeatedly), do a full
   fresh restart (kill both `gz sim` processes too, not just PX4) before the next
   trial - don't reuse a session where the vehicle is thousands of meters away.

## Known gotchas (each cost real time - don't rediscover these)

- **The real motor topic is `/x500_0/command/motor_speed`** (no `/model/` prefix).
  `/model/x500_0/command/motor_speed` also exists as a topic but has **zero real
  subscribers** - publishing there silently does nothing. This alone cost a long
  debugging detour.
- **Gazebo's native body frame is FLU** (X-forward, **Y-LEFT-positive**, Z-up), not
  the FRD (Y-right) convention common in aerospace/PX4 material. Reading the x500
  SDF's rotor positions assuming FRD gives every motor's left/right label backwards.
  This caused a real bug (see below) - if you're deriving anything from SDF
  coordinates or body-frame physics, double check which convention you're assuming.
- **`make px4_sitl gz_x500` reuses an already-running Gazebo session** if one exists,
  rather than always spawning fresh. If you need a genuinely fresh/upright spawn (e.g.
  after a crash), kill **both** `bin/px4` and both `gz sim` processes before
  relaunching, not just PX4.
- **`/tmp/*.log` is wiped on every `wsl --shutdown`** - lost a full run's log this way
  mid-session. Logs now go to the Windows-mounted repo path (`run_sim.log`,
  `rate_test.log`, both git-ignorable) instead of `/tmp`, so they survive.
- PX4's `DONT_RUN=1 make px4_sitl gz_x500` is supposed to build without launching -
  empirically it still launched Gazebo anyway. Didn't investigate why; just don't
  rely on it.
- `pkill -f <pattern>` occasionally appears to kill more than intended (background
  Bash calls returning exit code 9/15 without the expected output) - if a `pkill`
  call's output looks wrong, just re-check with `ps aux` and kill by explicit PID
  instead of trusting the pattern match.
- **The `gz sim` server process dies on its own fairly often**, independent of whether
  the test that just ran was clean or divergent - confirmed at least once after a
  visibly well-behaved yaw test with no oscillation or saturation. Best explanation:
  `test_rate_loop.py`/`run_sim.py` just stop publishing motor commands and exit the
  moment their loop ends, even if the vehicle is still climbing (it often is - see the
  liftoff/thrust-margin note below), so it free-falls from altitude with no one
  commanding thrust and eventually hits the ground hard enough to crash the physics
  server. Not itself a sign of a bad test result - just means "check `ps aux` before
  assuming the next test can reuse this session" is now a near-every-run step, not an
  occasional one.

## What's confirmed fixed (real bugs, not just tuning)

1. **`cascade.mix()` had all three rotation signs backwards** relative to the x500's
   actual rotor layout. Root cause: assumed FRD when reading the SDF (see gotcha
   above). Re-derived properly using actual rotation matrices against Gazebo's real
   FLU frame + Newton's-third-law reaction-torque physics for yaw. The corrected
   derivation is documented in `mix()`'s own comments - trust those over redoing the
   derivation from scratch, they've been checked multiple times.
2. **`velocity_loop`'s `roll_sp` sign was backwards** - caused 50+ meter uncontrolled
   horizontal drift on an early test. Fixed, verified against the same FLU rotation
   math as above.
3. Calibration now uses the live bridge's real sensors (dwell-only, no wiggle) when
   `DroneEKF(sensors=...)` is passed something, instead of always using the synthetic
   `_cal_get_*` stand-ins. **Caveat: my original justification for this fix was
   WRONG** - I initially claimed the synthetic calibration path converges to a
   "phantom" ~2°/s gyro bias matching `TRUE_GYRO_BIAS`. Checked empirically: it
   doesn't, it converges to ~0. The fix is still more *correct* in principle (why
   calibrate against a disconnected synthetic sensor when a real one is available?),
   but it's probably not fixing anything significant. Don't re-chase this as a lead.
4. Added `integral_limits` (anti-windup) and `d_filter_alpha` (derivative-on-raw-noisy-
   measurement filtering) to several loops in `run_sim.py`'s `GAINS` where they were
   simply never set. `pid.py`'s `PID` class already supported both; they just weren't
   being passed.
5. **`test_rate_loop.py` never actually got the vehicle airborne (2026-09-26).** Root
   cause: `HOVER_THRUST` (757 rad/s) is the exact theoretical steady-state hover value
   (thrust == weight), giving zero net vertical force - fine once flying, but there was
   nothing to actually break contact with the ground, and calibration itself runs ~2s
   before any motor command is published at all, letting the vehicle settle onto the
   pad first. Confirmed directly via Gazebo's own `/world/default/pose/info` ground
   truth: without a takeoff phase, `x500_0` sat at `z≈-0.013` (on the pad, level
   orientation) for the entire "flight". This fully explained two things that looked
   like separate bugs: pitch showing **zero** measured rate response no matter how much
   `tau_pitch` wound up (the differential was being absorbed by ground contact), and an
   earlier roll test's catastrophic single-tick jump to `(roll=-8.4, pitch=+10.3,
   yaw=+4.7)` that saturated everything and killed the `gz sim` server outright (almost
   certainly a real ground-contact/tip-over impulse, not a control or EKF bug - motor
   torque alone is orders of magnitude too weak to produce that big a change in one
   0.2ms tick). **Fix**: `test_rate_loop.py` now has a `wait_for_liftoff()` phase that
   climbs on `TAKEOFF_THRUST` (1.15x hover) until the EKF's altitude estimate clears
   `LIFTOFF_ALT=0.5m` (aborting after `LIFTOFF_TIMEOUT=5.0s` instead of silently testing
   on the ground), *then* resets the clock and switches to the original constant
   `HOVER_THRUST` for the actual measurement window. Verified fixed: pitch now tracks
   cleanly to ~0.48 rad/s on a 0.5 rad/s step; roll went from a 264% overshoot (ground
   contact) to no overshoot at all once airborne.
6. **Yaw had ~10x less physical authority than roll/pitch for the same commanded omega
   differential (2026-09-26), so `rate_yaw`'s old `kp=15` (same as `rate_rp`) never gave
   it enough torque to move.** Roll/pitch authority comes from thrust-differential ×
   arm length (`motorConstant` × 0.174m); yaw authority comes from reaction torque
   (`momentConstant=0.016` × thrust) which is a fundamentally weaker effect - worked
   through the actual x500 constants at hover, the same `tau` value produces roughly
   10x less angular acceleration in yaw than in roll. Confirmed empirically: a `yaw 0.3
   5.0` trial (post-liftoff-fix, so genuinely airborne) held `tau_yaw` pinned near its
   pure-P value for the *entire* step with essentially zero measured rate response
   (stayed ~0.000-0.002 the whole time) - not slow, no response at all. **Fix**: raised
   `rate_yaw`'s `kp` from 15 to 150. Retested clean: rises to ~0.27 on a 0.3 rad/s step,
   no overshoot, settles back to ~0 in a clean exponential decay after the step ends -
   arguably the best-behaved axis now, a big change from being "the worst axis" in
   every previous full-cascade run.
7. **The intermittent single-tick EKF attitude corruption (the "probable EKF robustness
   gap" flagged earlier this session) was actually corrupted sensor data, not an EKF
   math bug.** Root-caused via `FINAL_gps.py`'s `EKF_ACCEL` diagnostic (still in place,
   `TEMP DIAGNOSTIC` comment) correlated against `test_velocity_loop.py` output: right
   before every corruption event, raw accel deviation spiked to 400-700+ m/s² (43-73g,
   physically impossible), correctly REJECTED by the chi-squared gate - but sandwiched
   between two such spikes was one `FIRED` correction with `d2` just under threshold and
   a large `dtheta` (~0.53 rad). `gz-transport` was occasionally delivering genuinely
   corrupted IMU messages. **Fix**: `gz_bridge.py`'s `_on_imu()` now rejects any accel
   reading over 6g and holds the last good value instead of handing it to the EKF at
   all. Confirmed fixed: max deviation across a full test run dropped from 713 m/s² to
   2.76 m/s² (physical).
8. **Separately, `state['vel']` could freeze bit-for-bit for 0.5+ seconds during real,
   sustained acceleration** (found immediately after fix #7, via `test_velocity_loop.py
   vz 0.5 3.0` - confirmed genuine, not a display bug: altitude kept integrating
   linearly from the frozen velocity value the whole time). Root cause: `FINAL_gps.py`'s
   accel deviation/gate (`accel_magnitude`/`deviation`/`R_accel`) was computed from RAW
   accel, but the actual prediction step and residual both use BIAS-CORRECTED accel -
   different signals. A real, sustained, growing acceleration (e.g. this test's own
   climbing thrust) got accepted through the gate as a series of individually-small
   corrections (each fine per raw deviation), each one nudging `accel_bias` a little,
   until the bias had absorbed enough of the real signal that the bias-corrected accel
   fed to prediction stayed anomalously close to gravity-only - so velocity stopped
   responding to real thrust changes. **Fix**: moved the `accel_magnitude`/`deviation`/
   `R_accel` computation to after `corrected_accel_x/y/z`, using the bias-corrected
   values (see the comment in `FINAL_gps.py`'s `step()`). Confirmed fixed: velocity now
   updates continuously through sustained acceleration, no more freezing.
9. **`vel_z` had no feedforward term - its PID had to build the entire ~757 hover
   thrust from its own integral alone.** With `ki=20`, reaching hover requires the
   integral to reach ~38 (38 (m/s)·s of accumulated velocity error) - confirmed via
   `test_velocity_loop.py`: holding vel_sp=0 from a genuinely airborne state (not
   resting on the ground, which hides this - see the ground-contact bug above), thrust
   only reached ~153 after 2s and the vehicle fell over 3.5m. **Fix**: added
   `VEL_Z_HOVER_THRUST_FF=757.0` in `cascade.py`, added to `vel_pid_z`'s output before
   returning as `thrust`. `vel_z`'s `output_limits`/`integral_limits` (in `run_sim.py`'s
   `GAINS`/`LIMITS`) changed from `(0,1000)` to a trim range `(-400,240)` accordingly.
   Confirmed fixed: warmup thrust now reaches ~733-757 and holds altitude stable
   immediately, no fall.
10. **`velocity_loop()`'s `a_right` term had a sign bug, separate from the roll_sp sign
    bug already fixed earlier (item #2)** - `a_right = -ax*sin(yaw) + ay*cos(yaw)`
    reduces to `+ay` at yaw≈0, but body-right = world -Y in FLU (Y=left), so `a_right`
    (desired RIGHTWARD accel) should be `-ay`, not `+ay`. Confirmed empirically via
    `test_velocity_loop.py` with `roll_sp`/`pitch_sp` logged: with `vy` very negative
    (drifting right) and `vel_pid_y` correctly outputting positive `ay` to correct it,
    `roll_sp` saturated at its positive `max_tilt_rad` ceiling and STAYED there while
    `vy` kept getting more negative all test - the controller was trying harder in
    exactly the direction that made it worse (per `mix()`'s own already-verified "+roll
    = rightward accel" convention). **Fix**: `a_right = ax*sin(yaw) - ay*cos(yaw)` (full
    sign flip). After the fix alone, horizontal drift got worse but qualitatively
    different - `roll_sp`/`pitch_sp` oscillating between +/- max instead of pinned
    one-sided, the signature of a now-correctly-signed but underdamped negative-feedback
    loop, not a wrong-signed one. Also re-tuned `vel_xy`: `kp` 2.0->0.8, `kd` 0.3->0.6.
    Confirmed fixed together: `vx`/`vy` now stay bounded and settle over a full 3s test
    instead of diverging - first time any loop above rate has stayed stable that long.
11. **`attitude_loop()`'s derivative term was completely dead, structurally, since the
    loop was written - not just untuned, literally incapable of contributing anything
    no matter what `kd` was set to.** It called `PID.update(setpoint=err_roll,
    measurement=0.0, dt)` - `measurement` hardcoded to the literal constant `0.0`.
    `pid.py`'s derivative-on-measurement computes `raw_derivative = -(measurement -
    last_measurement)/dt`; with `measurement` always `0.0`, `last_measurement` becomes
    `0.0` after the first call and stays there forever, so `raw_derivative` is exactly
    `0.0` on every call, always. Confirmed by direct code inspection (not
    speculation), and confirmed as the ONE anomalous loop - `rate_loop`/
    `velocity_loop`/`position_loop` all pass their real, continuously-changing
    `state[...]` value as `measurement` and their D-terms work correctly. **Fix**:
    `self.att_pid_x.update(0.0, -err_roll, dt)` (same for pitch/yaw) - `error = 0 -
    (-err_roll) = err_roll`, identical P/I math, but `measurement=-err_roll` now
    genuinely evolves, so `kd` finally damps how fast the attitude error is changing.
12. **Immediately after fix #11, `att_rp`/`att_yaw` needed the same D-term noise filter
    `rate_rp`/`rate_yaw` already have and they didn't.** First post-fix `run_sim.py`
    trial hit `tau=(100,100,100)` (full 3-axis saturation) and `rate_sp` pinned at its
    3.0 rad/s cap on two axes, from a modest ~4-6 degree tilt state - a derivative
    kick from differentiating a real but EKF-noisy error signal with zero filtering,
    not a real disturbance response. **Fix**: added `d_filter_alpha: 0.2` to both
    (matches `rate_rp`/`rate_yaw`'s existing value and its own stated rationale).
13. **`gz_bridge.py`'s gyro channel was the one sensor field with no corruption
    guard**, even though the accel channel on the exact same IMU message already had
    one (fix #7). `FINAL_gps.py`'s `state['rate']` is a straight, ungated passthrough
    of the raw gyro reading (unlike accel, which goes through the chi-squared gate),
    so a single corrupted gyro reading hits the rate loop completely unprotected.
    Confirmed via a full `run_sim.py` log: `rate_meas` jumped from ~0 to
    `(-9.84,-22.36,-3.29)` rad/s in one tick, immediately saturating all 3 torque
    channels, right at a divergence onset. **Fix**: same pattern as the accel guard -
    reject any gyro reading with magnitude over 50 rad/s (far beyond anything this
    vehicle's real torque authority produces in one ~200Hz tick, far below what's
    needed to reject genuinely fast real rotation) and hold the last good value.
    **Caveat discovered later**: once a vehicle is ACTUALLY tumbling with sustained
    real rates over 50 rad/s, this same guard will permanently reject all further
    readings and freeze `state['rate']` at a stale value forever, which looks
    identical to a dead sensor from the logs - seen in the rate_yaw kd=5.0 trial
    below. Not a bug in the guard itself (50 rad/s is still a reasonable real-vs-
    corrupted threshold), just a reminder that a frozen `rate_meas` late in an
    already-diverging run means "it's spinning too fast to read," not "gyro broke."
14. **`att_rp`/`att_yaw`'s `kd` values (0.5/0.05) had literally never been tested at a
    working, non-zero effective value before fix #11** - they were numbers sitting in
    `GAINS` that happened to do nothing. First real tuning pass, 2026-09-26:
    `att_rp` kd 0.5->1.5 improved things substantially (roll/pitch stayed under ~9
    degrees through t=5.3s, vs ~2s before). `att_yaw` kd 0.05->0.3 (matching `att_rp`'s
    new kp:kd ratio, since `att_yaw`'s was proportionally much weaker) - result
    unclear on its own since it was tested together with the `att_rp` change. Trying
    to also damp `rate_yaw` (kd 0.0->5.0, the last remaining undamped loop) made
    things clearly worse (see #13's caveat) and was reverted. **Re-ran the exact same
    gains as a confirmation check - diverged much earlier (t~2.9s vs t=5.3s) with
    literally identical code.** These gains are a reasonable, well-reasoned starting
    point (each change individually justified against real symptoms), not a confirmed
    "best" configuration - the run-to-run noise is large enough that a single trial,
    good or bad, isn't strong evidence either way. Kept as-is since there's no better
    alternative yet, not because they're proven.
15. **The run-to-run noise referenced throughout items #7-14 above (and used as the
    reason several comparisons, like the att_rp kp=3-vs-6 test, were declared
    inconclusive) had a real root cause: an unsynchronized busy-loop control loop
    racing against asynchronous sensor delivery - see the 2026-09-27 session section at
    the top of this file for the full diagnosis and fix (`GazeboBridge.wait_for_imu()`
    + `MAX_DT` clamps in `gz_bridge.py`/`FINAL_gps.py`/`run_sim.py` and all three
    `test_*_loop.py` harnesses).** Confirmed via actual dt measurements (loop rate
    dropped from ~2000-3000Hz chaotic busy-spin to a consistent ~250-260Hz matching
    Gazebo's real IMU rate) and via a genuinely new result: `run_sim.py`'s full cascade
    survived a full 20s test with `pos_xy=0` (previous best was t=5.3s before an
    unrecoverable yaw spike). This means every "inconclusive due to noise" comparison
    earlier in this file (att_rp kp=3-vs-6, the ki addition to att_rp/att_yaw, the
    kp=1.5-vs-6.0-with-identical-gains divergence-timing discrepancy) is worth
    re-running now that trials should actually be comparable to each other - don't
    assume any of those conclusions still hold, but don't assume they're wrong either,
    they were just never measurable before.

## What's NOT resolved

- **CURRENT blocker (updated 2026-09-27, supersedes the paragraph below): with
  `pos_xy=0`, the full cascade (velocity->attitude->rate) now flies genuinely stable -
  survived a full 20s test, roll/pitch under ~6 degrees almost throughout, no tumble.
  The blocker now is specifically that ANY nonzero `pos_xy` gain (tried kp=1.0/kd=0.5
  AND a much gentler kp=0.15/kd=0.0) reproducibly excites a real resonance and tumbles
  the vehicle around t=4.3-5.1s.** See the 2026-09-27 session section at the top of
  this file and the comment on `pos_xy` in `run_sim.py`'s `GAINS` for full detail. Key
  evidence: the `EKF_ACCEL` diagnostic shows a smoothly growing/oscillating accel
  deviation (0.4->1.0+ m/s², cyclic over about a second) building right up to each
  tumble - a real physical oscillation, not corrupted sensor data (compare to the
  already-fixed corruption bug's signature: isolated 400+ m/s² single-tick spikes, not
  a smooth build-up). Since this reproduces at TWO very different `pos_xy` magnitudes,
  the problem is most likely insufficient damping margin somewhere in the inner
  velocity/attitude/rate cascade (it can currently only just barely stay stable with
  NO outer-loop disturbance at all), not the specific `pos_xy` gain chosen. Concretely
  unexplored: whether the resonance is roll/pitch-dominated, yaw-dominated, or both
  (the one tumble inspected closely had large rate spikes on both roll AND yaw -
  20-30 rad/s each) - worth a closer look at which axis leads next time before
  guessing at more gain changes.
  - **Not EKF/sensor corruption** - confirmed twice now, both for the original
    t~1.85s divergence (`EKF_ACCEL` deviation stayed small, <0.04 m/s², through that
    window) and for the new pos_xy-triggered tumbles (deviation grows smoothly, not a
    single-tick spike - see above).
  - **`pos_xy` causing earlier divergence is real, not a timing-noise artifact** -
    re-tested 2026-09-27 after fixing the actual root cause of the run-to-run noise
    (see top of file); the result reproduced at a very different gain value, so this
    is no longer just "one noisy trial." Don't re-attempt `pos_xy` at any gain without
    first addressing whatever margin gap lets it destabilize the cascade.
  - **Not fixed by damping `rate_yaw`** - tried kd 0.0->5.0 on the one remaining
    undamped loop: made things clearly worse (diverged by t=2.0s, permanent lockup
    with `rate_meas` frozen at a stale value - see #13's caveat about the gyro guard
    behaving that way once a tumble becomes genuinely fast). Reverted to kd=0.0.
  - **Still open**: yaw is the axis that keeps failing last and worst across trials -
    worth focusing there specifically rather than continuing to adjust roll/pitch.
    Also still unchecked: `pos_z`'s behavior right as altitude crosses/approaches the
    2m setpoint (kp=1.5, ki=0, kd=0.3, never revisited) - the 20s pos_xy=0 test showed
    real altitude oscillation (dipped to 0.05m off the ground once, recovered) worth
    investigating on its own now that trials are actually trustworthy run-to-run.
  - Original paragraph, now historical (predates the 2026-09-27 timing fix, kept for
    context): full-cascade flight diverged, failure point moved from t~1.85s to
    t~5.3s across items #11-14 (attitude D-term dead-code fix, filtering, gyro guard,
    damping re-tuning). The best trial then (`att_rp` kd=1.5, `att_yaw` kd=0.3,
    `rate_yaw` kd=0.0) kept roll/pitch under ~9 degrees through t=5.3s before a
    sudden violent yaw rate spike. Those are still the current gains and still a
    reasonable starting point, but "t=5.3s was the best ever achieved" is now stale -
    pos_xy=0 alone gets a clean 20s with the same gains.
- **`torque_range_rp`/`torque_range_yaw` loosening is no longer the obvious next
  experiment it was.** Yaw's fix was a `kp` increase (15->150) that works fine within
  the existing `±100` range without needing to loosen it - `tau_yaw` reached ~45 on a
  0.3 rad/s step, nowhere near saturating. Revisit only if a future test shows genuine
  saturation-limited tracking, not preemptively.
- **The most recent pre-fix change (adding `ki` to `att_rp`/`att_yaw`) is still
  unconfirmed** - the run that tested it diverged *faster* (35s vs 71s) than the
  previous run, but the early-flight log data needed to tell whether roll/pitch drift
  specifically improved was lost (see `/tmp` gotcha above) before it could be checked.
  Don't assume this change helped or hurt without re-testing with logging intact -
  this is now naturally the next thing to check once the attitude-loop harness exists.
- **Isolated rate-loop tests show a real velocity-driven drift on roll/pitch that is
  NOT a rate-loop bug** - a sustained ~0.3 rad/s roll/pitch command tilts the vehicle
  enough (via nothing holding attitude level) that it picks up real horizontal velocity
  under gravity (confirmed via logged `vel(x,y)`: grows smoothly to -5.5 m/s over the
  back half of a 5s test, tracking `g*sin(tilt)` closely), and the x500 SDF's
  `rotorDragCoefficient`/`rollingMomentCoefficient` produce a real disturbance torque
  that scales with that velocity - this is what causes roll/pitch to drift away from
  setpoint (and eventually 0) well after the step ends, growing roughly with velocity,
  not oscillating. This is expected to disappear once the velocity loop is active
  (`max_vel_xy=5.0` in `LIMITS` would cap it long before it matters) - don't chase this
  with more rate-loop `kp`/`ki`/`kd`, it isn't fixable at that level by design.
- **Root cause of the OLD full-cascade instability was never isolated to one loop**
  before the fixes above - worth re-establishing whether it's still true now that the
  rate loop is actually validated.
- **RESOLVED, moved to "What's confirmed fixed" (items #7/#8)**: the intermittent
  EKF attitude corruption first found via `test_attitude_loop.py` was root-caused to
  two separate real bugs (corrupted IMU sensor data + raw-vs-bias-corrected accel
  gating mismatch), both fixed. The `EKF_ACCEL` diagnostic in `FINAL_gps.py` is still
  in place (harmless, verbose) - fine to remove now that both root causes are fixed and
  confirmed, but not urgent.
- `FINAL_gps.py`'s covariance matrix `P` has no growth cap. Confirmed once that a
  sufficiently wild, prolonged uncontrolled flight can overflow it to `NaN` via
  `RuntimeWarning: invalid value encountered in matmul`, permanently breaking that
  `DroneEKF` instance (needs a fresh restart, not a real fix, just a symptom of the
  underlying instability). Worth a bounding fix once flight is otherwise stable, not
  urgent before then.
- Live-bridge calibration is dwell-only (no wiggle maneuver is actually commanded, so
  `accel_bias`/`mag_bias` may retain some ambiguity relative to a real wiggle -
  `gyro_bias` is fine since it's fully observable at rest regardless). Not verified
  whether this matters in practice for the Gazebo x500's very clean simulated
  sensors - probably low priority.

## Recommended next steps (tune inside-out, one loop at a time)

**Immediate next step as of 2026-09-27**: with the run-to-run noise fixed and
`pos_xy=0`, `run_sim.py` is genuinely stable for 20s+ - the position-hold resonance
(see "CURRENT blocker" above) is now the thing standing between here and real
station-keeping flight. Before touching `pos_xy`'s gain again:
1. Re-run the 20s `pos_xy=0` baseline once or twice more to confirm it's actually
   reproducible now (trials should finally be comparable to each other) - don't
   assume one clean 20s run is proof, same caution as everything else in this file.
2. When a `pos_xy` trial tumbles, look closely at which axis leads the resonance
   (roll, pitch, or yaw first) in the second or so of growing `EKF_ACCEL` deviation
   before the tumble - the one tumble inspected so far had both roll and yaw
   saturated simultaneously, not cleanly one axis, so this isn't confirmed yet.
3. Once the leading axis is identified, that's the loop (`att_rp`/`att_yaw`/
   `rate_rp`/`rate_yaw`) that most likely needs more damping margin - re-tune it in
   isolation first (via its own `test_*_loop.py` harness) before re-enabling
   `pos_xy`, rather than guessing at `pos_xy`'s own gain again (already tried two very
   different values, both failed the same way - the problem isn't there).
4. Now that trials are trustworthy, the previously-inconclusive `att_rp` kp=3.0-vs-6.0
   comparison and the unconfirmed `ki` addition to `att_rp`/`att_yaw` (both noted
   below as stale) are worth actually re-running before more new changes.

The historical notes below (mostly 2026-09-26 and earlier) are still accurate as a
record of what was tried, but "run_sim.py not flying yet" is now stale given the
20s pos_xy=0 result - read them for the reasoning/gotchas, not as the current status.

This is standard practice for cascaded flight controllers (matches how real FCs like
Betaflight/PX4 are tuned in practice: rate/acro mode first, then angle mode, then
position hold) - **don't go back to testing the full cascade until the rate loop is
confirmed solid on its own.**

**New this session**: `test_attitude_loop.py` + `run_attitude_test.sh` (attitude-loop
harness, same pattern as the rate-loop one), `test_common.py` (shared
`wait_for_liftoff()`/`reset_cascade_pids()` used by both harnesses - the PID reset
matters because `wait_for_liftoff()`'s variable-length climb otherwise leaves a
variable amount of pre-accumulated integral windup before the timed test starts, which
was causing real run-to-run inconsistency).

**Rate loop: DONE as of 2026-09-26** (see fixes #5/#6 above) - all three axes verified
clean (no overshoot, no oscillation) once the liftoff fix made the test actually
airborne and the yaw `kp` fix gave it real authority:
```
bash run_rate_test.sh roll 0.3 5.0     # axis, step size (rad/s), duration (s)
```
(default step is now `0.3`, not the old `1.0` - see the comment at the top of
`test_rate_loop.py` for why: `1.0` held for the standard 1.667s middle-third accumulates
~95 degrees of real attitude angle since nothing holds attitude level here, which pushes
the vehicle into large-angle territory that has nothing to do with rate-loop tracking).
Read the first ~1-1.5s after step onset (rise time, overshoot, initial settle) as the
trustworthy signal - don't read too much into the later portion of a long-held step;
see the velocity-driven-drift note above for why roll/pitch specifically wander late in
the window.

**Attitude-loop kp tuning attempted 2026-09-26, inconclusive - paused, not abandoned.**
Ran 3 fresh `test_attitude_loop.py roll 0.2 3.0` trials each at `att_rp` `kp=3.0` and
`kp=6.0`. Peak roll tracking ranged **0.10-0.207 rad (against a 0.2 target) at BOTH
values** - run-to-run noise (almost certainly the same source as the intermittent
EKF_ACCEL jump above) is bigger than any effect a 2x kp change should plausibly cause,
so the comparison wasn't actually measurable. kp=6.0 had 0/3 blowups vs kp=3.0's 1/3,
too small a sample to credit the gain for it. **Kept kp=6.0 pragmatically, not because
it's confirmed better** - see the comment on `att_rp` in `run_sim.py`'s `GAINS`.

**Attitude-loop kp noise is now explained** (was the EKF corruption bug, items #7/#8 -
fixed) - the inconclusive kp=3.0-vs-6.0 comparison above should be considered stale and
worth re-running now that the underlying noise source is fixed, before trusting kp=6.0.

**Velocity loop: real bugs found and fixed as of 2026-09-26** (items #7-10) -
`test_velocity_loop.py` + `run_velocity_test.sh`, same harness pattern, extended one
loop further out (velocity_loop -> attitude_loop -> rate_loop, position_loop bypassed).
Unlike the other harnesses, this one can't hold a fixed baseline thrust (vel_z's own
output IS thrust) - see the WARMUP phase in its own docstring for why, and don't reset
`vel_pid_z`'s integral after warmup (only rate/attitude PIDs) or you'll recreate the
sag the warmup exists to avoid. Usage:
```
bash run_velocity_test.sh vz 0.5 3.0     # axis (vx/vy/vz), step (m/s), duration (s)
```
Confirmed clean after fixes #7-10: `vz 0.5 3.0` now shows `vx`/`vy` staying bounded and
settling (not diverging) for a full 3s test, thrust climbing smoothly without
saturating. Not yet tested: `vx`/`vy` as the STEPPED axis (only tested them as the
"should stay at 0" background axes so far) - worth doing before fully trusting
`vel_xy`'s retuned gains (kp=0.8/ki=0.2/kd=0.6, itself not yet independently verified
against a real step, only against arresting drift).

**`run_sim.py` tried for real, extensively, 2026-09-26 - climbs cleanly every time,
divergence point pushed from t~1.85s to t~5.3s across a chain of real fixes (D-term
dead code, D-term filtering, gyro guard, damping re-tuning - see items #11-14). Not
flying yet, but this is now iterative tuning territory, not bug-hunting.**

1. **Continue iterative full-cascade tuning, focused on yaw** - it's been the axis
   that fails last and worst in the most recent trials (a sudden, violent rate spike
   after roll/pitch have already been held stable for several seconds). Concrete next
   moves, roughly in order of how cheap they are to check:
   - **First, just re-run the current gains** (`att_rp` kd=1.5, `att_yaw` kd=0.3,
     `rate_yaw` kd=0.0) once or twice more before changing anything else - the best
     result so far is a single trial, and this session's own data (the attitude-loop
     kp=3-vs-6 comparison) shows single trials aren't reliable evidence on their own.
   - If it reproduces, look specifically at what's happening to yaw (`rate_sp`/`tau`
     on the 3rd axis, and `pos.x`/`pos.y` growth) in the ~1s before the spike, the
     same way the roll/pitch divergence was diagnosed earlier - `run_sim.py`'s own
     print already has everything needed, just needs reading closely around that
     specific window.
   - `att_yaw`'s `kp=1.5` has never been revisited (only `kd` was touched this
     session) - worth considering whether it's simply too weak relative to how hard
     `rate_yaw`'s own `kp=150` can react once attitude error builds up.
   - Add `pos.z` overshoot-checking and a horizontal-speed readout to `run_sim.py`'s
     print if the position loop still looks implicated once yaw is better understood.
   - Build `test_position_loop.py` (same pattern as the others, one loop further out)
     if isolating position specifically still seems necessary - not yet built.
2. **Standard per-axis tuning heuristic** (still applies): raise `kp` until you see
   sustained oscillation in the log, back off to ~50-70% of that value, add `kd` to
   damp remaining overshoot, add a small `ki` last only if there's steady-state error
   that `kp`+`kd` alone don't close. Edit gains in `run_sim.py`'s `GAINS` dict (both
   scripts import from there). Don't trust a single trial's result for any change -
   this session has repeatedly confirmed real run-to-run noise; average/range over a
   few before concluding a gain change helped or hurt, and revert immediately (like
   the `rate_yaw` kd=5.0 attempt) the moment a change looks clearly worse rather than
   pushing further on it.
3. Not yet tested: `test_velocity_loop.py` with `vx`/`vy` as the STEPPED axis (only
   tested as "should stay at 0" background axes so far) - worth doing before fully
   trusting `vel_xy`'s retuned gains against a real step, not just against arresting
   drift.
4. Re-check the `ki` addition to `att_rp`/`att_yaw` noted as unconfirmed above once
   there's bandwidth - it predates all of this session's fixes and was never resolved
   either way.

## Useful physical constants already derived (x500 model, don't re-derive)

- Mass: 2.0 kg. `motorConstant`: 8.54858e-06 (thrust = motorConstant * omega², N).
  `momentConstant`: 0.016 (yaw reaction torque coefficient). `maxRotVelocity`: 1000
  rad/s (this is what `Actuators.velocity` values mean - real rad/s, not 0-1).
- Hover thrust: ~757 rad/s per motor (mass·g / 4, solved through the thrust formula).
- Rotor layout (Gazebo's real FLU frame, from `Tools/simulation/gz/models/x500/model.sdf`
  and `x500_base/model.sdf`): motor0=front-right(ccw), motor1=back-left(ccw),
  motor2=front-left(cw), motor3=back-right(cw). Arm offset ±0.174m in x/y.
- IMU/accel/gyro/mag all come through raw, unconverted from Gazebo (no FLU->FRD
  conversion applied, unlike PX4's own `gz_bridge.cpp`) - this is intentional and
  correct as long as everything downstream (the EKF, the mixer) stays consistently
  built against that same raw FLU frame, which it currently is. Don't add a FRD
  conversion without also re-deriving the mixer signs to match - they'd no longer
  agree.

## Roads not taken (context, not TODOs)

- Considered running PX4 SITL for real and having this Python stack act as either
  (a) a companion-computer sending PX4 offboard setpoints over MAVLink, or (b) an
  external estimator publishing vision/odometry into PX4 via MAVLink/DDS. Both are
  legitimate real-world patterns, but the user specifically wants to validate their
  *own* full stack (including the mixer, which neither pattern would exercise) - the
  current direct-Gazebo-bridge approach was chosen deliberately for that reason.
  QGroundControl was installed while exploring option (a); it's not used now.
- Considered a custom hand-rolled physics simulator instead of Gazebo. Rejected in
  favor of Gazebo since PX4's own SITL toolchain already provides real, validated
  rigid-body physics + sensor simulation for free once installed.
