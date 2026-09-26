# Handoff: PID cascade tuning against Gazebo/x500

## Goal

Custom Python EKF (`state estimation/FINAL_gps.py`) + custom Python PID cascade
(`PID/cascade.py`, `PID/pid.py`) flying a simulated PX4 x500 quadrotor directly in
Gazebo (Harmonic, via `gz.transport13`), bypassing PX4's own estimator/controller
entirely. This is deliberate: the user is building their own EKF+PID stack from
scratch as a learning project, and wants to validate it against real physics before
ever touching real hardware or PX4/Betaflight integration (a separate, later step).

**Current status (updated after the 2026-09-26 session): the rate loop is now
confirmed solid on all three axes in isolation** (see "What's confirmed fixed" below -
a ground-contact bug meant every earlier `test_rate_loop.py` run was never actually
airborne, and a yaw-authority gap meant yaw never really got tested at all). With both
fixed, roll/pitch/yaw all show clean, no-overshoot step responses. Full-cascade flight
(this session's earlier best run flew ~71s before diverging into an unrecoverable
runaway yaw spin) has **not been re-tested since these fixes** - that diagnosis predates
discovering the rate loop wasn't being validly tested at all, so don't assume the old
71s-runaway failure mode is still accurate. Next step is the attitude-loop test harness
(see "Recommended next steps"), not jumping straight back to full-cascade flight.

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

## What's NOT resolved

- **Full-cascade flight has not been re-tested since the 2026-09-26 rate-loop fixes.**
  Everything below this line about the 71s runaway/instability predates discovering
  that `test_rate_loop.py` wasn't validly testing anything (see fixes #5/#6 above) - it
  may or may not still apply once the attitude loop is validated and a real full-cascade
  run is tried again. Don't assume the yaw-spin failure mode is still the live bug;
  it's quite possibly explained by the same yaw-authority gap that's now fixed.
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
- **Probable EKF robustness gap under real linear acceleration, found via
  `test_attitude_loop.py` (2026-09-26).** Every attitude-loop trial so far (roll, 0.2
  rad step) eventually suffers a sudden, large, single-print-interval jump in computed
  attitude (e.g. tilt.roll: 0.061 -> 2.379 rad between two ~0.03s-apart samples) that
  coincides with an altitude/velocity discontinuity. Two things rule out the simpler
  explanations: (1) `state['rate']` (raw bias-corrected gyro, doesn't depend on the
  quaternion at all) stays smooth and physically continuous straight through the same
  jump (0.590 -> 0.755, no discontinuity) - so this isn't a real physical event or a
  control/mixer bug, it's specifically the EKF's `self.q` that's discontinuous. (2)
  Resetting the rate/attitude PID integrators right after liftoff (see
  `test_common.reset_cascade_pids()`, added this session) changed the timing of the
  blowup (~1.5s vs ~2.4s into a similar trial) but did not prevent it - so it isn't PID
  windup either. **Leading theory, not yet confirmed**: by the time this happens the
  vehicle has picked up real horizontal velocity/acceleration from the sustained tilt
  (same mechanism as the velocity-driven-drift note above) - `FINAL_gps.py`'s own
  docstring already flags that the accelerometer correction assumes hover-like
  (low-acceleration) conditions ("accelerometer doesn't work when drone accelerating
  hard"), so a bad accel correction slipping past the chi-squared gate during a real
  acceleration burst could inject a large one-step `d_theta` via `apply_correction()` -
  which would look exactly like this: smooth gyro, corrupted quaternion. **Diagnostic
  added, not yet confirmed**: `FINAL_gps.py`'s accel-correction block now has a `TEMP
  DIAGNOSTIC` print (`EKF_ACCEL t=... FIRED/REJECTED dev=... d2=... dtheta=...`) on
  every tick - deliberately left in place, don't remove until this is resolved. **Also
  confirmed intermittent**: 5 fresh `test_attitude_loop.py roll 0.2 3.0` trials this
  session (after the liftoff/sub-rate/PID-reset fixes) - only 1 blew up, the other 4
  were clean. So whatever this is doesn't fire every time; next time it recurs, check
  the `attitude_test.log`/diagnostic output for an `EKF_ACCEL FIRED` entry at the exact
  wall-clock tick the jump happens (correlate via the harness's own `start_t_epoch=...`
  line against the diagnostic's `t=` epoch timestamps). This is a strictly deeper
  problem than rate/attitude gain tuning - a corrupted state estimate looks like a
  control failure no matter how good the gains are, and it's also the leading suspect
  for the general run-to-run noise that made attitude-loop kp tuning unmeasurable this
  session (see "Recommended next steps" below).
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

1. **Chase the EKF-under-acceleration theory** (see "What's NOT resolved" above) when it
   next recurs - a corrupted attitude estimate looks exactly like a control failure, and
   it's also the best current explanation for why gain comparisons aren't measurable.
   The `EKF_ACCEL` diagnostic print in `FINAL_gps.py` is already in place for this.
2. **Standard per-axis tuning heuristic** (still applies once the noise is understood):
   raise `kp` until you see sustained oscillation in the log, back off to ~50-70% of
   that value, add `kd` to damp remaining overshoot, add a small `ki` last only if
   there's steady-state error that `kp`+`kd` alone don't close. Edit gains in
   `run_sim.py`'s `GAINS` dict (both scripts import from there). Don't trust a single
   trial's result for any change here - this session's data says the noise floor is
   large enough that single trials aren't meaningful; average/range over several.
3. Once attitude is validated, re-check the `ki` addition to `att_rp`/`att_yaw` noted
   as unconfirmed above, then move to velocity/position, then finally a full-cascade
   flight - the old 71s-runaway diagnosis should be treated as possibly-stale until
   re-observed post-fix.

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
