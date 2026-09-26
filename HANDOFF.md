# Handoff: PID cascade tuning against Gazebo/x500

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

**Full-cascade flight has still not been re-tested since ANY of these fixes.** Every
prior full-cascade result (including the 71s-runaway/yaw-spin diagnosis) predates
discovering the vehicle usually wasn't even airborne, so treat all of it as stale.
Next concrete step: try `run_sim.py` for real now that rate/attitude/velocity have each
been individually validated - see "Recommended next steps".

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

1. **Try `run_sim.py` for real** - this is the actual goal ("does it fly"), and rate,
   attitude, and velocity have each now been individually validated in isolation for
   the first time. Expect new problems to surface (every layer so far has had at least
   one) - that's normal, not a sign the lower layers are wrong. `pos_xy`/`pos_z` still
   use old, never-revisited gains (`pos_z`: kp=1.5/ki=0/kd=0.3; `pos_xy` still zeroed) -
   don't assume position loop is validated just because velocity is.
2. **If it doesn't fly cleanly**, isolate `test_velocity_loop.py`'s `vx`/`vy` step
   response (not yet done) and/or build a `test_position_loop.py` (same pattern, one
   loop further out) before going back to full-cascade debugging blind - this session's
   whole arc has been "isolate before trusting the full stack," don't abandon that now
   that the goal is close.
3. **Standard per-axis tuning heuristic** (still applies): raise `kp` until you see
   sustained oscillation in the log, back off to ~50-70% of that value, add `kd` to
   damp remaining overshoot, add a small `ki` last only if there's steady-state error
   that `kp`+`kd` alone don't close. Edit gains in `run_sim.py`'s `GAINS` dict (both
   scripts import from there). Don't trust a single trial's result for any change here -
   real-time sensor/timing jitter under WSL means single trials carry real noise;
   average/range over a few before concluding a gain change helped or hurt.
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
