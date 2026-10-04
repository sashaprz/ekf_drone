# EKF stress-test report — baseline `86193e6` (dirty), 2026-09-30

Plan: `EKF_TEST_PLAN.md`. Suite: `testing/suite/`. Baseline artefacts:
`testing/results/baseline_86193e6/` — `results.json` (267 Tier A runs), `REPORT.md`
(auto-generated tables), `plots/<mission>__<fault>.png`, `TIERB.md` / `tierB.json`,
`VALIDATION.md` + `fault_channels.png`, `RECORDINGS.md`. Recordings:
`testing/data/flight_<mission>.csv` (+ `.cal.csv`), live trials in `testing/data/live/`
(gitignored). Nothing was committed.

Code under test: `state estimation/FINAL_gps.py` sha1 prefix recorded in
`results.json → meta.filter_sha1` (the only diffs vs `86193e6` are the pre-approved
instrumentation, §6).

---

## 1. Headline

- **Tier A: 1 PASS / 28 WARN / 238 FAIL** (267 runs). 185 of those FAILs are attitude
  NEES alone. Grading everything except NEES gives 76 PASS / 138 WARN / 53 FAIL.
- **The single most important weakness is calibration, not flight.** The live dwell-only
  calibration lands 0.5–1.4° off in tilt and 1–7° off in heading (driven by sensor noise;
  Gazebo's noise is tiny). The pinned accel/mag biases then freeze that error for the
  whole flight, while P claims σ_pitch ≈ 0.12°. Replay the *same flights* with a
  noise-free calibration dwell (`cal_ideal`) and error drops to **0.005–0.15° tilt,
  0.01–0.26° yaw on every mission, including circle_fast at 13° sustained tilt**.
- **`combined_realistic` (tilt RMS / max / yaw RMS, deg): FAIL on all 9 missions.**
  hover 3.0/3.7/0.9 · box 5.1/6.0/7.0 · circle_slow 0.4/1.1/4.3 ·
  **circle_fast 8.2/27.1/21.5** (mag gate locked out ~54 s, 9 GPS resets) · stops
  3.9/7.1/14.0 · yaw_steps† 4.8/16.6/25.4 · yaw_spin† 3.2/6.2/8.0 · takeoff_land
  5.0/5.9/21.8 · patrol_long 2.5/6.4/16.7. With the same noise and biases applied **in
  flight only** (clean calibration), circle_fast is 1.1/2.3/3.9 with zero mag rejections.
  The realistic-hardware failure is seeded by the calibration.
- **Tier B (closed loop on the EKF): 12/12 trials survived** (hover, box, circle_slow,
  takeoff_land × 3). The same calibration signature shows up live: heading error spans
  −7.3…+5.8° across trials, and hover true position wanders 0.14–0.40 m RMS vs 0.00 in
  ORACLE.
- Found along the way, **outside the filter**: a controller frame bug that tumbles the
  vehicle once yaw ≳ 90°, even on ground truth (§4 F8). Yaw missions are scored only up
  to that point (†).

## 2. Summary grid

Tier A overall grade per (mission × fault). `P`/`W`/`F`. Fault keys are in
`baseline_86193e6/REPORT.md`. † = truncated recording (see F8).

```
                none   drop5  drop15  drop30  stale5 stale15 stale30    gpsN    outl     1Hz   gb0.2     gb1   gb1ir  gdrift abx0.05  abx0.2  abx0.5 abz0.05  abz0.2  abz0.5 abx0.2ir  mb0.05  mb0.15 mint0.3   mint1    imuN   spike   idrop  gap0.2    gap1    REAL calIdeal
hover              F       F       F       F       F       F       F       F       F       F       W       F       F       F       F       F       F       F       F       W       F       F       F       F       F       F       F       F       F       F       F       F
box                F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       W       F       F       F       F       F       F       F       F       F       F
circle_slow        F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F
circle_fast        W       W       W       W       W       F       F       W       W       W       W       W       W       W       F       F       F       W       W       W       F       F       F       W       W       F       W       W       W       F       F       F
stops              F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F
yaw_steps†         F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F
yaw_spin†          F       F       F       F       F       F       F       F       F       F       F       F       F       F       P       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F
patrol_long        W                       W                               W                                               W       F                                                       F                               F       F                               F       F
takeoff_land       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F       F
ref_live1          W   (2026-09-30 reference flight, first 12 s, "none" only)
```

How to read it: the grid is almost uniformly `F` because attitude NEES fails nearly
everywhere, and for two opposite reasons (§3 H5). Filter changes should be judged with
`compare.py` per metric, not by counting letters. `cal_ideal` is also `F`, but only on
NEES (≪1, underconfident). Its errors are the best in the suite.

**Tier B** (`TIERB.md`; mean [min..max] over 3 trials; truth vs the mission's ideal path):

| mission | survived | true track_h RMS, EKF | same, ORACLE | est tilt RMS | est yaw RMS | est pos_h RMS |
|---|---|---|---|---|---|---|
| hover | 3/3 | 0.25 [0.14..0.40] m | 0.00 | 0.64 [0.41..0.92]° | 5.75 [4.15..7.33]° | 0.04 m |
| box | 3/3 | 0.95 [0.93..0.96] | 0.93 | 0.61 [0.09..0.93] | 1.26 [0.19..1.88] | 0.01 |
| circle_slow | 3/3 | 0.54 [0.39..0.80] | 0.48 | 0.86 [0.46..1.24] | 4.77 [1.74..6.33] | 0.05 |
| takeoff_land | 3/3 | 0.17 [0.05..0.37] | 0.00 | 0.74 [0.21..1.35] | 2.85 [1.18..5.25] | 0.01 |

Not flown in Tier B: circle_fast and stops (the plan says fly aggressive missions only if
Tier A passes them, and it didn't), and yaw_steps (would tumble from the controller bug
regardless of estimator, F8). Zero `EKF_GPS_RESET` and no >0.11 s loop stalls in any trial.

## 3. Hypothesis verdicts

**H1 — tilt during GPS loss: CONFIRMED, but not by the assumed mechanism.** Pure
translation without GPS barely drifts tilt. `gps_dropout(30 s)` tilt drift is
|≤0.03|°/s on hover/box/circles/stops/patrol (e.g. circle_fast 0.028°/s, stops
0.019°/s), which PASSes. The bad case is **rotation without GPS**: `yaw_steps` + 15 s
dropout drifts **0.32°/s** and ends at 4.6° pitch / 7.9° yaw error (13.6° mid-turn),
with pos error 28 m. The same with `cal_ideal`: 0.13° / 0.36°. So the drift is the
frozen calibration biases turning inconsistent as the body rotates (F3). GPS velocity
normally masks it. Plot: `plots/yaw_steps__gps_dropout(duration_s=15).png`. Separately,
`gps_stale` (hung receiver) is much worse than dropout: circle_fast `stale30` ends at
10.5° tilt with 104 rejections and 20 lockout resets (F6).

**H2 — pinned biases: CONFIRMED.** An in-run accel bias the filter can't learn
(`accel_bias(x, 0.2, from_cal=False)`) gives pitch error ≈ −b/g (b/g = 1.17°): −1.15…−1.86°
on hover/box/circle_fast/yaw_steps/takeoff_land, and pos_h 0.19–0.41 m. It is partly
observed where GPS velocity sees it during sustained acceleration: circle_slow −0.26°,
stops −0.56°. A bias present from power-on
is worse. The calibration splits it between tilt and bias and, through the inclined mag
field, into heading: `accel_bias(x, 0.2)` gives hover yaw RMS **17.6°**, stops 13.8°.
Z-axis accel bias is harmless to tilt, as expected. Hard-iron `mag_bias 0.05` gives a
1.7–8.4° yaw offset that is never learned (`mag_bias_final_err` ≈ injected on every
mission). The gyro bias (not pinned) is learned fine: `gyro_bias(1 °/s)` in-run converges
in 0.4–1.2 s to <0.05°/s on the non-yaw missions. Exception: under rotation it absorbs
the calibration inconsistency (0.7–0.8°/s wrong on yaw_steps/yaw_spin, F3).

**H3 — heading from calibration: CONFIRMED, and larger than thought.** Calibrated heading
error per recording: hover +2.6°, box +1.0°, circle_slow +2.3°, stops −4.1°,
takeoff_land +1.9°. Tier B live: −7.3…+5.8°. It is noise-driven and chaotic. Change the
hover dwell samples by a 0.2°/s constant (`gyro_bias 0.2`) and the same flight's
calibrated yaw goes from 2.58° to 0.01°. **Rotation does not converge it**:
yaw_steps with GPS is −0.89° before the 90° step and −0.88° after (tilt partially heals,
1.18° → −0.04° roll, via GPS velocity). Without GPS, rotation makes it much worse (H1).
Mechanism in F1.

**H4 — sustained maneuvers: REFUTED for the in-flight filter.** With a noise-free
calibration, circle_fast (12.9° mean / 16.5° peak true tilt, 2.3 m/s) scores 0.15° tilt
RMS / 0.43° max and 0.26° yaw RMS. stops scores 0.05° / 0.27°. The `none` numbers for
these missions (0.67° / 1.13° tilt) are calibration offsets carried along.
GPS-velocity tilt observability under sustained acceleration works.

**H5 — overconfidence: CONFIRMED (mixed sign).** Attitude P doesn't track the actual
error at all. The median σ is ~0.72° roll / 0.12–0.2° pitch / 1.43° yaw on *every* run,
whatever the error. So NEES_att is 6.6–69 in `none` (overconfident about calibration
errors) and 0.0006–0.08 in `cal_ideal` (underconfident about in-flight error). The
overconfidence does real damage through the mag gate:
- `combined_realistic` / `imu_noise` on circle_fast: **13,300–13,600 mag rejections**
  (the gate is closed ~54 s of 75), 8–27° tilt.
- `imu_gap(1 s)` on circle_fast: 656 mag rejections, 15.6° tilt transient, 2 GPS resets.
  yaw_spin: 745 rejections.
- The recorded patrol_long had real 0.6–1.1 s loop stalls at t=59–65 s. Result: 4–5°
  error, then **430 consecutive mag rejections** until a GPS lockout reset at t=66.3 s
  rescued it.

The attitude-Q cut is not the problem in-flight (in-flight error is tiny). P is just
never told about calibration ambiguity or missed integration time, and there's no mag
lockout recovery.

**H6 — GPS realism: REFUTED (handles it), with two caveats.** `gps_noise` (1.5 m/3 m +
0.1 m/s + wander) gives pos_h RMS 0.13–0.65 m (≤0.43σ, PASS) and NEES pos 0.7–2.2 /
vel 0.24–2.3 (consistent), with no resets. patrol_long reaches 1.19 m / NEES_pos 6.4 (the
0.5 m/√min wander builds up over 10 min, and R_gps has no time-correlated term): WARN. 1 Hz GPS: pos_h ≤0.31 m, no tilt effect.
Caveats: (a) the 6-DOF gate passes any horizontal jump under ~10.25 m
(d² = off²/6.25 < 16.81). circle_slow/stops/takeoff_land accepted 1 of 2–4 injected
outliers (small effect here). (b) A *frozen* fix is accepted and then causes repeated
lockout resets (F6). On clean Gazebo GPS, NEES pos/vel ≈ 1e-3 because `R_gps` is a real
receiver's. That's a simulator artefact, reported but not graded (§7).

**Also tested (not in the plan's list):** `mag_interference` 0.3/1.0: the gate rejects
100% of the 5 s burst (1,250/1,250), with zero effect on error. PASS in spirit.
`imu_spikes` (0.1%): tilt max +1–2°, no lockouts, except patrol (10.2° max, stall-related).
`imu_dropouts` 2%: no effect. takeoff_land: ground contact and re-takeoff produced no
GPS resets and no extra error (the reset path wasn't needed).

## 4. Findings, ranked by severity

### F1 — Dwell-only calibration locks in a noise-driven attitude error that P doesn't know about

*Symptom.* In `none`, tilt error is a constant offset from t=0 (hover 0.65° roll / −0.38°
pitch / 2.58° yaw, flat for 75 s; plot `plots/hover__none.png`). With `cal_ideal` it's
0.005° / 0.008°. IMU noise in the dwell only → box 5.6° tilt RMS; in flight only → 0.70°
(the same as `none`). Tier B heading spread is ±7°.

*Mechanism.*
- `FINAL_gps.py:394` calibrates dwell-only (`wiggle_steps=0`). At one orientation,
  tilt vs accel bias and heading vs mag bias are unobservable combinations.
- `calibration.py:122-123` runs that with `P = np.eye(12)` and **attitude Q = 0.01
  rad²/step**. That's the value `FINAL_gps.py:312` already cut to 1e-7 for live mode
  because it was ~1e9× too big. It's still in calibration. Bias Q is 1e-6/step. So the
  filter never averages its 200 samples. Measurement noise random-walks the estimate
  along the unobservable directions. 200 samples of 0.006 m/s² accel noise should pin
  tilt to ~0.003°; instead the result lands ~1° off, and heading 1–7°.
- `FINAL_gps.py:429-432` then pins both biases (Q 1e-12, P 1e-6) at those values. The
  wrong split is permanent.
- `FINAL_gps.py:404` seeds P_att from calibration's P, which is only small in the
  observable directions, hence NEES 7–70.

*Candidate changes* (describe, don't implement):
1. **Make the dwell calibration average.** Either set calibration's attitude Q to
   something gyro-realistic (~1e-9–1e-7 rad²/step, as in live) with a finite bias prior
   (P0 bias σ ≈ expected turn-on bias, not 1), or skip the filter for dwell-only: average
   accel and mag over the dwell and solve attitude directly (two-vector/TRIAD against
   gravity and `mag_ref`), with biases held at their prior. Expected: `none` → ≈
   `cal_ideal` in Gazebo. With a real accel bias b, tilt error → b/g (unavoidable at rest
   without a wiggle), and heading no longer inherits accel bias through the inclined
   field. Risk: none for Gazebo. On hardware the bias-free assumption turns b into tilt
   `b/g`. Caught by the `accel_bias(from_cal)` rows.
2. **Seed P honestly.** Add the unobservable-direction variance to P_att after
   calibration: tilt (σ_b,accel/g)², heading (σ_b,mag/|B_h|)². Also keep the cross-
   covariance with the (pinned) biases, so a later observation can move the right state.
   Expected: NEES_att in `none` → ~1, and gates stay open after a bad start (F2). Risk:
   larger P lets GPS/mag move attitude more per update. Watch `tilt_max` on stops /
   circle_fast and `gps_noise` rows.
3. (Longer term) a real pre-flight wiggle, as HANDOFF already suggests.

*Evidence to verify a fix:* improve `none`, `gb*`, `abz*`, `imuN`, `REAL` on all missions
toward the `cal_ideal` row, Tier B yaw spread. Must not regress: `cal_ideal` (in-flight
accuracy), `mint*` (gate still rejects interference), `gpsN`.

### F2 — Overconfident attitude P + mag gate = lockout with no recovery path

*Symptom.* circle_fast `REAL`: 13,572 mag rejections, 27.1° tilt max, 9 GPS resets
(`plots/circle_fast__combined_realistic.png`). `imu_gap(1 s)` circle_fast: 656
rejections, 15.6°. patrol_long `none`: 430 consecutive rejections after natural stalls,
rescued only by a GPS reset.

*Mechanism.* The mag gate at `FINAL_gps.py:637` uses S = H P Hᵀ + R_mag with R_mag σ =
0.01 on a unit vector (~0.6°) and σ_pitch ≈ 0.12°. Once attitude is a few degrees off,
every mag update fails the gate. Unlike GPS (`FINAL_gps.py:573-584`, reset after 5
rejections), the mag path has no lockout recovery, and nothing inflates P while
rejecting. GPS velocity is then the only thing pulling attitude back.

*Candidates.*
1. Mag lockout recovery mirroring the GPS one: after N consecutive rejections (~1 s),
   inflate P_att (at least the heading/tilt block) so the next update is accepted, or
   re-initialise heading from mag. Risk: the interference burst (`mint1`) would get
   accepted after 1 s. Mitigate with a magnitude/inclination sanity check on the raw
   reading first (interference changes |B| and dip; a real attitude error doesn't).
2. F1.2 (honest P) removes most triggers.
3. Inflate R_mag a little (e.g. 0.02–0.03), since it's tighter than real mags justify.
   Watch `yaw_rms` in `cal_ideal`.

*Verify:* `REAL`/`imuN`/`gap1` on circle_fast, yaw_spin, patrol_long (mag_rejected →
~0, tilt_max down). Must not regress: `mint0.3`/`mint1` (still 100% rejected during
the burst, `yaw_rms` unchanged).

### F3 — Frozen biases become inconsistent under rotation; without GPS the attitude runs away

*Symptom.* yaw_steps `drop15`: 13.6° yaw / 6.5° roll error mid-turn, 4.6°/7.9° after
(`cal_ideal` + same dropout: 0.13° / 0.36°). With GPS, the gyro-bias estimate absorbs
the inconsistency: 0.70°/s z-bias after one 90° turn (yaw_steps), 0.8°/s y (yaw_spin).
With `cal_ideal`: ≤0.04°/s.

*Mechanism.* The calibrated (tilt, accel-bias) and (heading, mag-bias) pairs are
self-consistent only at the calibration heading. A body-fixed bias error rotates in the
world frame as the vehicle yaws, but it's pinned (`FINAL_gps.py:429-432`). So the
residual goes into the free states instead: attitude via mag updates, gyro bias via
cross-covariance. GPS velocity corrects tilt when present (1.18° → −0.04° roll over the
turn).

*Candidates.*
1. F1 removes the source.
2. Unpin mag bias (and later accel bias) with a small Q **only while the vehicle is
   rotating in yaw** (observable then). Pinned at hover, where HANDOFF #5 showed they
   absorb heading/tilt. Risk: re-creating HANDOFF #5. Verify with `EXP="YAW0=10"`
   replay and the `none` hover rows.
3. Lower the gyro-bias process noise (module Q 1e-6/step ≈ 1°/s/√s random walk) so a
   0.7°/s swing in 2 s isn't cheap. Risk: slower learning; watch `gb1ir`/`gdrift`
   converge_s (currently 0.4–3.4 s).

*Verify:* yaw_steps/yaw_spin `none`, `drop5/15`, `mb*` (after F8 is fixed, full-length
yaw recordings). Must not regress: hover `none`, `gb1ir`.

### F4 — In-run accel bias is unobservable to the pinned filter (H2)

*Symptom / mechanism.* `abx0.2ir`: pitch ≈ −b/g (−1.15…−1.86° on 5 of 8 missions;
−0.26°/−0.56° on circle_slow/stops), pos_h 0.19–0.41 m. `FINAL_gps.py:429` pins it. Tier B already shows this magnitude of tilt error
costing ~0.1–0.4 m of true position.
*Candidates.* Small accel-bias Q (≈1e-10–1e-9/step), learnt mainly under rotation or
acceleration. Risk: HANDOFF #5's −3 m/s² wander. Keep the accel correction off.
*Verify:* `abx*ir` improves. Must not regress: `none`, `cal_ideal`, stops/circle_fast
`tilt_max`.

### F5 — `MAX_DT` clamp silently discards rotation during loop stalls

*Symptom.* `gap1`: circle_fast 15.6°, yaw_spin 3.2° and 745 mag rejections, stops 1
reset. Natural 0.6–1.1 s stalls in the patrol recording (F2). `gap0.2`: ≤2.3° (circle_fast).
*Mechanism.* `FINAL_gps.py:517`, `dt = min(dt, MAX_DT)`. After a 1 s stall the
predict step integrates the held gyro for only 50 ms, and **P grows by one step's Q**.
So the filter is both wrong and certain. (On real hardware a FIFO'd IMU would deliver
the missing samples. Here the bridge only keeps the latest one.)
*Candidates.* When the measured dt exceeds MAX_DT, keep the clamp for velocity/position
if desired, but inflate P_att (and vel/pos) by Q × (dt_true/dt_nominal), so the gates
reopen. Optionally integrate attitude over the true dt with the held rate. Risk: small.
*Verify:* `gap0.2`/`gap1` rows, patrol `none` (mag_rejected 430 → ~0).

### F6 — A frozen ("stale") GPS fix is accepted, then triggers repeated lockout resets

*Symptom.* circle_fast `stale15/30`: 55/104 rejections, 11/20 resets, tilt 9.4/10.5° at
the end of the window. stops `stale15`: 6 resets, recovery 16.6 s.
*Mechanism.* Stale fixes pass the gate until the vehicle has moved ~10 m. Each
`EKF_GPS_RESET` (`FINAL_gps.py:573-584`) snaps pos/vel to the **frozen** fix and
reopens P, so the next fresh-vs-frozen mismatch repeats the cycle, and the vel snap
disturbs attitude through cross-covariance.
*Candidates.* Treat a fix bit-identical to the previous one (or with velocity
inconsistent with IMU-predicted motion) as "no fix", which the new `None` path supports.
Don't reset to a fix that failed the freshness check. *Verify:* `stale*` rows. Must not
regress: `drop*`, `1Hz`.

### F7 — The 6-DOF GPS gate accepts horizontal jumps up to ~10 m (minor)

`outl`: circle_slow 3/4, stops 1/2, takeoff_land 1/2 outliers rejected. The rest were
accepted with small effect here. d² = off²/σ² with σ = 2.5 m. *Candidate:* gate
position and velocity separately (3-DOF each), so a position jump isn't diluted by a
good velocity. *Verify:* `outliers_rejected == outliers_consumed`.

### F8 — (controller, not EKF) attitude error is computed in the world frame

> **FIXED 2026-09-30 (later the same day)** — `cascade.py` now uses `q⁻¹ ⊗ q_sp`. yaw_steps
> and yaw_spin were re-recorded in ORACLE mode and fly clean (max true tilt 0.0°), and the
> truncation was removed. Full-length yaw results (vs the new recordings, so not
> comparable to the baseline's truncated rows) are in `testing/results/yaw_fixed_controller/`:
> yaw_steps `none` tilt 0.44°/yaw 2.92° RMS vs `cal_ideal` 0.04°/0.09°, so rotation does
> **not** remove the calibration heading error (H3). `mag_bias 0.05` now also triggers 4–5 GPS
> lockout resets on the yaw missions with no GPS fault (pinned bias vs rotation, F3). The
> original text below is kept for the record.

`PID/cascade.py:145`, `q_err = quat_mult(att_setpoint, quat_conjugate(state["quat"]))`,
is `q_sp ⊗ q⁻¹`, a **world-frame** rotation, but its components feed body-rate PIDs.
Verified numerically: a +5° roll request reads as roll +5° at yaw 0, **pitch +5° at yaw
90**, and **roll −5° at yaw 180** (positive feedback). ORACLE recordings of yaw_steps
and yaw_spin tumble at yaw ≈ 95° / ≈ 150° (`RECORDINGS.md`,
`testing/data/logs/rec_yaw_*.log`). Every earlier flight held yaw ≈ 0, which is why it
never showed. Candidate fix: `q_err = quat_mult(quat_conjugate(state["quat"]),
att_setpoint)` (body-frame; gives roll +5° at all three yaws). **Not applied**: the plan
reserves controller changes for the owner. After fixing, re-record yaw_steps/yaw_spin
(`record.ps1 yaw_steps yaw_spin`) and delete their `TRUNCATE` entries in `run_suite.py`.

### F9 — (simulator, not EKF) Gazebo GPS is noise-free, R_gps is a real receiver's

NEES pos/vel ≈ 1e-3 on clean-GPS runs. Not a filter defect. Keep R_gps for hardware, and
judge pos/vel consistency on `gpsN`/`REAL` rows only (they read 0.7–2.2).

> **Follow-up 2026-09-30 (owner asked for these):**
> - **F5 stall handling — IMPLEMENTED** in `FINAL_gps.py` (`missed = dt_raw - dt`): coast
>   attitude on the held gyro rate (exact rotation), velocity/position on the held accel,
>   and add the skipped steps' Q plus `STALL_RATE_STD`/`STALL_ACCEL_STD` uncertainty to P.
>   Full suite vs `baseline_86193e6_yawfix` (`testing/results/candidate_stall_fix/COMPARE.md`):
>   77 improved, 2 grade changes (both up), no graded regressions; every run without a stall
>   is bit-identical. circle_fast 1 s gap 15.6 -> 2.1 deg tilt max, mag rejections 656 -> 0;
>   patrol_long `none` 6.3 -> 0.43 deg, 430 -> 0 rejections. The 57 flagged "regressions" are
>   ungraded clean-GPS pos/vel NEES (F9) and patrol `cal_ideal` NEES_att getting smaller.
> - **F2 mag lockout reset — implemented but DISABLED** (`MAG_LOCKOUT_STEPS = 0`): with it on,
>   0.3-amplitude interference got through the dip-angle guard (tilt 0.7 -> 10-14 deg) and
>   calibration-caused lockouts got worse (circle_fast imu_noise 9 -> 26 deg). Revisit after F1.

> **In-flight calibration — IMPLEMENTED 2026-10-01** (owner's go-ahead; fixes F1, most of F2/F3).
> - `calibration.calibrate_dwell` (live mode): averages the dwell, attitude from the two mean
>   vectors (triad: gravity exact, mag for heading only), and a covariance from one Kalman
>   update of a finite bias prior. It keeps the tilt↔accel-bias and heading↔mag-bias links in P
>   instead of resolving them at random. Priors `ACCEL_BIAS_PRIOR_STD` 0.005,
>   `MAG_BIAS_PRIOR_STD` 0.02 (a bench-calibrated compass).
> - Mag bias **unpinned**: learned in flight once the body rotates. Accel bias **stays pinned**:
>   unpinned it diverged at hover (0.75 m/s², yaw −16°), and with its calibration link an
>   in-flight bias shift leaked into heading.
> - `gz_bridge.py` scales mag by one latched field strength instead of normalizing each
>   reading. Normalizing made a hard iron orientation-dependent, so it was unlearnable. Suite
>   v2 injects mag faults additively to match.
> - `missions.py`: optional **calibration turn** (smooth 360° yaw over 12 s + 3 s hold after the
>   settle hover). `hover_ct`/`box_ct` missions, or `CAL_TURN=1` for any mission.
> - Results (`testing/results/candidate_calturn/`, vs the stall-fix and the original baseline):
>   grades excl. NEES 76/139/52 → **216/32/19** (P/W/F); median tilt 0.72 → **0.05°**, heading
>   2.33 → **0.12°**; GPS resets 169 → 76. circle_fast `combined_realistic` 27° max → 1.5°
>   (PASS). Hard iron 0.05 / 0.15 after the turn: heading **0.11° / 1.1°** (3.5° / 10° without).
> - Live (`testing/results/live_calturn/TIERB.md`, 3 trials each, all survived): hover true
>   position error 0.25 → **0.01 m**, est tilt 0.64 → 0.01°, heading 5.75 → 0.02°; hover_ct /
>   box_ct heading 0.06–0.07°.
> - Known remaining: (a) without a turn, heading is honestly uncertain and model errors at
>   hover can move it (in-flight accel-bias jump: 6.5° heading, pos 0.43 m; was 2.6°/0.19 m);
>   (b) stale-GPS recovery 1–2 s → 3.5–5.5 s (WARN); (c) attitude NEES now ≪1 everywhere: the
>   filter is *under*confident now that errors are ~0.02°, so the live attitude Q (1e-7) is
>   the next thing to look at.

> **Harder scenarios, 2026-10-02** (GPS latency offline + live realistic sensors; filter = in-flight calibration version).
> - **New fault `gps_latency` (100/200 ms)**, `testing/results/gps_latency/`: harmless at hover and in gentle flight,
>   but circle_fast goes 0.15° → 1.3° / 2.8° tilt RMS (5.3° max, 9 GPS resets at 200 ms). The EKF compares a
>   150–200 ms-old GPS velocity with *now*, and under sustained acceleration the difference (a × latency ≈
>   0.35–0.5 m/s) reads as attitude error.
> - **Live sensor degradation** (`PID/sim_degrade.py`, off by default): `SIM_REALISTIC=1` applies the
>   `combined_realistic` set in the bridge (controller flies on it too), `SIM_GPS_LATENCY_MS=N` delays fixes.
>   `run_live.py` gained `--tag/--env/--cal-turn`. 30 live trials, all with the calibration turn:
>
>   | condition | hover | box | yaw_steps | stops | circle_fast |
>   |---|---|---|---|---|---|
>   | clean (`live_clean_ct`) | – | – | 3/3 | 3/3 | 3/3 |
>   | realistic + 150 ms latency (`live_real_lat_ct`) | 3/3 | 3/3 | 3/3 | 3/3 | **0/3** |
>   | realistic, no latency (`live_real_ct`) | – | – | – | **2/3** | 3/3 |
>
>   Clean: tilt error 0.05–0.15°. Realistic, gentle missions: est position ~0.9 m RMS (= GPS noise), constant
>   −0.2° pitch (pinned 0.05 m/s² accel bias / g, as predicted).
> - **Failures, diagnosed from the logs** (replay reproduces stops t3 and circle_fast t1 exactly, so they are pure
>   estimator failures; all four are in the suite as regression recordings, `testing/results/live_crash_regressions/`,
>   via `run_suite.py --extra <csv>`):
>   1. circle_fast + latency (t2, t3): heading error builds to −4…+8° as speed reaches 3 m/s, then GPS
>      rejections → lockout reset → divergence. The latency mechanism above, now closed-loop with noise.
>   2. circle_fast + latency (t1): host loop stalls up to **2.1 s** (36 > 50 ms) during the calibration turn; the
>      stall inflation opens P_att to ~10°, and the next *delayed, noisy* GPS velocity is mapped into tilt
>      through the cross-covariance (3° → 40° in 3 s). Stall handling + latency interact badly.
>   3. stops, realistic, no latency (t3): 84 loop stalls (max 0.35 s) plus a mag lockout (437 rejections) during
>      3 m/s reversals, then GPS rejections and two lockout resets at t=55–56 s. Tilt error goes 2° → 24° in
>      1 s. The GPS reset repairs position/velocity, not the attitude damage, so the controller flies on it.
> - **Next filter work, in priority order**: (a) **latency compensation**: fuse each GPS fix against the state
>   at the time it was measured (keep a ~0.5 s ring buffer of predicted pos/vel; residual = fix − buffered
>   state, correction applied now). This is the standard "delayed fusion" PX4's EKF2 uses. (b) After a stall,
>   don't let GPS-velocity residuals rotate attitude more than the coasted rotation could have
>   (e.g. cap the P_att inflation or skip GPS attitude coupling for ~0.5 s). (c) GPS-reset hygiene: after a
>   lockout reset, check attitude consistency rather than trusting it.

> **Latency + stall fixes — IMPLEMENTED 2026-10-02** (`FINAL_gps.py`; compare `testing/results/candidate_latency/`):
> - **Delayed GPS fusion**: a ~1.5 s history of predicted pos/vel; each fix is compared with the state at
>   `now − gps_latency`, and the correction is applied now and to the history. The lockout reset carries the
>   fix forward the same way. The latency comes from the sensor object (`gps_latency`: the bridge uses
>   `EKF_GPS_LATENCY_MS`, else the simulated latency; recordings carry it in `<csv>.meta.json`). With latency 0
>   it is bit-identical to before. circle_fast with 200 ms latency: 2.82°/5.25° tilt RMS/max and 9 resets →
>   **0.22°/0.59°, 0 resets** (no-latency level: 0.15°/0.42°). The `gps_latency(ekf_latency_ms=0)` row keeps
>   the uncompensated case as a reference.
> - **Stall handling, revised**: roll/pitch rates and the acceleration are coasted with a 0.25 s decay
>   (`STALL_COAST_TAU`), because tilt is bounded and those rates must reverse. The yaw rate is held for the
>   whole gap. Holding the full roll rate through a 1.1 s stall had added 15° that never happened (live crash
>   t1). The uncoasted roll/pitch rotation is added to P about that axis only. For 1 s after a stall
>   (`STALL_GPS_ATT_HOLD_S`) GPS corrects only pos/vel.
> - Live re-test: circle_fast realistic + 150 ms latency **3/3** (was 0/3), est tilt 0.6° RMS / 1.8° max;
>   stops realistic **3/3**. The host had no stalls during these runs; stalls are covered by `imu_gap` and the
>   crash replays.
> - Crash replays: circle_fast t2/t3 → WARN. circle_fast t1 holds ~1° through the stall burst where it used to
>   reach 19–40°; the row still grades F because the recording itself contains the real tumble. **stops t3 is
>   not fixed**, and it's a different problem: IMU *delivery* stalled (four 101 ms `wait_for_imu` timeouts,
>   then bursts of queued messages 1–4 ms apart). The EKF integrates by arrival time, so ~100 ms of motion
>   per burst is lost. Fix: integrate on the IMU message's own timestamp (on hardware, the IMU FIFO
>   timestamps). Not done yet.
> - Suite, all 361 runs: excluding NEES 299 PASS / 40 WARN / 22 FAIL; no graded regressions vs the previous
>   filter.

> **IMU timestamps — IMPLEMENTED 2026-10-03** (fixes the stops_real_ct_t3 failure mode):
> - `gz_bridge.py` queues **every** IMU sample with Gazebo's `header.stamp` (`drain_imu()`, `use_imu_sample()`,
>   `get_imu_time()`). `run_sim.py` steps the EKF once per queued sample, then the controller once. `FINAL_gps.py`
>   takes `dt` from the sample stamps (`self._clock`) when the sensor provides them. SENSOR_LOG now writes one row
>   per EKF step with `t` = IMU stamp, `tw` = wall time and `tt` = truth-pose stamp (used by `metrics.truth_arrays`).
>   `EKF_IMU_TIMESTAMPS=0` restores the old latest-sample/arrival-time loop; harnesses that never drain the queue
>   are unchanged. Test hook: `SIM_LOOP_STALL=prob,ms` freezes the loop on purpose.
> - Live check: sim-time steps exactly 4.00 ms on all 18,742 rows (77 bunched arrivals handled); replay matches
>   the live estimate to 0.003° / 1.7 cm.
> - A/B under injected 150 ms stalls (~1/s), stops with realistic sensors, 3 trials each: old loop lost ~2,850
>   samples and coasted 76 gaps, est tilt 0.70° / 2.48° and heading 1.41°. **New loop processed every sample (0
>   gaps), 0.43° / 1.32° and 0.71° — the same as stall-free flight.** circle_fast with realistic sensors + 150 ms
>   latency + stalls: 3/3, 0.62° / 1.98°. (The injected stalls didn't crash the old loop either; the original
>   crash had 4–9 stalls/s plus delivery bursts.) The stall-coasting code stays as a backstop for genuinely lost
>   samples (`imu_gap` faults, hardware dropouts).
> - Offline suite unchanged (replay never used arrival time): bit-identical to `candidate_latency`.

> **Hardware prep, 2026-10-03/04:**
> - **Barometer**: 19th EKF state `baro_bias` (appended; all older indices unchanged), baro fused only on new
>   samples, gate + re-reference on lockout; `gz_bridge.py` reads Gazebo's `air_pressure` (50 Hz) as height above
>   the boot reading; `SIM_REALISTIC` adds 0.3 m noise + 0.3 m/√min drift; `EKF_USE_BARO=0` for A/B. Live, realistic
>   sensors, 3 trials each: altitude-estimate error 0.48 → **0.13 m RMS** (max 0.92 → 0.28), true altitude tracking
>   0.55 → **0.14 m** (hover/box). Offline suite bit-identical (old recordings have no baro).
> - **Timing**: `testing/bench_timing.py` — run on the Pi. Dev laptop: 0.17 ms/step EKF+controller (4 % of a core).
> - **Shadow mode**: `testing/ulog_to_csv.py` (PX4 .ulg → suite csv, FRD/NED → FLU/ENU, dwell, mag scale + field
>   direction from data, `EKF2_GPS_DELAY`) and `testing/suite/px4_shadow_flight.sh` (PX4 flies itself in SITL with
>   `SDLOG_PROFILE=3`, `SDLOG_MODE=1`). Validated end to end in SITL vs ground truth: your EKF tilt 0.25° / heading
>   0.75° / pos 3 cm; PX4 EKF2 0.03° / 5.6° / 4 cm. Found: **mag is fused every IMU step even without a new reading**
>   (PX4 mag ~14 Hz → ~18× over-fusion). Plan in `SHADOW_MODE.md`.
> - **Failsafes** (`PID/failsafe.py`, wired into `run_sim.py`): HealthMonitor NORMAL → LAND / FALLBACK → DISARMED
>   (NaN or tilt σ > 20° → FALLBACK on a backup estimator = truth in sim / PX4 on hardware; GPS lost 3 s, ≥3 GPS
>   resets in 10 s, geofence 50 m / 15 m, loop stall 0.5 s, mission end → LAND; touchdown = low height OR
>   "commanded down but not descending"); no-GPS landing descends level (`cascade.step` `level_xy` flag, opt-in);
>   Watchdog thread cuts motors after 1 s of silence; any exception → motors off. Test hooks `SIM_GPS_LOSS_AT`,
>   `SIM_NAN_AT`, `SIM_HANG_AT`. Live: normal / NaN / GPS-loss (after the level-landing fix: 2/2, tilt ≤ 0.4°,
>   disarmed at 0.00 m) / hang (watchdog cut in 1.02 s) all behave; no false triggers on circle_fast or stops with
>   realistic sensors + latency. The first GPS-loss version tipped over at touchdown and disarmed at 4.5 m true
>   height — fixed by the level descent + stuck-detection.

## 5. Symptom → cause lookup (extended)

| Symptom | Likely cause | Knob / change to consider |
|---|---|---|
| Constant tilt/heading offset from t=0, `cal_ideal` row clean | calibration split noise-driven + biases pinned (F1) | `calibration.py:122-123` Q/P0, averaging/TRIAD for dwell-only |
| NEES_att ≫ 1 in `none`, ≪ 1 in `cal_ideal` | P never includes calibration ambiguity | seed P_att with bias-ambiguity variance (F1.2) |
| `mag_rejected` in the thousands, error grows | mag gate lockout, overconfident P, no recovery | mag lockout → inflate P_att; |B|/dip sanity check (F2) |
| Error only after yawing, worse without GPS; gyro-bias estimate jumps | pinned body-frame biases vs rotating world (F3) | F1; rotation-gated bias learning; lower gyro-bias Q |
| Tilt ≈ b/g after an in-run accel bias | accel bias pinned (F4) | small accel-bias Q |
| Error step after a loop stall, then lockout | `MAX_DT` clamp without P inflation (F5) | inflate P by true dt |
| Repeated `EKF_GPS_RESET` with a frozen receiver | stale fix accepted, reset to stale value (F6) | freshness check → `None` |
| NEES ≫ 1 (att), errors fine in `none` | P too small: attitude Q too low | live `Q[0:3,0:3]` (1e-7) — *not observed; in-flight error is tiny* |
| Tilt drifts only during GPS loss | no tilt reference without GPS | only happens under rotation (F3) — fix F1 first |
| Gate rejects many good fixes, resets | overconfident P or R_gps too small | not observed except stale/gap cases |
| Large error only in fast/aggressive missions | linearization / tilt observability | *refuted* — circle_fast `cal_ideal` 0.15° |
| NaN / divergence | numerical | none seen in 267 runs |
| Vehicle tumbles past yaw 90° even in ORACLE | controller world-frame error (F8) | `cascade.py:145` |

## 6. Instrumentation changes (plan §1 exceptions) + proof behaviour is unchanged

`FINAL_gps.py` only:
1. `self.stats` counters (`gps_updates/rejected/resets/no_fix`, `accel_updates/rejected`,
   `mag_updates/rejected`) and `self.last_d2` (last gate distances). Read-only, set next
   to the existing gate decisions.
2. `get_gps()` may return `None`: the GPS update is skipped and `last_gps_time` is left
   alone, so the next tick polls again. The block is otherwise unchanged.

Proof: the full per-step trace (q, pos, vel and all 324 entries of P) for the reference
flight `ref_sensors_live1.csv` (4,512 steps) is **bit-identical** with the original
`FINAL_gps.py` (git stash) and the patched one. Attitude RMS on the reference is still
**0.14 / 0.12 / 0.20°** through both `replay_ekf.py` and the suite (`VALIDATION.md` §1).

Other code changes (test infrastructure, no filter/controller behaviour change):
- `run_sim.py`: `MISSION=` (setpoint schedule from `testing/suite/missions.py`, clean
  exit after the mission), `MAX_VEL_XY`/`MAX_TILT_DEG` overrides (applied to a copy of
  LIMITS; ORACLE recordings only), extra `SENSOR_LOG` columns (mission time, setpoint,
  live EKF estimate), and `CalRecorder`, a passthrough that saves the calibration reads to
  `<log>.cal.csv`. Defaults are unchanged: plain `python3 run_sim.py` behaves as before.
- `testing/replay_ekf.py`: refactored into importable `load_rows`/`Replay`/
  `ArrayReplay`/`run_replay` (shared with the suite). It now calibrates on the
  `.cal.csv` samples when present (`CAL_ROW0=1` restores the old behaviour) and patches
  only `FINAL_gps.time`, not the global `time` module. Same CLI and output.
- `.gitignore`: `testing/data/`.

## 7. Deviations from the plan (and why)

1. **Replay calibration uses the live calibration samples.** The old replay calibrated
   on row 0 × 200, which is noise-free and hides the largest error source (F1). Faults
   are applied to calibration rows too, so power-on biases/noise hit calibration as on
   hardware. `from_cal=False` variants model in-run shifts.
2. **NEES pos/vel graded only on runs with injected GPS noise** (`gpsN`, `REAL`), for F9.
   They're still reported everywhere.
3. **Missions use setpoint lead** (`p + (v + 2a)/kp_pos`): `pos_xy` kp=0.15 would otherwise
   fly a 3 m circle as ~1 m. Achieved dynamics are in `RECORDINGS.md`: circle_fast 12.9°
   mean / 16.5° peak tilt at 2.3 m/s (plan: ~17°, 3 m/s); circle_slow 1.08 m/s, 2.3°;
   stops 9.1°/13.8°; box only 1.0°/2.1° (5 m in 8 s isn't aggressive with these gains).
4. **yaw_steps / yaw_spin truncated** (F8): scored to t=24 s (0→90° step + 4 s hold) and
   t=15 s (0→135°). Fault windows start at the profile start on these, and windows that
   outlast the data skip recovery grading. yaw_steps not flown in Tier B.
5. **patrol_long runs a 10-fault subset** by default (`PATROL_FAULTS`, `--full-patrol` for
   all). Its recording contains natural 0.6–1.1 s loop stalls at t=59–65 s (host load while
   recording). Kept deliberately (F2/F5); `imu_gap` reproduces them on the other missions.
6. **Extra faults**: `imu_gap` (0.2/1 s stall), `cal_ideal` (counterfactual noise-free
   dwell), in-run `gyro_bias`/`accel_bias` variants.
7. **Runtime**: full Tier A takes **~9–10 min** (546–622 s, 267 runs, 10 workers) on this machine, not
   <5 min. ~0.28 ms per EKF step in pure Python, and the time is inside `DroneEKF.step`
   (profiled). `--missions/--faults` subsets are fast (a 60 s mission × 1 fault ≈ 5 s).
8. Recovery thresholds: max(1.5 × no-fault p95, floor), floors 0.2° tilt / 0.3° yaw /
   0.2 m (`metrics.RECOVERY_FLOOR`), so "recovered" doesn't demand sub-noise accuracy.
   The ~0 no-fault errors in Gazebo would otherwise make it unattainable.
9. Velocity truth = finite difference of the ~50 Hz pose topic, 0.1 s moving average.
   Truth attitude/position are interpolated between pose updates (pose latency vs IMU
   fitted as ~0 ms on circle_fast/stops, so no shift).

Validation (§3.8), all PASS (`VALIDATION.md`): (1) reference 0.14/0.12/0.20°; (2)
`gyro_bias(0)` ≡ `none` bit-identical, and every fault changes only its own channels
(table + `fault_channels.png`); (3) NEES of 10k N(0,P) draws = 0.994; (4) determinism:
two full independent runs on the same code/data give **byte-identical** `results.json`, and
`compare.py` reports 0 improved / 0 regressed / 0 grade changes for the baseline vs itself
(`COMPARE_self.md`) and vs the re-run (`COMPARE_rerun.md`).

## 8. How to re-run

```powershell
# record (WSL Gazebo, fresh restart per mission, ORACLE=1) - ~25 min for all 9
powershell -File testing/suite/record.ps1                    # or: record.ps1 yaw_steps yaw_spin
python testing/suite/validate.py --recordings --out <dir>    # RECORDINGS.md
python testing/suite/validate.py --out <dir>                 # tooling checks, VALIDATION.md

# Tier A - offline, deterministic (~9-10 min all; subsets in seconds)
python testing/suite/run_suite.py --out testing/results/<name>
python testing/suite/run_suite.py --missions circle_fast --faults none combined_realistic --no-plots

# compare a filter change against the baseline
python testing/suite/compare.py testing/results/baseline_86193e6/results.json testing/results/<name>/results.json
#   -> <name>/COMPARE.md ; --fail-on-regression for a nonzero exit

# Tier B - closed loop on the EKF, 3 trials each (~100 s/trial); don't run heavy CPU alongside
python testing/suite/run_live.py --missions hover box circle_slow takeoff_land --out testing/results/<name>
python testing/suite/run_live.py --score-only --out <dir>     # rescore existing trial csvs

# one flight by hand (inside WSL), or the old single-file replay
bash testing/suite/fly.sh <mission> testing/data/x.csv testing/data/logs/x.log [ORACLE=1 ...]
python testing/replay_ekf.py testing/data/flight_hover.csv    # MAG_REMAPPED=0 for new recordings
```

Suggested order for the owner's changes, since each is independently checkable with
`compare.py`: F1 (calibration) → F5 (dt/P inflation) → F2 (mag lockout) → F6 → F3/F4
(bias learning), and F8 in the controller whenever convenient (then re-record the yaw
missions).
