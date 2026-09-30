# EKF stress-test suite — plan for a handoff agent

## 0. Read this first

**Your job:** build a repeatable test suite that measures how the EKF (`state estimation/FINAL_gps.py`) performs in scenarios harder than hover, run it once to produce a **baseline**, and write a report that tells the owner *what about the filter needs to change and why*. You are **not** fixing the filter. The owner makes filter changes themselves (they want to understand and write that code — explain, don't implement). The suite must be re-runnable after each of their changes and say, per scenario, whether it got better or worse.

**Read before doing anything:** `HANDOFF.md` (top section "2026-09-30 session" in full; environment + standard test procedure + known gotchas sections), `README.md` (last two sections), `testing/replay_ekf.py`, `run_sim.py`, `PID/gz_bridge.py`. `FINAL_gps.py`'s live-mode block in `DroneEKF.__init__` (everything under `if sensors is not None`) is the current tuning you're evaluating.

**Current state (so you know what "good" means today):** 3/3 live 60 s hovers level at 2.00 m on the EKF. Replay of a maneuvering-then-crashing flight scores attitude RMS 0.14 / 0.12 / 0.20° (roll/pitch/yaw). That's the *only* dynamic evidence so far — everything else in this plan is untested.

**Known weak spots to target** (hypotheses, not facts — the suite should confirm or refute each):
- **H1 — tilt during GPS loss.** The accel gravity correction is OFF in live mode, so tilt is corrected only through GPS velocity. With GPS gone, nothing corrects tilt; drift rate is unknown.
- **H2 — pinned biases.** Accel/mag bias states are pinned (Q 1e-12, P 1e-6) because Gazebo's IMU has zero bias. A real IMU has bias; the pinned filter can't learn it.
- **H3 — heading from calibration.** Calibrated heading varies −4.7…+1.4° run to run (dwell-only calibration). Unknown whether yaw rotation in flight converges it out.
- **H4 — sustained maneuvers.** Tilt observability via GPS velocity is weakest under sustained acceleration (circles, hard stops). Only one short maneuvering flight has been evaluated.
- **H5 — overconfidence.** Attitude Q was cut 1e5x; if P is now too small the filter is overconfident — errors exceed what P claims, gates reject good data, and lockouts follow. Never measured.
- **H6 — GPS realism.** Gazebo GPS is near-noiseless; `R_gps` assumes 2.5 m / 5 m std. Behavior under realistic GPS noise/outliers/low rate is unknown.

## 1. Rules

- **Do not change filter behavior** (`FINAL_gps.py`, `calibration.py`) or controller gains (`run_sim.py` GAINS/LIMITS defaults). Pre-approved exceptions, each must be listed in the report:
  1. **Read-only instrumentation** in `FINAL_gps.py`: counters/attributes (e.g. `self.stats = {"gps_rejected": 0, ...}`) and exposing values already computed (residuals, d²). No change to any state/P/gain math.
  2. **"No GPS fix" support:** if `get_gps()` returns `None`, skip the GPS update for that tick. Needed for honest dropout tests (today a dropout would have to be faked as a *stale* fix, which is a different, wrong test). Identical behavior whenever GPS is present — prove it by showing the baseline replay numbers are unchanged before/after.
  - Anything else → stop and ask the owner.
- **Determinism:** every fault injection is seeded; the offline suite must give bit-identical results on re-run.
- **Environment gotchas** (from HANDOFF): WSL2 `Ubuntu-24.04`; call `wsl` from PowerShell with a script file (Git Bash mangles `/mnt/...` paths); run Gazebo headless (`HEADLESS=1`); kill only `bin/px4` after launch; fresh full restart (kill `gz sim` too) before every live run; the `gz sim` server dies often — check before each run.
- **Don't commit** unless the owner asks. Recordings/logs are gitignored; ask before committing any data.
- Record the git commit hash (and "dirty" flag) of the code under test in every results file.

## 2. Suite architecture

```
testing/suite/
  missions.py        setpoint schedules: mission(t) -> {"pos": np.array, "yaw": float}
  record.ps1/.sh     flies each mission in Gazebo and saves a recording
  faults.py          seeded sensor corruptions applied to a recording at replay time
  metrics.py         error/consistency/recovery metrics from a replay trace
  run_suite.py       offline tier: every (recording x fault) -> results.json + REPORT.md + plots
  run_live.py        closed-loop tier: flies missions on the EKF in Gazebo, N trials each
  compare.py         candidate results.json vs baseline -> regression/improvement table
testing/data/        recordings (flight_<mission>.csv), gitignored
testing/results/<timestamp>_<githash>/   results.json, REPORT.md, plots/
```

Two tiers:
- **Tier A — offline replay (the workhorse).** Recordings are flown once in Gazebo, then every EKF version is scored against the *same* data + faults. Fast (target: full tier < 5 min), deterministic, fair comparison. Answers "is the estimate right?"
- **Tier B — closed-loop live.** The EKF actually flies the mission. Slow (~2 min/trial), noisy, needs repeats. Answers "does estimator error make the vehicle fly badly or crash?" Run after Tier A, and on any candidate that passes Tier A.

## 3. Phase 1 — tooling

### 3.1 Missions (`missions.py`) + `run_sim.py` hook
Add `MISSION=<name>` env var to `run_sim.py` (default = today's fixed hover, so plain runs are unchanged). Setpoint = `missions.get(name)(t_since_takeoff)`. Also add `MISSION_END` behavior: after the mission's duration, hold last setpoint for 5 s then exit cleanly (so the log has a defined end).

Respect current limits (`max_vel_xy` 1.5 m/s, `max_tilt` 15°) — a setpoint moving faster than the vehicle can follow just saturates. For the aggressive set, allow env overrides `MAX_VEL_XY`, `MAX_TILT_DEG` in `run_sim.py` (default unchanged), used only for ORACLE recordings.

| Mission | Duration | Profile | Targets |
|---|---|---|---|
| `hover` | 60 s | hold (0,0,2) | control/sanity |
| `box` | 60 s | 5 m square at 2 m, 8 s legs with 4 s holds | H4 transients, tilt at max |
| `circle_slow` | 60 s | r=3 m, v=1.0 m/s | H4 sustained accel (~0.3 m/s², ~2°) |
| `circle_fast` | 60 s | r=3 m, v=3 m/s (needs MAX_VEL_XY=3.5, MAX_TILT_DEG=25) | H4 sustained ~3 m/s², ~17° tilt |
| `stops` | 45 s | 8 m dashes with abrupt reversals (aggressive limits) | H4 hardest transients |
| `yaw_steps` | 60 s | hover, yaw 0→90→180→−90→0°, 10 s each | H3, mag under rotation |
| `yaw_spin` | 60 s | hover, yaw rate 30°/s continuous | H3, mag/gyro consistency |
| `patrol_long` | 600 s | box repeated | slow drift, P health over time |
| `takeoff_land` | 40 s | 2 m hover, descend to touchdown, sit 5 s, re-takeoff | ground contact, GPS reset path |

### 3.2 Recording (`record`)
Fly each mission with **`ORACLE=1`** (controller flies on ground truth) + `SENSOR_LOG=testing/data/flight_<mission>.csv`. Oracle guarantees the recorded trajectory is the intended one regardless of how good the EKF is, so every EKF version is scored on identical, clean flights. Fresh Gazebo restart per mission. Validate each recording: truth attitude/position follow the mission, no crash (true tilt < 45°, z > 0.3 m while airborne except `takeoff_land`), sample rate ~250 Hz.

CSV columns (from `run_sim.py`): `t, gx..gz, ax..az, mx..mz, px,py,pz, vx,vy,vz, tqw..tqz, tpx..tpz`. All ENU world / FLU body. **New recordings have raw mag → replay with `MAG_REMAPPED=0`** (old ones from 2026-09-30 need `=1`; see `replay_ekf.py`).

Known gap: GPS columns are the bridge's latest cached value logged every IMU tick (Gazebo NavSat publishes at 30 Hz; the EKF consumes at 5 Hz). Fine for replay.

### 3.3 Faults (`faults.py`)
Each fault: `name`, params, `seed`, and `apply(rows) -> rows` (or a per-tick sensor wrapper). Magnitudes are chosen from typical real hardware (cheap MEMS IMU + consumer GNSS) — state the source/assumption in the report.

| Fault | Parameters (run each level) | Targets |
|---|---|---|
| `none` | — | baseline per mission |
| `gps_dropout` | 5, 15, 30 s; start at mission's most dynamic segment; uses `None` = no fix | H1 |
| `gps_stale` | same windows, but GPS frozen at last value (what the current EKF would see from a hung receiver) | H1, robustness |
| `gps_noise` | white 1.5 m horiz / 3 m vert + 0.1 m/s vel, plus random-walk wander 0.5 m/√min | H6 |
| `gps_outliers` | 1% of fixes offset 10–50 m | H6, gate |
| `gps_rate` | 1 Hz fixes only | H1/H6 |
| `gyro_bias` | constant 0.2, 1.0 °/s per axis; and drift 0.5 °/s over 60 s | gyro-bias state (not pinned) |
| `accel_bias` | 0.05, 0.2, 0.5 m/s² on x and on z | H2 |
| `mag_bias` (hard iron) | 0.05, 0.15 (unit-field fraction), fixed body vector | H2, H3 |
| `mag_interference` | 5 s burst, 0.3–1.0 magnitude, e.g. motor-current-correlated | gate, yaw holdover |
| `imu_noise` | gyro 0.005 rad/s, accel 0.05 m/s² white (≈5–8x Gazebo) | H5 |
| `imu_spikes` | 0.1% samples, gyro ±5 rad/s, accel ±30 m/s² | robustness |
| `imu_dropouts` | drop 2% of samples (dt doubles) | timing robustness |
| `combined_realistic` | gps_noise + imu_noise + gyro_bias 0.2 + accel_bias 0.05 + mag_bias 0.05 | "real hardware" headline number |

### 3.4 Metrics (`metrics.py`)
Computed per (mission, fault) over the airborne window (exclude calibration and the first 2 s after takeoff):

- **Attitude error** per axis (roll/pitch/yaw, deg; body-frame small-angle vector from `q_true⁻¹ ⊗ q_est`, same as `replay_ekf.quat_err_deg`): RMS, p95, max. Report **tilt** (√(roll²+pitch²)) separately from yaw — they fail for different reasons.
- **Position error** (m, horiz / vert) and **velocity error** (m/s): RMS, p95, max. Velocity truth: finite-difference of truth position (smoothed) — say so in the report.
- **Fault-window metrics** (dropout/interference/outlier faults): error at fault end, **drift rate** during fault (deg/s, m/s — linear fit), **recovery time** after fault ends (time until error back within 1.5x the no-fault p95), max transient on recovery.
- **Consistency (NEES)** — the key filter-health check (H5): for attitude, `e_attᵀ P_att⁻¹ e_att` (P rows/cols 0:3), same for velocity (6:9) and position (9:12). Report mean NEES / dof and fraction of samples outside the 95% chi² bounds. Mean ≈ 1 = honest; ≫1 = overconfident (P too small — Q too low or R too low); ≪1 = underconfident. Needs P per tick from the replay loop (already accessible as `ekf.P`).
- **Gate/robustness counters:** GPS/mag/accel gate rejections, `EKF_GPS_RESET` count, any NaN/inf in state or P (instant FAIL).
- **Bias estimates** vs injected truth (for bias faults): final error and convergence time for gyro/accel/mag bias states.

### 3.5 Grading
Each (mission, fault) gets **PASS / WARN / FAIL** per metric and an overall (worst of). Thresholds (owner may retune — keep them in one dict at the top of `metrics.py`):

| Metric | PASS | WARN | FAIL |
|---|---|---|---|
| Tilt RMS | < 1° | < 3° | ≥ 3° |
| Tilt max | < 3° | < 8° | ≥ 8° |
| Yaw RMS | < 2° | < 5° | ≥ 5° |
| Horiz pos RMS (no GPS fault) | < 0.3 m | < 1 m | ≥ 1 m |
| Horiz pos RMS (GPS noise faults) | < 1.5x GPS noise σ | < 3x | ≥ 3x |
| Tilt drift during GPS dropout | < 0.5°/s | < 2°/s | ≥ 2°/s |
| Recovery time after fault | < 3 s | < 10 s | ≥ 10 s / never |
| NEES mean / dof (att, vel, pos) | 0.3–3 | 0.1–10 | outside |
| GPS resets (no-GPS-fault runs) | 0 | 1 | > 1 |
| NaN / divergence | never | — | any |

Rationale for the tilt thresholds: the controller holds a tilt setpoint; 3° of tilt error ≈ 0.5 m/s² of unwanted horizontal acceleration the position loop must fight; 8°+ is where the old tumbles began.

### 3.6 Suite runner (`run_suite.py`)
`python testing/suite/run_suite.py [--missions ...] [--faults ...] [--out testing/results/...]`
- Runs every recording × fault through `DroneEKF` (reuse `replay_ekf.py`'s `Replay` source / patched `time.time` — factor shared code into an importable function rather than duplicating).
- Outputs `results.json` (schema below), `REPORT.md`, and `plots/<mission>__<fault>.png`: truth vs estimate (roll/pitch/yaw, pos), error with ±3σ band from P, fault window shaded.
- Also prints a one-screen summary grid: missions as rows, faults as columns, cells PASS/WARN/FAIL.

`results.json` schema (one entry per run):
```json
{
  "meta": {"git": "abc1234", "dirty": true, "date": "...", "suite_version": 1, "thresholds": {...}},
  "runs": [{
    "mission": "circle_fast", "fault": "gps_dropout", "params": {"duration_s": 15, "start_s": 20}, "seed": 1,
    "metrics": {"tilt_rms_deg": 0.8, "tilt_max_deg": 2.1, "yaw_rms_deg": 1.2, "pos_h_rms_m": 0.4,
                "tilt_drift_deg_s": 0.3, "recovery_s": 2.1, "nees_att": 1.4, "nees_vel": 0.9,
                "nees_pos": 1.1, "gps_rejected": 3, "gps_resets": 0, "nan": false, "...": "..."},
    "grades": {"tilt_rms_deg": "PASS", "...": "..."}, "overall": "PASS",
    "hypotheses": ["H1"]
  }]
}
```

### 3.7 Compare (`compare.py`)
`python testing/suite/compare.py baseline/results.json candidate/results.json` → `COMPARE.md`: per (mission, fault), each key metric's baseline → candidate, % change, and a flag: **REGRESSION** (grade worsened, or metric > 20% worse and > a small absolute floor), **IMPROVED**, or unchanged. Top of the file: counts of improved / regressed / grade changes, and a list of every regression. This is what the owner runs after each filter change.

### 3.8 Validate the tooling before trusting it
1. Replay the existing reference flight through the new runner with fault `none`: `git show 33be40e:sensors_live1.csv > testing/data/ref_sensors_live1.csv`, `MAG_REMAPPED=1`, first 12 s → must reproduce attitude RMS **0.14 / 0.12 / 0.20°**.
2. Fault sanity: `gyro_bias` at 0 magnitude == `none` exactly; each fault visibly changes only the channels it should (plot one example of each).
3. NEES code sanity: feed the metric synthetic errors drawn from N(0, P) for a known P → mean NEES/dof must come out ≈ 1 (±0.05 over 10k samples). This tests the metric code only — don't expect the synthetic (`sensors=None`) EKF to read ≈ 1, its Q is deliberately inflated, so it will legitimately read ≪ 1.
4. Determinism: run the suite twice, `results.json` identical.

## 4. Phase 2 — Tier A baseline
Record all missions (§3.2), run the full offline suite on the current code, save as `testing/results/baseline_<githash>/`. This is the reference every future filter change is compared against.

## 5. Phase 3 — Tier B closed loop
`run_live.py`: for each mission in {hover, box, circle_slow, yaw_steps, takeoff_land} (skip the aggressive-limit ones unless Tier A passes them), fly **on the EKF** (no ORACLE), **3 trials** each, fresh Gazebo restart per trial, `SENSOR_LOG` on (so failures can be replayed offline). Per trial: survived (no true tilt > 45° / no unplanned ground contact), true position tracking error vs setpoint (RMS, max), EKF-vs-truth error (same metrics as Tier A). Report mean and spread across trials — never a single trial as evidence (HANDOFF's recurring lesson).

Optional stretch (ask owner first): wind — the default world has no wind; adding Gazebo's `WindEffects` system to a *copy* of the world file tests rotor-drag-driven accel disturbances.

## 6. Phase 4 — the report (`REPORT.md`)
Write it for two readers: the owner (wants to understand *why* and decide what to change) and a future agent (wants unambiguous, machine-checkable conclusions). Structure:

1. **Headline** (≤ 5 lines): overall grade counts; the single most important weakness found; `combined_realistic` result per mission.
2. **Summary grid**: missions × faults, PASS/WARN/FAIL, Tier B survival rates.
3. **Hypotheses verdicts**: H1–H6 each → CONFIRMED / REFUTED / INCONCLUSIVE, with the specific numbers and plot that decide it.
4. **Findings, ranked by severity** — one block each:
   - *Symptom* (metric, scenario, number, plot link)
   - *Mechanism* — which part of the filter causes it, with file:line (e.g. "`self.use_accel_correction = False` → no tilt reference during dropout")
   - *Candidate changes* (1–3), each with expected effect and the risk (what it might break — cite the scenario that would catch it). Describe; don't implement.
   - *Evidence needed to verify a fix*: which suite rows should improve, which must not regress.
5. **Symptom → cause lookup** (use and extend):

| Symptom | Likely cause | Knob / change to consider |
|---|---|---|
| NEES ≫ 1 (att), errors fine in `none` | P too small: attitude Q too low | live `Q[0:3,0:3]` (1e-7) |
| Tilt drifts only during GPS loss | no tilt reference without GPS | conditional accel correction (only when \|a\|≈g AND low rotation AND low GPS-vel change), or a drag-aware accel model |
| Yaw offset constant, not converging | heading absorbed by pinned mag bias / calibration | unpin mag bias only while yaw-rotating; calibration heading init |
| Error grows with injected accel/mag bias | biases pinned (H2) | live bias Q/P; observability-gated bias learning |
| Gate rejects many good fixes, resets | overconfident P or R_gps too small | R_gps, Q vel/pos, reset threshold |
| Large error only in fast/aggressive missions | linearization / tilt observability under sustained accel | Q attitude, GPS velocity weight |
| NaN / divergence | numerical — P asymmetric / not PSD | Joseph form everywhere, P symmetrization, P caps |

6. **Instrumentation changes made** (per §1 exceptions) + proof behavior was unchanged.
7. **How to re-run**: exact commands for record / suite / compare / live.

## 7. Done when
- [ ] Tooling validated (§3.8 all four checks pass).
- [ ] All missions recorded and validated.
- [ ] Baseline `results.json` + `REPORT.md` + plots committed-to-disk under `testing/results/baseline_<hash>/`.
- [ ] Tier B run with 3 trials per mission.
- [ ] Report answers H1–H6 with numbers, ranks findings, and gives candidate changes with the suite rows that would verify them.
- [ ] `compare.py` demonstrated: baseline vs itself → zero changes.
- [ ] `HANDOFF.md` gets a short dated section pointing to the suite, the baseline, and the headline findings.

## 8. When to stop and ask the owner
- Any change to filter math/tuning or controller gains seems necessary to proceed.
- A mission can't be flown even in ORACLE mode (controller limits) — propose limit changes, don't just apply them.
- A result contradicts the "current state" numbers in §0 by a lot (e.g. hover baseline no longer ~0.1°) — something in the environment changed; find out what before continuing.
- Tier B trials crash the physics server repeatedly.
