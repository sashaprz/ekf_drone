# Filter benchmark v2: your EKF vs PX4 EKF2 vs Madgwick vs Mahony

The first benchmark (`testing/benchmark_filters.py`) was biased in your EKF's favour. Its
recordings were flown by your own stack and used to debug the EKF; Gazebo's stock IMU is
near-ideal; EKF2 showed up on only one hover log; north came from ground truth; and there
was one trial per case. This pipeline fixes each of those:

| Problem | Fix |
|---|---|
| Home advantage | Every flight is flown by **PX4 itself** (its EKF2 + controller). None of this data has been used to develop your EKF. |
| EKF2 on different data | EKF2's numbers come from **PX4's own EKF2 replay** of the exact log the other filters read, including the fault-injected ones. On a clean log the replay matches live EKF2 to ≤ 0.06° after its first 2 s. |
| Ideal IMU | Each scenario is also flown with `x500_bench`, an x500 with a hobby-grade IMU: 0.29 °/s gyro noise, a random turn-on gyro bias (σ 0.5 °/s), Gauss-Markov bias wander, 0.1 m/s² accel noise and accel bias. PX4's EKF2 flies with it live. |
| North from truth | Gazebo's magnetic field is rotated onto the World Magnetic Model field. EKF2 uses that field and ignores the declination parameter while GPS is valid. Doing this removes the 2.7° mismatch behind the old "EKF2 heading error = 5.6°". |
| One trial | Each condition is flown 3 times, each from a fresh boot with new random noise and bias draws. Results are reported as 95% CIs, paired ratios, and a flight-level bootstrap. |
| Tuning on the test set | Madgwick/Mahony are scored with their reference default gains *and* with leave-one-trial-out cross-validated gains. |

## Pipeline

```
make_gz_model.py   (once)  writes ~/PX4-Autopilot/Tools/simulation/gz/models/x500_bench{,_base}
campaign.sh        flies scenarios x {x500_bench, x500} x trials -> ~/bench/flights/*.ulg
  fly_px4.sh         one fresh PX4 + Gazebo boot, runs px4_mission.py, copies the .ulg
  px4_mission.py     offboard setpoints: dwell, climb, 360° calibration turn, profile, land
  prepare.py         per flight x fault:
    ulog_faults.py     field alignment + fault injection, in place in the .ulg
    PX4 EKF2 replay    `make px4_sitl none` with replay=<log>, replay_mode=ekf2
    ulog_to_csv.py     -> testing/data/benchmark/<scenario>__<model>__t<k>__<fault>.csv
score.py           (Windows) replays your EKF / Madgwick / Mahony on every csv, scores all
                   four against ground truth -> testing/results/benchmark_v2/REPORT.md
```

Run it:

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Sasha/repos/python_drone/testing/benchmark/campaign.sh 3
python testing/benchmark/score.py
```

Both steps are resumable. The campaign skips flights and recordings that already exist,
and `score.py` caches each run in `runs.json`. To add trials, run `campaign.sh 5`.

**Scenarios** (`px4_mission.py`): hover, box (8 m at 2 m/s), circle_fast (r 5 m at 4 m/s),
figure8 (≈5 m/s, ±1 m altitude, yaw follows velocity), stops (5 m/s sprints with hard stops),
yaw_spin (60 °/s). The aggressive ones raise PX4's speed, acceleration and tilt limits.

**Faults** (`ulog_faults.py`, applied to `x500_bench` flights only; stock-IMU flights are
the clean control): none, gyro_bias_step, gyro_drift, accel_bias_step, gps_dropout (15 s),
gps_outliers, mag_bias, mag_interference, imu_spikes.

**Realistic GPS** (`gps_realistic`, applied to both IMUs, scored separately from the faults): this is
not a fault. Gazebo's GPS is within about 4 cm of truth, while each fix *reports* the accuracy of a
real receiver (eph 0.9 m, epv 1.8 m). So this condition adds error of that size: Gauss-Markov wander
(τ 60 s, 0.7 m per horizontal axis, 1.4 m vertical) plus fast noise. On the raw simulator GPS, a filter
that follows GPS closely looks far better on position than it would on real hardware. Use this
condition for the position and velocity comparison.

## Gotchas found while building this

- PX4's replay reader is extremely slow on `/mnt/c`: more than 120 s, versus 8 s from the Linux
  filesystem. `prepare.py` works in `~/bench/work`.
- Replay re-logs ground truth at only ~7 Hz; the flight log has 50 Hz. Truth is taken from
  the original log (`ulog_to_csv.py --truth-from`) and interpolated onto the IMU timeline.
  Holding it instead makes a correct estimate look 3–5° wrong in aggressive flight.
- `EKF2_MAG_DECL` / `EKF2_DECL_TYPE` overrides have no effect while GPS is valid
  (`mag_control.cpp` resets to the WMM field vector). That's why the field alignment is needed.
- PX4 SITL's shell spins on EOF. Never pipe its stdout into a Python buffer
  (python grew to 4.8 GB); write it to a file.
- Don't run a replay while a SITL flight is running: both are PX4 instances.
- PX4 backdates every GPS fix by `SENS_GPS0_DELAY` (110 ms), but Gazebo's GPS has no real
  latency. Measured against truth velocity, the data lines up with the *arrival* time to
  about 10 ms. Left alone, EKF2 fused GPS 110 ms early. The converter fused it 220 ms early,
  because it also told your EKF to delay it by another 110 ms. A timing error during a turn
  looks exactly like a heading error. `ulog_faults.py --gps-arrival-stamp` and
  `ulog_to_csv.py --gps-latency 0` fix it for both filters.
- **The SITL magnetometer is wrong whenever the vehicle tilts.** gz-sim's mag plugin uses a
  left-handed frame (gz-sim PR #2460). PX4's bridge swaps axes to compensate, which is
  correct only when level. At 15–30° tilt the logged field is about 35° off the true
  attitude. EKF2 rejects it then, with a mag test ratio above 1. The other filters fuse the
  bad readings or gate them out. `ulog_faults.py --fix-gz-mag` applies `(m_y, −m_x, m_z)`
  in FRD. This was the only one of 48 signed axis permutations that is consistent with
  ground truth, at about 1–3° residual at any tilt. Without the fix, declination learning
  learned the bug instead of the field.
