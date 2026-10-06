# Shadow mode: your EKF on real sensor data, before it flies anything

Target hardware: **Pixhawk-class flight controller running PX4** + **Raspberry Pi** companion.
Goal: measure how `state estimation/FINAL_gps.py` behaves on real sensors (noise, vibration,
GPS latency, magnetic interference) while PX4 flies the drone — no crash risk — using the same
replay/suite tooling the simulation work used.

## Stage 1 — offline shadow (no Pi needed) ← start here

PX4 already records every sensor at full rate in its `.ulg` log if told to. Fly normally on
PX4, then convert the log and replay your EKF on it.

**One-time PX4 parameters** (QGroundControl → Parameters, then reboot):

| Param | Value | Why |
|---|---|---|
| `SDLOG_PROFILE` | 3 | default set + "Estimator replay": IMU, mag, baro, GPS at **full rate** (default logs mag/baro at 5 Hz) |
| `SDLOG_MODE` | 1 | log from boot, so the log contains the vehicle at rest before arming (→ calibration dwell) |
| `EKF2_GPS_DELAY` | your receiver's latency (default 110 ms) | read by the converter/bridge for your EKF's delayed GPS fusion. **Only exists on PX4 ≤ 1.15** - newer PX4 dropped it; then 110 ms is assumed, or set `EKF_GPS_LATENCY_MS` |

**Before the first flights (bench):** PX4's own accel calibration (6 orientations), mag
calibration (rotate through all orientations, ideally also *compass-motor calibration* with
props off/motors on), gyro calibration. Your EKF uses PX4's calibrated outputs
(`sensor_combined`, `vehicle_magnetometer`), so these matter for it too. Your EKF still keeps
its own dwell calibration + in-flight mag-bias learning on top.

**Each flight:**
1. Power on, leave the vehicle still for ≥ 5 s (dwell), arm, fly. Include, if you can:
   a slow 360° yaw turn early (calibration turn), gentle translations, a hover, a landing.
2. Download the `.ulg` (QGC → Analyze → Log Download).
3. Convert and replay:
   ```bash
   python3 testing/ulog_to_csv.py flight.ulg testing/data/shadow/flight1.csv
   python3 testing/replay_ekf.py testing/data/shadow/flight1.csv          # quick look
   python3 testing/suite/run_suite.py --missions none --extra testing/data/shadow/flight1.csv
   ```
   `ulog_to_csv.py` converts PX4's FRD body / NED world into this project's FLU / ENU,
   picks the at-rest stretch as the calibration dwell, scales the magnetometer like
   `gz_bridge.py` does, **estimates the local magnetic-field direction from the data**, and
   writes `<csv>.meta.json` (field direction, GPS delay, rates). On hardware there's no ground
   truth: the reference is PX4's EKF2 estimate — a good but not perfect yardstick (in SITL it
   had a constant 5.6° heading offset from truth).

**Validated in SITL (2026-10-04):** `testing/suite/px4_shadow_flight.sh` flies PX4 itself in
Gazebo with these params; the converted log replayed through your EKF gave tilt 0.25° RMS,
heading 0.75°, position 3 cm vs ground truth (PX4's EKF2: 0.03° / 5.6° / 4 cm).
Rates seen: IMU 250 Hz, GPS 31 Hz, baro 20 Hz, mag 14 Hz.

**What to look at in the replay** (in order):
1. Frames: mag-field spread in `meta.json` (`mag_reference_spread_deg` should be ≲ 2°
   after a yaw turn — a large value means a frame/sign error or a bad compass calibration).
2. Attitude vs EKF2 during hover and during the yaw turn; heading after the turn.
3. Innovation/gate stats in the suite output (`gps_rejected`, `mag_rejected`, `baro_*`).
4. Vibration: accel noise in hover (`std` of `ax..az`) vs the 0.05 m/s² the suite assumes.
5. Then tune from real data: `Q_att` (NEES ≈ 1 against EKF2 is the target), `R_mag`,
   `BARO_STD`, `EKF2_GPS_DELAY`.

## Known issues to fix before/while doing this

- ~~Mag fused every IMU step~~ **fixed 2026-10-04**: each reading is fused once (stamps from the
  bridge / log `mgt` column). On the SITL log it changed tilt only 0.25° → 0.24° and heading
  0.75° → 0.66°, so it is NOT what separates your tilt from EKF2's 0.03° - still open.
- ~~Stale GPS accepted~~ **fixed 2026-10-05**: a fix whose timestamp hasn't changed
  (`GPS_RAW_INT.time_usec` live, `gpt` column in logs) is skipped like "no fix". Suite: circle_fast +
  5 s stale GPS 25° → 0.6° tilt max.
- **Accel bias stays pinned** (unpinning diverged in sim) — rely on PX4's 6-point accel cal.
- **`Q_att` is ~1000× gyro noise** — tune from real data (step 5 above), not from the sim.
- The magnetic-field direction must come from the data (converter) or the WMM for your
  location — not Gazebo's `MAG_REFERENCE_ENU`.

## Stage 2 — live shadow on the Pi  (bridge written: `PID/px4_bridge.py`)

**Status 2026-10-04: implemented and tested against PX4 SITL** (`testing/suite/px4_live_shadow.sh`):
`BRIDGE=px4 PX4_URL=... SENSOR_LOG=x.csv RUN_SECONDS=300 python3 run_sim.py` runs your EKF +
controller live on PX4's MAVLink stream (HIGHRES_IMU 250 Hz with PX4 timestamps, GPS_RAW_INT,
PX4's attitude/position as reference) and never sends motor commands. SITL result over a
takeoff-hover-land: 250 Hz with clean 4 ms stamps, mag 14 Hz fused once per reading, baro 20 Hz,
GPS fused without vertical velocity (MAVLink doesn't carry it); your live estimate vs PX4's:
tilt 0.25° RMS, heading +0.33°, position 2 cm / 9 cm. On the Pi: `PX4_URL=/dev/serial0`
(or `/dev/ttyAMA0`), `PX4_BAUD=921600`, PX4 `MAV_1_CONFIG=TELEM 2`, `MAV_1_MODE=Onboard`,
`SER_TEL2_BAUD=921600`.


Same EKF, running live on the Pi from PX4's sensor stream, still not controlling anything —
this tests timing and the data link, which Stage 1 can't.

1. **Timing first:** `python3 testing/bench_timing.py` on the Pi. On the dev laptop:
   0.17 ms/step (4 % of a core at 250 Hz); expect a Pi 4 to be ~5–10× slower. Verdict line
   tells you if it fits.
2. **Data path:** Pixhawk TELEM2 → Pi UART at 921600 baud, MAVLink 2. Stream `HIGHRES_IMU`
   (accel/gyro/mag/baro with `time_usec`) at 250 Hz (or uXRCE-DDS / ROS 2 for full uORB
   topics), `GPS_RAW_INT`, and `ATTITUDE_QUATERNION` + `LOCAL_POSITION_NED` as the reference.
   Use the **sensor timestamps** (`time_usec`), never arrival time — the IMU-timestamp work in
   `gz_bridge.py`/`run_sim.py` is exactly this pattern (queue every sample, step the EKF once
   per sample on its own stamp).
3. Write a `px4_bridge.py` with the same interface as `gz_bridge.GazeboBridge`
   (`get_gyro/get_accel/get_mag/get_gps/get_baro`, `drain_imu`, `get_imu_time`,
   `mag_reference`, `gps_latency`, `use_timestamps`) — then `run_sim.py`'s loop and logging
   work unchanged, and `SENSOR_LOG` recordings feed the same suite.

## Stage 3 — your stack flies (offboard)

Only after Stage 1–2 show your EKF tracking EKF2 closely over many flights.

- PX4 **offboard mode** with actuator-level setpoints (`ActuatorMotors` via uXRCE-DDS) so your
  controller and mixer run on the Pi; motor command mapping (Gazebo rad/s → normalized
  thrust) needs a thrust curve or at least the measured hover throttle; verify motor order and
  spin direction props-off.
- **Safety, outer to inner:** RC kill switch (hardware) → RC switch back to PX4 Position/Stabilized
  → PX4 offboard-loss failsafe (`COM_OF_LOSS_T` ≈ 0.5 s, `COM_OBL_RC_ACT` = land) →
  your `PID/failsafe.py` (FALLBACK = leave offboard so PX4 takes over; LAND; watchdog).
- Progression: props off (signs/directions) → tethered → low hover over grass with the RC
  fallback in hand → missions. Re-tune gains starting from the rate loop.
