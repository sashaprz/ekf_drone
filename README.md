# python_drone

Attitude estimation experiments for a drone using gyro + accelerometer + magnetometer, starting from a complementary filter and working up to an Extended Kalman Filter (EKF). All tests below use hardcoded constant sensor values (no real hardware yet) to validate the filter logic in isolation.

## Files

- `complementary.py` — fixed-weight complementary filter (roll/pitch/yaw blended from gyro integration + accel/mag).
- `ekf.py` — EKF version: dynamically computed Kalman gain, plus a real (non-identity) process-model Jacobian `F` derived from the coupled Euler rate equations. Measurement Jacobian `H` is identity by design (see "Design notes" below).

## Test scenarios and results

All tests simulate a drone tilted 10° in roll (`accel_y = sin(10°)`, `accel_z = cos(10°)`) and pointing magnetic north (`mag_x = 1.0`).

### 1. Complementary filter — static tilt, zero rotation
**Setup:** all gyro axes = 0, `weight = 0.98`, `sleep(0.5)`.
**Result:** `roll` climbs from 0 and asymptotically approaches the accel-derived ~10° tilt, following `roll ≈ 10° * (1 - 0.98^n)`. After ~90 iterations (45s) it reached **8.38°**, matching the predicted curve almost exactly. `pitch`/`yaw` stayed at 0.0 throughout.
**Takeaway:** confirms the gyro-integration → accel/mag correction → blend loop is wired correctly for the simple static case.

### 2. Complementary filter — constant gyro bias
**Setup:** `gyro_x_dps = 5` (constant 5°/s roll rate), same 10° tilt accel, `sleep(0.01)`.
**Result:** `roll` converges to a steady state around **12.6–12.7°** — not the true 10°. `pitch`/`yaw` stayed at 0.0.
**Takeaway:** a fixed-weight complementary filter cannot fully reject a *sustained* gyro rate — it settles at an offset set by the balance between `weight` and the loop's `dt`, not at the true value. This is the fundamental limitation that motivated moving to an EKF (specifically, a gyro-bias state — not yet implemented, see "Known limitations").

### 3. EKF — same gyro bias scenario, identity Jacobians
**Setup:** same as test 2, but using the EKF's dynamically computed Kalman gain (`Q = 0.01·I`, `R = 0.1·I`) instead of a fixed weight. `F`/`H` both identity (valid for this test since `gyro_y_dps = gyro_z_dps = 0` makes the coupling terms vanish).
**Result:** `roll` converges to **~10.14°** — much closer to the true 10° than the complementary filter's 12.6–12.7°. `pitch`/`yaw` stayed at 0.0 (expected — this test never exercises the coupling).
**Why it's better:** the steady-state Kalman gain for this `Q`/`R` works out to `K ≈ 0.27` (trusting the accel measurement ~27% per step vs. the complementary filter's fixed 2%). This is a like-for-like demonstration that a gain derived from actual noise assumptions can outperform an arbitrary hand-tuned blend weight — not because of anything Jacobian-related yet.

### 4. EKF — coupling exercised via nonzero gyro_y
**Setup:** `gyro_x_dps = 5`, `gyro_y_dps = 3` (new), everything else as in test 3.
**Result:** `roll` still converges to ~10.14° as before, but now `pitch` settles around **0.084 rad (~4.8°)** and `yaw` around **0.015 rad (~0.86°)** — neither pinned at exactly 0 anymore.
**Why:** once `roll` is nonzero, the `sin(roll)`/`cos(roll)` coupling terms in the Euler rate equations start "leaking" `gyro_y`'s body-frame rotation into both `pitch_dot` and `yaw_dot`. With the old decoupled equations, `gyro_y` could only ever have affected `pitch` — never `yaw`. This is the first test where the process-model Jacobian `F` is genuinely non-identity.

## Design notes

- **Gyro units**: raw constants are stored in deg/s and converted to rad/s inside `get_gyro()`, to match the radian-based state (`roll`/`pitch`/`yaw` from `atan2`).
- **`H` is identity by design, not a placeholder.** Accel/mag readings are pre-converted into pseudo-angle measurements (`accel_roll`, `accel_pitch`, `accel_yaw`) *before* being compared against the state, so the measurement model is `h(state) = state` — genuinely linear, not an approximation. `F` needed to become a real Jacobian because the *process* model (Euler rate coupling) is nonlinear in the state; `H` only would need the same treatment if the measurement model were changed to compare raw accel/mag vectors directly against a rotated reference vector.

## Known limitations / next steps

- **`R` is still static.** The plan to make it dynamically inflate based on accel-magnitude deviation from 1g (to auto-distrust accel during hard maneuvering) was discussed but not yet implemented.
- **No gyro-bias state.** The residual offset in tests 2-4 (converging near, but not exactly at, the true angle) is the signature of an un-modeled constant gyro bias. Adding bias as an estimated state (expanding the state vector to 6 elements) is the proper fix, not yet implemented.
- **No real hardware input yet.** `get_gyro()`/`get_accel()`/`get_mag()` all return hardcoded constants.
