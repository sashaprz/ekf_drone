"""
Why this file exists: from a single orientation, a constant accelerometer bias and a
constant attitude/tilt error produce the exact same accelerometer reading - the filter
literally cannot tell them apart. Same problem, different axis, for the magnetometer:
a constant mag bias and a constant yaw error produce the same mag reading from one
heading. This runs a pre-flight routine that gives it enough attitude diversity (a
dwell, then a small deliberate wiggle - roll+pitch to separate accel_bias from tilt,
yaw to separate mag_bias from heading error) to break both ties before the main
navigation filter has to rely on good bias estimates.

Pre-flight calibration: gyro bias (simple averaging while stationary - always fully
observable, no ambiguity) + attitude/accel_bias/mag_bias (a restricted 12-state EKF run
over a dwell phase followed by a small deliberate wiggle).

The dwell-only part cannot separate accel_bias from a fixed tilt error, or mag_bias
from a fixed heading error - a constant sensor offset and a constant attitude error
look identical from a single orientation (see gps.py's update_H_accel/update_H_mag and
testing/test_gps_tracking.py's accel_bias fix check, where enabling H[0:3,12:15] made
things WORSE without attitude diversity). The wiggle phase exists specifically to break
both ambiguities: as attitude changes, the tilt/heading contribution to the
accel/mag reading changes with it while a true sensor bias does not, so the two become
separable.

Velocity/position aren't modeled at all here - calibration assumes the vehicle is
genuinely stationary throughout (dwell AND wiggle), so there's no navigation state to
carry, just attitude(3) + gyro_bias(3) + accel_bias(3) + mag_bias(3) = 12 states.

calibrate() is the reusable routine - it only reads from the get_gyro/get_accel/get_mag
callables it's given, so it works against real sensors during an actual pre-arm wiggle
maneuver. The __main__ block below builds synthetic sensor closures (same style as
testing/test_gps_tracking.py) with known injected biases, purely to validate that the
algorithm actually recovers them - it is not what gps.py calls.

Caveat carried over from the README/conversation: on real hardware, magnetometer
interference is often current-dependent (varies with motor throttle), not a fixed
constant like this treats it - this models the constant-bias case only. If your real
interference isn't roughly constant, a state like this won't track it well.
"""

import math
import numpy as np

def quat_mult(q1, q2):
    w = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]
    x = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]
    y = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]
    z = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]
    return np.array([w, x, y, z])

def quat_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def rotate_by_quat(q, v):
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
    v_quat = np.array([0, v[0], v[1], v[2]])
    rotated_v_quat = quat_mult(quat_mult(q, v_quat), q_conj)
    return rotated_v_quat[1:]

def skew(v):
    return np.array([[0,-v[2],v[1]], [v[2],0,-v[0]], [-v[1],v[0],0]])

def update_F(w, dt, F):
    #no velocity/position states here (vehicle assumed stationary throughout),
    #so unlike gps.py's update_F there's no accel_bias->velocity coupling block -
    #accel_bias/mag_bias only show up through H, never through F
    F[0:3, 0:3] = np.eye(3) - dt * skew(w)
    F[0:3, 3:6] = -dt * np.eye(3)
    return F

def update_H_accel(predicted_accel, H):
    H[0:3, 0:3] = skew(predicted_accel)
    H[0:3, 3:6] = 0
    H[0:3, 6:9] = np.eye(3)    #accel rows' Jacobian wrt accel_bias - only separable
                               #from H[0:3,0:3]'s attitude term because the wiggle
                               #phase varies predicted_accel while accel_bias stays fixed
    H[0:3, 9:12] = 0
    return H

def update_H_mag(predicted_mag, H):
    H[0:3, 0:3] = skew(predicted_mag)
    H[0:3, 3:6] = 0
    H[0:3, 6:9] = 0
    H[0:3, 9:12] = np.eye(3)   #mag rows' Jacobian wrt mag_bias - same derivation as
                               #accel_bias's +I (see gps.py's update_H_mag), separable
                               #from heading error only because the wiggle varies yaw too
    return H

def calibrate(get_gyro, get_accel, get_mag, dwell_steps=200, wiggle_steps=400, dt=0.01,
              gravity=1.0, accel_noise_var=0.1, mag_noise_var=0.1):
    """
    Runs dwell_steps + wiggle_steps predict/correct cycles against the given sensor
    readers (no wiggle command is issued here - that has to come from whatever's
    driving get_gyro/get_accel/get_mag, e.g. a real pre-arm maneuver). Returns
    (q, gyro_bias, accel_bias, mag_bias, P) to seed the main navigation filter with - P
    is the converged 12x12 covariance (attitude, gyro_bias, accel_bias, mag_bias
    blocks, in that order), NOT just np.eye(12): a blind identity guess for the main
    filter's own P is what let the pooled accel+mag correction overcorrect badly enough
    to lock the accel gate on ~30% of cold starts (see testing/test_gate_monte_carlo.py)
    - actually reflecting how much calibration narrowed things down avoids that. The
    caller (gps.py) has to map this 12x12 block layout onto its own 18x18 P (which also
    has velocity/position), since calibration never modeled those.

    gravity: the magnitude get_accel() reports at rest (1.0 if it's 1g-normalized, or
    the real ~9.80665 m/s² if it isn't) - must match the caller's convention, or the
    residual will be dominated by this mismatch rather than the actual attitude/bias
    error, and accel_bias will absorb the difference almost entirely.

    accel_noise_var/mag_noise_var: R for the accel/mag corrections - the defaults (0.1)
    assume 1g-normalized accel; pass the caller's actual noise variance (e.g. accel
    noise-std² in the same units as `gravity`) when it isn't, or the filter will be
    badly over/under-confident about how much to trust each reading.
    """
    q = np.array([1.0, 0.0, 0.0, 0.0])
    gyro_bias = np.zeros(3)
    accel_bias = np.zeros(3)
    mag_bias = np.zeros(3)

    P = np.eye(12)
    Q = np.diag([0.01,0.01,0.01, 1e-6,1e-6,1e-6, 1e-6,1e-6,1e-6, 1e-6,1e-6,1e-6])
    R_accel = np.diag([accel_noise_var]*3)
    R_mag = np.diag([mag_noise_var]*3)
    F = np.eye(12)
    I = np.eye(12)
    H_accel = np.zeros((3,12))
    H_mag = np.zeros((3,12))

    def apply_correction(correction):
        #injects immediately and re-normalizes q, rather than pooling accel+mag against
        #one frozen q and injecting once - see gps.py's apply_correction for why this
        #matters (the pooled version let two large-gain corrections double-count against
        #the same stale linearization when P was large, which is exactly the situation
        #at the start of calibration before anything has converged yet)
        nonlocal q, gyro_bias, accel_bias, mag_bias
        d_theta = correction[0:3]
        gyro_bias = gyro_bias + correction[3:6]
        accel_bias = accel_bias + correction[6:9]
        mag_bias = mag_bias + correction[9:12]
        dq = np.array([1.0, d_theta[0]/2, d_theta[1]/2, d_theta[2]/2])
        q = quat_mult(q, dq)
        q = q / np.linalg.norm(q)

    for _ in range(dwell_steps + wiggle_steps):
        gyro_x, gyro_y, gyro_z = get_gyro()
        mag_x, mag_y, mag_z = get_mag()
        accel_x, accel_y, accel_z = get_accel()

        corrected_gyro = np.array([gyro_x, gyro_y, gyro_z]) - gyro_bias
        corrected_accel = np.array([accel_x, accel_y, accel_z]) - accel_bias
        corrected_mag = np.array([mag_x, mag_y, mag_z]) - mag_bias

        w_quat = [0, corrected_gyro[0], corrected_gyro[1], corrected_gyro[2]]
        q_dot = 0.5 * quat_mult(q, w_quat)
        q = q + q_dot * dt
        q = q / np.linalg.norm(q)

        F = update_F(corrected_gyro, dt, F)
        P = F @ P @ F.T + Q

        #accel correction
        predicted_accel = rotate_by_quat(quat_conjugate(q), np.array([0.0, 0.0, gravity]))
        H_accel = update_H_accel(predicted_accel, H_accel)
        K_accel = P @ H_accel.T @ np.linalg.inv(H_accel @ P @ H_accel.T + R_accel)
        residual_accel = corrected_accel - predicted_accel
        apply_correction(K_accel @ residual_accel)
        #Joseph form - numerically robust to floating-point drift (keeps P symmetric/PSD).
        #Matters here specifically because gps.py now seeds its own P from this function's
        #returned P - a P that's silently lost PSD would corrupt the main loop from the start.
        P = (I - K_accel @ H_accel) @ P @ (I - K_accel @ H_accel).T + K_accel @ R_accel @ K_accel.T

        #mag correction - predicted_mag uses q AFTER the accel correction just applied;
        #residual compares against corrected_mag (bias-subtracted), same pattern as accel
        predicted_mag = rotate_by_quat(quat_conjugate(q), np.array([1.0, 0.0, 0.0]))
        H_mag = update_H_mag(predicted_mag, H_mag)
        K_mag = P @ H_mag.T @ np.linalg.inv(H_mag @ P @ H_mag.T + R_mag)
        residual_mag = corrected_mag - predicted_mag
        apply_correction(K_mag @ residual_mag)
        P = (I - K_mag @ H_mag) @ P @ (I - K_mag @ H_mag).T + K_mag @ R_mag @ K_mag.T

    return q, gyro_bias, accel_bias, mag_bias, P


if __name__ == "__main__":
    #---- synthetic ground truth: the vehicle rests at a small, UNKNOWN static tilt AND
    #     heading throughout (e.g. sitting on slightly uneven ground) - the filter starts
    #     at q=identity, so this is exactly the kind of thing that's indistinguishable
    #     from accel_bias/mag_bias during a plain dwell. Then a small roll+pitch+yaw
    #     wiggle, purely to prove the algorithm needs it to separate the two ----
    true_gyro_bias = np.array([math.radians(2), math.radians(-1), math.radians(0.5)])
    true_accel_bias = np.array([0.05, -0.02, 0.03])
    true_mag_bias = np.array([0.04, 0.06, -0.03])
    base_tilt_roll = math.radians(3)   #unknown static mounting tilt/heading, present the whole time
    base_tilt_pitch = math.radians(-2)
    base_heading_yaw = math.radians(7)
    dt = 0.01
    dwell_duration = 200 * dt
    wiggle_amp_roll = math.radians(8)
    wiggle_freq_roll = 0.8
    wiggle_amp_pitch = math.radians(6)
    wiggle_freq_pitch = 0.5
    wiggle_amp_yaw = math.radians(10)
    wiggle_freq_yaw = 0.4

    def true_roll(t, wiggle_enabled):
        if t < dwell_duration or not wiggle_enabled:
            return base_tilt_roll
        return base_tilt_roll + wiggle_amp_roll * math.sin(wiggle_freq_roll * (t - dwell_duration))

    def true_roll_dot(t, wiggle_enabled):
        if t < dwell_duration or not wiggle_enabled:
            return 0.0
        return wiggle_amp_roll * wiggle_freq_roll * math.cos(wiggle_freq_roll * (t - dwell_duration))

    def true_pitch(t, wiggle_enabled):
        if t < dwell_duration or not wiggle_enabled:
            return base_tilt_pitch
        return base_tilt_pitch + wiggle_amp_pitch * math.sin(wiggle_freq_pitch * (t - dwell_duration))

    def true_pitch_dot(t, wiggle_enabled):
        if t < dwell_duration or not wiggle_enabled:
            return 0.0
        return wiggle_amp_pitch * wiggle_freq_pitch * math.cos(wiggle_freq_pitch * (t - dwell_duration))

    def true_yaw(t, wiggle_enabled):
        if t < dwell_duration or not wiggle_enabled:
            return base_heading_yaw
        return base_heading_yaw + wiggle_amp_yaw * math.sin(wiggle_freq_yaw * (t - dwell_duration))

    def true_yaw_dot(t, wiggle_enabled):
        if t < dwell_duration or not wiggle_enabled:
            return 0.0
        return wiggle_amp_yaw * wiggle_freq_yaw * math.cos(wiggle_freq_yaw * (t - dwell_duration))

    def make_sensors(wiggle_enabled):
        state = {"t": 0.0}

        def get_gyro():
            t = state["t"]
            roll, pitch = true_roll(t, wiggle_enabled), true_pitch(t, wiggle_enabled)
            roll_dot, pitch_dot, yaw_dot = (true_roll_dot(t, wiggle_enabled), true_pitch_dot(t, wiggle_enabled),
                                             true_yaw_dot(t, wiggle_enabled))
            wx = roll_dot - yaw_dot * math.sin(pitch)
            wy = pitch_dot * math.cos(roll) + yaw_dot * math.sin(roll) * math.cos(pitch)
            wz = -pitch_dot * math.sin(roll) + yaw_dot * math.cos(roll) * math.cos(pitch)
            return (wx + true_gyro_bias[0], wy + true_gyro_bias[1], wz + true_gyro_bias[2])

        def get_accel():
            t = state["t"]
            roll, pitch = true_roll(t, wiggle_enabled), true_pitch(t, wiggle_enabled)
            ax = -math.sin(pitch)
            ay = math.sin(roll) * math.cos(pitch)
            az = math.cos(roll) * math.cos(pitch)
            return (ax + true_accel_bias[0], ay + true_accel_bias[1], az + true_accel_bias[2])

        def get_mag():
            t = state["t"]
            roll, pitch, yaw = true_roll(t, wiggle_enabled), true_pitch(t, wiggle_enabled), true_yaw(t, wiggle_enabled)
            mx = math.cos(pitch) * math.cos(yaw)
            my = math.sin(roll) * math.sin(pitch) * math.cos(yaw) - math.cos(roll) * math.sin(yaw)
            mz = math.cos(roll) * math.sin(pitch) * math.cos(yaw) + math.sin(roll) * math.sin(yaw)
            #renormalize so tilt/heading don't change the vector's magnitude, THEN add the bias
            #(bias is a sensor offset, not part of the "true field direction" being renormalized)
            v = np.array([mx, my, mz])
            v = v / np.linalg.norm(v)
            return tuple(v + true_mag_bias)

        def advance(step_dt):
            state["t"] += step_dt

        return get_gyro, get_accel, get_mag, advance

    def run_calibration(wiggle_enabled, dwell_steps=200, wiggle_steps=400):
        get_gyro, get_accel, get_mag, advance = make_sensors(wiggle_enabled)
        total_steps = dwell_steps + (wiggle_steps if wiggle_enabled else 0)

        #calibrate() calls get_gyro/get_mag/get_accel once per step, in that order, but
        #has no way to advance simulated time itself (real sensors don't need that) -
        #advance time on the LAST read of that sequence so all three see the same t
        def gyro_wrapped():
            return get_gyro()
        def mag_wrapped():
            return get_mag()
        def accel_wrapped():
            r = get_accel()
            advance(dt)
            return r

        return calibrate(gyro_wrapped, accel_wrapped, mag_wrapped, dwell_steps=dwell_steps,
                          wiggle_steps=(wiggle_steps if wiggle_enabled else 0), dt=dt)

    print("Calibration accuracy - dwell-only vs dwell+wiggle")
    print(f"true gyro_bias (deg/s):  {[round(math.degrees(b),3) for b in true_gyro_bias]}")
    print(f"true accel_bias:         {[round(b,3) for b in true_accel_bias]}")
    print(f"true mag_bias:           {[round(b,3) for b in true_mag_bias]}\n")

    for label, wiggle_enabled in [("Dwell only (no wiggle)", False), ("Dwell + wiggle", True)]:
        q, gyro_bias, accel_bias, mag_bias, _P = run_calibration(wiggle_enabled)
        print(f"{label}:")
        print(f"  gyro_bias est (deg/s):  {[round(math.degrees(b),3) for b in gyro_bias]}")
        print(f"  accel_bias est:         {[round(b,3) for b in accel_bias]}")
        print(f"  accel_bias error:       {[round(float(b),3) for b in (accel_bias - true_accel_bias)]}")
        print(f"  mag_bias est:           {[round(b,3) for b in mag_bias]}")
        print(f"  mag_bias error:         {[round(float(b),3) for b in (mag_bias - true_mag_bias)]}\n")
