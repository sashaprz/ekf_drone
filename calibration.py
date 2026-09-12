"""
Why this file exists: from a single orientation, a constant accelerometer bias and a
constant attitude/tilt error produce the exact same accelerometer reading - the filter
literally cannot tell them apart. This runs a pre-flight routine that gives it enough
attitude diversity (a dwell, then a small deliberate wiggle) to break that tie before
the main navigation filter has to rely on a good accel_bias estimate.

Pre-flight calibration: gyro bias (simple averaging while stationary - always fully
observable, no ambiguity) + attitude/accel_bias (a restricted 9-state EKF run over a
dwell phase followed by a small deliberate wiggle).

The dwell-only part cannot separate accel_bias from a fixed attitude/tilt error - a
constant accelerometer offset and a constant tilt error look identical from a single
orientation (see gps.py's update_H_mag_accel and testing/test_gps_tracking.py's
accel_bias fix check, where enabling H[0:3,12:15] made things WORSE without attitude
diversity). The wiggle phase exists specifically to break that ambiguity: as attitude
changes, the tilt contribution to the accelerometer reading changes with it while a
true sensor bias does not, so the two become separable.

Velocity/position aren't modeled at all here - calibration assumes the vehicle is
genuinely stationary throughout (dwell AND wiggle), so there's no navigation state to
carry, just attitude(3) + gyro_bias(3) + accel_bias(3) = 9 states.

calibrate() is the reusable routine - it only reads from the get_gyro/get_accel/get_mag
callables it's given, so it works against real sensors during an actual pre-arm wiggle
maneuver. The __main__ block below builds synthetic sensor closures (same style as
testing/test_gps_tracking.py) with known injected biases, purely to validate that the
algorithm actually recovers them - it is not what gps.py calls.
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
    #accel_bias only shows up through H, never through F
    F[0:3, 0:3] = np.eye(3) - dt * skew(w)
    F[0:3, 3:6] = -dt * np.eye(3)
    return F

def update_H(predicted_accel, predicted_mag, H):
    H[0:3, 0:3] = skew(predicted_accel)
    H[0:3, 3:6] = 0
    H[0:3, 6:9] = np.eye(3)   #accel rows' Jacobian wrt accel_bias - only separable
                              #from H[0:3,0:3]'s attitude term because the wiggle
                              #phase varies predicted_accel while accel_bias stays fixed
    H[3:6, 0:3] = skew(predicted_mag)
    H[3:6, 3:6] = 0
    H[3:6, 6:9] = 0
    return H

def calibrate(get_gyro, get_accel, get_mag, dwell_steps=200, wiggle_steps=400, dt=0.01):
    """
    Runs dwell_steps + wiggle_steps predict/correct cycles against the given sensor
    readers (no wiggle command is issued here - that has to come from whatever's
    driving get_gyro/get_accel/get_mag, e.g. a real pre-arm maneuver). Returns
    (q, gyro_bias, accel_bias) to seed the main navigation filter with.
    """
    q = np.array([1.0, 0.0, 0.0, 0.0])
    gyro_bias = np.zeros(3)
    accel_bias = np.zeros(3)

    P = np.eye(9)
    Q = np.diag([0.01,0.01,0.01, 1e-6,1e-6,1e-6, 1e-6,1e-6,1e-6])
    R_accel = np.diag([0.1,0.1,0.1])
    R_mag = np.diag([0.1,0.1,0.1])
    F = np.eye(9)
    I = np.eye(9)
    H = np.zeros((6,9))

    for _ in range(dwell_steps + wiggle_steps):
        gyro_x, gyro_y, gyro_z = get_gyro()
        mag_x, mag_y, mag_z = get_mag()
        accel_x, accel_y, accel_z = get_accel()

        corrected_gyro = np.array([gyro_x, gyro_y, gyro_z]) - gyro_bias
        corrected_accel = np.array([accel_x, accel_y, accel_z]) - accel_bias

        w_quat = [0, corrected_gyro[0], corrected_gyro[1], corrected_gyro[2]]
        q_dot = 0.5 * quat_mult(q, w_quat)
        q = q + q_dot * dt
        q = q / np.linalg.norm(q)

        F = update_F(corrected_gyro, dt, F)
        P = F @ P @ F.T + Q

        predicted_accel = rotate_by_quat(quat_conjugate(q), np.array([0.0, 0.0, 1.0]))
        predicted_mag = rotate_by_quat(quat_conjugate(q), np.array([1.0, 0.0, 0.0]))
        H = update_H(predicted_accel, predicted_mag, H)
        H_accel = H[0:3, :]
        H_mag = H[3:6, :]

        K_accel = P @ H_accel.T @ np.linalg.inv(H_accel @ P @ H_accel.T + R_accel)
        residual_accel = corrected_accel - predicted_accel
        error_state = K_accel @ residual_accel
        P = (I - K_accel @ H_accel) @ P

        K_mag = P @ H_mag.T @ np.linalg.inv(H_mag @ P @ H_mag.T + R_mag)
        residual_mag = np.array([mag_x, mag_y, mag_z]) - predicted_mag
        error_state = error_state + K_mag @ residual_mag
        P = (I - K_mag @ H_mag) @ P

        d_theta = error_state[0:3]
        gyro_bias = gyro_bias + error_state[3:6]
        accel_bias = accel_bias + error_state[6:9]

        dq = np.array([1.0, d_theta[0]/2, d_theta[1]/2, d_theta[2]/2])
        q = quat_mult(q, dq)
        q = q / np.linalg.norm(q)

    return q, gyro_bias, accel_bias


if __name__ == "__main__":
    #---- synthetic ground truth: the vehicle rests at a small, UNKNOWN static tilt
    #     throughout (e.g. sitting on slightly uneven ground) - the filter starts at
    #     q=identity, so this tilt is exactly the kind of thing that's indistinguishable
    #     from accel_bias during a plain dwell. Then a small roll+pitch wiggle, purely
    #     to prove the algorithm needs it to separate the two ----
    true_gyro_bias = np.array([math.radians(2), math.radians(-1), math.radians(0.5)])
    true_accel_bias = np.array([0.05, -0.02, 0.03])
    base_tilt_roll = math.radians(3)   #unknown static mounting tilt, present the whole time
    base_tilt_pitch = math.radians(-2)
    dt = 0.01
    dwell_duration = 200 * dt
    wiggle_amp_roll = math.radians(8)
    wiggle_freq_roll = 0.8
    wiggle_amp_pitch = math.radians(6)
    wiggle_freq_pitch = 0.5

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

    def make_sensors(wiggle_enabled):
        state = {"t": 0.0}

        def get_gyro():
            t = state["t"]
            roll, pitch = true_roll(t, wiggle_enabled), true_pitch(t, wiggle_enabled)
            roll_dot, pitch_dot = true_roll_dot(t, wiggle_enabled), true_pitch_dot(t, wiggle_enabled)
            wx = roll_dot
            wy = pitch_dot * math.cos(roll)
            wz = -pitch_dot * math.sin(roll)
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
            roll, pitch = true_roll(t, wiggle_enabled), true_pitch(t, wiggle_enabled)
            mx = math.cos(pitch)
            my = math.sin(roll) * math.sin(pitch)
            mz = math.cos(roll) * math.sin(pitch)
            #renormalize so a nonzero mz/my from tilt doesn't change the vector's magnitude
            v = np.array([mx, my, mz])
            return tuple(v / np.linalg.norm(v))

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
    print(f"true accel_bias:         {[round(b,3) for b in true_accel_bias]}\n")

    for label, wiggle_enabled in [("Dwell only (no wiggle)", False), ("Dwell + wiggle", True)]:
        q, gyro_bias, accel_bias = run_calibration(wiggle_enabled)
        print(f"{label}:")
        print(f"  gyro_bias est (deg/s):  {[round(math.degrees(b),3) for b in gyro_bias]}")
        print(f"  accel_bias est:         {[round(b,3) for b in accel_bias]}")
        print(f"  accel_bias error:       {[round(float(b),3) for b in (accel_bias - true_accel_bias)]}\n")
