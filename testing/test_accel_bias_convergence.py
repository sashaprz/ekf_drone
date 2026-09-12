"""
Verifies accel_bias actually converges to its true injected value under a rotating
trajectory - the thing calibration.py's dwell+wiggle exists to bootstrap, checked here
against sustained real motion instead of a short pre-flight routine. Continuous
roll/pitch/yaw rotation (same sinusoidal ground truth as ekf_rms.py) gives the filter
the attitude diversity that a single fixed orientation can't: a true accel_bias stays
constant in the body frame while attitude keeps changing, so over time the two stop
looking like the same thing (see gps.py's update_H_mag_accel and calibration.py for why
they're otherwise indistinguishable).

Mirrors gps.py's current math exactly (GRAVITY, Joseph-form P updates, all three
chi-squared gates, the accel_bias H column). The vehicle hovers (true velocity/position
= 0) while rotating, so GPS keeps velocity/position pinned near truth and any residual
attitude/bias confusion shows up cleanly in the accel_bias estimate rather than being
masked by translational drift.
"""

import math
import numpy as np

GRAVITY = 9.80665
chi2_threshold = 11.34
chi2_threshold_gps = 16.81

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

def quat_to_R(q):
    return np.array([rotate_by_quat(q, e) for e in np.eye(3)]).T

def update_F(w, dt, F, q, accel_body):
    F[0:3, 0:3] = np.eye(3) - dt * skew(w)
    F[0:3, 3:6] = -dt * np.eye(3)
    F[3:6, 0:3] = 0
    F[3:6, 3:6] = np.eye(3)
    F[6:9, 0:3] = -dt * quat_to_R(q) @ skew(accel_body)
    F[9:12, 6:9] = dt * np.eye(3)
    F[12:15, 12:15] = np.eye(3)
    F[6:9, 12:15] = -dt * quat_to_R(q)
    return F

def update_H_mag_accel(predicted_accel, predicted_mag, H):
    H[0:3, 0:3] = skew(predicted_accel)
    H[0:3, 3:6] = 0
    H[0:3, 12:15] = np.eye(3)
    H[3:6, 0:3] = skew(predicted_mag)
    H[3:6, 3:6] = 0
    H[3:6, 12:15] = 0
    return H

def update_H_gps(H):
    H[:, 0:6] = 0
    H[0:3, 9:12] = np.eye(3)
    H[3:6, 6:9] = np.eye(3)
    return H

#---- ground truth: continuous roll/pitch/yaw rotation, hovering (zero true velocity/
#     position) - same sinusoids as ekf_rms.py, so this is a known-good attitude
#     trajectory, just re-used here to test accel_bias instead of pure attitude error ----
amplitude_roll = math.radians(15); freq_roll = 0.5
amplitude_pitch = math.radians(10); freq_pitch = 0.3; phase_pitch = math.pi / 2
amplitude_yaw = math.radians(5); freq_yaw = 0.1; phase_yaw = math.pi

def true_roll(t): return amplitude_roll * math.sin(freq_roll * t)
def true_roll_dot(t): return amplitude_roll * freq_roll * math.cos(freq_roll * t)
def true_pitch(t): return amplitude_pitch * math.sin(freq_pitch * t + phase_pitch)
def true_pitch_dot(t): return amplitude_pitch * freq_pitch * math.cos(freq_pitch * t + phase_pitch)
def true_yaw(t): return amplitude_yaw * math.sin(freq_yaw * t + phase_yaw)
def true_yaw_dot(t): return amplitude_yaw * freq_yaw * math.cos(freq_yaw * t + phase_yaw)

def get_gyro(t, gyro_bias_true):
    roll, pitch = true_roll(t), true_pitch(t)
    roll_dot, pitch_dot, yaw_dot = true_roll_dot(t), true_pitch_dot(t), true_yaw_dot(t)
    wx = roll_dot - yaw_dot * math.sin(pitch)
    wy = pitch_dot * math.cos(roll) + yaw_dot * math.sin(roll) * math.cos(pitch)
    wz = -pitch_dot * math.sin(roll) + yaw_dot * math.cos(roll) * math.cos(pitch)
    return (wx + gyro_bias_true[0], wy + gyro_bias_true[1], wz + gyro_bias_true[2])

def get_accel(t, accel_bias_true):
    #hovering - specific force is gravity-only, rotated into the body frame by the true attitude
    roll, pitch = true_roll(t), true_pitch(t)
    ax = -GRAVITY * math.sin(pitch)
    ay = GRAVITY * math.sin(roll) * math.cos(pitch)
    az = GRAVITY * math.cos(roll) * math.cos(pitch)
    return (ax + accel_bias_true[0], ay + accel_bias_true[1], az + accel_bias_true[2])

def get_mag(t):
    roll, pitch, yaw = true_roll(t), true_pitch(t), true_yaw(t)
    mx = math.cos(pitch) * math.cos(yaw)
    my = math.sin(roll) * math.sin(pitch) * math.cos(yaw) - math.cos(roll) * math.sin(yaw)
    mz = math.cos(roll) * math.sin(pitch) * math.cos(yaw) + math.sin(roll) * math.sin(yaw)
    return (mx, my, mz)

def get_gps():
    return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0   #hovering: true position/velocity are always zero

def run(true_gyro_bias=(math.radians(2), math.radians(-1), math.radians(0.5)),
        true_accel_bias=(0.05, -0.02, 0.03), num_steps=2000, dt=0.01, gps_period=0.2):
    true_gyro_bias = np.array(true_gyro_bias)
    true_accel_bias = np.array(true_accel_bias)

    q = np.array([1.0, 0.0, 0.0, 0.0])
    bias_x = bias_y = bias_z = 0.0
    accel_bias_x = accel_bias_y = accel_bias_z = 0.0
    velocity = np.array([0.0, 0.0, 0.0])
    position = np.array([0.0, 0.0, 0.0])

    P = np.eye(15)
    Q = np.diag([0.01,0.01,0.01, 1e-6,1e-6,1e-6, 1e-6,1e-6,1e-6, 1e-6,1e-6,1e-6, 1e-6,1e-6,1e-6])
    R_accel_base = np.diag([0.1,0.1,0.1])
    R_mag = np.diag([0.1,0.1,0.1])
    R_gps = np.diag([1.0,1.0,1.0, 0.5,0.5,0.5])
    F = np.eye(15)
    I = np.eye(15)
    H_accel_mag = np.zeros((6,15))
    H_gps = np.zeros((6,15))

    k = 1
    last_gps_time = 0.0
    t = 0.0
    log = []   #(t, accel_bias_x, accel_bias_y, accel_bias_z, gyro_bias_x, gyro_bias_y, gyro_bias_z)

    for _ in range(num_steps):
        gyro_x, gyro_y, gyro_z = get_gyro(t, true_gyro_bias)
        mag_x, mag_y, mag_z = get_mag(t)
        accel_x, accel_y, accel_z = get_accel(t, true_accel_bias)

        accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        deviation = abs(accel_magnitude - GRAVITY)
        R_accel = R_accel_base * (1 + k * deviation ** 2)

        corrected_gyro_x = gyro_x - bias_x
        corrected_gyro_y = gyro_y - bias_y
        corrected_gyro_z = gyro_z - bias_z
        corrected_accel_x = accel_x - accel_bias_x
        corrected_accel_y = accel_y - accel_bias_y
        corrected_accel_z = accel_z - accel_bias_z

        w_quat = [0, corrected_gyro_x, corrected_gyro_y, corrected_gyro_z]
        q_dot = 0.5 * quat_mult(q, w_quat)
        q = q + q_dot * dt
        q = q / np.linalg.norm(q)

        corrected_accel = np.array([corrected_accel_x, corrected_accel_y, corrected_accel_z])
        accel_world = rotate_by_quat(q, corrected_accel)
        accel_world = accel_world - np.array([0.0, 0.0, GRAVITY])
        velocity = velocity + accel_world * dt
        position = position + velocity * dt

        w = np.array([corrected_gyro_x, corrected_gyro_y, corrected_gyro_z])
        F = update_F(w, dt, F, q, corrected_accel)
        P = F @ P @ F.T + Q

        error_state = np.zeros(15)

        if t - last_gps_time >= gps_period:
            north, east, up, vel_north, vel_east, vel_up = get_gps()
            gps_measurement = np.array([north, east, up, vel_north, vel_east, vel_up])

            H_gps = update_H_gps(H_gps)
            S_gps = H_gps @ P @ H_gps.T + R_gps
            K_gps = P @ H_gps.T @ np.linalg.inv(S_gps)
            predicted_gps = np.concatenate([position, velocity])
            residual_gps = gps_measurement - predicted_gps
            d_squared_gps = residual_gps.T @ np.linalg.inv(S_gps) @ residual_gps

            if d_squared_gps <= chi2_threshold_gps:
                error_state = error_state + K_gps @ residual_gps
                P = (I - K_gps @ H_gps) @ P @ (I - K_gps @ H_gps).T + K_gps @ R_gps @ K_gps.T

            last_gps_time = t

        predicted_accel = rotate_by_quat(quat_conjugate(q), np.array([0.0, 0.0, GRAVITY]))
        predicted_mag = rotate_by_quat(quat_conjugate(q), np.array([1.0, 0.0, 0.0]))
        H_accel_mag = update_H_mag_accel(predicted_accel, predicted_mag, H_accel_mag)
        H_accel = H_accel_mag[0:3, :]

        S_accel = H_accel @ P @ H_accel.T + R_accel
        K_accel = P @ H_accel.T @ np.linalg.inv(S_accel)
        residual_accel = corrected_accel - predicted_accel
        S_accel_gate = H_accel @ P @ H_accel.T + R_accel_base
        d_squared = residual_accel.T @ np.linalg.inv(S_accel_gate) @ residual_accel
        if d_squared <= chi2_threshold:
            error_state = error_state + K_accel @ residual_accel
            P = (I - K_accel @ H_accel) @ P @ (I - K_accel @ H_accel).T + K_accel @ R_accel @ K_accel.T

        H_mag = H_accel_mag[3:]
        S_mag = H_mag @ P @ H_mag.T + R_mag
        K_mag = P @ H_mag.T @ np.linalg.inv(S_mag)
        residual_mag = np.array([mag_x, mag_y, mag_z]) - predicted_mag
        d_squared_mag = residual_mag.T @ np.linalg.inv(S_mag) @ residual_mag
        if d_squared_mag <= chi2_threshold:
            error_state = error_state + K_mag @ residual_mag
            P = (I - K_mag @ H_mag) @ P @ (I - K_mag @ H_mag).T + K_mag @ R_mag @ K_mag.T

        d_theta = error_state[0:3]
        d_bias = error_state[3:6]
        d_velocity = error_state[6:9]
        d_position = error_state[9:12]
        d_accel_bias = error_state[12:15]

        bias_x += d_bias[0]; bias_y += d_bias[1]; bias_z += d_bias[2]
        accel_bias_x += d_accel_bias[0]; accel_bias_y += d_accel_bias[1]; accel_bias_z += d_accel_bias[2]
        dq = np.array([1.0, d_theta[0]/2, d_theta[1]/2, d_theta[2]/2])
        q = quat_mult(q, dq)
        q = q / np.linalg.norm(q)
        velocity = velocity + d_velocity
        position = position + d_position

        log.append((t, accel_bias_x, accel_bias_y, accel_bias_z, bias_x, bias_y, bias_z))
        t += dt

    return log, true_accel_bias, true_gyro_bias


if __name__ == "__main__":
    log, true_accel_bias, true_gyro_bias = run()
    final = log[-1]
    #settle window: average over the last 2s to smooth out residual oscillation
    settle = log[-200:]
    settled_bias = np.mean([[row[1], row[2], row[3]] for row in settle], axis=0)

    print(f"true accel_bias:       {[round(b,4) for b in true_accel_bias]}")
    print(f"final accel_bias est:  {[round(b,4) for b in [final[1], final[2], final[3]]]}")
    print(f"settled accel_bias est (last 2s avg): {[round(b,4) for b in settled_bias]}")
    print(f"error (settled - true): {[round(float(b),4) for b in (settled_bias - true_accel_bias)]}")
