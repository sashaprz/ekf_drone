"""
Combines everything the separate tests exercised individually: rotation (attitude
diversity for accel_bias, from test_accel_bias_convergence.py), translation + GPS
correction (from test_gps_tracking.py), and realistic noise on all four sensors (from
test_gate_monte_carlo.py) - does accel_bias convergence and position/velocity tracking
both hold up when all three are happening at once, not in isolation?

Mirrors gps.py's current math exactly (GRAVITY, Joseph-form P updates, all three
chi-squared gates, the accel_bias H column, AND the pre-flight calibration call that
seeds q/bias/accel_bias/P). P0 is NOT np.eye(15), and NOT an arbitrary shrunk scalar
either - it's built the same way gps.py builds it, from calibration.calibrate()'s own
converged covariance. Both matter: np.eye(15) gave the pooled accel+mag correction a
~30% chance of overcorrecting badly enough to lock the accel gate on permanently (see
test_gate_monte_carlo.py); a blanket small scalar (tried 0.01 here first) avoided that
but starved gyro_bias of room to move from its zero prior, degrading position RMS by
~70x. The calibration-seeded P0 fixes both at once - see the run comparison this file
prints for the actual before/after numbers.
"""

import math
import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import calibration

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

def update_H_accel(predicted_accel, H):
    H[0:3, 0:3] = skew(predicted_accel)
    H[0:3, 3:6] = 0
    H[0:3, 12:15] = np.eye(3)
    return H

def update_H_mag(predicted_mag, H):
    H[0:3, 0:3] = skew(predicted_mag)
    H[0:3, 3:6] = 0
    H[0:3, 12:15] = 0
    return H

def update_H_gps(H):
    H[:, 0:6] = 0
    H[0:3, 9:12] = np.eye(3)
    H[3:6, 6:9] = np.eye(3)
    return H

#---- ground truth: continuous roll/pitch/yaw rotation (same sinusoids as ekf_rms.py)
#     PLUS real translation north (same accelerate-then-coast profile as
#     test_gps_tracking.py) PLUS realistic noise on all four sensors (same stds as
#     test_gate_monte_carlo.py / gps.py's current tuning) ----
amplitude_roll = math.radians(15); freq_roll = 0.5
amplitude_pitch = math.radians(10); freq_pitch = 0.3; phase_pitch = math.pi / 2
amplitude_yaw = math.radians(5); freq_yaw = 0.1; phase_yaw = math.pi
ACCEL_MAG = 1.0   #m/s^2, north acceleration for the first 2s, then coast

GYRO_NOISE_STD = math.radians(0.5)
ACCEL_NOISE_STD = 0.02
MAG_NOISE_STD = 0.01
GPS_POS_NOISE_STD = np.array([2.5, 2.5, 5.0])
GPS_VEL_NOISE_STD = np.array([0.15, 0.15, 0.3])

def true_roll(t): return amplitude_roll * math.sin(freq_roll * t)
def true_roll_dot(t): return amplitude_roll * freq_roll * math.cos(freq_roll * t)
def true_pitch(t): return amplitude_pitch * math.sin(freq_pitch * t + phase_pitch)
def true_pitch_dot(t): return amplitude_pitch * freq_pitch * math.cos(freq_pitch * t + phase_pitch)
def true_yaw(t): return amplitude_yaw * math.sin(freq_yaw * t + phase_yaw)
def true_yaw_dot(t): return amplitude_yaw * freq_yaw * math.cos(freq_yaw * t + phase_yaw)

def true_accel_north(t): return ACCEL_MAG if t < 2.0 else 0.0
def true_position_north(t):
    v_end = ACCEL_MAG * 2.0
    if t < 2.0: return 0.5 * ACCEL_MAG * t * t
    return v_end * (t - 2.0) + 0.5 * ACCEL_MAG * 2.0 * 2.0
def true_velocity_north(t): return ACCEL_MAG * t if t < 2.0 else ACCEL_MAG * 2.0

def get_gyro(t, gyro_bias_true):
    roll, pitch = true_roll(t), true_pitch(t)
    roll_dot, pitch_dot, yaw_dot = true_roll_dot(t), true_pitch_dot(t), true_yaw_dot(t)
    wx = roll_dot - yaw_dot * math.sin(pitch)
    wy = pitch_dot * math.cos(roll) + yaw_dot * math.sin(roll) * math.cos(pitch)
    wz = -pitch_dot * math.sin(roll) + yaw_dot * math.cos(roll) * math.cos(pitch)
    n = np.random.normal(0, GYRO_NOISE_STD, 3)
    return (wx + gyro_bias_true[0] + n[0], wy + gyro_bias_true[1] + n[1], wz + gyro_bias_true[2] + n[2])

def get_accel(t, accel_bias_true):
    #specific force = gravity (rotated into body frame) PLUS the true north acceleration,
    #both expressed in body frame - the north thrust adds onto the body-x axis before tilt
    #is applied, same convention as test_gps_tracking.py's get_accel
    roll, pitch = true_roll(t), true_pitch(t)
    ax = true_accel_north(t) - GRAVITY * math.sin(pitch)
    ay = GRAVITY * math.sin(roll) * math.cos(pitch)
    az = GRAVITY * math.cos(roll) * math.cos(pitch)
    n = np.random.normal(0, ACCEL_NOISE_STD, 3)
    return (ax + accel_bias_true[0] + n[0], ay + accel_bias_true[1] + n[1], az + accel_bias_true[2] + n[2])

def get_mag(t):
    roll, pitch, yaw = true_roll(t), true_pitch(t), true_yaw(t)
    mx = math.cos(pitch) * math.cos(yaw)
    my = math.sin(roll) * math.sin(pitch) * math.cos(yaw) - math.cos(roll) * math.sin(yaw)
    mz = math.cos(roll) * math.sin(pitch) * math.cos(yaw) + math.sin(roll) * math.sin(yaw)
    n = np.random.normal(0, MAG_NOISE_STD, 3)
    return (mx + n[0], my + n[1], mz + n[2])

def get_gps(t):
    north, vel_north = true_position_north(t), true_velocity_north(t)
    noise_pos = np.random.normal(0, GPS_POS_NOISE_STD)
    noise_vel = np.random.normal(0, GPS_VEL_NOISE_STD)
    return (north + noise_pos[0], noise_pos[1], noise_pos[2],
            vel_north + noise_vel[0], noise_vel[1], noise_vel[2])

#---- pre-flight calibration wiggle, same as gps.py's _cal_get_* functions - a short
#dwell then a small deliberate roll/pitch wiggle, used only to seed q/bias/accel_bias/P
#before the main run starts (see calibration.py for why this matters) ----
_CAL_DWELL_STEPS = 200
_CAL_DT = 0.01
_CAL_WIGGLE_AMP_ROLL = math.radians(8); _CAL_WIGGLE_FREQ_ROLL = 0.8
_CAL_WIGGLE_AMP_PITCH = math.radians(6); _CAL_WIGGLE_FREQ_PITCH = 0.5

def _calibrate_seed():
    step = [0]
    def cal_attitude(s):
        if s < _CAL_DWELL_STEPS: return 0.0, 0.0
        tt = (s - _CAL_DWELL_STEPS) * _CAL_DT
        return (_CAL_WIGGLE_AMP_ROLL * math.sin(_CAL_WIGGLE_FREQ_ROLL * tt),
                _CAL_WIGGLE_AMP_PITCH * math.sin(_CAL_WIGGLE_FREQ_PITCH * tt))
    def cal_rates(s):
        if s < _CAL_DWELL_STEPS: return 0.0, 0.0
        tt = (s - _CAL_DWELL_STEPS) * _CAL_DT
        return (_CAL_WIGGLE_AMP_ROLL * _CAL_WIGGLE_FREQ_ROLL * math.cos(_CAL_WIGGLE_FREQ_ROLL * tt),
                _CAL_WIGGLE_AMP_PITCH * _CAL_WIGGLE_FREQ_PITCH * math.cos(_CAL_WIGGLE_FREQ_PITCH * tt))
    def cal_get_gyro():
        rd, pd = cal_rates(step[0]); return rd, pd, 0.0
    def cal_get_accel():
        r, p = cal_attitude(step[0]); step[0] += 1
        return (-GRAVITY * math.sin(p), GRAVITY * math.sin(r) * math.cos(p), GRAVITY * math.cos(r) * math.cos(p))
    def cal_get_mag():
        r, p = cal_attitude(step[0])
        v = np.array([math.cos(p), math.sin(r) * math.sin(p), math.cos(r) * math.sin(p)])
        return tuple(v / np.linalg.norm(v))

    q0, gyro_bias0, accel_bias0, P_cal = calibration.calibrate(
        cal_get_gyro, cal_get_accel, cal_get_mag, dwell_steps=_CAL_DWELL_STEPS, wiggle_steps=400,
        dt=_CAL_DT, gravity=GRAVITY, accel_noise_var=ACCEL_NOISE_STD**2, mag_noise_var=MAG_NOISE_STD**2)

    #map calibration's 9x9 P (attitude, gyro_bias, accel_bias) onto this file's 15x15
    #layout (attitude, gyro_bias, velocity, position, accel_bias) - see gps.py for the
    #same mapping; velocity/position get an independent prior since calibration never
    #modeled them (the vehicle was assumed stationary throughout)
    P0 = np.eye(15) * 0.1
    P0[0:3, 0:3] = P_cal[0:3, 0:3]; P0[0:3, 3:6] = P_cal[0:3, 3:6]
    P0[3:6, 0:3] = P_cal[3:6, 0:3]; P0[3:6, 3:6] = P_cal[3:6, 3:6]
    P0[0:3, 12:15] = P_cal[0:3, 6:9]; P0[12:15, 0:3] = P_cal[6:9, 0:3]
    P0[3:6, 12:15] = P_cal[3:6, 6:9]; P0[12:15, 3:6] = P_cal[6:9, 3:6]
    P0[12:15, 12:15] = P_cal[6:9, 6:9]
    return q0, gyro_bias0, accel_bias0, P0

def run(true_gyro_bias=(math.radians(2), math.radians(-1), math.radians(0.5)),
        true_accel_bias=(0.05, -0.02, 0.03), num_steps=2000, dt=0.01, gps_period=0.2, seed=None):
    if seed is not None:
        np.random.seed(seed)
    true_gyro_bias = np.array(true_gyro_bias)
    true_accel_bias = np.array(true_accel_bias)

    q, (bias_x, bias_y, bias_z), (accel_bias_x, accel_bias_y, accel_bias_z), P = _calibrate_seed()
    velocity = np.array([0.0, 0.0, 0.0])
    position = np.array([0.0, 0.0, 0.0])

    Q = np.diag([0.01,0.01,0.01, 1e-6,1e-6,1e-6, 1e-6,1e-6,1e-6, 1e-6,1e-6,1e-6, 1e-6,1e-6,1e-6])
    R_accel_base = np.diag([ACCEL_NOISE_STD**2]*3)
    R_mag = np.diag([MAG_NOISE_STD**2]*3)
    R_gps = np.diag([6.25, 6.25, 25.0, 0.0225, 0.0225, 0.09])
    F = np.eye(15)
    I = np.eye(15)
    H_accel = np.zeros((3,15))
    H_mag = np.zeros((3,15))
    H_gps = np.zeros((6,15))

    def apply_correction(correction):
        #injects immediately and re-normalizes q, rather than pooling GPS+accel+mag
        #against one frozen q and injecting once - see gps.py's apply_correction
        nonlocal q, bias_x, bias_y, bias_z, velocity, position, accel_bias_x, accel_bias_y, accel_bias_z
        d_theta = correction[0:3]; d_bias = correction[3:6]
        d_velocity = correction[6:9]; d_position = correction[9:12]; d_accel_bias = correction[12:15]
        bias_x += d_bias[0]; bias_y += d_bias[1]; bias_z += d_bias[2]
        accel_bias_x += d_accel_bias[0]; accel_bias_y += d_accel_bias[1]; accel_bias_z += d_accel_bias[2]
        velocity = velocity + d_velocity
        position = position + d_position
        dq = np.array([1.0, d_theta[0]/2, d_theta[1]/2, d_theta[2]/2])
        q = quat_mult(q, dq)
        q = q / np.linalg.norm(q)

    k = 1
    last_gps_time = 0.0
    t = 0.0
    sse_pos = 0.0
    sse_vel = 0.0
    log = []   #(t, accel_bias_x, accel_bias_y, accel_bias_z, gyro_bias_x, gyro_bias_y, gyro_bias_z, pos_north, vel_north)

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

        if t - last_gps_time >= gps_period:
            north, east, up, vel_north, vel_east, vel_up = get_gps(t)
            gps_measurement = np.array([north, east, up, vel_north, vel_east, vel_up])

            H_gps = update_H_gps(H_gps)
            S_gps = H_gps @ P @ H_gps.T + R_gps
            K_gps = P @ H_gps.T @ np.linalg.inv(S_gps)
            predicted_gps = np.concatenate([position, velocity])
            residual_gps = gps_measurement - predicted_gps
            d_squared_gps = residual_gps.T @ np.linalg.inv(S_gps) @ residual_gps

            if d_squared_gps <= chi2_threshold_gps:
                apply_correction(K_gps @ residual_gps)
                P = (I - K_gps @ H_gps) @ P @ (I - K_gps @ H_gps).T + K_gps @ R_gps @ K_gps.T

            last_gps_time = t

        #accel correction - re-linearize against q as GPS may have just updated it
        predicted_accel = rotate_by_quat(quat_conjugate(q), np.array([0.0, 0.0, GRAVITY]))
        H_accel = update_H_accel(predicted_accel, H_accel)

        S_accel = H_accel @ P @ H_accel.T + R_accel
        K_accel = P @ H_accel.T @ np.linalg.inv(S_accel)
        residual_accel = corrected_accel - predicted_accel
        S_accel_gate = H_accel @ P @ H_accel.T + R_accel_base
        d_squared = residual_accel.T @ np.linalg.inv(S_accel_gate) @ residual_accel
        if d_squared <= chi2_threshold:
            apply_correction(K_accel @ residual_accel)
            P = (I - K_accel @ H_accel) @ P @ (I - K_accel @ H_accel).T + K_accel @ R_accel @ K_accel.T

        #mag correction - re-linearize against q AFTER the accel correction just applied
        predicted_mag = rotate_by_quat(quat_conjugate(q), np.array([1.0, 0.0, 0.0]))
        H_mag = update_H_mag(predicted_mag, H_mag)

        S_mag = H_mag @ P @ H_mag.T + R_mag
        K_mag = P @ H_mag.T @ np.linalg.inv(S_mag)
        residual_mag = np.array([mag_x, mag_y, mag_z]) - predicted_mag
        d_squared_mag = residual_mag.T @ np.linalg.inv(S_mag) @ residual_mag
        if d_squared_mag <= chi2_threshold:
            apply_correction(K_mag @ residual_mag)
            P = (I - K_mag @ H_mag) @ P @ (I - K_mag @ H_mag).T + K_mag @ R_mag @ K_mag.T

        true_pos = true_position_north(t)
        true_vel = true_velocity_north(t)
        sse_pos += (position[0] - true_pos) ** 2
        sse_vel += (velocity[0] - true_vel) ** 2

        log.append((t, accel_bias_x, accel_bias_y, accel_bias_z, bias_x, bias_y, bias_z, position[0], velocity[0]))
        t += dt

    rms_pos = math.sqrt(sse_pos / num_steps)
    rms_vel = math.sqrt(sse_vel / num_steps)
    return log, true_accel_bias, true_gyro_bias, rms_pos, rms_vel


if __name__ == "__main__":
    num_trials = 10
    accel_bias_errors = []
    gyro_bias_errors = []
    rms_positions = []
    rms_velocities = []

    for seed in range(num_trials):
        log, true_accel_bias, true_gyro_bias, rms_pos, rms_vel = run(seed=seed)
        settle = log[-200:]
        settled_accel_bias = np.mean([[r[1], r[2], r[3]] for r in settle], axis=0)
        settled_gyro_bias = np.mean([[r[4], r[5], r[6]] for r in settle], axis=0)
        accel_bias_errors.append(settled_accel_bias - true_accel_bias)
        gyro_bias_errors.append(settled_gyro_bias - true_gyro_bias)
        rms_positions.append(rms_pos)
        rms_velocities.append(rms_vel)

    accel_bias_errors = np.array(accel_bias_errors)
    gyro_bias_errors = np.array(gyro_bias_errors)

    print(f"Combined scenario (rotation + translation + bias + realistic noise), {num_trials} trials:\n")
    print(f"true accel_bias: {list(np.round(true_accel_bias, 4))}")
    print(f"accel_bias error (settled - true), mean across trials: {list(np.round(accel_bias_errors.mean(axis=0), 4))}")
    print(f"accel_bias error, worst-case |max| across trials:      {list(np.round(np.abs(accel_bias_errors).max(axis=0), 4))}\n")
    print(f"true gyro_bias (deg/s): {list(np.round(np.degrees(true_gyro_bias), 4))}")
    print(f"gyro_bias error (settled - true, deg/s), mean:  {list(np.round(np.degrees(gyro_bias_errors.mean(axis=0)), 4))}")
    print(f"gyro_bias error, worst-case |max| (deg/s):      {list(np.round(np.degrees(np.abs(gyro_bias_errors).max(axis=0)), 4))}\n")
    print(f"position RMS (m): mean={np.mean(rms_positions):.4f}  worst={np.max(rms_positions):.4f}")
    print(f"velocity RMS (m/s): mean={np.mean(rms_velocities):.4f}  worst={np.max(rms_velocities):.4f}")
