"""
Checks the chi-squared gates are calibrated, not just protective: on clean (outlier-free)
but genuinely NOISY data, a 99%-confidence gate should reject roughly 1% of legitimate
readings by chance - not 0% (too loose to ever catch a real outlier) and not way more
than 1% (starving the filter of real corrections). test_gates.py already proved the
gates catch deliberate outliers; this checks the other side of that tradeoff.

Uses gps.py's actual current trajectory/noise model (rotating attitude, hovering
translation, realistic noise stds) across many independent trials for enough GPS gate
evaluations to get a meaningful empirical rate (GPS only gates once per gps_period,
so one trial's ~25 evaluations per 5s isn't enough on its own).

Also seeds q/bias/accel_bias/P from calibration.calibrate(), same as gps.py's actual
pre-flight step - this file is what FOUND the ~30% lockout that a blind np.eye(15) P0
caused (see the git history / conversation for the np.eye(15) numbers); with the real
calibration-seeded P0, that lockout should be gone, which is what this file now checks
as a standing regression test rather than a one-off diagnosis.
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

#---- ground truth: same rotating/hovering scenario as gps.py's live loop ----
TRUE_AMP_ROLL = math.radians(15); TRUE_FREQ_ROLL = 0.5
TRUE_AMP_PITCH = math.radians(10); TRUE_FREQ_PITCH = 0.3; TRUE_PHASE_PITCH = math.pi / 2
TRUE_AMP_YAW = math.radians(5); TRUE_FREQ_YAW = 0.1; TRUE_PHASE_YAW = math.pi
TRUE_GYRO_BIAS = np.array([math.radians(2), math.radians(-1), math.radians(0.5)])
TRUE_ACCEL_BIAS = np.array([0.05, -0.02, 0.03])
GYRO_NOISE_STD = math.radians(0.5)
ACCEL_NOISE_STD = 0.02
MAG_NOISE_STD = 0.01
GPS_POS_NOISE_STD = np.array([2.5, 2.5, 5.0])
GPS_VEL_NOISE_STD = np.array([0.15, 0.15, 0.3])

def true_roll(t): return TRUE_AMP_ROLL * math.sin(TRUE_FREQ_ROLL * t)
def true_roll_dot(t): return TRUE_AMP_ROLL * TRUE_FREQ_ROLL * math.cos(TRUE_FREQ_ROLL * t)
def true_pitch(t): return TRUE_AMP_PITCH * math.sin(TRUE_FREQ_PITCH * t + TRUE_PHASE_PITCH)
def true_pitch_dot(t): return TRUE_AMP_PITCH * TRUE_FREQ_PITCH * math.cos(TRUE_FREQ_PITCH * t + TRUE_PHASE_PITCH)
def true_yaw(t): return TRUE_AMP_YAW * math.sin(TRUE_FREQ_YAW * t + TRUE_PHASE_YAW)
def true_yaw_dot(t): return TRUE_AMP_YAW * TRUE_FREQ_YAW * math.cos(TRUE_FREQ_YAW * t + TRUE_PHASE_YAW)

def get_gyro(t):
    roll, pitch = true_roll(t), true_pitch(t)
    roll_dot, pitch_dot, yaw_dot = true_roll_dot(t), true_pitch_dot(t), true_yaw_dot(t)
    wx = roll_dot - yaw_dot * math.sin(pitch)
    wy = pitch_dot * math.cos(roll) + yaw_dot * math.sin(roll) * math.cos(pitch)
    wz = -pitch_dot * math.sin(roll) + yaw_dot * math.cos(roll) * math.cos(pitch)
    n = np.random.normal(0, GYRO_NOISE_STD, 3)
    return (wx+TRUE_GYRO_BIAS[0]+n[0], wy+TRUE_GYRO_BIAS[1]+n[1], wz+TRUE_GYRO_BIAS[2]+n[2])

def get_accel(t):
    roll, pitch = true_roll(t), true_pitch(t)
    ax = -GRAVITY * math.sin(pitch)
    ay = GRAVITY * math.sin(roll) * math.cos(pitch)
    az = GRAVITY * math.cos(roll) * math.cos(pitch)
    n = np.random.normal(0, ACCEL_NOISE_STD, 3)
    return (ax+TRUE_ACCEL_BIAS[0]+n[0], ay+TRUE_ACCEL_BIAS[1]+n[1], az+TRUE_ACCEL_BIAS[2]+n[2])

def get_mag(t):
    roll, pitch, yaw = true_roll(t), true_pitch(t), true_yaw(t)
    mx = math.cos(pitch) * math.cos(yaw)
    my = math.sin(roll) * math.sin(pitch) * math.cos(yaw) - math.cos(roll) * math.sin(yaw)
    mz = math.cos(roll) * math.sin(pitch) * math.cos(yaw) + math.sin(roll) * math.sin(yaw)
    n = np.random.normal(0, MAG_NOISE_STD, 3)
    return (mx+n[0], my+n[1], mz+n[2])

def get_gps():
    np_ = np.random.normal(0, GPS_POS_NOISE_STD)
    nv_ = np.random.normal(0, GPS_VEL_NOISE_STD)
    return (np_[0], np_[1], np_[2], nv_[0], nv_[1], nv_[2])

#---- pre-flight calibration wiggle, same as gps.py's _cal_get_* functions ----
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

    P0 = np.eye(15) * 0.1
    P0[0:3, 0:3] = P_cal[0:3, 0:3]; P0[0:3, 3:6] = P_cal[0:3, 3:6]
    P0[3:6, 0:3] = P_cal[3:6, 0:3]; P0[3:6, 3:6] = P_cal[3:6, 3:6]
    P0[0:3, 12:15] = P_cal[0:3, 6:9]; P0[12:15, 0:3] = P_cal[6:9, 0:3]
    P0[3:6, 12:15] = P_cal[3:6, 6:9]; P0[12:15, 3:6] = P_cal[6:9, 3:6]
    P0[12:15, 12:15] = P_cal[6:9, 6:9]
    return q0, gyro_bias0, accel_bias0, P0

def run_trial(seed, num_steps=2000, dt=0.01, gps_period=0.2):
    np.random.seed(seed)
    k = 1
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

    last_gps_time = 0.0
    t = 0.0
    accel_evals = accel_rejects = 0
    mag_evals = mag_rejects = 0
    gps_evals = gps_rejects = 0

    for _ in range(num_steps):
        gyro_x, gyro_y, gyro_z = get_gyro(t)
        mag_x, mag_y, mag_z = get_mag(t)
        accel_x, accel_y, accel_z = get_accel(t)

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
        accel_world = rotate_by_quat(q, corrected_accel) - np.array([0.0, 0.0, GRAVITY])
        velocity = velocity + accel_world * dt
        position = position + velocity * dt

        w = np.array([corrected_gyro_x, corrected_gyro_y, corrected_gyro_z])
        F = update_F(w, dt, F, q, corrected_accel)
        P = F @ P @ F.T + Q

        if t - last_gps_time >= gps_period:
            north, east, up, vel_north, vel_east, vel_up = get_gps()
            gps_measurement = np.array([north, east, up, vel_north, vel_east, vel_up])
            H_gps = update_H_gps(H_gps)
            S_gps = H_gps @ P @ H_gps.T + R_gps
            K_gps = P @ H_gps.T @ np.linalg.inv(S_gps)
            predicted_gps = np.concatenate([position, velocity])
            residual_gps = gps_measurement - predicted_gps
            d_squared_gps = residual_gps.T @ np.linalg.inv(S_gps) @ residual_gps

            gps_evals += 1
            if d_squared_gps <= chi2_threshold_gps:
                apply_correction(K_gps @ residual_gps)
                P = (I - K_gps @ H_gps) @ P @ (I - K_gps @ H_gps).T + K_gps @ R_gps @ K_gps.T
            else:
                gps_rejects += 1
            last_gps_time = t

        #accel correction - re-linearize against q as GPS may have just updated it
        predicted_accel = rotate_by_quat(quat_conjugate(q), np.array([0.0, 0.0, GRAVITY]))
        H_accel = update_H_accel(predicted_accel, H_accel)

        S_accel = H_accel @ P @ H_accel.T + R_accel
        K_accel = P @ H_accel.T @ np.linalg.inv(S_accel)
        residual_accel = corrected_accel - predicted_accel
        S_accel_gate = H_accel @ P @ H_accel.T + R_accel_base
        d_squared = residual_accel.T @ np.linalg.inv(S_accel_gate) @ residual_accel

        accel_evals += 1
        if d_squared <= chi2_threshold:
            apply_correction(K_accel @ residual_accel)
            P = (I - K_accel @ H_accel) @ P @ (I - K_accel @ H_accel).T + K_accel @ R_accel @ K_accel.T
        else:
            accel_rejects += 1

        #mag correction - re-linearize against q AFTER the accel correction just applied
        predicted_mag = rotate_by_quat(quat_conjugate(q), np.array([1.0, 0.0, 0.0]))
        H_mag = update_H_mag(predicted_mag, H_mag)

        S_mag = H_mag @ P @ H_mag.T + R_mag
        K_mag = P @ H_mag.T @ np.linalg.inv(S_mag)
        residual_mag = np.array([mag_x, mag_y, mag_z]) - predicted_mag
        d_squared_mag = residual_mag.T @ np.linalg.inv(S_mag) @ residual_mag

        mag_evals += 1
        if d_squared_mag <= chi2_threshold:
            apply_correction(K_mag @ residual_mag)
            P = (I - K_mag @ H_mag) @ P @ (I - K_mag @ H_mag).T + K_mag @ R_mag @ K_mag.T
        else:
            mag_rejects += 1

        t += dt

    return accel_evals, accel_rejects, mag_evals, mag_rejects, gps_evals, gps_rejects


if __name__ == "__main__":
    num_trials = 20
    tot_accel_evals = tot_accel_rejects = 0
    tot_mag_evals = tot_mag_rejects = 0
    tot_gps_evals = tot_gps_rejects = 0

    for seed in range(num_trials):
        ae, ar, me, mr, ge, gr = run_trial(seed)
        tot_accel_evals += ae; tot_accel_rejects += ar
        tot_mag_evals += me; tot_mag_rejects += mr
        tot_gps_evals += ge; tot_gps_rejects += gr

    print(f"{num_trials} trials x 2000 steps (20s each), no injected outliers - expected false-reject rate ~1% (99% CI threshold)\n")
    print(f"{'gate':10s} | {'evals':>8s} | {'rejects':>8s} | {'empirical rate':>15s}")
    print(f"{'accel':10s} | {tot_accel_evals:8d} | {tot_accel_rejects:8d} | {100*tot_accel_rejects/tot_accel_evals:14.2f}%")
    print(f"{'mag':10s} | {tot_mag_evals:8d} | {tot_mag_rejects:8d} | {100*tot_mag_rejects/tot_mag_evals:14.2f}%")
    print(f"{'gps':10s} | {tot_gps_evals:8d} | {tot_gps_rejects:8d} | {100*tot_gps_rejects/tot_gps_evals:14.2f}%")
