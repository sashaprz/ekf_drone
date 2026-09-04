"""
RMS comparison: complementary filter (fixed weight, no coupling, no bias) vs
angle-based EKF (ekf_angle_based.py's design - atan2 measurement, no bias state)
vs raw-vector+bias EKF (ekf_gyro_bias.py's design - raw accel/mag vectors,
gyro-bias state) - all three run against the SAME dynamic trajectory and the
SAME synthesized sensor readings each step (fixed RNG seed per trial), so the
comparison isn't polluted by different noise draws. Fixed simulated dt
throughout (see ekf_gyro_bias.py for why).
"""

import math
import numpy as np

#---- shared ground-truth trajectory ----
amplitude_roll = math.radians(15)
freq_roll = 0.5

amplitude_pitch = math.radians(10)
freq_pitch = 0.3
phase_pitch = math.pi / 2

amplitude_yaw = math.radians(5)
freq_yaw = 0.1
phase_yaw = math.pi

def true_roll(t):
    return amplitude_roll * math.sin(freq_roll * t)

def true_roll_dot(t):
    return amplitude_roll * freq_roll * math.cos(freq_roll * t)

def true_pitch(t):
    return amplitude_pitch * math.sin(freq_pitch * t + phase_pitch)

def true_pitch_dot(t):
    return amplitude_pitch * freq_pitch * math.cos(freq_pitch * t + phase_pitch)

def true_yaw(t):
    return amplitude_yaw * math.sin(freq_yaw * t + phase_yaw)

def true_yaw_dot(t):
    return amplitude_yaw * freq_yaw * math.cos(freq_yaw * t + phase_yaw)

#---- shared sensor noise (bias magnitude is set per-trial) ----
gyro_noise_std = math.radians(0.5)
accel_noise_std = 0.02
mag_noise_std = 0.01

def get_gyro(t, bias_x, bias_y, bias_z):
    simulated_gyro_x = true_roll_dot(t) - true_yaw_dot(t) * math.sin(true_pitch(t))
    simulated_gyro_y = true_pitch_dot(t) * math.cos(true_roll(t)) + true_yaw_dot(t) * math.sin(true_roll(t)) * math.cos(true_pitch(t))
    simulated_gyro_z = -true_pitch_dot(t) * math.sin(true_roll(t)) + true_yaw_dot(t) * math.cos(true_roll(t)) * math.cos(true_pitch(t))
    return (simulated_gyro_x + bias_x + np.random.normal(0, gyro_noise_std),
            simulated_gyro_y + bias_y + np.random.normal(0, gyro_noise_std),
            simulated_gyro_z + bias_z + np.random.normal(0, gyro_noise_std))

def get_accel(t):
    true_ax = -math.sin(true_pitch(t))
    true_ay = math.sin(true_roll(t)) * math.cos(true_pitch(t))
    true_az = math.cos(true_roll(t)) * math.cos(true_pitch(t))
    return (true_ax + np.random.normal(0, accel_noise_std),
            true_ay + np.random.normal(0, accel_noise_std),
            true_az + np.random.normal(0, accel_noise_std))

def get_mag(t):
    true_mx = math.cos(true_pitch(t)) * math.cos(true_yaw(t))
    true_my = math.sin(true_roll(t)) * math.sin(true_pitch(t)) * math.cos(true_yaw(t)) - math.cos(true_roll(t)) * math.sin(true_yaw(t))
    true_mz = math.cos(true_roll(t)) * math.sin(true_pitch(t)) * math.cos(true_yaw(t)) + math.sin(true_roll(t)) * math.sin(true_yaw(t))
    return (true_mx + np.random.normal(0, mag_noise_std),
            true_my + np.random.normal(0, mag_noise_std),
            true_mz + np.random.normal(0, mag_noise_std))

def update_F_angle(pitch_dot, pitch, yaw_dot, dt, F):
    F[0][0] = 1 + dt * math.tan(pitch) * pitch_dot
    F[0][1] = dt * yaw_dot / math.cos(pitch)
    F[0][2] = 0
    F[1][0] = -dt * yaw_dot * math.cos(pitch)
    F[1][1] = 1
    F[1][2] = 0
    F[2][0] = dt * pitch_dot / math.cos(pitch)
    F[2][1] = dt * yaw_dot * math.tan(pitch)
    F[2][2] = 1
    return F

def update_F_raw(pitch_dot, pitch, yaw_dot, dt, roll, F):
    F[0][0] = 1 + dt * math.tan(pitch) * pitch_dot
    F[0][1] = dt * yaw_dot / math.cos(pitch)
    F[0][2] = 0
    F[1][0] = -dt * yaw_dot * math.cos(pitch)
    F[1][1] = 1
    F[1][2] = 0
    F[2][0] = dt * pitch_dot / math.cos(pitch)
    F[2][1] = dt * yaw_dot * math.tan(pitch)
    F[2][2] = 1
    F[0][3] = -dt
    F[0][4] = -dt * math.sin(roll) * math.tan(pitch)
    F[0][5] = -dt * math.cos(roll) * math.tan(pitch)
    F[1][3] = 0
    F[1][4] = -dt * math.cos(roll)
    F[1][5] = dt * math.sin(roll)
    F[2][3] = 0
    F[2][4] = -dt * math.sin(roll) / math.cos(pitch)
    F[2][5] = -dt * math.cos(roll) / math.cos(pitch)
    return F

def update_H_raw(roll, pitch, yaw, H_raw):
    H_raw[0] = [0, -math.cos(pitch), 0, 0, 0, 0]
    H_raw[1] = [math.cos(roll)*math.cos(pitch), -math.sin(roll)*math.sin(pitch), 0, 0, 0, 0]
    H_raw[2] = [-math.sin(roll)*math.cos(pitch), -math.cos(roll)*math.sin(pitch), 0, 0, 0, 0]
    my = math.sin(roll)*math.sin(pitch)*math.cos(yaw) - math.cos(roll)*math.sin(yaw)
    mz = math.cos(roll)*math.sin(pitch)*math.cos(yaw) + math.sin(roll)*math.sin(yaw)
    H_raw[3] = [0, -math.sin(pitch)*math.cos(yaw), -math.cos(pitch)*math.sin(yaw), 0, 0, 0]
    H_raw[4] = [mz, math.sin(roll)*math.cos(pitch)*math.cos(yaw), -math.sin(roll)*math.sin(pitch)*math.sin(yaw) - math.cos(roll)*math.cos(yaw), 0, 0, 0]
    H_raw[5] = [-my, math.cos(roll)*math.cos(pitch)*math.cos(yaw), -math.cos(roll)*math.sin(pitch)*math.sin(yaw) + math.sin(roll)*math.cos(yaw), 0, 0, 0]
    return H_raw

def run_trial(bias_x, bias_y, bias_z, num_steps=2000, dt=0.01, seed=0):
    np.random.seed(seed) #same noise draws across trials - only the bias magnitude differs

    #--- Filter C: complementary (fixed weight, no coupling, no bias correction) ---
    weight = 0.98
    roll_c, pitch_c, yaw_c = 0.0, 0.0, 0.0

    #--- Filter A: angle-based EKF (3-state, atan2 measurement, identity H, no bias) ---
    roll_angle, pitch_angle, yaw_angle = 0.0, 0.0, 0.0
    P_angle = np.eye(3)
    Q_angle = np.diag([0.01, 0.01, 0.01])
    R_angle = np.diag([0.1, 0.1, 0.1])
    F_angle = np.eye(3)
    H_angle = np.eye(3)
    I_angle = np.eye(3)

    #--- Filter B: raw-vector + bias EKF (6-state, raw accel/mag vectors, real H jacobian) ---
    roll_raw, pitch_raw, yaw_raw = 0.0, 0.0, 0.0
    bias_x_raw, bias_y_raw, bias_z_raw = 0.0, 0.0, 0.0
    P_raw = np.eye(6)
    Q_raw = np.diag([0.01, 0.01, 0.01, 1e-6, 1e-6, 1e-6])
    R_raw = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    F_raw = np.eye(6)
    H_raw = np.zeros((6, 6))
    I_raw = np.eye(6)

    t = 0.0
    sse_comp = np.zeros(3)
    sse_angle = np.zeros(3)
    sse_raw = np.zeros(3)

    for _ in range(num_steps):

        gyro_x, gyro_y, gyro_z = get_gyro(t, bias_x, bias_y, bias_z)
        accel_x, accel_y, accel_z = get_accel(t)
        mag_x, mag_y, mag_z = get_mag(t)

        #--- Filter C step (complementary - decoupled, fixed weight, no bias correction) ---
        roll_c += gyro_x * dt
        pitch_c += gyro_y * dt
        yaw_c += gyro_z * dt

        mag_x_comp_c = mag_x*math.cos(pitch_c) + mag_y*math.sin(roll_c)*math.sin(pitch_c) + mag_z*math.cos(roll_c)*math.sin(pitch_c)
        mag_y_comp_c = mag_y*math.cos(roll_c) - mag_z*math.sin(roll_c)
        accel_roll_c = math.atan2(accel_y, accel_z)
        accel_pitch_c = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))
        accel_yaw_c = -math.atan2(mag_y_comp_c, mag_x_comp_c)

        roll_c = weight*roll_c + (1-weight)*accel_roll_c
        pitch_c = weight*pitch_c + (1-weight)*accel_pitch_c
        yaw_c = weight*yaw_c + (1-weight)*accel_yaw_c

        #--- Filter A step (angle-based, no bias correction) ---
        roll_dot_a = gyro_x + gyro_y*math.sin(roll_angle)*math.tan(pitch_angle) + gyro_z*math.cos(roll_angle)*math.tan(pitch_angle)
        pitch_dot_a = gyro_y*math.cos(roll_angle) - gyro_z*math.sin(roll_angle)
        yaw_dot_a = (gyro_y*math.sin(roll_angle) + gyro_z*math.cos(roll_angle)) / math.cos(pitch_angle)

        F_angle = update_F_angle(pitch_dot_a, pitch_angle, yaw_dot_a, dt, F_angle)

        roll_angle += roll_dot_a * dt
        pitch_angle += pitch_dot_a * dt
        yaw_angle += yaw_dot_a * dt

        mag_x_comp = mag_x*math.cos(pitch_angle) + mag_y*math.sin(roll_angle)*math.sin(pitch_angle) + mag_z*math.cos(roll_angle)*math.sin(pitch_angle)
        mag_y_comp = mag_y*math.cos(roll_angle) - mag_z*math.sin(roll_angle)
        accel_roll = math.atan2(accel_y, accel_z)
        accel_pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))
        accel_yaw = -math.atan2(mag_y_comp, mag_x_comp)

        P_angle = F_angle @ P_angle @ F_angle.T + Q_angle
        K_angle = P_angle @ H_angle.T @ np.linalg.inv(H_angle @ P_angle @ H_angle.T + R_angle)

        measurement_angle = np.array([accel_roll, accel_pitch, accel_yaw])
        predicted_state_angle = np.array([roll_angle, pitch_angle, yaw_angle])
        residual_angle = measurement_angle - H_angle @ predicted_state_angle

        corrected_angle = predicted_state_angle + K_angle @ residual_angle
        P_angle = (I_angle - K_angle @ H_angle) @ P_angle

        roll_angle, pitch_angle, yaw_angle = corrected_angle

        #--- Filter B step (raw-vector + bias) ---
        corrected_gyro_x = gyro_x - bias_x_raw
        corrected_gyro_y = gyro_y - bias_y_raw
        corrected_gyro_z = gyro_z - bias_z_raw

        roll_dot_b = corrected_gyro_x + corrected_gyro_y*math.sin(roll_raw)*math.tan(pitch_raw) + corrected_gyro_z*math.cos(roll_raw)*math.tan(pitch_raw)
        pitch_dot_b = corrected_gyro_y*math.cos(roll_raw) - corrected_gyro_z*math.sin(roll_raw)
        yaw_dot_b = (corrected_gyro_y*math.sin(roll_raw) + corrected_gyro_z*math.cos(roll_raw)) / math.cos(pitch_raw)

        F_raw = update_F_raw(pitch_dot_b, pitch_raw, yaw_dot_b, dt, roll_raw, F_raw)
        H_raw = update_H_raw(roll_raw, pitch_raw, yaw_raw, H_raw)

        roll_raw += roll_dot_b * dt
        pitch_raw += pitch_dot_b * dt
        yaw_raw += yaw_dot_b * dt

        predicted_accel = np.array([-math.sin(pitch_raw), math.sin(roll_raw)*math.cos(pitch_raw), math.cos(roll_raw)*math.cos(pitch_raw)])
        predicted_mag = np.array([math.cos(pitch_raw)*math.cos(yaw_raw),
                                   math.sin(roll_raw)*math.sin(pitch_raw)*math.cos(yaw_raw) - math.cos(roll_raw)*math.sin(yaw_raw),
                                   math.cos(roll_raw)*math.sin(pitch_raw)*math.cos(yaw_raw) + math.sin(roll_raw)*math.sin(yaw_raw)])

        P_raw = F_raw @ P_raw @ F_raw.T + Q_raw
        K_raw = P_raw @ H_raw.T @ np.linalg.inv(H_raw @ P_raw @ H_raw.T + R_raw)

        measurement_raw = np.array([accel_x, accel_y, accel_z, mag_x, mag_y, mag_z])
        predicted_state_raw = np.array([roll_raw, pitch_raw, yaw_raw, bias_x_raw, bias_y_raw, bias_z_raw])
        predicted_measurement_raw = np.concatenate([predicted_accel, predicted_mag])
        residual_raw = measurement_raw - predicted_measurement_raw

        corrected_raw = predicted_state_raw + K_raw @ residual_raw
        P_raw = (I_raw - K_raw @ H_raw) @ P_raw

        roll_raw, pitch_raw, yaw_raw, bias_x_raw, bias_y_raw, bias_z_raw = corrected_raw

        #--- accumulate squared error against ground truth, in degrees ---
        true_r, true_p, true_y = math.degrees(true_roll(t)), math.degrees(true_pitch(t)), math.degrees(true_yaw(t))

        sse_comp += np.array([math.degrees(roll_c) - true_r,
                               math.degrees(pitch_c) - true_p,
                               math.degrees(yaw_c) - true_y]) ** 2
        sse_angle += np.array([math.degrees(roll_angle) - true_r,
                                math.degrees(pitch_angle) - true_p,
                                math.degrees(yaw_angle) - true_y]) ** 2
        sse_raw += np.array([math.degrees(roll_raw) - true_r,
                              math.degrees(pitch_raw) - true_p,
                              math.degrees(yaw_raw) - true_y]) ** 2

        t += dt

    return np.sqrt(sse_comp / num_steps), np.sqrt(sse_angle / num_steps), np.sqrt(sse_raw / num_steps)

if __name__ == "__main__":
    for label, (bx, by, bz) in [
        ("Original bias", (math.radians(2), math.radians(-1), math.radians(0.5))),
        ("10x larger bias", (math.radians(20), math.radians(-10), math.radians(5))),
    ]:
        rms_comp, rms_angle, rms_raw = run_trial(bx, by, bz)
        print(f"\n{label}:")
        print(f"  Complementary     RMS error (deg): roll={rms_comp[0]:.3f}  pitch={rms_comp[1]:.3f}  yaw={rms_comp[2]:.3f}")
        print(f"  Angle-based EKF   RMS error (deg): roll={rms_angle[0]:.3f}  pitch={rms_angle[1]:.3f}  yaw={rms_angle[2]:.3f}")
        print(f"  Raw-vector+bias   RMS error (deg): roll={rms_raw[0]:.3f}  pitch={rms_raw[1]:.3f}  yaw={rms_raw[2]:.3f}")
