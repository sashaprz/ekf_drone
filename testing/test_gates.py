"""
Checks what the chi-squared gates added to gps.py's GPS and mag corrections actually
do: (1) do they fire at all during normal, clean operation (they shouldn't - a false
rejection would starve the filter of real corrections), and (2) do they actually
protect the state against a single bad reading (GPS multipath spike, mag interference
spike) the way the existing accel gate already did?

Self-contained, mirrors gps.py's current math (GRAVITY-corrected, Joseph-form P
update, all three gates) - ground truth is a vehicle at rest (zero velocity/position,
level attitude) so any deviation after an injected outlier is directly attributable to
that outlier, not to real dynamics. gate_enabled=False bypasses only the gate check
(the correction itself, and its Joseph-form P update, are unchanged) so the comparison
isolates exactly what the `if d_squared <= threshold:` line buys you.
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

def run(gate_enabled, num_steps=300, dt=0.01, gps_period=0.2,
        gps_outlier_step=100, gps_outlier_north=50.0,
        mag_outlier_step=200):
    k = 1
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

    last_gps_time = 0.0
    t = 0.0
    log = []           #(t, |position|, |attitude error proxy|, gps_rejected, mag_rejected)
    gps_rejections = 0
    mag_rejections = 0
    step = 0

    for step in range(num_steps):
        gyro_x, gyro_y, gyro_z = 0.0, 0.0, 0.0                  #at rest, no rotation
        mag_x, mag_y, mag_z = 1.0, 0.0, 0.0                      #clean, level
        accel_x, accel_y, accel_z = 0.0, 0.0, GRAVITY            #clean, level, at rest

        if mag_outlier_step is not None and step == mag_outlier_step:
            mag_x, mag_y, mag_z = 0.0, 1.0, 0.0                  #90deg-wrong mag reading

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
        gps_rejected_this_step = False
        mag_rejected_this_step = False

        if t - last_gps_time >= gps_period:
            north, east, up = 0.0, 0.0, 0.0
            vel_north, vel_east, vel_up = 0.0, 0.0, 0.0
            if step == gps_outlier_step:
                north = gps_outlier_north                        #multipath-style spike

            gps_measurement = np.array([north, east, up, vel_north, vel_east, vel_up])

            H_gps = update_H_gps(H_gps)
            S_gps = H_gps @ P @ H_gps.T + R_gps
            K_gps = P @ H_gps.T @ np.linalg.inv(S_gps)
            predicted_gps = np.concatenate([position, velocity])
            residual_gps = gps_measurement - predicted_gps
            d_squared_gps = residual_gps.T @ np.linalg.inv(S_gps) @ residual_gps

            if (not gate_enabled) or d_squared_gps <= chi2_threshold_gps:
                error_state = error_state + K_gps @ residual_gps
                P = (I - K_gps @ H_gps) @ P @ (I - K_gps @ H_gps).T + K_gps @ R_gps @ K_gps.T
            else:
                gps_rejected_this_step = True
                gps_rejections += 1

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

        if (not gate_enabled) or d_squared_mag <= chi2_threshold:
            error_state = error_state + K_mag @ residual_mag
            P = (I - K_mag @ H_mag) @ P @ (I - K_mag @ H_mag).T + K_mag @ R_mag @ K_mag.T
        else:
            mag_rejected_this_step = True
            mag_rejections += 1

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

        att_err_deg = 2 * math.degrees(math.asin(min(1.0, np.linalg.norm(q[1:]))))
        log.append((t, float(np.linalg.norm(position)), att_err_deg, gps_rejected_this_step, mag_rejected_this_step))

        t += dt

    return log, gps_rejections, mag_rejections


if __name__ == "__main__":
    print("Phase 1: clean run (no outliers) - do the gates ever fire on legitimate data?")
    log_clean, gps_rej_clean, mag_rej_clean = run(gate_enabled=True, gps_outlier_step=None, mag_outlier_step=None)
    print(f"  GPS rejections: {gps_rej_clean}, mag rejections: {mag_rej_clean} (both should be 0)\n")

    print("Phase 2: one bad GPS fix (+50m north spike at step 100) and one bad mag reading (90deg off at step 200)")
    print(f"{'':16s} | {'gated?':>7s} | {'GPS rejected':>13s} | {'mag rejected':>13s} | {'peak |pos| (m)':>15s} | {'peak att err (deg)':>19s}")
    for label, gated in [("Gates ON", True), ("Gates OFF", False)]:
        log, gps_rej, mag_rej = run(gate_enabled=gated)
        peak_pos = max(row[1] for row in log)
        peak_att = max(row[2] for row in log)
        print(f"{label:16s} | {str(gated):>7s} | {gps_rej:13d} | {mag_rej:13d} | {peak_pos:15.4f} | {peak_att:19.4f}")

    print("\nDetail around the GPS outlier (step 100, t=1.00s), gates ON vs OFF:")
    log_on, _, _ = run(gate_enabled=True)
    log_off, _, _ = run(gate_enabled=False)
    print(f"{'t':>6s} | {'|pos| ON':>10s} | {'|pos| OFF':>10s} | {'att err ON':>11s} | {'att err OFF':>12s}")
    for i in [95, 99, 100, 101, 105, 110, 150]:
        t_on, pos_on, att_on, _, _ = log_on[i]
        t_off, pos_off, att_off, _, _ = log_off[i]
        print(f"{t_on:6.2f} | {pos_on:10.4f} | {pos_off:10.4f} | {att_on:11.4f} | {att_off:12.4f}")

    print("\nDetail around the mag outlier (step 200, t=2.00s), gates ON vs OFF:")
    for i in [195, 199, 200, 201, 205, 210, 250]:
        t_on, pos_on, att_on, _, _ = log_on[i]
        t_off, pos_off, att_off, _, _ = log_off[i]
        print(f"{t_on:6.2f} | {pos_on:10.4f} | {pos_off:10.4f} | {att_on:11.4f} | {att_off:12.4f}")
