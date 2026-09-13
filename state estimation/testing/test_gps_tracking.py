"""
Position tracking test for the quaternion MEKF + GPS extension (gps.py's design):
inject a constant accelerometer error (a real-world bias the filter has no state
for) so pure inertial dead-reckoning drifts away from ground truth, then compare
WITH GPS correction enabled vs WITHOUT - isolates whether the H_gps/K_gps/R_gps
pipeline is actually pulling the position estimate back toward truth, not just
running without crashing. Fixed dt (deterministic), no wall-clock, no sleep.

Also exercises the accel_bias state + H[0:3,12:15] Jacobian (gps.py's
update_H_mag_accel): `use_bias_column` toggles that Jacobian term on/off so the
accel-bias scenarios below show whether the filter actually estimates and
corrects the injected accelerometer bias, or just carries it as unobservable
dead weight (the bug state, before the fix).
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

def update_H_mag_accel(predicted_accel, predicted_mag, H, use_bias_column):
    H[0:3, 0:3] = skew(predicted_accel)
    H[0:3, 3:6] = 0
    H[0:3, 12:15] = np.eye(3) if use_bias_column else 0
    H[3:6, 0:3] = skew(predicted_mag)
    H[3:6, 3:6] = 0
    H[3:6, 12:15] = 0
    return H

def update_H_gps(H):
    H[:, 0:6] = 0
    H[0:3, 9:12] = np.eye(3)
    H[3:6, 6:9] = np.eye(3)
    return H

#---- ground truth: accelerate north (x) at accel_mag m/s^2 for 2s, then coast at constant velocity.
#     roll_tilt_deg holds a static roll throughout (rotation axis = x = the direction of travel,
#     so it doesn't affect the north-axis numbers - it's there to exercise quat_to_R(q) at a genuinely
#     non-identity attitude, not just q~=identity, while accelerating) ----

def true_accel_north(t, accel_mag):
    return accel_mag if t < 2.0 else 0.0

def true_position_north(t, accel_mag):
    v_end = accel_mag * 2.0
    if t < 2.0:
        return 0.5 * accel_mag * t * t
    return v_end * (t - 2.0) + 0.5 * accel_mag * 2.0 * 2.0

def true_velocity_north(t, accel_mag):
    return accel_mag * t if t < 2.0 else accel_mag * 2.0

def get_gyro():
    return 0.0, 0.0, 0.0 #non-rotating throughout - isolates translational tracking (roll_tilt_deg is a fixed, not moving, tilt)

def get_accel(t, accel_mag, bias_error, roll_tilt_rad):
    #roll is about the x-axis (direction of travel), so it only ever redistributes
    #gravity across y/z - the x (north) component is untouched by tilt.
    #bias_error is injected directly onto the raw body-x reading, same as a real
    #accelerometer bias would be - this is what accel_bias_x is supposed to estimate.
    return true_accel_north(t, accel_mag) + bias_error, math.sin(roll_tilt_rad), math.cos(roll_tilt_rad)

def get_mag():
    return 1.0, 0.0, 0.0 #world-north reference is along the roll axis too, so also untouched by roll tilt

def get_gps(t, accel_mag):
    #noiseless GPS matching ground truth exactly - position and velocity
    return true_position_north(t, accel_mag), 0.0, 0.0, true_velocity_north(t, accel_mag), 0.0, 0.0

def run(gps_enabled, accel_mag=1.0, bias_error=0.0, roll_tilt_deg=0.0, use_bias_column=True, num_steps=400, dt=0.01, gps_period=0.2):
    roll_tilt_rad = math.radians(roll_tilt_deg)
    k = 1
    chi2_threshold = 11.34
    last_gps_time = 0.0 #matches gps.py's gating: GPS only corrects once per gps_period, not every predict step

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

    t = 0.0
    sse_position = 0.0
    sse_velocity = 0.0
    log = []

    for _ in range(num_steps):
        gyro_x, gyro_y, gyro_z = get_gyro()
        mag_x, mag_y, mag_z = get_mag()
        accel_x, accel_y, accel_z = get_accel(t, accel_mag, bias_error, roll_tilt_rad)

        accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        deviation = abs(accel_magnitude - 1.0)
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
        accel_world = accel_world - np.array([0.0, 0.0, 1.0])
        velocity = velocity + accel_world * dt
        position = position + velocity * dt

        w = np.array([corrected_gyro_x, corrected_gyro_y, corrected_gyro_z])
        F = update_F(w, dt, F, q, corrected_accel)
        P = F @ P @ F.T + Q

        predicted_accel = rotate_by_quat(quat_conjugate(q), np.array([0.0, 0.0, 1.0]))
        predicted_mag = rotate_by_quat(quat_conjugate(q), np.array([1.0, 0.0, 0.0]))
        H_accel_mag = update_H_mag_accel(predicted_accel, predicted_mag, H_accel_mag, use_bias_column)
        H_accel = H_accel_mag[0:3, :]
        error_state = np.zeros(15)

        S_accel = H_accel @ P @ H_accel.T + R_accel
        K_accel = P @ H_accel.T @ np.linalg.inv(S_accel)
        residual_accel = corrected_accel - predicted_accel
        S_accel_gate = H_accel @ P @ H_accel.T + R_accel_base
        d_squared = residual_accel.T @ np.linalg.inv(S_accel_gate) @ residual_accel
        if d_squared <= chi2_threshold:
            error_state = error_state + K_accel @ residual_accel
            P = (I - K_accel @ H_accel) @ P

        H_mag = H_accel_mag[3:]
        K_mag = P @ H_mag.T @ np.linalg.inv(H_mag @ P @ H_mag.T + R_mag)
        residual_mag = np.array([mag_x, mag_y, mag_z]) - predicted_mag
        error_state = error_state + K_mag @ residual_mag
        P = (I - K_mag @ H_mag) @ P

        if gps_enabled and (t - last_gps_time >= gps_period):
            north, east, up, vel_north, vel_east, vel_up = get_gps(t, accel_mag)
            gps_measurement = np.array([north, east, up, vel_north, vel_east, vel_up])

            H_gps = update_H_gps(H_gps)
            S_gps = H_gps @ P @ H_gps.T + R_gps
            K_gps = P @ H_gps.T @ np.linalg.inv(S_gps)
            predicted_gps = np.concatenate([position, velocity])
            residual_gps = gps_measurement - predicted_gps
            error_state = error_state + K_gps @ residual_gps
            P = (I - K_gps @ H_gps) @ P

            last_gps_time = t

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

        true_pos = true_position_north(t, accel_mag)
        true_vel = true_velocity_north(t, accel_mag)
        sse_position += (position[0] - true_pos) ** 2
        sse_velocity += (velocity[0] - true_vel) ** 2
        log.append((t, position[0], true_pos, velocity[0], true_vel))

        t += dt

    rms_pos = math.sqrt(sse_position / num_steps)
    rms_vel = math.sqrt(sse_velocity / num_steps)
    return rms_pos, rms_vel, log, (accel_bias_x, accel_bias_y, accel_bias_z)

if __name__ == "__main__":
    #(label, accel_mag [m/s^2], bias_error [m/s^2], roll_tilt_deg)
    scenarios = [
        ("Gentle accel, clean sensor, level",        1.0, 0.00, 0.0),
        ("Aggressive accel, clean sensor, level",    4.0, 0.00, 0.0),
        ("Gentle accel, with accel bias, level",     1.0, 0.05, 0.0),
        ("Aggressive accel, with accel bias, level", 4.0, 0.05, 0.0),
        ("Gentle accel, clean sensor, 10deg tilt",   1.0, 0.00, 10.0),
        ("Aggressive accel, clean sensor, 10deg tilt", 4.0, 0.00, 10.0),
    ]

    print(f"{'scenario':44s} | {'pos RMS (GPS)':>13s} | {'pos RMS (no GPS)':>16s} | {'vel RMS (GPS)':>13s} | {'vel RMS (no GPS)':>16s}")
    results = {}
    for label, accel_mag, bias_error, tilt in scenarios:
        rms_pos_with, rms_vel_with, log_with, _ = run(gps_enabled=True, accel_mag=accel_mag, bias_error=bias_error, roll_tilt_deg=tilt)
        rms_pos_without, rms_vel_without, log_without, _ = run(gps_enabled=False, accel_mag=accel_mag, bias_error=bias_error, roll_tilt_deg=tilt)
        results[label] = (log_with, log_without)
        print(f"{label:44s} | {rms_pos_with:13.4f} | {rms_pos_without:16.4f} | {rms_vel_with:13.4f} | {rms_vel_without:16.4f}")

    #detailed trajectory table for the most demanding case (aggressive accel + bias + tilt combined isn't
    #in the matrix above on purpose - keeping each scenario isolated to one variable at a time - so show
    #the aggressive-accel/clean-sensor/tilted case here, since it's the strongest test of quat_to_R(q)
    #being applied correctly at a genuinely non-identity attitude while accelerating)
    label = "Aggressive accel, clean sensor, 10deg tilt"
    log_with, log_without = results[label]
    print(f"\nDetail for: {label}")
    print("  t   | pos est (GPS) | pos est (no GPS) | pos true | vel est (GPS) | vel true")
    for i in [0, 50, 100, 150, 199, 200, 250, 300, 350, 399]:
        t_i, pos_gps, pos_true, vel_gps, vel_true = log_with[i]
        _, pos_nogps, _, _, _ = log_without[i]
        print(f"{t_i:5.2f} | {pos_gps:13.4f} | {pos_nogps:17.4f} | {pos_true:8.4f} | {vel_gps:13.4f} | {vel_true:8.4f}")

    #---- accel_bias Jacobian fix: does the filter actually estimate + correct the
    #     constant accelerometer bias injected via bias_error, or is it unobservable
    #     without H[0:3,12:15]? Compare use_bias_column=True (fix) vs False (the bug -
    #     accel_bias state exists and propagates through F, but the measurement never
    #     tells the filter anything about it directly). GPS enabled in both, since
    #     without GPS the velocity/bias/attitude coupling has no independent truth
    #     reference to correct against at all. ----
    print("\naccel_bias fix check (GPS enabled) - does the filter estimate the injected bias?")
    print(f"{'scenario':44s} | {'pos RMS (fix)':>13s} | {'pos RMS (no fix)':>16s} | {'bias_x est (fix)':>16s} | {'bias_x est (no fix)':>19s}")
    bias_scenarios = [
        ("Gentle accel, with accel bias, level",      1.0, 0.05, 0.0),
        ("Aggressive accel, with accel bias, level",  4.0, 0.05, 0.0),
        ("Gentle accel, with accel bias, 10deg tilt", 1.0, 0.05, 10.0),
    ]
    for label, accel_mag, bias_error, tilt in bias_scenarios:
        rms_pos_fix, rms_vel_fix, _, bias_fix = run(gps_enabled=True, accel_mag=accel_mag, bias_error=bias_error, roll_tilt_deg=tilt, use_bias_column=True)
        rms_pos_nofix, rms_vel_nofix, _, bias_nofix = run(gps_enabled=True, accel_mag=accel_mag, bias_error=bias_error, roll_tilt_deg=tilt, use_bias_column=False)
        print(f"{label:44s} | {rms_pos_fix:13.4f} | {rms_pos_nofix:16.4f} | {bias_fix[0]:16.4f} | {bias_nofix[0]:19.4f}")
    print("\n(true injected bias_x = 0.05 for all three scenarios above - bias_x est should converge toward that with the fix)")
