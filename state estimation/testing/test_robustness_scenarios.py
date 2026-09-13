"""
Three robustness scenarios beyond what the other tests cover - none of these were
known-broken, they were just untested:

1. Sustained sensor interference (many consecutive bad mag readings, not just one) -
   test_gates.py only ever injects a single outlier. Does a burst of interference
   (e.g. magnetic noise near motors during sustained high throttle) get rejected the
   whole way through, or does the filter eventually get dragged off and lock up the
   way the pre-fix pooled correction did?

2. Extended GPS dropout - does dead-reckoning degrade predictably (bounded, roughly
   linear/quadratic drift) during a long gap with no GPS, and recover cleanly once GPS
   returns, rather than compounding with bias-estimation weaknesses into something worse?

3. A sudden bias step mid-flight (e.g. real damage/miscalibration event) instead of a
   constant bias present from t=0 - everything else so far assumed the bias was fixed
   for the whole run. How long does the filter take to notice and re-converge?

Mirrors gps.py's current math: GRAVITY, realistic noise, Joseph-form P, sequential
apply_correction, mag_bias, all three gates.
"""

import math
import numpy as np

GRAVITY = 9.80665
chi2_threshold = 11.34
chi2_threshold_gps = 16.81
GYRO_NOISE_STD = math.radians(0.5)
ACCEL_NOISE_STD = 0.02
MAG_NOISE_STD = 0.01
GPS_POS_NOISE_STD = np.array([2.5, 2.5, 5.0])
GPS_VEL_NOISE_STD = np.array([0.15, 0.15, 0.3])

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
    F[15:18, 15:18] = np.eye(3)
    return F

def update_H_accel(predicted_accel, H):
    H[0:3, 0:3] = skew(predicted_accel)
    H[0:3, 3:6] = 0
    H[0:3, 12:15] = np.eye(3)
    H[0:3, 15:18] = 0
    return H

def update_H_mag(predicted_mag, H):
    H[0:3, 0:3] = skew(predicted_mag)
    H[0:3, 3:6] = 0
    H[0:3, 12:15] = 0
    H[0:3, 15:18] = np.eye(3)
    return H

def update_H_gps(H):
    H[:, 0:6] = 0
    H[0:3, 9:12] = np.eye(3)
    H[3:6, 6:9] = np.eye(3)
    return H

#---- ground truth: rotating, hovering (matches gps.py's live loop) ----
AMP_ROLL = math.radians(15); FREQ_ROLL = 0.5
AMP_PITCH = math.radians(10); FREQ_PITCH = 0.3; PHASE_PITCH = math.pi / 2
AMP_YAW = math.radians(5); FREQ_YAW = 0.1; PHASE_YAW = math.pi

def true_roll(t): return AMP_ROLL * math.sin(FREQ_ROLL * t)
def true_roll_dot(t): return AMP_ROLL * FREQ_ROLL * math.cos(FREQ_ROLL * t)
def true_pitch(t): return AMP_PITCH * math.sin(FREQ_PITCH * t + PHASE_PITCH)
def true_pitch_dot(t): return AMP_PITCH * FREQ_PITCH * math.cos(FREQ_PITCH * t + PHASE_PITCH)
def true_yaw(t): return AMP_YAW * math.sin(FREQ_YAW * t + PHASE_YAW)
def true_yaw_dot(t): return AMP_YAW * FREQ_YAW * math.cos(FREQ_YAW * t + PHASE_YAW)

def attitude_error_deg(q):
    #angle between q and identity - a clean scalar "how wrong is attitude" metric
    return 2 * math.degrees(math.asin(min(1.0, np.linalg.norm(q[1:]))))

def run(num_steps=6000, dt=0.01, gps_period=0.2,
        true_gyro_bias=(math.radians(2), math.radians(-1), math.radians(0.5)),
        true_accel_bias=(0.05, -0.02, 0.03), true_mag_bias=(0.04, 0.06, -0.03),
        mag_interference=None,      # (start_step, end_step) - mag reads garbage in this window
        gps_dropout=None,           # (start_step, end_step) - no GPS updates in this window
        accel_bias_step=None,       # (step, new_bias np.array(3)) - true_accel_bias jumps at this step
        seed=0):
    np.random.seed(seed)
    true_gyro_bias = np.array(true_gyro_bias)
    accel_bias_true = np.array(true_accel_bias)
    mag_bias_true = np.array(true_mag_bias)

    q = np.array([1.0, 0.0, 0.0, 0.0])
    bias_x = bias_y = bias_z = 0.0
    accel_bias_x = accel_bias_y = accel_bias_z = 0.0
    mag_bias_x = mag_bias_y = mag_bias_z = 0.0
    velocity = np.array([0.0, 0.0, 0.0])
    position = np.array([0.0, 0.0, 0.0])

    P = np.eye(18) * 0.1
    Q = np.diag([0.01,0.01,0.01, 1e-6,1e-6,1e-6, 1e-6,1e-6,1e-6, 1e-6,1e-6,1e-6, 1e-6,1e-6,1e-6, 1e-6,1e-6,1e-6])
    R_accel_base = np.diag([ACCEL_NOISE_STD**2]*3)
    R_mag = np.diag([MAG_NOISE_STD**2]*3)
    R_gps = np.diag([6.25, 6.25, 25.0, 0.0225, 0.0225, 0.09])
    F = np.eye(18)
    I = np.eye(18)
    H_accel = np.zeros((3,18))
    H_mag = np.zeros((3,18))
    H_gps = np.zeros((6,18))

    def apply_correction(correction):
        nonlocal q, bias_x, bias_y, bias_z, velocity, position, accel_bias_x, accel_bias_y, accel_bias_z, mag_bias_x, mag_bias_y, mag_bias_z
        d_theta = correction[0:3]; d_bias = correction[3:6]
        d_velocity = correction[6:9]; d_position = correction[9:12]
        d_accel_bias = correction[12:15]; d_mag_bias = correction[15:18]
        bias_x += d_bias[0]; bias_y += d_bias[1]; bias_z += d_bias[2]
        accel_bias_x += d_accel_bias[0]; accel_bias_y += d_accel_bias[1]; accel_bias_z += d_accel_bias[2]
        mag_bias_x += d_mag_bias[0]; mag_bias_y += d_mag_bias[1]; mag_bias_z += d_mag_bias[2]
        velocity = velocity + d_velocity
        position = position + d_position
        dq = np.array([1.0, d_theta[0]/2, d_theta[1]/2, d_theta[2]/2])
        q = quat_mult(q, dq)
        q = q / np.linalg.norm(q)

    k = 1
    last_gps_time = -1e9
    t = 0.0
    accel_rejects = mag_rejects = gps_rejects = 0
    log = []   #(t, attitude_error_deg, |position|, accel_bias_x, mag_gated)

    for step in range(num_steps):
        if accel_bias_step is not None and step == accel_bias_step[0]:
            accel_bias_true = np.array(accel_bias_step[1])

        roll, pitch, yaw = true_roll(t), true_pitch(t), true_yaw(t)
        roll_dot, pitch_dot, yaw_dot = true_roll_dot(t), true_pitch_dot(t), true_yaw_dot(t)
        wx = roll_dot - yaw_dot * math.sin(pitch)
        wy = pitch_dot * math.cos(roll) + yaw_dot * math.sin(roll) * math.cos(pitch)
        wz = -pitch_dot * math.sin(roll) + yaw_dot * math.cos(roll) * math.cos(pitch)
        n_gyro = np.random.normal(0, GYRO_NOISE_STD, 3)
        gyro_x, gyro_y, gyro_z = wx+true_gyro_bias[0]+n_gyro[0], wy+true_gyro_bias[1]+n_gyro[1], wz+true_gyro_bias[2]+n_gyro[2]

        ax = -GRAVITY*math.sin(pitch); ay = GRAVITY*math.sin(roll)*math.cos(pitch); az = GRAVITY*math.cos(roll)*math.cos(pitch)
        n_accel = np.random.normal(0, ACCEL_NOISE_STD, 3)
        accel_x, accel_y, accel_z = ax+accel_bias_true[0]+n_accel[0], ay+accel_bias_true[1]+n_accel[1], az+accel_bias_true[2]+n_accel[2]

        if mag_interference is not None and mag_interference[0] <= step < mag_interference[1]:
            #sustained interference: mag reads a wildly wrong, fixed direction (like a
            #nearby current-carrying wire dominating the real field) rather than the true one
            mag_x, mag_y, mag_z = 0.0, 1.0, 0.0
        else:
            mx = math.cos(pitch)*math.cos(yaw); my = math.sin(roll)*math.sin(pitch)*math.cos(yaw) - math.cos(roll)*math.sin(yaw)
            mz = math.cos(roll)*math.sin(pitch)*math.cos(yaw) + math.sin(roll)*math.sin(yaw)
            n_mag = np.random.normal(0, MAG_NOISE_STD, 3)
            mag_x, mag_y, mag_z = mx+mag_bias_true[0]+n_mag[0], my+mag_bias_true[1]+n_mag[1], mz+mag_bias_true[2]+n_mag[2]

        accel_magnitude = math.sqrt(accel_x**2+accel_y**2+accel_z**2)
        deviation = abs(accel_magnitude - GRAVITY)
        R_accel = R_accel_base * (1 + k*deviation**2)

        cgx, cgy, cgz = gyro_x-bias_x, gyro_y-bias_y, gyro_z-bias_z
        cax, cay, caz = accel_x-accel_bias_x, accel_y-accel_bias_y, accel_z-accel_bias_z
        cmx, cmy, cmz = mag_x-mag_bias_x, mag_y-mag_bias_y, mag_z-mag_bias_z

        w_quat = [0, cgx, cgy, cgz]
        q_dot = 0.5 * quat_mult(q, w_quat)
        q = q + q_dot*dt; q = q/np.linalg.norm(q)

        corrected_accel = np.array([cax, cay, caz])
        accel_world = rotate_by_quat(q, corrected_accel) - np.array([0.0,0.0,GRAVITY])
        velocity = velocity + accel_world*dt
        position = position + velocity*dt

        w = np.array([cgx, cgy, cgz])
        F = update_F(w, dt, F, q, corrected_accel)
        P = F@P@F.T + Q

        gps_active = gps_dropout is None or not (gps_dropout[0] <= step < gps_dropout[1])
        if gps_active and t - last_gps_time >= gps_period:
            n_pos = np.random.normal(0, GPS_POS_NOISE_STD)
            n_vel = np.random.normal(0, GPS_VEL_NOISE_STD)
            gps_measurement = np.array([n_pos[0], n_pos[1], n_pos[2], n_vel[0], n_vel[1], n_vel[2]])  #hovering: true pos/vel = 0
            H_gps = update_H_gps(H_gps)
            S_gps = H_gps@P@H_gps.T + R_gps
            K_gps = P@H_gps.T@np.linalg.inv(S_gps)
            predicted_gps = np.concatenate([position, velocity])
            residual_gps = gps_measurement - predicted_gps
            d2 = residual_gps.T@np.linalg.inv(S_gps)@residual_gps
            if d2 <= chi2_threshold_gps:
                apply_correction(K_gps@residual_gps)
                P = (I-K_gps@H_gps)@P@(I-K_gps@H_gps).T + K_gps@R_gps@K_gps.T
            else:
                gps_rejects += 1
            last_gps_time = t

        predicted_accel = rotate_by_quat(quat_conjugate(q), np.array([0.0,0.0,GRAVITY]))
        H_accel = update_H_accel(predicted_accel, H_accel)
        S_accel = H_accel@P@H_accel.T + R_accel
        K_accel = P@H_accel.T@np.linalg.inv(S_accel)
        residual_accel = corrected_accel - predicted_accel
        S_accel_gate = H_accel@P@H_accel.T + R_accel_base
        d2 = residual_accel.T@np.linalg.inv(S_accel_gate)@residual_accel
        if d2 <= chi2_threshold:
            apply_correction(K_accel@residual_accel)
            P = (I-K_accel@H_accel)@P@(I-K_accel@H_accel).T + K_accel@R_accel@K_accel.T
        else:
            accel_rejects += 1

        predicted_mag = rotate_by_quat(quat_conjugate(q), np.array([1.0,0.0,0.0]))
        H_mag = update_H_mag(predicted_mag, H_mag)
        S_mag = H_mag@P@H_mag.T + R_mag
        K_mag = P@H_mag.T@np.linalg.inv(S_mag)
        corrected_mag = np.array([cmx, cmy, cmz])
        residual_mag = corrected_mag - predicted_mag
        d2 = residual_mag.T@np.linalg.inv(S_mag)@residual_mag
        mag_gated = d2 > chi2_threshold
        if not mag_gated:
            apply_correction(K_mag@residual_mag)
            P = (I-K_mag@H_mag)@P@(I-K_mag@H_mag).T + K_mag@R_mag@K_mag.T
        else:
            mag_rejects += 1

        log.append((t, attitude_error_deg(q), float(np.linalg.norm(position)), accel_bias_x, mag_gated))
        t += dt

    return log, accel_rejects, mag_rejects, gps_rejects


if __name__ == "__main__":
    dt = 0.01

    print("=== Scenario 1: sustained mag interference (5s burst, steps 2000-2500) ===")
    log, ar, mr, gr = run(num_steps=6000, mag_interference=(2000, 2500))
    burst_start, burst_end = 2000*dt, 2500*dt
    peak_during = max(row[1] for row in log if burst_start <= row[0] < burst_end)
    peak_after = max(row[1] for row in log if burst_end <= row[0] < burst_end + 2.0)
    final_att_err = log[-1][1]
    rejected_during_burst = sum(1 for row in log if burst_start <= row[0] < burst_end and row[4])
    total_during_burst = sum(1 for row in log if burst_start <= row[0] < burst_end)
    print(f"mag rejected during burst: {rejected_during_burst}/{total_during_burst}")
    print(f"peak attitude error during burst: {peak_during:.3f} deg")
    print(f"peak attitude error in the 2s after burst ends: {peak_after:.3f} deg")
    print(f"final attitude error (run end): {final_att_err:.3f} deg")

    print("\n=== Scenario 2: extended GPS dropout (30s, steps 2000-5000) ===")
    log, ar, mr, gr = run(num_steps=8000, gps_dropout=(2000, 5000))
    drop_start, drop_end = 2000*dt, 5000*dt
    for check_t in [drop_start, drop_start+10, drop_start+20, drop_end-0.01, drop_end+5, 79.99]:
        closest = min(log, key=lambda r: abs(r[0]-check_t))
        print(f"t={closest[0]:6.2f}s  |position|={closest[2]:8.3f}m")

    print("\n=== Scenario 3: sudden accel_bias step mid-flight (t=20s: 0.05->0.20 on x) ===")
    log, ar, mr, gr = run(num_steps=6000, accel_bias_step=(2000, np.array([0.20, -0.02, 0.03])))
    for check_t in [19.9, 20.0, 20.5, 22, 25, 30, 40, 59.9]:
        closest = min(log, key=lambda r: abs(r[0]-check_t))
        print(f"t={closest[0]:6.2f}s  accel_bias_x est={closest[3]:.4f}  (true: {'0.05' if closest[0]<20 else '0.20'})")
