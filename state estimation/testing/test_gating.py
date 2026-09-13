"""
Stress test for accel gating: inject an artificial, physically-absurd accel
spike for a few iterations (simulating a violent jolt/collision, not just
normal maneuvering) and compare roll/pitch tracking WITH gating enabled vs
disabled. Adaptive R stays on in both runs, so this isolates what gating adds
on top of it. Fixed dt (deterministic), no wall-clock, no sleep - fast to run.
"""

import math
import numpy as np

def run(gating_enabled, num_steps=400, spike_start=150, spike_end=160):
    k = 1
    chi2_threshold = 11.34 if gating_enabled else 1e18 #effectively disables the gate

    gyro_x_dps, gyro_y_dps, gyro_z_dps = 5, 3, 0
    mag_x, mag_y, mag_z = 1.0, 0, 0

    roll, pitch, yaw = 0.0, 0.0, 0.0
    bias_x, bias_y, bias_z = 0.0, 0.0, 0.0

    P = np.eye(6)
    Q = np.diag([0.01, 0.01, 0.01, 1e-6, 1e-6, 1e-6])
    R_accel_base = np.diag([0.1, 0.1, 0.1])
    R_mag = np.diag([0.1, 0.1, 0.1])
    F = np.eye(6)
    I = np.eye(6)
    H = np.zeros((6, 6))

    def update_F(pitch_dot, pitch, yaw_dot, dt, roll, F):
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

    def update_H(roll, pitch, yaw):
        H[0] = [0, -math.cos(pitch), 0, 0, 0, 0]
        H[1] = [math.cos(roll)*math.cos(pitch), -math.sin(roll)*math.sin(pitch), 0, 0, 0, 0]
        H[2] = [-math.sin(roll)*math.cos(pitch), -math.cos(roll)*math.sin(pitch), 0, 0, 0, 0]
        my = math.sin(roll)*math.sin(pitch)*math.cos(yaw) - math.cos(roll)*math.sin(yaw)
        mz = math.cos(roll)*math.sin(pitch)*math.cos(yaw) + math.sin(roll)*math.sin(yaw)
        H[3] = [0, -math.sin(pitch)*math.cos(yaw), -math.cos(pitch)*math.sin(yaw), 0, 0, 0]
        H[4] = [mz, math.sin(roll)*math.cos(pitch)*math.cos(yaw), -math.sin(roll)*math.sin(pitch)*math.sin(yaw) - math.cos(roll)*math.cos(yaw), 0, 0, 0]
        H[5] = [-my, math.cos(roll)*math.cos(pitch)*math.cos(yaw), -math.cos(roll)*math.sin(pitch)*math.sin(yaw) + math.sin(roll)*math.cos(yaw), 0, 0, 0]
        return H

    dt = 0.01
    roll_log = []

    for i in range(num_steps):
        gyro_x = math.radians(gyro_x_dps)
        gyro_y = math.radians(gyro_y_dps)
        gyro_z = math.radians(gyro_z_dps)

        if spike_start <= i < spike_end:
            #physically-absurd jolt: nowhere near 1g, wildly inconsistent with gravity-only tilt
            accel_x, accel_y, accel_z = 5.0, 5.0, 5.0
        else:
            accel_x = 0
            accel_y = math.sin(math.radians(10))
            accel_z = math.cos(math.radians(10))

        accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        deviation = abs(accel_magnitude - 1.0)
        R_accel = R_accel_base * (1 + k * deviation ** 2)

        corrected_gyro_x = gyro_x - bias_x
        corrected_gyro_y = gyro_y - bias_y
        corrected_gyro_z = gyro_z - bias_z

        roll_dot = corrected_gyro_x + corrected_gyro_y * math.sin(roll) * math.tan(pitch) + corrected_gyro_z * math.cos(roll) * math.tan(pitch)
        pitch_dot = corrected_gyro_y * math.cos(roll) - corrected_gyro_z * math.sin(roll)
        yaw_dot = (corrected_gyro_y * math.sin(roll) + corrected_gyro_z * math.cos(roll)) / math.cos(pitch)

        roll += roll_dot * dt
        pitch += pitch_dot * dt
        yaw += yaw_dot * dt

        F = update_F(pitch_dot, pitch, yaw_dot, dt, roll, F)
        H = update_H(roll, pitch, yaw)

        P = F @ P @ F.T + Q

        predicted_accel = np.array([-math.sin(pitch), math.sin(roll)*math.cos(pitch), math.cos(roll)*math.cos(pitch)])

        state = np.array([roll, pitch, yaw, bias_x, bias_y, bias_z])
        H_accel = H[0:3, :]

        S_accel = H_accel @ P @ H_accel.T + R_accel
        K_accel = P @ H_accel.T @ np.linalg.inv(S_accel)
        residual_accel = np.array([accel_x, accel_y, accel_z]) - predicted_accel

        #gate against BASE noise, not the already-inflated adaptive R - see chat
        S_accel_gate = H_accel @ P @ H_accel.T + R_accel_base
        d_squared = residual_accel.T @ np.linalg.inv(S_accel_gate) @ residual_accel

        if spike_start <= i < spike_end:
            print(f"  iter {i}: d_squared={d_squared:.2f}  threshold={chi2_threshold:.2f}  R_accel_diag={R_accel[0][0]:.2f}  gated_out={d_squared > chi2_threshold}")

        if d_squared <= chi2_threshold:
            state = state + K_accel @ residual_accel
            P = (I - K_accel @ H_accel) @ P

        roll, pitch, yaw, bias_x, bias_y, bias_z = state

        update_H(roll, pitch, yaw)
        predicted_mag = np.array([math.cos(pitch)*math.cos(yaw),
                                   math.sin(roll)*math.sin(pitch)*math.cos(yaw) - math.cos(roll)*math.sin(yaw),
                                   math.cos(roll)*math.sin(pitch)*math.cos(yaw) + math.sin(roll)*math.sin(yaw)])

        H_mag = H[3:]
        K_mag = P @ H_mag.T @ np.linalg.inv(H_mag @ P @ H_mag.T + R_mag)
        residual_mag = np.array([mag_x, mag_y, mag_z]) - predicted_mag
        state = state + K_mag @ residual_mag
        P = (I - K_mag @ H_mag) @ P

        roll, pitch, yaw, bias_x, bias_y, bias_z = state

        roll_log.append(math.degrees(roll))

    return roll_log

if __name__ == "__main__":
    log_with_gating = run(gating_enabled=True)
    log_without_gating = run(gating_enabled=False)

    print("iter | roll WITH gating | roll WITHOUT gating")
    for i in [145, 149, 150, 152, 155, 159, 160, 165, 170, 180, 200]:
        print(f"{i:4d} | {log_with_gating[i]:16.3f} | {log_without_gating[i]:19.3f}")

    print(f"\nmax roll during spike window (150-160):")
    print(f"  WITH gating:    {max(log_with_gating[150:160]):.3f}")
    print(f"  WITHOUT gating: {max(log_without_gating[150:160]):.3f}")
