"""
Finite-difference check for F[6:9,12:15] (velocity_error's sensitivity to accel_bias
error), the one F block gps.py's own comment admits was only derived analytically, not
verified numerically the way F[6:9,0:3] (attitude->velocity coupling) was.

Method: perturb accel_bias_hat by +-eps along each axis, recompute the one-step
world-frame acceleration used in gps.py's propagation (rotate_by_quat(q, accel_raw -
accel_bias_hat) - gravity), and central-difference to get a numeric d(accel_world)/d(bias)
Jacobian. F[6:9,12:15] = -dt * quat_to_R(q) should match -dt * (that numeric Jacobian).
"""

import math
import numpy as np

GRAVITY = 9.80665

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

def quat_to_R(q):
    return np.array([rotate_by_quat(q, e) for e in np.eye(3)]).T

def world_accel(q, accel_raw, accel_bias_hat):
    corrected = accel_raw - accel_bias_hat
    return rotate_by_quat(q, corrected) - np.array([0.0, 0.0, GRAVITY])

def numeric_jacobian_wrt_bias(q, accel_raw, accel_bias_hat, eps=1e-6):
    J = np.zeros((3, 3))
    for i in range(3):
        d = np.zeros(3); d[i] = eps
        f_plus = world_accel(q, accel_raw, accel_bias_hat + d)
        f_minus = world_accel(q, accel_raw, accel_bias_hat - d)
        J[:, i] = (f_plus - f_minus) / (2 * eps)
    return J

def random_unit_quat(rng):
    v = rng.normal(size=4)
    return v / np.linalg.norm(v)

if __name__ == "__main__":
    rng = np.random.default_rng(0)
    dt = 0.01
    max_err = 0.0

    print(f"{'trial':>5s} | {'max |analytic - numeric|':>26s}")
    for trial in range(10):
        q = random_unit_quat(rng)                       #genuinely non-identity attitude each trial
        accel_bias_hat = rng.normal(scale=0.1, size=3)
        accel_raw = rng.normal(scale=1.0, size=3) + np.array([0, 0, GRAVITY])

        numeric_dworld_dbias = numeric_jacobian_wrt_bias(q, accel_raw, accel_bias_hat)
        F_numeric = dt * numeric_dworld_dbias   #velocity_error propagates as dt * d(world_accel)/d(bias)
        F_analytic = -dt * quat_to_R(q)

        err = np.max(np.abs(F_analytic - F_numeric))
        max_err = max(max_err, err)
        print(f"{trial:5d} | {err:26.2e}")

    print(f"\nmax error across all trials: {max_err:.2e}")
    print("PASS - F[6:9,12:15] = -dt*quat_to_R(q) matches finite difference to float precision"
          if max_err < 1e-8 else "FAIL - analytic and numeric Jacobians disagree beyond float precision")
