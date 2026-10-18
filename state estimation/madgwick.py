"""Madgwick filter: gradient-descent orientation filter (AHRS), written as a benchmark
baseline for FINAL_gps.py.

Reference: S. Madgwick, "An efficient orientation filter for inertial and inertial/magnetic
sensor arrays", 2010 (report), and Madgwick, Harrison, Vaidyanathan, ICORR 2011. Same
algorithm as the widely used MadgwickAHRS.c, but the objective function and its Jacobian
are written in vector form (one formula for any reference direction) instead of the
paper's hand-expanded scalars, so the steps can be followed.

---------------------------------------------------------------------------------------
THE IDEA IN ONE PARAGRAPH
---------------------------------------------------------------------------------------
Two estimates of how the attitude is changing are fused every step:

  1. from the gyro:  q_dot_gyro = 1/2 * q (x) (0, omega)          (smooth, drifts)
  2. from accel/mag: "which way should q move to make the predicted up/north match the
     measured up/north?" This is posed as minimizing a cost f(q) = |predicted - measured|^2
     and answered with ONE step of gradient descent per sample:  -grad f / |grad f|.

  q_dot = q_dot_gyro - beta * grad_f / |grad_f|

Because the gradient is normalized to unit length, beta is simply "how fast (rad/s) the
accel/mag are allowed to pull the attitude". Madgwick's rule of thumb: set beta to the
expected gyro error rate, beta = sqrt(3/4) * gyro_error. If the true error is larger than
beta can correct, the filter lags; if beta is much larger, accel noise leaks in. It plays
the same role as Kp in mahony.py.

Optional gyro-bias estimation (zeta): the normalized gradient points in the direction the
estimate is being pushed away from the measurements. Converted to a body rate, that is the
"error rate" the gyro is adding; integrating it slowly (gain zeta) learns the gyro bias.
This is the counterpart of Mahony's integral term.

Compared with the EKF: no covariance, no Kalman gain, no gates, attitude only. beta and
zeta are fixed numbers you choose instead of gains recomputed from uncertainty.

---------------------------------------------------------------------------------------
FRAMES (same as the rest of the project)
---------------------------------------------------------------------------------------
  body  = FLU (x forward, y left, z up)        world = ENU (x east, y north, z up)
  q = [w, x, y, z] rotates body -> world:  v_world = R(q) @ v_body
  The paper uses the opposite convention (q from earth to sensor), which is why its
  formulas look like the transposes of the ones here. The algorithm is the same.
"""
import numpy as np


def quat_mult(a, b):
    # Hamilton product a (x) b - "apply b, then a"
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def quat_to_R(q):
    # rotation matrix of q (body -> world)
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def skew(v):
    # [v]x : skew(v) @ u == cross(v, u)
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def residual_and_jacobian(q, d, s):
    """The cost term for one sensor, and how it changes with q.

    d = reference direction in the WORLD frame (e.g. up = (0,0,1)), s = the same direction
    as MEASURED in the body frame (unit vectors). If q is right, d rotated into the body
    frame equals s, so the residual

        f(q) = R(q)^T d - s                                   (3 numbers)

    is zero. Writing q = (w, u) with u the vector part, rotating d into the body frame is

        R^T d = (w^2 - u.u) d + 2 (u.d) u - 2 w (u x d)

    (the standard quaternion rotation formula, with the sign of the w-term flipped because
    R^T is the inverse rotation). Differentiating each term by w and by u gives the 3x4
    Jacobian J = [df/dw | df/du]:

        df/dw = 2 w d - 2 (u x d)
        df/du = -2 d u^T + 2 ((u.d) I + u d^T) + 2 w [d]x

    The paper writes out these same 12 entries by hand for the two special cases it needs
    (gravity along one axis; magnetic field in a vertical plane)."""
    w, u = q[0], q[1:]
    f = (w * w - u @ u) * d + 2 * (u @ d) * u - 2 * w * np.cross(u, d) - s
    J = np.empty((3, 4))
    J[:, 0] = 2 * w * d - 2 * np.cross(u, d)
    J[:, 1:] = -2 * np.outer(d, u) + 2 * ((u @ d) * np.eye(3) + np.outer(u, d)) + 2 * w * skew(d)
    return f, J


class Madgwick:
    def __init__(self, beta=0.1, zeta=0.0, q0=(1.0, 0.0, 0.0, 0.0), gyro_bias0=(0.0, 0.0, 0.0),
                 mag_reference=(0.0, 1.0, 0.0)):
        """beta  - accel/mag correction rate, rad/s (MadgwickAHRS.c default 0.1).
        zeta     - gyro-bias learning rate, rad/s^2 (0 = off, as in MadgwickAHRS.c).
        q0, gyro_bias0 - starting attitude and gyro-bias guess (from the pre-flight dwell,
                   the same data the EKF's calibration sees).
        mag_reference - Earth's field direction in ENU; only its horizontal direction (where
                   magnetic north points) is used - see update()."""
        self.beta, self.zeta = beta, zeta
        self.q = np.array(q0, float) / np.linalg.norm(q0)
        self.gyro_bias = np.array(gyro_bias0, float)
        ref = np.array(mag_reference, float)
        self.north_h = np.array([ref[0], ref[1], 0.0]) / np.hypot(ref[0], ref[1])

    def update(self, gyro, accel, mag, dt):
        """One step. gyro rad/s, accel m/s^2 (direction only), mag any units (direction only)
        or None. Returns the new attitude quaternion."""
        q = self.q
        grad = np.zeros(4)

        # ---- 1. gradient of the cost: one residual per sensor, summed ----------------------
        # cost = 1/2 |f_acc|^2 + 1/2 |f_mag|^2, so grad = J_acc^T f_acc + J_mag^T f_mag
        a_norm = np.linalg.norm(accel)
        if a_norm > 1e-6:
            # accelerometer: world "up" (0,0,1) should appear along the measured accel
            f, J = residual_and_jacobian(q, np.array([0.0, 0.0, 1.0]), np.asarray(accel, float) / a_norm)
            grad += J.T @ f
        if mag is not None and np.linalg.norm(mag) > 1e-6:
            m = np.asarray(mag, float) / np.linalg.norm(mag)
            # Magnetic reference built from the measurement itself, so the mag only corrects
            # heading (same trick as mahony.py, and as the paper's "magnetic distortion
            # compensation"): rotate the reading into the world frame, keep its vertical
            # part, and point its horizontal part at magnetic north. Any tilt disagreement
            # is removed by construction; only a heading disagreement is left.
            h = quat_to_R(q) @ m
            b = np.hypot(h[0], h[1]) * self.north_h
            b[2] = h[2]
            f, J = residual_and_jacobian(q, b, m)
            grad += J.T @ f

        # ---- 2. gyro bias (optional) -------------------------------------------------------
        g_norm = np.linalg.norm(grad)
        step = grad / g_norm if g_norm > 1e-12 else np.zeros(4)
        if self.zeta > 0:
            # Turn the quaternion-rate direction into a body rotation rate:
            # q_dot = 1/2 q (x) (0, omega)  <=>  omega = 2 q* (x) q_dot.
            # 'step' points the way the estimate has drifted AWAY from the measurements, which
            # is the direction a gyro bias pushes it; integrating it learns the bias.
            q_conj = q * np.array([1.0, -1.0, -1.0, -1.0])
            omega_err = 2 * quat_mult(q_conj, step)[1:]
            self.gyro_bias += self.zeta * omega_err * dt

        # ---- 3. fuse: gyro rate of change minus a beta-sized step down the gradient --------
        omega = np.asarray(gyro, float) - self.gyro_bias
        q_dot = 0.5 * quat_mult(q, np.array([0.0, *omega])) - self.beta * step

        # ---- 4. integrate and renormalize --------------------------------------------------
        self.q = q + q_dot * dt
        self.q /= np.linalg.norm(self.q)
        return self.q
