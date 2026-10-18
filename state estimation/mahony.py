"""Mahony filter: a nonlinear complementary filter for attitude (AHRS), written as a
benchmark baseline for FINAL_gps.py.

Reference: R. Mahony, T. Hamel, J.-M. Pflimlin, "Nonlinear Complementary Filters on the
Special Orthogonal Group", IEEE Trans. Automatic Control, 2008. The quaternion form here
follows the widely used MahonyAHRS.c (S. Madgwick's open-source port), rewritten with
vectors instead of hand-expanded scalar algebra so each step is readable.

---------------------------------------------------------------------------------------
THE IDEA IN ONE PARAGRAPH
---------------------------------------------------------------------------------------
Integrating the gyro gives a smooth attitude that slowly drifts (gyro bias + noise add up).
The accelerometer (which way is up) and magnetometer (which way is north) never drift, but
are noisy. So: each step, ask "if my attitude estimate were right, where SHOULD up and north
appear in the body frame?", compare that with where the accel/mag say they ARE, and turn the
disagreement into a small extra rotation rate that is added to the gyro before integrating.
That added rate is a PI controller acting on the attitude error:

    omega_corrected = omega_gyro + Kp * e + Ki * integral(e)

  - Kp * e        pulls the estimate toward accel/mag. Bigger Kp = trust accel/mag more
                  (faster correction, more noise). This is the "blend weight" of the
                  complementary filter in complementary.py, expressed as a rate.
  - Ki * int(e)   is the part complementary.py does NOT have. A constant gyro bias causes a
                  constant error that never goes away; the integral keeps growing until it
                  exactly cancels the bias. So the integral IS the gyro-bias estimate
                  (with the opposite sign).

Compared with the EKF: there is no covariance P, no Kalman gain, no Jacobians. Kp and Ki are
fixed numbers you choose; the EKF recomputes its equivalent of them every step from how
uncertain it currently is. Mahony also only estimates attitude (+ gyro bias), not velocity
or position, and has no outlier gates.

---------------------------------------------------------------------------------------
FRAMES (same as the rest of the project)
---------------------------------------------------------------------------------------
  body  = FLU (x forward, y left, z up)        world = ENU (x east, y north, z up)
  q = [w, x, y, z] rotates body -> world:  v_world = R(q) @ v_body
  At rest and level the accelerometer reads about (0, 0, +9.81): it measures the force
  holding the drone up (specific force), which points UP, opposite to gravity.
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


class Mahony:
    def __init__(self, kp=0.5, ki=0.0, q0=(1.0, 0.0, 0.0, 0.0), gyro_bias0=(0.0, 0.0, 0.0),
                 mag_reference=(0.0, 1.0, 0.0)):
        """kp, ki     - the PI gains above (units: 1/s and 1/s^2). MahonyAHRS.c defaults are
                        kp = 0.5, ki = 0 (no bias estimation).
        q0          - starting attitude (the benchmark computes it from the pre-flight dwell,
                        the same data the EKF's calibration sees).
        gyro_bias0  - starting gyro-bias guess (rad/s), also from the dwell. Stored as the
                        initial value of the integral term: integral = -bias, because the
                        integral is ADDED to the gyro and has to cancel the bias.
        mag_reference - Earth's field direction in ENU. Only its HORIZONTAL direction is used
                        (where magnetic north points, i.e. the declination) - see update()."""
        self.kp, self.ki = kp, ki
        self.q = np.array(q0, float) / np.linalg.norm(q0)
        self.integral = -np.array(gyro_bias0, float)
        ref = np.array(mag_reference, float)
        self.north_h = np.array([ref[0], ref[1], 0.0]) / np.hypot(ref[0], ref[1])

    @property
    def gyro_bias(self):
        return -self.integral

    def update(self, gyro, accel, mag, dt):
        """One step. gyro rad/s, accel m/s^2 (any scale - only its direction is used), mag in
        any units (direction only) or None. Returns the new attitude quaternion."""
        omega = np.array(gyro, float)
        R = quat_to_R(self.q)
        e = np.zeros(3)

        # ---- 1. accelerometer: tilt error ------------------------------------------------
        a_norm = np.linalg.norm(accel)
        if a_norm > 1e-6:
            a = np.asarray(accel, float) / a_norm  # measured "up" in body frame
            # Where "up" SHOULD be in the body frame if q were right: world up (0,0,1)
            # rotated into the body frame. R maps body->world, so R.T maps world->body, and
            # R.T @ (0,0,1) is just the bottom row of R.
            v_up = R[2, :]
            # Cross product of measured x predicted: its direction is the axis you'd rotate
            # about to line them up, its length is sin(angle between them). For small errors
            # this is exactly the rotation error vector. Order matters: a x v (not v x a) is
            # the direction that, added to omega, rotates the estimate TOWARD the measurement.
            e += np.cross(a, v_up)

        # ---- 2. magnetometer: heading error ----------------------------------------------
        if mag is not None:
            m_norm = np.linalg.norm(mag)
            if m_norm > 1e-6:
                m = np.asarray(mag, float) / m_norm
                # The Earth's field dips steeply into the ground, so it carries tilt
                # information too - but the accelerometer already handles tilt, and a mag
                # disturbance shouldn't be able to tilt the estimate. Mahony's trick (same as
                # Madgwick's): rotate the measurement into the world frame with the current
                # estimate, then build a reference field that has the SAME vertical part as
                # the measurement and puts all the horizontal part along magnetic north.
                # The only disagreement left between measurement and reference is then a
                # rotation about the vertical axis, i.e. heading.
                h = R @ m                              # measured field in world frame
                b = np.hypot(h[0], h[1]) * self.north_h
                b[2] = h[2]                            # reference: north-pointing, same dip
                w = R.T @ b                            # reference seen from the body
                e += np.cross(m, w)                    # same "measured x predicted" error

        # ---- 3. PI feedback onto the gyro ------------------------------------------------
        if self.ki > 0:
            # integral term = running sum of error. A gyro bias b makes the estimate rotate
            # at rate b, which shows up as a steady error; the integral grows until
            # Ki*integral == -b and the drift stops. With ki == 0 the integral just stays at
            # its starting value (-gyro_bias0, the dwell estimate) - MahonyAHRS.c zeroes it
            # instead, because it has no dwell estimate to start from.
            self.integral += self.ki * e * dt
        omega = omega + self.integral + self.kp * e

        # ---- 4. integrate the corrected rate ---------------------------------------------
        # Quaternion kinematics: q_dot = 1/2 * q (x) (0, omega_body). Euler step, then
        # renormalize to stay on the unit sphere (|q| = 1, see README section 2). A first-
        # order step is fine at 250 Hz: the angle per step is tiny.
        q_dot = 0.5 * quat_mult(self.q, np.array([0.0, *omega]))
        self.q = self.q + q_dot * dt
        self.q /= np.linalg.norm(self.q)
        return self.q
