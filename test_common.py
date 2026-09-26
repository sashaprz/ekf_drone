"""
Shared setup for the isolated-loop test harnesses (test_rate_loop.py,
test_attitude_loop.py, ...).

wait_for_liftoff() exists because constant HOVER_THRUST (thrust == weight exactly)
gives zero net vertical force - fine once airborne, but it never generates the excess
lift needed to leave the ground, and calibration itself runs for ~2s with no motor
command published at all, letting the vehicle settle onto the pad first. Confirmed via
Gazebo's own /world/default/pose/info ground truth: without this phase, the vehicle
sits at z~-0.013 (on the pad, level) for an entire "flight" - see HANDOFF.md for the
full story (this bug made an earlier pitch-axis rate test show zero response and made
a roll-axis test's ground-contact tip-over look like a mid-air EKF glitch).
"""
import sys
import time
import numpy as np


def reset_cascade_pids(c):
    # zero every PID's integral/derivative history - call this right after
    # wait_for_liftoff() returns, before starting a timed measurement window.
    # wait_for_liftoff() runs rate_loop() (with real, if small, error - the vehicle isn't
    # perfectly level during the climb) for however long the climb happens to take, which
    # varies run to run (observed 0.7s-2.2s) - without this reset, each test starts with a
    # different amount of pre-accumulated integral windup depending on liftoff duration,
    # which showed up as real run-to-run inconsistency in test_attitude_loop.py results
    # before this existed.
    for pid in (c.pos_pid_x, c.pos_pid_y, c.pos_pid_z,
                c.vel_pid_x, c.vel_pid_y, c.vel_pid_z,
                c.att_pid_x, c.att_pid_y, c.att_pid_z,
                c.rate_pid_roll, c.rate_pid_pitch, c.rate_pid_yaw):
        pid.reset()


def wait_for_liftoff(ekf, c, bridge, mix_fn, limits, takeoff_thrust,
                      liftoff_alt=0.5, liftoff_timeout=5.0):
    # climb on takeoff_thrust (rate loop still running, sp=0, to keep it level) until
    # altitude clears liftoff_alt - caller should call reset_cascade_pids(c) right after
    # this returns, before starting its own timed measurement window (see that function's
    # comment for why)
    liftoff_start = time.time()
    last_t = liftoff_start
    while True:
        state = ekf.step()
        now = time.time()
        dt = now - last_t
        last_t = now

        roll_tau, pitch_tau, yaw_tau = c.rate_loop(np.zeros(3), state, dt)
        m1, m2, m3, m4 = mix_fn(takeoff_thrust, roll_tau, pitch_tau, yaw_tau, limits["motor_range"], frame=c.frame)
        bridge.publish_motors(m1, m2, m3, m4)

        alt = state["pos"][2]
        if alt >= liftoff_alt:
            print(f"liftoff confirmed: alt={alt:.3f}m after {now - liftoff_start:.2f}s", flush=True)
            return
        if now - liftoff_start > liftoff_timeout:
            print(f"ABORT: never reached liftoff alt ({liftoff_alt}m) within {liftoff_timeout}s "
                  f"(stuck at alt={alt:.3f}m) - not running the test on the ground", flush=True)
            sys.exit(1)
