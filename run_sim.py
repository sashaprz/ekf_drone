import sys
import os
import time
import math
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "state estimation"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "PID"))

from FINAL_gps import DroneEKF
from gz_bridge import GazeboBridge
import cascade

# STARTING-POINT gains/limits, not tuned - real numbers for the x500 Gazebo model
# (mass=2.0kg, motorConstant=8.54858e-06, maxRotVelocity=1000 rad/s), but the actual
# kp/ki/kd values below are first guesses and will need iteration once you can watch
# it fly. Motor commands are rad/s directly (see gz_bridge.py), not a normalized 0-1
# fraction, so "thrust"/torque outputs below are all in that same omega-ish scale,
# not physical Newtons/N*m - that's what makes cascade.mix()'s naive addition
# (thrust + roll_tau - pitch_tau - yaw_tau) work as a per-motor omega command.
# Hover needs ~757 rad/s per motor (mass*g / 4, solved through motorConstant*omega^2) -
# with zero ki that number never gets reached (P alone goes to 0 output once velocity
# error is zeroed), so vel_z's ki is what actually has to climb to that baseline over
# time - expect it to sag noticeably before ki catches up, there's no gravity
# feedforward term in cascade.py to shortcut that.
GAINS = {
    # DIAGNOSTIC: pos_xy/vel_xy zeroed out temporarily - isolates whether horizontal
    # drift correction is the source of the aggressive tilting, or whether it's
    # elsewhere (attitude/rate loop, EKF). Vehicle should just climb straight up,
    # staying level, with no tilt commands at all regardless of any drift.
    "pos_xy": {"kp": 0.0, "ki": 0.0, "kd": 0.0},
    "pos_z":  {"kp": 1.5, "ki": 0.0, "kd": 0.3},
    "vel_xy": {"kp": 0.0, "ki": 0.0, "kd": 0.0, "integral_limits": (-5.0, 5.0)},
    # cut further (40/60 -> 15/20) - across three runs now, instability tracked with
    # how fast/high thrust ramped up, not a fixed threshold - a gentler climb gives
    # attitude/rate loops room to keep up instead of fighting a fast-moving baseline
    "vel_z":  {"kp": 15.0, "ki": 20.0, "kd": 0.0, "integral_limits": (0.0, 1000.0)},
    "att_rp":  {"kp": 3.0, "ki": 0.0, "kd": 0.5},
    "att_yaw": {"kp": 1.5, "ki": 0.0, "kd": 0.05},
    # rate_rp's old kp=150 turned even a fractional rad/s rate error into a torque
    # command that immediately saturated the +/-250 limit - bang-bang, not smooth
    # control, and almost certainly what flipped it. Cut ~10x as a first pass.
    # kd bumped 0.1->0.5 for more damping - "climbed fine then tipped" looks like a
    # growing oscillation as thrust ramps up, not saturation this time.
    # d_filter_alpha added - kd differentiates the RAW gyro reading (real sensor noise),
    # and without filtering that noise gets amplified straight into torque chatter,
    # which is a better fit for "very very wobbly/chaotic but not decisively crashing"
    # than genuine instability would be
    "rate_rp":  {"kp": 15.0, "ki": 2.0, "kd": 0.5, "integral_limits": (-100.0, 100.0), "d_filter_alpha": 0.2},
    "rate_yaw": {"kp": 15.0, "ki": 2.0, "kd": 0.0, "integral_limits": (-100.0, 100.0), "d_filter_alpha": 0.2},
}
LIMITS = {
    "max_tilt_rad": math.radians(15), "max_rate": 3.0,  # down from 30 deg, extra margin while still tuning
    "thrust_range": (0.0, 1000.0),      # per-motor baseline omega command (rad/s)
    "motor_range": (0.0, 1000.0),       # matches x500's maxRotVelocity
    "max_vel_xy": 5.0, "max_vel_z": 3.0, "max_accel_xy": 5.0, "max_rate_yaw": 3.0,
    "torque_range_rp": (-100.0, 100.0),   # down from +/-250, omega perturbation not physical torque
    "torque_range_yaw": (-100.0, 100.0),
}


def main():
    bridge = GazeboBridge()
    ekf = DroneEKF(sensors=bridge)
    c = cascade.Cascade({"GAINS": GAINS, "LIMITS": LIMITS})

    setpoint = {"pos": np.array([0.0, 0.0, 2.0]), "yaw": 0.0}  # climb to 2m and hold

    last_t = time.time()
    print("running - Ctrl+C to stop")
    while True:
        state = ekf.step()

        now = time.time()
        dt = now - last_t
        last_t = now

        thrust, roll_tau, pitch_tau, yaw_tau = c.step(setpoint, state, dt)
        m1, m2, m3, m4 = cascade.mix(thrust, roll_tau, pitch_tau, yaw_tau, LIMITS["motor_range"], frame=c.frame)
        bridge.publish_motors(m1, m2, m3, m4)


if __name__ == "__main__":
    main()
