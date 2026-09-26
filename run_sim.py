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
    # ki added - logged data showed a slow, steady attitude drift (not oscillation) even
    # with pos_xy/vel_xy disabled: a pure-P attitude loop can't cancel a sustained
    # disturbance (motor spin-up/down asymmetry, gyroscopic coupling, etc), it just
    # settles at a nonzero offset - that drift is what eventually snowballed into the
    # full divergence by t=71s in the last run
    # kp doubled (3.0->6.0), 2026-09-26 - NOT confirmed to actually be better, kept
    # pragmatically. 3 fresh trials each of kp=3.0 and kp=6.0 (test_attitude_loop.py
    # roll 0.2 3.0) showed peak roll tracking ranging 0.10-0.207 (against a 0.2 target)
    # at BOTH values - run-to-run noise (likely the same timing-jitter/intermittent-EKF-
    # correction issue as the EKF_ACCEL diagnostic in FINAL_gps.py, see HANDOFF.md) is
    # larger than any effect a 2x kp change should plausibly cause, so this comparison
    # wasn't actually measurable. kp=6.0's 3 trials had 0 blowups vs kp=3.0's 1/3, but
    # that's far too small a sample to credit the gain for it. Don't read the tracking
    # quality of any single test_attitude_loop.py run as evidence this value is right -
    # revisit once the underlying noise source is understood, not before.
    "att_rp":  {"kp": 6.0, "ki": 0.5, "kd": 0.5, "integral_limits": (-1.0, 1.0)},
    "att_yaw": {"kp": 1.5, "ki": 0.3, "kd": 0.05, "integral_limits": (-1.0, 1.0)},
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
    # yaw authority comes from reaction torque (momentConstant=0.016 * per-motor thrust),
    # not thrust-differential-times-arm like roll/pitch - worked through the actual x500
    # constants at hover, the SAME commanded omega differential produces ~10x less angular
    # acceleration in yaw than in roll/pitch. Confirmed empirically via test_rate_loop.py:
    # yaw 0.3 5.0 with kp=15 held tau_yaw pinned near its pure-P value (~4.5) for the
    # entire step with essentially zero measured rate response (stayed ~0.000-0.002 the
    # whole time) - not a slow response, no response. kp raised ~10x to compensate for the
    # weaker physical gain, not because the loop was oscillating (start of tuning from
    # scratch for this axis, not a refinement of a working baseline).
    "rate_yaw": {"kp": 150.0, "ki": 2.0, "kd": 0.0, "integral_limits": (-100.0, 100.0), "d_filter_alpha": 0.2},
}
LIMITS = {
    "max_tilt_rad": math.radians(15), "max_rate": 3.0,  # down from 30 deg, extra margin while still tuning
    "thrust_range": (0.0, 1000.0),      # per-motor baseline omega command (rad/s)
    "motor_range": (0.0, 1000.0),       # matches x500's maxRotVelocity
    "max_vel_xy": 5.0, "max_vel_z": 3.0, "max_accel_xy": 5.0, "max_rate_yaw": 3.0,
    "torque_range_rp": (-100.0, 100.0),   # down from +/-250, omega perturbation not physical torque
    "torque_range_yaw": (-100.0, 100.0),
}


def quat_to_euler_deg(q):
    # [w,x,y,z] -> (roll,pitch,yaw) in degrees, ZYX convention, matches cascade.euler_to_quat
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def main():
    bridge = GazeboBridge()
    time.sleep(0.5)  # let the first real gz-transport messages arrive before calibration reads them -
                      # otherwise it'd read the bridge's placeholder zeros (e.g. accel=0 instead of ~9.8)
    ekf = DroneEKF(sensors=bridge)
    c = cascade.Cascade({"GAINS": GAINS, "LIMITS": LIMITS})

    setpoint = {"pos": np.array([0.0, 0.0, 2.0]), "yaw": 0.0}  # climb to 2m and hold

    last_t = time.time()
    start_t = last_t
    i = 0
    print("running - Ctrl+C to stop", flush=True)
    while True:
        state = ekf.step()

        now = time.time()
        dt = now - last_t
        last_t = now

        thrust, roll_tau, pitch_tau, yaw_tau = c.step(setpoint, state, dt)
        m1, m2, m3, m4 = cascade.mix(thrust, roll_tau, pitch_tau, yaw_tau, LIMITS["motor_range"], frame=c.frame)
        bridge.publish_motors(m1, m2, m3, m4)

        i += 1
        if i % 30 == 0:
            roll_deg, pitch_deg, yaw_deg = quat_to_euler_deg(state["quat"])
            print(f"t={now - start_t:6.2f} dt={dt*1000:6.2f}ms "
                  f"est(rpy)=({roll_deg:6.1f},{pitch_deg:6.1f},{yaw_deg:6.1f})deg "
                  f"rate_meas=({state['rate'][0]:5.2f},{state['rate'][1]:5.2f},{state['rate'][2]:5.2f}) "
                  f"rate_sp=({c._rate_sp[0]:5.2f},{c._rate_sp[1]:5.2f},{c._rate_sp[2]:5.2f}) "
                  f"thrust={thrust:6.1f} tau=({roll_tau:6.1f},{pitch_tau:6.1f},{yaw_tau:6.1f}) "
                  f"pos=({state['pos'][0]:5.2f},{state['pos'][1]:5.2f},{state['pos'][2]:5.2f})",
                  flush=True)


if __name__ == "__main__":
    main()
