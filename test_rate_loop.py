"""
Isolated rate-loop test - bypasses position_loop/velocity_loop/attitude_loop entirely.
Feeds cascade.Cascade.rate_loop() a scripted step rate-setpoint directly on one axis,
holds a constant near-hover thrust, and logs the tracking response. This is step 1 of
tuning inside-out: get the innermost/fastest loop solid in isolation before trusting
anything built on top of it (attitude, velocity, position) - see HANDOFF.md.

Usage:
    python3 test_rate_loop.py [axis] [step_value_rad_s] [duration_s]
    python3 test_rate_loop.py roll 0.3 5.0      (default if no args given)
    python3 test_rate_loop.py pitch 0.3 5.0
    python3 test_rate_loop.py yaw 0.3 5.0

Schedule: rate setpoint on the chosen axis is 0 for the first third of `duration`,
steps to step_value for the middle third, then back to 0 for the last third - a clean
step-up/step-down so you can read rise time, overshoot, and settling from the log
without anything else (position, velocity, attitude, EKF drift) confounding it.

step_value default kept small (0.3 rad/s, not 1.0) deliberately: this test only closes
the rate loop, nothing holds attitude level, so step_value*(duration/3) is roughly how
much real attitude angle accumulates over the held step - at 1.0 rad/s for a 1.667s hold
that's ~95 degrees of actual roll/pitch/yaw, enough that by the time the step drops back
to zero the vehicle is no longer close to level and "hover thrust" isn't pointing
anywhere near vertical anymore. That's a real, but entirely separate, large-angle flight
problem, not a rate-loop tracking problem - confirmed via a 1.0 rad/s roll trial that
tracked the step cleanly (no overshoot) but then diverged ~1s after the step ended,
consistent with ~90+ degrees of accumulated roll and nothing to recover attitude. 0.3
rad/s for the same hold length keeps accumulated angle to ~30 degrees, comfortably
inside the regime this test is actually meant to characterize.
"""
import sys
import os
import time
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "state estimation"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "PID"))

from FINAL_gps import DroneEKF
from gz_bridge import GazeboBridge
import cascade
from run_sim import GAINS, LIMITS  # single source of truth for gains - edit them there, not here
from test_common import wait_for_liftoff, reset_cascade_pids

AXIS = sys.argv[1] if len(sys.argv) > 1 else "roll"
STEP_VALUE = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3
DURATION = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
AXIS_INDEX = {"roll": 0, "pitch": 1, "yaw": 2}[AXIS]

# near-hover baseline (mass=2.0kg, motorConstant=8.54858e-06, g=9.80665 -> ~757 rad/s/motor)
# held CONSTANT throughout the actual measurement window - deliberately not using vel_z's
# own PID here, so thrust isn't a second time-varying moving part while reading the rate
# loop's response. This is exactly the theoretical steady-state hover value (thrust ==
# weight), which gives zero net vertical force - fine once airborne, but it means there's
# no excess lift to ever climb away from the ground if the vehicle starts sitting on it
# (which it does: calibration runs for ~2s before any motor command is published at all).
# Confirmed via gz's own /world/default/pose/info ground truth: without a real takeoff
# phase, the vehicle sits at z~-0.013 (on the pad) for the entire "flight" - so a short
# TAKEOFF_THRUST climb-out (below) runs first to actually get airborne before the
# constant-hover measurement window (and its scripted step) begins.
HOVER_THRUST = 757.0
TAKEOFF_THRUST = HOVER_THRUST * 1.15  # real excess lift (~3 m/s^2 net accel) to climb away from the pad
LIFTOFF_ALT = 0.5    # meters - comfortably clear of any ground-contact effects
LIFTOFF_TIMEOUT = 5.0  # seconds - abort rather than silently running the step on the ground


def rate_setpoint(t):
    sp = np.zeros(3)
    third = DURATION / 3.0
    if third <= t < 2 * third:
        sp[AXIS_INDEX] = STEP_VALUE
    return sp


def main():
    bridge = GazeboBridge()
    time.sleep(0.5)  # let the first real gz-transport messages arrive before calibration reads them
    ekf = DroneEKF(sensors=bridge)
    c = cascade.Cascade({"GAINS": GAINS, "LIMITS": LIMITS})

    wait_for_liftoff(ekf, c, bridge, cascade.mix, LIMITS, TAKEOFF_THRUST,
                      liftoff_alt=LIFTOFF_ALT, liftoff_timeout=LIFTOFF_TIMEOUT)
    reset_cascade_pids(c)

    last_t = time.time()
    start_t = last_t
    i = 0
    print(f"rate-loop isolation test: axis={AXIS} step={STEP_VALUE}rad/s duration={DURATION}s", flush=True)
    print("t       dt(ms)  sp      meas(r,p,y)                 tau(r,p,y)              alt(z)  vel(x,y)", flush=True)

    while True:
        state = ekf.step()

        now = time.time()
        dt = now - last_t
        last_t = now
        t = now - start_t

        sp = rate_setpoint(t)
        roll_tau, pitch_tau, yaw_tau = c.rate_loop(sp, state, dt)
        m1, m2, m3, m4 = cascade.mix(HOVER_THRUST, roll_tau, pitch_tau, yaw_tau, LIMITS["motor_range"], frame=c.frame)
        bridge.publish_motors(m1, m2, m3, m4)

        i += 1
        if i % 10 == 0:
            print(f"{t:6.2f}  {dt*1000:5.2f}  {sp[AXIS_INDEX]:5.2f}  "
                  f"({state['rate'][0]:6.3f},{state['rate'][1]:6.3f},{state['rate'][2]:6.3f})  "
                  f"({roll_tau:6.1f},{pitch_tau:6.1f},{yaw_tau:6.1f})  "
                  f"{state['pos'][2]:6.3f}  "
                  f"({state['vel'][0]:6.2f},{state['vel'][1]:6.2f})", flush=True)

        if t > DURATION:
            print("test complete", flush=True)
            break


if __name__ == "__main__":
    main()
