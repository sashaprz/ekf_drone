"""
Isolated attitude-loop test - bypasses position_loop/velocity_loop, but (unlike
test_rate_loop.py) runs rate_loop() normally underneath attitude_loop() instead of
driving it directly. Feeds cascade.Cascade.attitude_loop() a scripted step attitude
target on one axis, holds a constant near-hover thrust, and logs the tracking
response. This is step 2 of tuning inside-out (see HANDOFF.md) - the rate loop was
validated with test_rate_loop.py first; this validates attitude->rate->mix together
before trusting velocity_loop/position_loop or a full-cascade flight.

Usage:
    python3 test_attitude_loop.py [axis] [step_value_rad] [duration_s]
    python3 test_attitude_loop.py roll 0.2 3.0      (default if no args given)
    python3 test_attitude_loop.py pitch 0.2 3.0
    python3 test_attitude_loop.py yaw 0.2 3.0

Schedule: same as test_rate_loop.py - attitude target on the chosen axis is level (0)
for the first third of `duration`, steps to step_value for the middle third, then back
to level for the last third.

step_value default kept small (0.2 rad ~= 11 degrees), same reasoning as
test_rate_loop.py's shrunk default: nothing above attitude_loop is active either (no
velocity/position hold), so a sustained tilt lets real horizontal velocity build up and
feed back as a rotorDrag-driven disturbance (see HANDOFF.md's velocity-driven-drift
note). duration default shrunk further than the rate-loop harness's (3.0s, not 5.0s -
a 1.0s hold, not 1.667s): unlike a rate-loop step (a brief pulse that returns to zero
quickly), an attitude step commands a SUSTAINED tilt for the whole hold, which builds
disturbance-inducing velocity faster - confirmed empirically, a 0.2 rad roll step held
for 1.667s reached a clean ~0.26 tracking value by ~0.7s in, then suffered a sudden,
severe multi-axis breakdown (roll swinging to 160+ degrees, matching altitude/velocity
discontinuities) around 0.7s into the hold. Read the step-onset/rise/initial-overshoot
portion (roughly the first ~0.7s after the step begins) as the trustworthy signal -
same caveat as the rate-loop harness, just on a tighter clock here.
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
STEP_VALUE = float(sys.argv[2]) if len(sys.argv) > 2 else 0.2
DURATION = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0

HOVER_THRUST = 757.0
TAKEOFF_THRUST = HOVER_THRUST * 1.15
LIFTOFF_ALT = 0.5
LIFTOFF_TIMEOUT = 5.0


def attitude_setpoint(t):
    # target quaternion [w,x,y,z] - level, except a step on one axis during the middle
    # third of `duration` (same schedule as test_rate_loop.py's rate_setpoint)
    third = DURATION / 3.0
    roll = pitch = yaw = 0.0
    if third <= t < 2 * third:
        if AXIS == "roll":
            roll = STEP_VALUE
        elif AXIS == "pitch":
            pitch = STEP_VALUE
        else:
            yaw = STEP_VALUE
    return cascade.euler_to_quat(roll, pitch, yaw)


def current_tilt(state):
    # actual (roll, pitch, yaw) of the current attitude, standard ZYX Hamilton
    # decomposition matching euler_to_quat()'s own construction in cascade.py - NOT the
    # small-angle quat_conjugate(state["quat"]) trick, which gives roughly -tilt (the
    # correction needed to reach level), not the tilt itself, and reads backwards
    w, x, y, z = state["quat"]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = cascade.yaw_from_quat(state["quat"])
    return np.array([roll, pitch, yaw])


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
    # correlates with FINAL_gps.py's TEMP EKF_ACCEL diagnostic prints (raw time.time(),
    # not test-relative t) - EKF_ACCEL's t minus this equals this log's own t column
    print(f"start_t_epoch={start_t:.6f}", flush=True)
    # attitude_loop() must be recomputed at its own sub-rate and HELD between recomputes,
    # same as Cascade.step() does - calling it on every rate-loop tick instead (every
    # ~0.3ms rather than the intended ~5ms/200Hz) changes its P/I/D balance enough to
    # produce a real (but spurious) overshoot that has nothing to do with the actual gains
    attitude_period = c._periods["attitude"]
    attitude_elapsed = 0.0
    rate_sp = np.zeros(3)
    print(f"attitude-loop isolation test: axis={AXIS} step={STEP_VALUE}rad duration={DURATION}s", flush=True)
    print("t       dt(ms)  sp      tilt(r,p,y)                 rate_sp(r,p,y)          "
          "meas_rate(r,p,y)          alt(z)  vel(x,y)", flush=True)

    while True:
        state = ekf.step()

        now = time.time()
        dt = now - last_t
        last_t = now
        t = now - start_t

        attitude_elapsed += dt
        if attitude_elapsed >= attitude_period:
            att_sp = attitude_setpoint(t)
            rate_sp = c.attitude_loop(att_sp, state, attitude_elapsed)
            attitude_elapsed = 0.0
        roll_tau, pitch_tau, yaw_tau = c.rate_loop(rate_sp, state, dt)
        m1, m2, m3, m4 = cascade.mix(HOVER_THRUST, roll_tau, pitch_tau, yaw_tau, LIMITS["motor_range"], frame=c.frame)
        bridge.publish_motors(m1, m2, m3, m4)

        i += 1
        if i % 10 == 0:
            third = DURATION / 3.0
            sp_val = STEP_VALUE if third <= t < 2 * third else 0.0
            tilt = current_tilt(state)
            print(f"{t:6.2f}  {dt*1000:5.2f}  {sp_val:5.2f}  "
                  f"({tilt[0]:6.3f},{tilt[1]:6.3f},{tilt[2]:6.3f})  "
                  f"({rate_sp[0]:6.3f},{rate_sp[1]:6.3f},{rate_sp[2]:6.3f})  "
                  f"({state['rate'][0]:6.3f},{state['rate'][1]:6.3f},{state['rate'][2]:6.3f})  "
                  f"{state['pos'][2]:6.3f}  ({state['vel'][0]:6.2f},{state['vel'][1]:6.2f})", flush=True)

        if t > DURATION:
            print("test complete", flush=True)
            break


if __name__ == "__main__":
    main()
