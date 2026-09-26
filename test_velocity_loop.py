"""
Isolated velocity-loop test - bypasses position_loop, but runs attitude_loop() and
rate_loop() normally underneath velocity_loop() (same nested throttle-and-hold pattern
as test_attitude_loop.py, one loop further out). Feeds cascade.Cascade.velocity_loop()
a scripted step velocity target on one axis. This is step 3 of tuning inside-out (see
HANDOFF.md) - rate and attitude loops were validated first.

Usage:
    python3 test_velocity_loop.py [axis] [step_value_m_s] [duration_s]
    python3 test_velocity_loop.py vz 0.5 3.0      (default if no args given)
    python3 test_velocity_loop.py vx 0.5 3.0
    python3 test_velocity_loop.py vy 0.5 3.0

Schedule: same as the other harnesses - velocity target on the chosen axis is 0 for the
first third of `duration`, steps to step_value for the middle third, then back to 0 for
the last third.

Unlike test_rate_loop.py/test_attitude_loop.py, this harness can't hold a fixed
baseline thrust for its measurement window - vel_z's own PID output IS thrust here,
that's the entire point of testing it (run_sim.py's own comment: "with zero ki that
number never gets reached... expect it to sag noticeably before ki catches up"). So
after the fixed-thrust liftoff climb (same as the other harnesses, just to clear the
ground), there's a WARMUP phase: velocity_loop/attitude_loop/rate_loop all run for
real with vel_sp=[0,0,0] ("hold current velocity") so vel_z's ki has time to climb to
a real hover-sustaining value before the timed test starts. Only the rate/attitude
PIDs get reset after warmup (same reasoning as test_common.reset_cascade_pids) -
vel_z's integral is deliberately left alone, since zeroing it right as the test starts
would recreate exactly the sag this warmup exists to avoid.
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
from test_common import wait_for_liftoff

AXIS = sys.argv[1] if len(sys.argv) > 1 else "vz"
STEP_VALUE = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
DURATION = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
AXIS_INDEX = {"vx": 0, "vy": 1, "vz": 2}[AXIS]

TAKEOFF_THRUST = 757.0 * 1.15
LIFTOFF_ALT = 0.5
LIFTOFF_TIMEOUT = 5.0
WARMUP_DURATION = 2.0  # seconds - lets vel_z's ki climb to a real hover value


def vel_setpoint(t):
    sp = np.zeros(3)
    third = DURATION / 3.0
    if third <= t < 2 * third:
        sp[AXIS_INDEX] = STEP_VALUE
    return sp


def cascade_tick(c, state, dt, vel_sp, periods, elapsed, held):
    # replicates Cascade.step()'s velocity->attitude->rate sub-rate chain, minus
    # position_loop - same throttle-and-hold pattern Cascade.step() itself uses
    elapsed["velocity"] += dt
    if elapsed["velocity"] >= periods["velocity"]:
        roll_sp, pitch_sp, held["thrust"] = c.velocity_loop(vel_sp, state, elapsed["velocity"])
        held["att_sp"] = cascade.euler_to_quat(roll_sp, pitch_sp, 0.0)
        held["roll_sp"] = roll_sp
        held["pitch_sp"] = pitch_sp
        elapsed["velocity"] = 0.0

    elapsed["attitude"] += dt
    if elapsed["attitude"] >= periods["attitude"]:
        held["rate_sp"] = c.attitude_loop(held["att_sp"], state, elapsed["attitude"])
        elapsed["attitude"] = 0.0

    roll_tau, pitch_tau, yaw_tau = c.rate_loop(held["rate_sp"], state, dt)
    return held["thrust"], roll_tau, pitch_tau, yaw_tau


def main():
    bridge = GazeboBridge()
    time.sleep(0.5)  # let the first real gz-transport messages arrive before calibration reads them
    ekf = DroneEKF(sensors=bridge)
    c = cascade.Cascade({"GAINS": GAINS, "LIMITS": LIMITS})

    wait_for_liftoff(ekf, c, bridge, cascade.mix, LIMITS, TAKEOFF_THRUST,
                      liftoff_alt=LIFTOFF_ALT, liftoff_timeout=LIFTOFF_TIMEOUT)

    periods = {"velocity": c._periods["velocity"], "attitude": c._periods["attitude"]}
    elapsed = {"velocity": 0.0, "attitude": 0.0}
    held = {"thrust": TAKEOFF_THRUST, "att_sp": np.array([1.0, 0.0, 0.0, 0.0]), "rate_sp": np.zeros(3),
            "roll_sp": 0.0, "pitch_sp": 0.0}

    last_t = time.time()
    warmup_start = last_t
    while time.time() - warmup_start < WARMUP_DURATION:
        state = ekf.step()
        now = time.time()
        dt = now - last_t
        last_t = now
        thrust, roll_tau, pitch_tau, yaw_tau = cascade_tick(c, state, dt, np.zeros(3), periods, elapsed, held)
        m1, m2, m3, m4 = cascade.mix(thrust, roll_tau, pitch_tau, yaw_tau, LIMITS["motor_range"], frame=c.frame)
        bridge.publish_motors(m1, m2, m3, m4)
    print(f"warmup done: thrust={held['thrust']:.1f} (hover~757), alt={state['pos'][2]:.3f}", flush=True)

    # only reset rate/attitude PIDs - vel_z's integral is the whole point of the warmup
    c.rate_pid_roll.reset(); c.rate_pid_pitch.reset(); c.rate_pid_yaw.reset()
    c.att_pid_x.reset(); c.att_pid_y.reset(); c.att_pid_z.reset()

    last_t = time.time()
    start_t = last_t
    i = 0
    print(f"velocity-loop isolation test: axis={AXIS} step={STEP_VALUE}m/s duration={DURATION}s", flush=True)
    print(f"start_t_epoch={start_t:.6f}", flush=True)
    print("t       dt(ms)  sp      meas_vel(x,y,z)             thrust   alt(z)  roll_sp pitch_sp", flush=True)

    while True:
        state = ekf.step()
        now = time.time()
        dt = now - last_t
        last_t = now
        t = now - start_t

        sp = vel_setpoint(t)
        thrust, roll_tau, pitch_tau, yaw_tau = cascade_tick(c, state, dt, sp, periods, elapsed, held)
        m1, m2, m3, m4 = cascade.mix(thrust, roll_tau, pitch_tau, yaw_tau, LIMITS["motor_range"], frame=c.frame)
        bridge.publish_motors(m1, m2, m3, m4)

        i += 1
        if i % 10 == 0:
            print(f"{t:6.2f}  {dt*1000:5.2f}  {sp[AXIS_INDEX]:5.2f}  "
                  f"({state['vel'][0]:6.2f},{state['vel'][1]:6.2f},{state['vel'][2]:6.2f})  "
                  f"{thrust:7.1f}  {state['pos'][2]:6.3f}  "
                  f"{held['roll_sp']:7.3f} {held['pitch_sp']:7.3f}", flush=True)

        if t > DURATION:
            print("test complete", flush=True)
            break


if __name__ == "__main__":
    main()
