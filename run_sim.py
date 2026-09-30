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
    # pos_xy TRIED at kp=1.0/kd=0.5 (2026-09-26) and REVERTED - made the full-cascade
    # divergence happen even earlier (t~1.92s vs ~1.85-1.90s), not later. The theory
    # that motivated trying it (unbounded position drift from nothing correcting
    # position) is likely wrong, or at least incomplete - see HANDOFF.md's "full
    # cascade divergence, root cause still open" note. Back to zero until the real
    # cause is found - don't re-enable this blind based on the same reasoning that
    # motivated the reverted attempt.
    #
    # vel_xy given real gains, 2026-09-26 (was zeroed as a diagnostic predating this
    # session, to isolate whether horizontal drift correction caused the old "aggressive
    # tilting" failure - that diagnosis is stale now given everything found since: the
    # ground-contact bug, yaw-authority gap, and vel_z's missing feedforward were all
    # real, sufficient explanations on their own). Needed now because leaving vel_xy at
    # zero means NOTHING bounds horizontal velocity, ever - confirmed via
    # test_velocity_loop.py: even commanding vz alone (attitude target held level, so
    # ax=ay=0 by construction with vel_xy's old zero gains), residual bias let vy drift
    # unbounded past 40 m/s over a few seconds, which is almost certainly what triggered
    # a real acceleration burst and the known intermittent EKF jump (see HANDOFF.md).
    # kp/kd re-tuned 2026-09-26 after fixing a real a_right sign bug in cascade.py's
    # velocity_loop() (see that fix's own comment) - roll_sp was saturating in the
    # WRONG direction and monotonically failing to arrest drift before the fix; after
    # the fix, drift got even worse but qualitatively different (roll_sp/pitch_sp
    # oscillating between +/- max_tilt_rad rather than pinned one-sided) - the
    # signature of a now-correctly-signed but underdamped/too-aggressive negative
    # feedback loop (oscillation), not a wrong-sign one (which saturates one-sided and
    # sits there). kp cut 2.0->0.8, kd raised 0.3->0.6 for more damping. NOT yet
    # confirmed stable - re-test before trusting these values.
    # RE-TESTED 2026-09-27 at kp=1.0/kd=0.5, then kp=0.15/kd=0.0, after today's real
    # timing-noise fix (see gz_bridge.py's wait_for_imu()) - confirmed the 2026-09-26
    # revert wasn't a stale-noise artifact: BOTH gains caused a violent full tumble
    # (kp=1.0 at t=4.35s, kp=0.15 at t=5.10s - tau saturated +/-100, rate spikes of
    # 20-30 rad/s on multiple axes). The EKF_ACCEL diagnostic right before each tumble
    # shows a real, smoothly growing/oscillating accel deviation (0.4->1.0+ m/s²,
    # cyclic, not the 400+ m/s² single-tick garbage pattern from the already-fixed
    # sensor-corruption bug) - a genuine resonance building over ~1s, not corrupted
    # data. A pos_xy=0 run under the exact same fixed timing survived a full 20s with
    # no tumble at all (just unbounded +X drift, the problem pos_xy is meant to fix).
    # Conclusion: ANY nonzero pos_xy currently excites a real resonance/instability in
    # the velocity->attitude->rate cascade, regardless of magnitude - this points at
    # insufficient damping margin somewhere inside that cascade (attitude/rate loop
    # tuning, most likely yaw per the existing "yaw fails last/worst" pattern - see
    # HANDOFF.md), not at pos_xy's own gain value. Back to 0 until that margin is
    # established; re-attempting pos_xy at ANY gain before then is expected to
    # reproduce this same failure.
    # RE-TESTING 2026-09-27 at kp=0.15/kd=0.0 (the same gentle value that still
    # tumbled at t=5.10s before) now that att_yaw's kp has been raised 1.5->4.0 (see
    # that comment) - isolated attitude-loop testing showed insufficient att_yaw
    # authority was a real, separate bug, independent of pos_xy. Testing whether that
    # was also the pos_xy resonance's root cause.
    "pos_xy": {"kp": 0.15, "ki": 0.0, "kd": 0.0},
    # pos_z/vel_z re-tuned 2026-09-30 from the actual plant gain instead of by feel:
    # d(accel)/d(omega) = 8*motorConstant*omega_hover/mass ~= 0.026 m/s^2 per rad/s of
    # trim, so vel_z kp=15 was a ~0.4 rad/s inner loop sitting UNDER a faster pos_z
    # (kp=1.5) - a textbook slow outer-loop oscillation. Confirmed in ORACLE mode
    # (oracle_trial1.log: attitude dead level for 38s, altitude swinging 0<->4m every
    # ~6s, touching the ground) and reproduced by a 1-D sim of this exact loop
    # structure (old gains: peak 4.0m, 2.4m p-p sustained). New: vel_z kp=100 (~2.6
    # rad/s) with ki well below it, pos_z kp=1.0 below that - 1-D sim: 2.0m, ~4cm
    # overshoot, no sustained oscillation.
    "pos_z":  {"kp": 1.0, "ki": 0.0, "kd": 0.0},
    # d_filter_alpha added 2026-09-30 - kd differentiates the EKF's velocity estimate,
    # which steps at every 5Hz GPS correction; unfiltered, one 0.1 m/s step over a 75Hz
    # tick is ~7.5 m/s^2 * 0.6 = a full-scale accel command -> tilt_sp slammed to
    # +/-15deg on both axes at once (run_sim_tiltsp_trial1.log t=1.42).
    "vel_xy": {"kp": 0.8, "ki": 0.2, "kd": 0.6, "integral_limits": (-5.0, 5.0), "d_filter_alpha": 0.2},
    # cut further (40/60 -> 15/20) - across three runs now, instability tracked with
    # how fast/high thrust ramped up, not a fixed threshold - a gentler climb gives
    # attitude/rate loops room to keep up instead of fighting a fast-moving baseline
    #
    # output_limits/integral_limits changed from (0,1000) to a trim range, 2026-09-26 -
    # cascade.py's velocity_loop() now adds VEL_Z_HOVER_THRUST_FF (757, feedforward)
    # before this PID's output, so it only needs to trim around hover, not build the
    # whole thrust from its own integral (see VEL_Z_HOVER_THRUST_FF's comment in
    # cascade.py for why that was a real problem - confirmed via test_velocity_loop.py,
    # thrust only reached ~153 after 2s without the feedforward, vehicle fell >3.5m).
    # Range is asymmetric to match real headroom around the 757 baseline (757-400=357
    # min, 757+240=997 max, both comfortably inside the real 0-1000 motor range).
    "vel_z":  {"kp": 100.0, "ki": 25.0, "kd": 0.0, "integral_limits": (-16.0, 10.0)},
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
    # d_filter_alpha added, 2026-09-26 - att_rp/att_yaw's kd was DEAD until today (see
    # attitude_loop()'s fix comment in cascade.py: it called PID.update() with
    # measurement hardcoded to 0.0, so derivative-on-measurement always computed to
    # exactly zero regardless of kd's value). Now that it's genuinely active for the
    # first time, it's differentiating a real, EKF-noisy error signal with no
    # filtering - same risk rate_rp/rate_yaw's own d_filter_alpha already guards
    # against ("kd differentiates the RAW gyro reading... without filtering that noise
    # gets amplified straight into torque chatter"). Confirmed this bit immediately:
    # first `run_sim.py` test after the attitude_loop fix hit tau=(100,100,100) (full
    # saturation on all 3 axes) and rate_sp pinned at its 3.0 rad/s cap on two axes,
    # from a modest ~4-6 degree tilt state - a derivative kick, not a real disturbance
    # response. NOT yet re-tested with this filter in place.
    # kd raised 0.5->1.5, 2026-09-26 - post D-term-fix trials all show the same shape
    # (smooth, accelerating tilt growth over ~2-4s, no oscillation, then a final
    # saturation event) regardless of the filter/gyro-clamp fixes tried - the classic
    # signature of insufficient damping, not a bug. kd was NEVER tuned (it was dead
    # until today), so this is genuinely a first attempt, not a refinement.
    "att_rp":  {"kp": 6.0, "ki": 0.5, "kd": 1.5, "integral_limits": (-1.0, 1.0), "d_filter_alpha": 0.2},
    # kd raised 0.05->0.3, 2026-09-26 - with att_rp's kd fixed/raised, roll/pitch stayed
    # under 5 degrees far longer than any previous trial, but yaw diverged FIRST this
    # time (7.8->21.9->oscillating). att_yaw's kp:kd ratio was 30:1 (1.5:0.05) vs
    # att_rp's now much more damped 4:1 (6.0:1.5) - brought into a similar proportion
    # (1.5:0.3 = 5:1). Not yet tested.
    # kp raised 1.5->4.0, 2026-09-27 - the isolated attitude-loop yaw test
    # (HANDOFF.md "Same-day follow-up") showed rate_sp_yaw staying tiny (0.04-0.09
    # rad/s, nowhere near the 3.0 cap) while yaw_tilt drifted steadily away from its
    # level target the whole time - att_yaw wasn't saturating, it just wasn't
    # commanding enough correction relative to whatever real disturbance was pulling
    # yaw away (gyro_bias_z and calibration ambiguity both checked and ruled out as
    # too small to explain it - see HANDOFF.md). Plenty of headroom before hitting
    # max_rate_yaw, so raising kp is a direct, well-justified first test.
    "att_yaw": {"kp": 4.0, "ki": 0.3, "kd": 0.3, "integral_limits": (-1.0, 1.0), "d_filter_alpha": 0.2},
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
    # kd=5.0 TRIED and REVERTED, 2026-09-26 - made things clearly worse: diverged by
    # t=2.0s (vs ~5.3s for the best trial so far, att_yaw kd=0.3 with this at kd=0.0),
    # and never recovered - rate_meas froze at a constant nonsense value for the rest
    # of a 45s run, almost certainly gz_bridge.py's gyro sanity clamp correctly
    # rejecting a vehicle spinning fast enough to permanently exceed 50 rad/s, holding
    # stale data forever once the tumble became real. Back to kd=0.0 - this loop's
    # damping is not the fix, at least not at this magnitude. The best full-cascade
    # result so far used this exact set of gains (rate_yaw kd=0.0, att_rp kd=1.5,
    # att_yaw kd=0.3) - don't re-try kd>0 here without a specific new reason to.
    "rate_yaw": {"kp": 150.0, "ki": 2.0, "kd": 0.0, "integral_limits": (-100.0, 100.0), "d_filter_alpha": 0.2},
}
LIMITS = {
    "max_tilt_rad": math.radians(15), "max_rate": 3.0,  # down from 30 deg, extra margin while still tuning
    # vel_pid_z's OWN output_limits - a trim range around cascade.py's
    # VEL_Z_HOVER_THRUST_FF feedforward now, not absolute thrust (see that constant's
    # comment and vel_z's GAINS comment above for why this changed, 2026-09-26)
    "thrust_range": (-400.0, 240.0),
    "motor_range": (0.0, 1000.0),       # matches x500's maxRotVelocity
    # max_vel_xy cut 5.0->1.5, 2026-09-28 - the 3 post-att_yaw-fix pos_xy trials
    # (HANDOFF.md "CURRENT blocker") failed on a different axis combo each time
    # (roll alone, roll+pitch, roll+yaw) - no single weak attitude/rate loop, a
    # shared damping-margin gap the rotorDrag/velocity-feedback torque (already
    # confirmed via the isolated roll-attitude test) can exploit on whichever axis
    # is weakest that run. Capping how much horizontal velocity can ever build
    # bounds that disturbance torque directly, regardless of vel_xy's own reaction
    # speed - a more direct lever than retuning vel_xy's gains. vel_xy gains left
    # untouched so this is a single-variable test.
    "max_vel_xy": 1.5, "max_vel_z": 3.0, "max_accel_xy": 5.0, "max_rate_yaw": 3.0,
    "torque_range_rp": (-100.0, 100.0),   # down from +/-250, omega perturbation not physical torque
    "torque_range_yaw": (-100.0, 100.0),
}

# 2026-09-27: same defensive backstop as FINAL_gps.py's MAX_DT (see that comment) - now
# that the main loop below waits on GazeboBridge.wait_for_imu() instead of busy-spinning,
# this should essentially never bind, only during a genuine stall.
MAX_DT = 0.05


def quat_to_euler_deg(q):
    # [w,x,y,z] -> (roll,pitch,yaw) in degrees, ZYX convention, matches cascade.euler_to_quat
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def true_str(bridge):
    # Gazebo ground truth next to the estimate - see GazeboBridge.get_true_pose()
    tq, tp = bridge.get_true_pose()
    if tq is None:
        return ""
    r, p, y = quat_to_euler_deg(tq)
    return f" TRUE rpy=({r:6.1f},{p:6.1f},{y:6.1f}) pos=({tp[0]:5.2f},{tp[1]:5.2f},{tp[2]:5.2f})"


# Diagnostics, 2026-09-30 - both default off, so a plain `python3 run_sim.py` is unchanged.
# ORACLE=1: the controller flies on Gazebo's TRUE attitude/position (+ GPS velocity,
#   + raw gyro) instead of the EKF's estimate - the EKF still runs alongside and is
#   logged, but nothing reads it. Separates "controller can't fly" from "estimator is
#   lying to the controller", which were impossible to tell apart from a crash log.
# SENSOR_LOG=path.csv: every tick's raw gyro/accel/mag/gps + ground truth, so the EKF
#   can be replayed offline against truth (replay_ekf.py) in seconds instead of
#   minutes-per-Gazebo-crash.
ORACLE = os.environ.get("ORACLE") == "1"
SENSOR_LOG = os.environ.get("SENSOR_LOG")


def oracle_state(bridge, ekf_state):
    tq, tp = bridge.get_true_pose()
    if tq is None:
        return ekf_state
    gps = bridge.get_gps()
    return {"quat": np.array(tq), "pos": np.array(tp), "vel": np.array(gps[3:6]),
            "rate": np.array(bridge.get_gyro())}


def main():
    bridge = GazeboBridge()
    time.sleep(0.5)  # let the first real gz-transport messages arrive before calibration reads them -
                      # otherwise it'd read the bridge's placeholder zeros (e.g. accel=0 instead of ~9.8)
    ekf = DroneEKF(sensors=bridge)
    c = cascade.Cascade({"GAINS": GAINS, "LIMITS": LIMITS})

    setpoint = {"pos": np.array([0.0, 0.0, 2.0]), "yaw": 0.0}  # climb to 2m and hold

    log = open(SENSOR_LOG, "w") if SENSOR_LOG else None
    if log:
        log.write("t,gx,gy,gz,ax,ay,az,mx,my,mz,px,py,pz,vx,vy,vz,tqw,tqx,tqy,tqz,tpx,tpy,tpz\n")
    if ORACLE:
        print("ORACLE MODE - controller is flying on Gazebo ground truth, not the EKF", flush=True)

    last_t = time.time()
    start_t = last_t
    i = 0
    print("running - Ctrl+C to stop", flush=True)
    while True:
        bridge.wait_for_imu()  # paces the loop to real sensor arrival - see that method's comment
        state = ekf.step()
        est_state = state
        if ORACLE:
            state = oracle_state(bridge, est_state)

        now = time.time()
        if log:
            tq, tp = bridge.get_true_pose()
            tq, tp = tq or (1.0, 0.0, 0.0, 0.0), tp or (0.0, 0.0, 0.0)
            row = (now, *bridge.get_gyro(), *bridge.get_accel(), *bridge.get_mag(), *bridge.get_gps(), *tq, *tp)
            log.write(",".join(f"{v:.6f}" for v in row) + "\n")
        dt = min(now - last_t, MAX_DT)
        last_t = now

        thrust, roll_tau, pitch_tau, yaw_tau = c.step(setpoint, state, dt)
        m1, m2, m3, m4 = cascade.mix(thrust, roll_tau, pitch_tau, yaw_tau, LIMITS["motor_range"], frame=c.frame)
        bridge.publish_motors(m1, m2, m3, m4)

        i += 1
        if i % 30 == 0:
            roll_deg, pitch_deg, yaw_deg = quat_to_euler_deg(est_state["quat"])
            print(f"t={now - start_t:6.2f} dt={dt*1000:6.2f}ms "
                  f"est(rpy)=({roll_deg:6.1f},{pitch_deg:6.1f},{yaw_deg:6.1f})deg "
                  f"tilt_sp=({math.degrees(c._roll_sp):6.1f},{math.degrees(c._pitch_sp):6.1f})deg "
                  f"rate_meas=({state['rate'][0]:5.2f},{state['rate'][1]:5.2f},{state['rate'][2]:5.2f}) "
                  f"rate_sp=({c._rate_sp[0]:5.2f},{c._rate_sp[1]:5.2f},{c._rate_sp[2]:5.2f}) "
                  f"thrust={thrust:6.1f} tau=({roll_tau:6.1f},{pitch_tau:6.1f},{yaw_tau:6.1f}) "
                  f"pos=({est_state['pos'][0]:5.2f},{est_state['pos'][1]:5.2f},{est_state['pos'][2]:5.2f})"
                  + true_str(bridge), flush=True)


if __name__ == "__main__":
    main()
