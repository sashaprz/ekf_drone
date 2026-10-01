"""Setpoint schedules for the EKF stress-test suite (EKF_TEST_PLAN.md s3.1).

get(name)(t) -> {"pos": np.array([x, y, z]), "yaw": rad}, t = seconds since the control
loop started (motors spin up at t=0). Every mission starts with SETTLE_S of plain hover
at (0,0,2) - the same climb run_sim.py has always done - and its profile runs from
t=SETTLE_S for `duration` seconds. After that the setpoint holds the profile's final
position (no feedforward) - run_sim.py exits END_HOLD_S later.

Setpoint lead (why the position setpoint isn't just the path): pos_xy is P-only with
kp=0.15, so a setpoint moving at v sits v/0.15 metres ahead of the vehicle in steady
state (6.7 m at 1 m/s) - a 3 m circle setpoint would produce a ~1 m, 0.4 m/s circle.
Without touching the controller, each moving mission instead publishes
    p_des + (v_des + LEAD_ACCEL * a_des) / kp_pos
which makes position_loop's output ~= v_des (+ a velocity lead that roughly cancels
vel_xy's lag: with D-on-measurement kd=0.6 and kp=0.8, the loop's accel response is
about kp/(1+kd) per m/s of error -> lead = (1+kd)/kp * a_des). It's feedforward done in
setpoint shaping - approximate by design; recordings are validated on what the vehicle
actually did, not on the ideal path.
"""
import math
import numpy as np

SETTLE_S = 10.0
END_HOLD_S = 5.0
# in-flight calibration turn (2026-09-30): after the settle hover, one smooth 360deg yaw
# (CAL_TURN_S, peak ~45 deg/s) then CAL_HOLD_S of hover, before the profile starts. A
# dwell on the ground can't separate heading from mag bias; rotating the body under the
# fixed earth field does - the EKF learns the hard iron during the turn (EKF_TEST_REPORT.md).
CAL_TURN_S = 12.0
CAL_HOLD_S = 3.0
ALT = 2.0

# defaults = run_sim.py GAINS at the time of writing; run_sim passes the live values in
DEFAULT_KP_POS = 0.15
DEFAULT_LEAD_ACCEL = (1 + 0.6) / 0.8


class Mission:
    def __init__(self, name, duration, path, *, dynamic_start, aggressive=False, hypotheses=(), desc="",
                 cal_turn=False):
        # path(tau) -> (p, v, a, yaw) in profile time tau in [0, duration]
        self.name = name
        self.duration = duration
        self.path = path
        self.dynamic_start = dynamic_start  # profile time where fault windows start
        self.aggressive = aggressive        # needs MAX_VEL_XY=3.5 MAX_TILT_DEG=25 (ORACLE only)
        self.hypotheses = list(hypotheses)
        self.desc = desc
        self.kp_pos = DEFAULT_KP_POS
        self.lead_accel = DEFAULT_LEAD_ACCEL
        self.cal_turn = cal_turn

    @property
    def profile_start(self):
        # seconds from loop start until the mission profile begins
        return SETTLE_S + ((CAL_TURN_S + CAL_HOLD_S) if self.cal_turn else 0.0)

    @property
    def total(self):
        # seconds from loop start until run_sim.py exits
        return self.profile_start + self.duration + END_HOLD_S

    def ideal(self, t):
        # the intended trajectory (no lead) - for validation/plots
        tau = t - self.profile_start
        if tau < 0:
            yaw = 0.0
            if self.cal_turn and t >= SETTLE_S:
                yaw = 2 * math.pi * _smoothstep((t - SETTLE_S) / CAL_TURN_S)
            return np.array([0.0, 0.0, ALT]), np.zeros(3), np.zeros(3), yaw
        p, v, a, yaw = self.path(min(tau, self.duration))
        if tau > self.duration:
            v, a = np.zeros(3), np.zeros(3)
        return p, v, a, yaw

    def __call__(self, t):
        p, v, a, yaw = self.ideal(t)
        lead = (v + self.lead_accel * a) / self.kp_pos
        lead[2] = 0.0  # pos_z kp=1.0 tracks altitude fine without it
        return {"pos": p + lead, "yaw": yaw}


def _hold(tau):
    return np.array([0.0, 0.0, ALT]), np.zeros(3), np.zeros(3), 0.0


def _smoothstep(s):
    # 0..1 -> 0..1 with zero slope at both ends (used for ramps, not for "abrupt" legs)
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


BOX_CORNERS = [np.array(c, dtype=float) for c in [(0, 0), (5, 0), (5, 5), (0, 5)]]
BOX_MOVE, BOX_HOLD = 8.0, 4.0


def _box(tau):
    # constant-speed 5 m legs (0.625 m/s) with abrupt start/stop, 4 s hold at each corner
    leg_t = BOX_MOVE + BOX_HOLD
    k = int(tau // leg_t)
    s = tau - k * leg_t
    a_c, b_c = BOX_CORNERS[k % 4], BOX_CORNERS[(k + 1) % 4]
    if s < BOX_MOVE:
        vel = (b_c - a_c) / BOX_MOVE
        xy, v = a_c + vel * s, vel
    else:
        xy, v = b_c, np.zeros(2)
    return np.array([xy[0], xy[1], ALT]), np.array([v[0], v[1], 0.0]), np.zeros(3), 0.0


def _circle(radius, speed, ramp=5.0):
    # circle through the origin (centre (-r, 0)), angular speed ramped up over `ramp` s
    w_max = speed / radius

    def path(tau):
        if tau < ramp:
            w = w_max * tau / ramp
            wdot = w_max / ramp
            phi = 0.5 * w_max * tau ** 2 / ramp
        else:
            w, wdot = w_max, 0.0
            phi = 0.5 * w_max * ramp + w_max * (tau - ramp)
        c, s = math.cos(phi), math.sin(phi)
        p = np.array([-radius + radius * c, radius * s, ALT])
        v = np.array([-radius * w * s, radius * w * c, 0.0])
        a = np.array([-radius * wdot * s - radius * w * w * c, radius * wdot * c - radius * w * w * s, 0.0])
        return p, v, a, 0.0
    return path


def _stops(speed=3.0, half=4.0):
    # triangle wave in x between -half and +half at `speed`: 8 m dashes, instant reversals
    period = 4 * half / speed

    def path(tau):
        ph = (tau + half / speed) % period  # starts at x=0 heading +x
        if ph < period / 2:
            x, vx = -half + speed * ph, speed
        else:
            x, vx = half - speed * (ph - period / 2), -speed
        return np.array([x, 0.0, ALT]), np.array([vx, 0.0, 0.0]), np.zeros(3), 0.0
    return path


def _yaw_steps(tau):
    seq = [0, 90, 180, -90, 0, 0]
    return np.array([0.0, 0.0, ALT]), np.zeros(3), np.zeros(3), math.radians(seq[min(int(tau // 10), 5)])


def _yaw_spin(tau):
    return np.array([0.0, 0.0, ALT]), np.zeros(3), np.zeros(3), math.radians(30.0) * tau


def _takeoff_land(tau):
    # 0-8 hold 2 m | 8-16 descend to -0.3 (below ground: guarantees touchdown) |
    # 16-24 sit on the ground | 24-40 re-takeoff to 2 m and hold
    if tau < 8:
        z = ALT
    elif tau < 16:
        z = ALT + (-0.3 - ALT) * (tau - 8) / 8
    elif tau < 24:
        z = -0.3
    else:
        z = ALT
    return np.array([0.0, 0.0, z]), np.zeros(3), np.zeros(3), 0.0


MISSIONS = {
    "hover": Mission("hover", 60, _hold, dynamic_start=20, hypotheses=["control"],
                     desc="hold (0,0,2)"),
    "box": Mission("box", 60, _box, dynamic_start=24, hypotheses=["H4"],
                   desc="5 m square at 2 m, 8 s legs, 4 s holds"),
    "circle_slow": Mission("circle_slow", 60, _circle(3.0, 1.0), dynamic_start=15, hypotheses=["H4"],
                           desc="r=3 m, 1 m/s (~0.33 m/s^2)"),
    "circle_fast": Mission("circle_fast", 60, _circle(3.0, 3.0), dynamic_start=15, aggressive=True,
                           hypotheses=["H4"], desc="r=3 m, 3 m/s (~3 m/s^2, ~17 deg)"),
    "stops": Mission("stops", 45, _stops(), dynamic_start=10, aggressive=True, hypotheses=["H4"],
                     desc="8 m dashes at 3 m/s, abrupt reversals"),
    "yaw_steps": Mission("yaw_steps", 60, _yaw_steps, dynamic_start=9, hypotheses=["H3"],
                         desc="yaw 0/90/180/-90/0, 10 s each"),
    "yaw_spin": Mission("yaw_spin", 60, _yaw_spin, dynamic_start=15, hypotheses=["H3"],
                        desc="continuous 30 deg/s yaw"),
    "patrol_long": Mission("patrol_long", 600, _box, dynamic_start=100, hypotheses=["H5", "drift"],
                           desc="box repeated for 10 min"),
    "takeoff_land": Mission("takeoff_land", 40, _takeoff_land, dynamic_start=6, hypotheses=["ground"],
                            desc="hover, land, sit, re-takeoff"),
}

# same missions with the in-flight calibration turn first
MISSIONS["hover_ct"] = Mission("hover_ct", 60, _hold, dynamic_start=20, hypotheses=["H3", "cal"],
                               desc="calibration turn, then hold (0,0,2)", cal_turn=True)
MISSIONS["box_ct"] = Mission("box_ct", 60, _box, dynamic_start=24, hypotheses=["H3", "cal"],
                             desc="calibration turn, then the box", cal_turn=True)

# the aggressive-limit overrides these missions need (run_sim MAX_VEL_XY / MAX_TILT_DEG)
AGGRESSIVE_LIMITS = {"MAX_VEL_XY": "3.5", "MAX_TILT_DEG": "25"}


def get(name, gains=None, cal_turn=None):
    m = MISSIONS[name]
    if cal_turn is not None:
        m.cal_turn = m.cal_turn or cal_turn
    if gains is not None:
        m.kp_pos = gains["pos_xy"]["kp"]
        m.lead_accel = (1 + gains["vel_xy"]["kd"]) / gains["vel_xy"]["kp"]
    return m
