"""Fly one benchmark scenario on PX4 SITL (PX4's own EKF2 + controller fly; nothing of
this project is in the loop). Runs INSIDE WSL next to a running PX4 SITL instance:

    python3 testing/benchmark/px4_mission.py <scenario> [--url udpin:0.0.0.0:14540]

Streams SET_POSITION_TARGET_LOCAL_NED (position + velocity feedforward + yaw) at 20 Hz in
OFFBOARD mode, then lands and waits for disarm. PX4 logs everything (SDLOG_PROFILE=3, set by
testing/suite/px4_shadow_flight.sh) - the .ulg is the benchmark's raw data.

All setpoints are NED (PX4's frame), relative to the arming point; yaw 0 = north.
Each scenario: dwell on the ground (calibration stretch for the replayed filters), climb to
2 m, a 360 deg calibration yaw turn (so the replayed EKF can learn mag bias, as in the
suite's *_ct missions; EKF2 is given the same turn), then the profile, then land.
"""
import argparse
import math
import sys
import time

from pymavlink import mavutil

ALT = 2.0
DWELL_S = 10.0       # on the ground, disarmed, before arming
CLIMB_S = 8.0
CAL_TURN_S = 12.0    # one smooth 360 deg turn
HOLD_S = 3.0
RATE_HZ = 20.0

# type_mask: use position, velocity, yaw; ignore accel and yaw rate
MASK_PVY = 0b0000_1001_1100_0000
# use velocity + yaw only (position ignored) - for the hard-stop sprints
MASK_VY = 0b0000_1001_1100_0111


def _smooth(u):
    # 0..1 -> 0..1 with zero slope at both ends
    u = min(max(u, 0.0), 1.0)
    return u * u * (3 - 2 * u)


def hover(t):
    return (0.0, 0.0, -ALT), (0.0, 0.0, 0.0), 0.0


def profile_box(t, side=8.0, v=2.0):
    # square, constant speed along each edge, fixed yaw
    per = 4 * side / v
    s = (t % per) * v
    k, d = int(s // side), s % side
    corners = [(0, 0), (side, 0), (side, side), (0, side), (0, 0)]
    (x0, y0), (x1, y1) = corners[k], corners[k + 1]
    ux, uy = (x1 - x0) / side, (y1 - y0) / side
    return (x0 + ux * d, y0 + uy * d, -ALT), (ux * v, uy * v, 0.0), 0.0


def profile_circle(t, r=5.0, v=4.0, ramp=4.0):
    # circle around (r, 0) starting at the origin, speed ramped in over `ramp` s
    # (a = v^2/r = 3.2 m/s^2 -> ~18 deg tilt at full speed)
    th = (v * (t - ramp / 2) if t > ramp else v * t * t / (2 * ramp)) / r
    vv = v * min(t / ramp, 1.0)
    x, y = r - r * math.cos(th), r * math.sin(th)
    vx, vy = vv * math.sin(th), vv * math.cos(th)
    return (x, y, -ALT), (vx, vy, 0.0), 0.0


def profile_figure8(t, a=8.0, w=0.45):
    # lemniscate of Gerono, peak speed ~5 m/s, +-1 m altitude, yaw follows velocity
    x, y = a * math.sin(w * t), a * math.sin(w * t) * math.cos(w * t)
    vx = a * w * math.cos(w * t)
    vy = a * w * math.cos(2 * w * t)
    z = -ALT - 1.0 * math.sin(0.5 * w * t)
    vz = -0.5 * w * math.cos(0.5 * w * t)
    return (x, y, z), (vx, vy, vz), math.atan2(vy, vx)


def profile_yaw_spin(t, rate=math.radians(60)):
    return (0.0, 0.0, -ALT), (0.0, 0.0, 0.0), (rate * t + math.pi) % (2 * math.pi) - math.pi


# sprint / hard-stop legs: velocity setpoints (PX4 accelerates at MPC_ACC_HOR), 10 m each way
def profile_stops(t, v=5.0, leg_s=2.5, rest_s=3.0):
    per = 2 * (leg_s + rest_s)
    u = t % per
    if u < leg_s:
        return None, (v, 0.0, 0.0), 0.0
    if u < leg_s + rest_s:
        return None, (0.0, 0.0, 0.0), 0.0
    if u < 2 * leg_s + rest_s:
        return None, (-v, 0.0, 0.0), 0.0
    return None, (0.0, 0.0, 0.0), 0.0


SCENARIOS = {  # name -> (profile, duration_s, aggressive PX4 limits?)
    "hover": (hover, 40.0, False),
    "box": (profile_box, 48.0, False),
    "circle_fast": (profile_circle, 45.0, True),
    "figure8": (profile_figure8, 42.0, True),
    "stops": (profile_stops, 44.0, True),
    "yaw_spin": (profile_yaw_spin, 30.0, False),
}
AGGRESSIVE_PARAMS = {"MPC_XY_VEL_MAX": 8.0, "MPC_XY_CRUISE": 6.0, "MPC_ACC_HOR_MAX": 6.0, "MPC_ACC_HOR": 6.0,
                     "MPC_TILTMAX_AIR": 40.0, "MPC_JERK_MAX": 30.0}


class Px4:
    def __init__(self, url):
        self.m = mavutil.mavlink_connection(url, source_system=255)
        self.m.wait_heartbeat(timeout=60)
        self.boot = time.time()
        print(f"connected sys={self.m.target_system}", flush=True)

    def param(self, name, value):
        self.m.mav.param_set_send(self.m.target_system, self.m.target_component, name.encode(), float(value),
                                  mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.1)

    def cmd(self, command, *p):
        p = list(p) + [0.0] * (7 - len(p))
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, command, 0, *p)

    def setpoint(self, pos, vel, yaw):
        mask = MASK_PVY if pos is not None else MASK_VY
        pos = pos or (0.0, 0.0, 0.0)
        self.m.mav.set_position_target_local_ned_send(
            int((time.time() - self.boot) * 1000) & 0xFFFFFFFF, self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask, *pos, *vel, 0, 0, 0, yaw, 0)

    def armed(self):
        hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        return hb is not None and bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def drain(self):
        while self.m.recv_match(blocking=False) is not None:
            pass


def run_for(px, duration, fn):
    """Stream fn(t) -> (pos, vel, yaw) at RATE_HZ for `duration` seconds."""
    t0 = time.time()
    while (t := time.time() - t0) < duration:
        px.setpoint(*fn(t))
        px.drain()
        time.sleep(1.0 / RATE_HZ)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario", choices=sorted(SCENARIOS))
    ap.add_argument("--url", default="udpin:0.0.0.0:14540")
    a = ap.parse_args()
    profile, duration, aggressive = SCENARIOS[a.scenario]
    px = Px4(a.url)
    if aggressive:
        for k, v in AGGRESSIVE_PARAMS.items():
            px.param(k, v)

    print("PHASE dwell", flush=True)
    t_end = time.time() + DWELL_S
    while time.time() < t_end:
        px.drain(); time.sleep(0.05)

    # offboard needs a setpoint stream before the mode switch is accepted
    ground = lambda t: ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)
    run_for(px, 1.0, ground)
    for _ in range(10):
        px.cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, 6)            # custom mode: OFFBOARD
        px.cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        run_for(px, 0.5, ground)
        if px.armed():
            break
    else:
        print("BENCH_RESULT fail=arm", flush=True)
        sys.exit(2)

    print("PHASE climb", flush=True)
    run_for(px, CLIMB_S, lambda t: ((0.0, 0.0, -ALT * _smooth(t / (CLIMB_S - 2))), (0.0, 0.0, 0.0), 0.0))
    print("PHASE cal_turn", flush=True)
    run_for(px, CAL_TURN_S, lambda t: ((0.0, 0.0, -ALT), (0.0, 0.0, 0.0),
                                       (2 * math.pi * _smooth(t / CAL_TURN_S) + math.pi) % (2 * math.pi) - math.pi))
    run_for(px, HOLD_S, hover)
    print(f"PHASE profile {a.scenario}", flush=True)
    run_for(px, duration, profile)
    # come back and settle before landing (velocity profiles end wherever they end)
    print("PHASE return", flush=True)
    run_for(px, 8.0, hover)
    print("PHASE land", flush=True)
    px.cmd(mavutil.mavlink.MAV_CMD_NAV_LAND)
    t_end = time.time() + 40
    while time.time() < t_end and px.armed():
        pass
    print(f"BENCH_RESULT ok scenario={a.scenario} disarmed={not px.armed()}", flush=True)


if __name__ == "__main__":
    main()
