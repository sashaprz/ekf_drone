"""Seeded sensor corruptions applied to a recording at replay time (EKF_TEST_PLAN.md s3.3).

A recording is a dict of numpy column arrays (see run_suite.load_recording), with the
calibration samples prepended as rows flagged is_cal=1 (run_suite.with_cal_rows) so that
power-on faults (biases, noise) corrupt calibration exactly like they would on hardware.
A fault's apply(cols, ctx) returns (new_cols, info) and never mutates its input:
  - new_cols may drop rows (imu_dropouts) and may add a "gps_ok" column (0 = the
    receiver reports no fix -> get_gps() returns None -> the EKF skips its GPS update)
  - info: {"window": (t_start, t_end) absolute row time or None,
           "gyro_bias"/"accel_bias": (N,3) injected truth per row (body frame),
           "mag_bias": (3,) injected hard-iron vector, "gps_outlier": (N,) bool, ...}
ctx: {"t0": first row time, "dyn_t": absolute time where the mission's most dynamic
segment starts (fault windows start there)}.

Magnitudes - assumptions, all "cheap hobby hardware" class (stated again in the report):
  gyro: MPU6000/ICM-20602-class MEMS. Turn-on bias up to ~+/-1 deg/s after factory cal,
        in-run drift a few deg/s over temperature -> 0.2 / 1.0 deg/s constant, 0.5 deg/s
        ramp. Noise density ~0.005-0.01 deg/s/rtHz x ~100 Hz bandwidth -> ~0.005 rad/s
        white per sample at 250 Hz (Gazebo's x500 IMU: 0.00087 rad/s, ~6x less).
  accel: zero-g offset +/-50 mg typical (0.5 m/s^2) worst case, 0.05-0.2 after cal; noise
        ~0.05 m/s^2 per sample at 250 Hz (plus vibration, not modelled).
  mag: hard-iron residual after a rushed calibration 5-15% of field; motor-current
        interference on small quads is commonly 30-100% of the earth field.
  GPS: u-blox M8/M10 class: ~1.5 m CEP horizontal, ~2x vertical, 0.05-0.1 m/s velocity,
        slow multipath wander; occasional 10-50 m jumps in urban/tree cover; cheap modules
        default to 1 Hz.
"""
import zlib
import numpy as np

G = 9.80665
IMU = ("gx", "gy", "gz", "ax", "ay", "az", "mx", "my", "mz")
GPS_POS = ("px", "py", "pz")
GPS_VEL = ("vx", "vy", "vz")


def _rng(name, seed, salt=""):
    # stable across processes/runs (Python's hash() is randomized per process)
    return np.random.default_rng([seed, zlib.crc32((name + salt).encode())])


def _copy(cols):
    return {k: v.copy() for k, v in cols.items()}


def _xyz(cols, pre):
    return np.stack([cols[pre + "x"], cols[pre + "y"], cols[pre + "z"]], axis=1)


def _set_xyz(cols, pre, arr):
    cols[pre + "x"], cols[pre + "y"], cols[pre + "z"] = arr[:, 0].copy(), arr[:, 1].copy(), arr[:, 2].copy()


def _is_cal(cols):
    return cols["is_cal"] == 1 if "is_cal" in cols else np.zeros(len(cols["t"]), bool)


def _window(ctx, duration):
    return ctx["dyn_t"], ctx["dyn_t"] + duration


def _signs(rng):
    return rng.choice([-1.0, 1.0], size=3)


class Fault:
    def __init__(self, name, params=None, seed=1, hypotheses=()):
        self.name = name
        self.params = dict(params or {})
        self.seed = seed
        self.hypotheses = list(hypotheses)

    @property
    def key(self):
        # unique, stable id: name + sorted params
        if not self.params:
            return self.name
        return self.name + "(" + ",".join(f"{k}={v}" for k, v in sorted(self.params.items())) + ")"

    def apply(self, cols, ctx):
        c = _copy(cols)
        n = len(c["t"])
        info = {"window": None, "gyro_bias": np.zeros((n, 3)), "accel_bias": np.zeros((n, 3)),
                "mag_bias": np.zeros(3), "gps_outlier": np.zeros(n, bool)}
        return getattr(self, "_" + self.name)(c, ctx, info)

    # ---- GPS -------------------------------------------------------------------
    def _none(self, c, ctx, info):
        return c, info

    def _gps_dropout(self, c, ctx, info):
        ws, we = _window(ctx, self.params["duration_s"])
        ok = np.ones(len(c["t"]))
        ok[(c["t"] >= ws) & (c["t"] < we)] = 0.0
        c["gps_ok"] = ok
        info["window"] = (ws, we)
        return c, info

    def _gps_stale(self, c, ctx, info):
        # receiver hung: keeps reporting its last fix (position AND velocity) as valid
        ws, we = _window(ctx, self.params["duration_s"])
        m = (c["t"] >= ws) & (c["t"] < we)
        if m.any():
            k0 = max(np.argmax(m) - 1, 0)
            for k in GPS_POS + GPS_VEL:
                c[k][m] = c[k][k0]
        info["window"] = (ws, we)
        return c, info

    def _gps_noise(self, c, ctx, info, rng=None):
        rng = rng or _rng(self.name, self.seed)
        n, t = len(c["t"]), c["t"]
        h, v, vel = self.params.get("h_m", 1.5), self.params.get("v_m", 3.0), self.params.get("vel_ms", 0.1)
        rw = self.params.get("rw_m_per_sqrt_min", 0.5)
        dt = np.diff(t, prepend=t[0])
        walk = np.cumsum(rng.normal(0, 1, (n, 3)) * (rw * np.sqrt(dt / 60.0))[:, None], axis=0)
        white = rng.normal(0, 1, (n, 3)) * np.array([h, h, v])
        for j, k in enumerate(GPS_POS):
            c[k] = c[k] + white[:, j] + walk[:, j]
        vn = rng.normal(0, vel, (n, 3))
        for j, k in enumerate(GPS_VEL):
            c[k] = c[k] + vn[:, j]
        return c, info

    def _gps_outliers(self, c, ctx, info):
        rng = _rng(self.name, self.seed)
        n = len(c["t"])
        hit = rng.random(n) < self.params.get("frac", 0.01)
        hit[0] = False
        d = rng.normal(0, 1, (n, 3))
        d /= np.linalg.norm(d, axis=1, keepdims=True)
        off = d * rng.uniform(self.params.get("min_m", 10), self.params.get("max_m", 50), n)[:, None]
        for j, k in enumerate(GPS_POS):
            c[k] = np.where(hit, c[k] + off[:, j], c[k])
        info["gps_outlier"] = hit
        return c, info

    def _gps_rate(self, c, ctx, info):
        # only one row per period carries a fix; the EKF polls every tick while a fix is
        # due, so it consumes each 1 Hz fix on the row it arrives
        period = 1.0 / self.params["hz"]
        epoch = np.floor((c["t"] - ctx["t0"]) / period)
        c["gps_ok"] = np.r_[1.0, (np.diff(epoch) > 0).astype(float)]
        return c, info

    def _gps_latency(self, c, ctx, info):
        # every fix arrives latency_ms late: the row at time t carries the GPS values the
        # recording had at t - latency (real receivers: ~100-200 ms; Gazebo's: ~0). The EKF
        # fuses each fix as if it described "now" - this measures what that costs.
        t = c["t"]
        idx = np.clip(np.searchsorted(t, t - self.params["latency_ms"] / 1000.0, side="right") - 1, 0, len(t) - 1)
        for k in GPS_POS + GPS_VEL:
            c[k] = c[k][idx]
        # what the EKF is told (delayed fusion) - the true latency unless overridden, e.g.
        # ekf_latency_ms=0 to see the uncompensated filter
        info["gps_latency_s"] = self.params.get("ekf_latency_ms", self.params["latency_ms"]) / 1000.0
        return c, info

    # ---- IMU / mag -------------------------------------------------------------
    def _gyro_bias(self, c, ctx, info):
        rng = _rng(self.name, self.seed)
        b = np.radians(self.params["dps"]) * _signs(rng)
        bias = np.tile(b, (len(c["t"]), 1))
        if not self.params.get("from_cal", True):
            bias[_is_cal(c)] = 0.0  # in-run shift: appears after calibration
        _set_xyz(c, "g", _xyz(c, "g") + bias)
        info["gyro_bias"] = bias
        return c, info

    def _gyro_drift(self, c, ctx, info):
        # linear ramp from 0 (at calibration) to dps at over_s seconds, then held
        rng = _rng(self.name, self.seed)
        frac = np.clip((c["t"] - ctx["t0"]) / self.params["over_s"], 0, 1)[:, None]
        bias = frac * np.radians(self.params["dps"]) * _signs(rng)
        _set_xyz(c, "g", _xyz(c, "g") + bias)
        info["gyro_bias"] = bias
        return c, info

    def _accel_bias(self, c, ctx, info):
        b = np.zeros(3)
        b["xyz".index(self.params["axis"])] = self.params["ms2"]
        bias = np.tile(b, (len(c["t"]), 1))
        if not self.params.get("from_cal", True):
            bias[_is_cal(c)] = 0.0
        _set_xyz(c, "a", _xyz(c, "a") + bias)
        info["accel_bias"] = bias
        return c, info

    def _renorm_mag(self, c, m):
        # additive in the bridge's units (raw field / one fixed field strength) - gz_bridge
        # stopped normalizing each reading 2026-09-30, so a hard iron stays a constant
        # body-frame offset, as on real hardware. (Suite v1 renormalized here, which made
        # the offset orientation-dependent - results for mag_bias/mag_interference before
        # that change aren't comparable.)
        _set_xyz(c, "m", m)

    def _mag_bias(self, c, ctx, info, rng=None):
        # hard iron: fixed body-frame vector added to the field (unit-field fraction),
        # present from power-on (calibration sees it too)
        rng = rng or _rng(self.name, self.seed)
        d = rng.normal(0, 1, 3)
        b = self.params["frac"] * d / np.linalg.norm(d)
        self._renorm_mag(c, _xyz(c, "m") + b)
        info["mag_bias"] = b
        return c, info

    def _mag_interference(self, c, ctx, info):
        # 5 s burst of motor-current-like interference: fixed body direction, amplitude
        # scaled by specific force along body z (a thrust proxy - motor current tracks thrust)
        rng = _rng(self.name, self.seed)
        ws, we = _window(ctx, self.params.get("duration_s", 5.0))
        d = rng.normal(0, 1, 3)
        d /= np.linalg.norm(d)
        on = ((c["t"] >= ws) & (c["t"] < we)).astype(float)
        amp = self.params["amp"] * np.clip(c["az"] / G, 0, 2) * on
        m = _xyz(c, "m")
        hit = on > 0
        m[hit] = m[hit] + amp[hit, None] * d  # only the burst rows; additive, see _renorm_mag
        _set_xyz(c, "m", m)
        info["window"] = (ws, we)
        return c, info

    def _imu_noise(self, c, ctx, info, rng=None):
        rng = rng or _rng(self.name, self.seed)
        n = len(c["t"])
        _set_xyz(c, "g", _xyz(c, "g") + rng.normal(0, self.params.get("gyro", 0.005), (n, 3)))
        _set_xyz(c, "a", _xyz(c, "a") + rng.normal(0, self.params.get("accel", 0.05), (n, 3)))
        return c, info

    def _imu_spikes(self, c, ctx, info):
        rng = _rng(self.name, self.seed)
        n = len(c["t"])
        for pre, mag in (("g", self.params.get("gyro", 5.0)), ("a", self.params.get("accel", 30.0))):
            hit = (rng.random(n) < self.params.get("frac", 0.001)) & ~_is_cal(c)  # in flight only
            axis = rng.integers(0, 3, n)
            sign = rng.choice([-1.0, 1.0], n)
            for j, ax in enumerate("xyz"):
                sel = hit & (axis == j)
                c[pre + ax][sel] += sign[sel] * mag
        return c, info

    def _imu_dropouts(self, c, ctx, info):
        # lost samples: the row simply never arrives, so the next step's dt doubles
        rng = _rng(self.name, self.seed)
        keep = (rng.random(len(c["t"])) >= self.params.get("frac", 0.02)) | _is_cal(c)
        keep[0] = True
        c = {k: v[keep] for k, v in c.items()}
        info.update(gyro_bias=info["gyro_bias"][keep], accel_bias=info["accel_bias"][keep],
                    gps_outlier=info["gps_outlier"][keep], keep=keep)
        return c, info

    def _imu_gap(self, c, ctx, info):
        # the loop stalls: NO samples at all for gap_s (host hiccup / bus lockup), as seen
        # naturally in the patrol_long recording (0.6-1.1 s stalls at t=59-65 s). The EKF
        # then sees one step with dt = gap_s, which FINAL_gps clamps to MAX_DT.
        ws, we = _window(ctx, self.params["gap_s"])
        keep = ~((c["t"] >= ws) & (c["t"] < we)) | _is_cal(c)
        c = {k: v[keep] for k, v in c.items()}
        info.update(gyro_bias=info["gyro_bias"][keep], accel_bias=info["accel_bias"][keep],
                    gps_outlier=info["gps_outlier"][keep], keep=keep, window=(ws, we))
        return c, info

    def _cal_ideal(self, c, ctx, info):
        # counterfactual, not a fault: every calibration sample replaced by the mean of the
        # dwell (a noise-free dwell). Separates error the live calibration locked in from
        # error that arises in flight (H3/H2).
        cal = _is_cal(c)
        for k in IMU:
            c[k][cal] = c[k][cal].mean()
        return c, info

    def _combined_realistic(self, c, ctx, info):
        subs = [Fault("gps_noise", {}, self.seed), Fault("imu_noise", {}, self.seed),
                Fault("gyro_bias", {"dps": 0.2}, self.seed), Fault("accel_bias", {"axis": "x", "ms2": 0.05}, self.seed),
                Fault("mag_bias", {"frac": 0.05}, self.seed)]
        for f in subs:
            c, sub = f.apply(c, ctx)
            for k in ("gyro_bias", "accel_bias", "mag_bias"):
                info[k] = info[k] + sub[k]
        return c, info


def _mk(name, hyp, **params):
    return Fault(name, params, 1, hyp)


# every fault level the suite runs, in report order
ALL_FAULTS = (
    [_mk("none", [])]
    + [_mk("gps_dropout", ["H1"], duration_s=d) for d in (5, 15, 30)]
    + [_mk("gps_stale", ["H1"], duration_s=d) for d in (5, 15, 30)]
    + [_mk("gps_noise", ["H6"]), _mk("gps_outliers", ["H6"], frac=0.01), _mk("gps_rate", ["H1", "H6"], hz=1.0)]
    + [_mk("gps_latency", ["H6", "latency"], latency_ms=ms) for ms in (100, 200)]
    + [_mk("gps_latency", ["H6", "latency"], latency_ms=200, ekf_latency_ms=0)]  # uncompensated reference
    + [_mk("gyro_bias", ["gyro_bias"], dps=d) for d in (0.2, 1.0)]
    + [_mk("gyro_bias", ["gyro_bias"], dps=1.0, from_cal=False), _mk("gyro_drift", ["gyro_bias"], dps=0.5, over_s=60)]
    + [_mk("accel_bias", ["H2"], axis=a, ms2=m) for a in ("x", "z") for m in (0.05, 0.2, 0.5)]
    + [_mk("accel_bias", ["H2"], axis="x", ms2=0.2, from_cal=False)]
    + [_mk("mag_bias", ["H2", "H3"], frac=f) for f in (0.05, 0.15)]
    + [_mk("mag_interference", ["H3"], amp=a) for a in (0.3, 1.0)]
    + [_mk("imu_noise", ["H5"]), _mk("imu_spikes", ["robustness"]), _mk("imu_dropouts", ["timing"])]
    + [_mk("imu_gap", ["timing", "H5"], gap_s=g) for g in (0.2, 1.0)]
    + [_mk("combined_realistic", ["H1", "H2", "H5", "H6"])]
    + [_mk("cal_ideal", ["H3", "H2"])]
)

GPS_OUTAGE = {"gps_dropout", "gps_stale"}
GPS_NOISY = {"gps_noise", "combined_realistic"}
GPS_FAULTS = GPS_OUTAGE | GPS_NOISY | {"gps_outliers", "gps_rate", "gps_latency"}
WINDOWED = {"gps_dropout", "gps_stale", "mag_interference", "imu_gap"}


def by_key():
    return {f.key: f for f in ALL_FAULTS}
