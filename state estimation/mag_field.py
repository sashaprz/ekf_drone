"""Earth magnetic field detection at startup - what FINAL_gps.py needs as `mag_reference`
when there's no simulator or PX4 EKF2 attitude to measure it against (real hardware).

The field direction in the world (ENU) frame has two angles:

  inclination (dip)  how steeply the field points into the ground (~63 deg here). OBSERVABLE
                     from your own sensors at rest: the accelerometer gives "up" in the body
                     frame, the magnetometer gives the field in the body frame, and the angle
                     between them doesn't depend on the attitude at all.
  declination        how far magnetic north is from true north. NOT observable from the IMU
                     and magnetometer: turning the drone by 5 deg and turning the field by
                     5 deg give the same magnetometer reading. It has to come from outside:
                       1. the World Magnetic Model at the GPS position (pip install pygeomag),
                       2. a value you supply (EKF_DECL_DEG, e.g. read off a WMM web calculator),
                       3. otherwise 0 deg, flagged as unknown with a large uncertainty - then
                          FINAL_gps.py's declination state (EKF_LEARN_DECL=1) can learn it in
                          flight from GPS velocity.

Hard-iron mag bias during the dwell tilts the measured dip (a bias along "up" changes the
angle), so the dwell should be done with the motors off, away from steel, like any compass
calibration. The EKF's mag-bias state is learned later, during the calibration turn.

    ref, info = field_reference(accel_mean, mag_mean, lat_deg, lon_deg)
"""
import datetime
import math
import os

import numpy as np

# 1-sigma uncertainty of the declination source (deg) - read by FINAL_gps.py to size the
# declination state's prior when EKF_LEARN_DECL=1
DECL_STD_WMM = 1.0       # WMM vs local reality, typical away from anomalies
DECL_STD_USER = 1.0
DECL_STD_UNKNOWN = 20.0  # no source at all


def inclination_deg(accel_mean, mag_mean):
    """Dip angle from one at-rest accel + mag average (positive = field points down)."""
    up = np.asarray(accel_mean, float)
    up = up / np.linalg.norm(up)  # at rest the accelerometer reads +1 g along "up"
    m = np.asarray(mag_mean, float)
    m = m / np.linalg.norm(m)
    return math.degrees(math.asin(np.clip(-(m @ up), -1.0, 1.0)))


def declination_deg(lat_deg=None, lon_deg=None, alt_m=0.0, when=None):
    """-> (declination deg east of true north, 1-sigma deg, source)"""
    env = os.environ.get("EKF_DECL_DEG")
    if env is not None:
        return float(env), DECL_STD_USER, "EKF_DECL_DEG"
    if lat_deg is not None and lon_deg is not None:
        try:
            from pygeomag import GeoMag
            when = when or datetime.date.today()
            year = when.year + (when.timetuple().tm_yday - 1) / 365.25
            r = GeoMag().calculate(glat=float(lat_deg), glon=float(lon_deg), alt=float(alt_m) / 1000.0, time=year)
            return float(r.d), DECL_STD_WMM, "WMM (pygeomag)"
        except Exception:  # not installed, or its API/coefficients differ - fall through
            pass
    return 0.0, DECL_STD_UNKNOWN, "unknown (assumed 0)"


def field_reference(accel_mean, mag_mean, lat_deg=None, lon_deg=None, alt_m=0.0):
    """Unit field direction in ENU from the dwell averages + declination source.

    ENU: x east, y north, z up. A field with dip I (down positive) and declination D (east
    positive) points  horizontal * (sin D, cos D)  and  -sin I  vertically."""
    incl = inclination_deg(accel_mean, mag_mean)
    decl, decl_std, src = declination_deg(lat_deg, lon_deg, alt_m)
    I, D = math.radians(incl), math.radians(decl)
    ref = np.array([math.cos(I) * math.sin(D), math.cos(I) * math.cos(D), -math.sin(I)])
    info = {"inclination_deg": incl, "declination_deg": decl, "declination_std_deg": decl_std,
            "declination_source": src}
    return ref, info
