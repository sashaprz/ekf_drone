"""Create the 'x500_bench' Gazebo model: PX4's x500 with a realistic (hobby-grade, vibrating
airframe) IMU instead of the near-ideal stock one. Runs INSIDE WSL; writes two new model
folders next to the stock ones and leaves the stock x500 untouched:

    python3 testing/benchmark/make_gz_model.py [~/PX4-Autopilot]

Launch it with PX4_SIM_MODEL=gz_x500_bench (testing/benchmark/fly_px4.sh does).

Why: stock x500 gyro noise is 0.05 deg/s with no bias at all, so any filter that just
integrates the gyro looks perfect (the first benchmark's 'tuned' Madgwick/Mahony did exactly
that). Gazebo's Gaussian noise model also supports a random turn-on bias (drawn once per
run: bias_mean +- bias_stddev) and a slowly wandering bias (first-order Gauss-Markov:
dynamic_bias_stddev, dynamic_bias_correlation_time). PX4's EKF2 sees exactly this IMU live,
and the replayed filters see it in the log - the same data for everyone.
"""
import os
import re
import sys

# per axis.  gyro rad/s, accel m/s^2
GYRO = dict(stddev=0.005,             # 0.29 deg/s white noise incl. vibration (suite imu_noise default)
            bias_mean=0.0, bias_stddev=0.0087,  # turn-on bias ~0.5 deg/s (after factory cal)
            dynamic_bias_stddev=0.0015, dynamic_bias_correlation_time=200.0)  # ~0.1 deg/s wander
ACCEL = dict(stddev=0.1,              # 2x the suite's 0.05: vibration on a real frame is worse
             bias_mean=0.0, bias_stddev=0.05,
             dynamic_bias_stddev=0.01, dynamic_bias_correlation_time=300.0)


def noise_xml(p):
    return ('<noise type="gaussian">' + "".join(f"<{k}>{v}</{k}>" for k, v in p.items()) + "</noise>")


def replace_noise(block, p):
    return re.sub(r"<noise type=\"gaussian\">.*?</noise>", noise_xml(p), block, flags=re.S)


def main():
    px4 = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else "~/PX4-Autopilot")
    models = os.path.join(px4, "Tools", "simulation", "gz", "models")
    base = open(os.path.join(models, "x500_base", "model.sdf")).read()
    base = re.sub(r"<angular_velocity>.*?</angular_velocity>",
                  lambda m: replace_noise(m.group(0), GYRO), base, count=1, flags=re.S)
    base = re.sub(r"<linear_acceleration>.*?</linear_acceleration>",
                  lambda m: replace_noise(m.group(0), ACCEL), base, count=1, flags=re.S)
    base = base.replace("name='x500_base'", "name='x500_bench_base'").replace('name="x500_base"', 'name="x500_bench_base"')
    top = open(os.path.join(models, "x500", "model.sdf")).read()
    top = top.replace("model://x500_base", "model://x500_bench_base").replace("name='x500'", "name='x500_bench'")
    assert base.count("<dynamic_bias_stddev>") == 6, "expected 3 gyro + 3 accel noise blocks"
    for name, sdf in (("x500_bench_base", base), ("x500_bench", top)):
        d = os.path.join(models, name)
        os.makedirs(d, exist_ok=True)
        open(os.path.join(d, "model.sdf"), "w").write(sdf)
        cfg = open(os.path.join(models, "x500" if name == "x500_bench" else "x500_base", "model.config")).read()
        open(os.path.join(d, "model.config"), "w").write(re.sub(r"<name>.*?</name>", f"<name>{name}</name>", cfg, count=1))
        print("wrote", d)


if __name__ == "__main__":
    main()
