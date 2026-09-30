# Recording validation

## Recordings

| recording | dur s | IMU Hz | pose Hz | GPS Hz | ended cleanly | max true tilt | min z airborne | tilt mean/max (profile) | speed mean/max | g*tan(tilt) mean/max | yaw rate max | path err mean/max | verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| hover | 75 | 250 | 50 | 28 | True | 0.0 | 1.99 | 0.0/0.0 | 0.00/0.00 | 0.00/0.00 | 0 | 0.00/0.00 | OK |
| box | 75 | 248 | 50 | 30 | True | 2.1 | 1.99 | 1.0/2.1 | 0.52/1.29 | 0.17/0.36 | 0 | 0.86/1.39 | OK |
| circle_slow | 75 | 249 | 50 | 30 | True | 2.8 | 1.99 | 2.3/2.8 | 1.08/1.30 | 0.39/0.49 | 0 | 0.46/0.69 | OK |
| circle_fast | 75 | 249 | 50 | 30 | True | 16.5 | 1.93 | 12.9/16.5 | 2.31/3.01 | 2.25/2.90 | 5 | 1.14/3.14 | OK |
| stops | 60 | 250 | 50 | 30 | True | 13.8 | 1.95 | 9.1/13.8 | 1.26/2.41 | 1.57/2.40 | 1 | 2.35/4.48 | OK |
| yaw_steps | 75 | 249 | 50 | 30 | True | 179.0 | -1.20 | 54.3/179.0 | 10.93/82.98 | 19.59/215097.09 | 28831 | 107.76/358.82 | BAD |
| yaw_spin | 75 | 250 | 50 | 30 | True | 180.0 | 0.31 | 156.3/180.0 | 2.20/19.30 | 0.53/4974.85 | 4802 | 22.71/34.95 | BAD |
| patrol_long | 615 | 248 | 50 | 30 | True | 3.7 | 1.99 | 1.0/3.7 | 0.52/2.39 | 0.17/0.63 | 1 | 0.88/1.43 | OK |
| takeoff_land | 55 | 250 | 45 | 25 | True | 0.0 | -0.00 | 0.0/0.0 | 0.00/0.00 | 0.00/0.00 | 0 | 0.00/0.00 | OK |

Units: deg, m/s, m/s^2, deg/s, m. Speed/accel/tilt are Gazebo truth during the mission profile (not the settle/end-hold); path err = truth vs the mission's ideal path (no lead).

Truth pose latency vs IMU (from aligning pose-derived body rate with the gyro):

- circle_fast: best-fit pose latency -0 ms (residual 0.035 rad/s)
- stops: best-fit pose latency -0 ms (residual 0.035 rad/s)
- yaw_steps: best-fit pose latency -2 ms (residual 47.753 rad/s)
- yaw_spin: best-fit pose latency -2 ms (residual 0.972 rad/s)
