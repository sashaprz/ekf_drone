#!/bin/bash
# WSL-side twin of record.ps1: bash testing/suite/record.sh [mission ...]
cd "$(dirname "$0")/../.."
missions=${*:-hover box circle_slow circle_fast stops yaw_steps yaw_spin takeoff_land patrol_long}
for m in $missions; do
    envs="ORACLE=1"
    case $m in circle_fast|stops) envs="$envs MAX_VEL_XY=3.5 MAX_TILT_DEG=25";; esac
    echo "=== recording $m ($envs)"
    bash testing/suite/fly.sh "$m" "testing/data/flight_$m.csv" "testing/data/logs/rec_$m.log" $envs
done
