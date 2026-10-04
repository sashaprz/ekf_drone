#!/bin/bash
# Live shadow-mode test in SITL (2026-10-04): PX4 flies itself; run_sim.py runs your EKF +
# controller LIVE on PX4's MAVLink onboard link (BRIDGE=px4) exactly as it would on the Pi,
# sending nothing to the motors. Runs INSIDE WSL:
#   bash testing/suite/px4_live_shadow.sh testing/data/shadow/live_shadow1.csv
# (needs the SITL params set once by px4_shadow_flight.sh: no GCS/RC required to arm)
set -u
CSV=$1
R=/mnt/c/Users/Sasha/repos/python_drone
PX=~/PX4-Autopilot
RF=$PX/build/px4_sitl_default/rootfs
LOG=/tmp/px4_live_shadow.log
cmd() { (cd $RF && ../bin/px4-$1 "${@:2}"); }
kill_all() { pkill -9 -f bin/px4; pkill -9 -f "make px4_sitl"; pkill -9 -f "gz sim"; pkill -9 -f "sleep infinity"; pkill -9 -f run_sim.py; sleep 2; }
kill_all
cd $PX; nohup bash -c "sleep infinity | HEADLESS=1 make px4_sitl gz_x500" > $LOG 2>&1 &
for i in $(seq 1 90); do sleep 2; grep -q "Ready for takeoff" $LOG && break; done
grep -q "Ready for takeoff" $LOG || { echo "PX4 never ready"; kill_all; exit 1; }
cd $R
mkdir -p "$(dirname "$CSV")"
BRIDGE=px4 PX4_URL=udpin:0.0.0.0:14540 SENSOR_LOG=$CSV RUN_SECONDS=75 \
    nohup python3 -u run_sim.py > ${CSV%.csv}.log 2>&1 &
sleep 15                       # bridge connects, measures the field, EKF dwell-calibrates at rest
cmd commander takeoff
sleep 25
cmd commander land
sleep 30
for i in $(seq 1 20); do pgrep -f run_sim.py > /dev/null || break; sleep 2; done
kill_all
echo "LIVE_SHADOW done: $(($(wc -l < $CSV) - 1)) rows"
grep -E "PX4 bridge|SHADOW|RUN_END|FAILSAFE|Traceback|Error" ${CSV%.csv}.log | head -12
