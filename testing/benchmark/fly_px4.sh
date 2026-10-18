#!/bin/bash
# One PX4-flown benchmark flight (PX4's EKF2 + controller fly; the .ulg is the raw data).
# Runs INSIDE WSL:
#   bash testing/benchmark/fly_px4.sh <scenario> <model: x500|x500_bench> <out.ulg>
# Fresh PX4 + Gazebo per flight (independent noise draws / turn-on biases per trial).
# Needs the logger params saved once by testing/suite/px4_shadow_flight.sh
# (SDLOG_PROFILE=3, SDLOG_MODE=1, no GCS/RC required to arm) and the x500_bench model
# from testing/benchmark/make_gz_model.py.
# Last line printed: FLY_PX4 scenario=.. model=.. rc=.. ulg=..
set -u
SCEN=$1; MODEL=$2; OUT=$3
R=/mnt/c/Users/Sasha/repos/python_drone
PX=~/PX4-Autopilot
RF=$PX/build/px4_sitl_default/rootfs
LOG=/tmp/fly_px4.log
kill_all() { pkill -9 -f bin/px4; pkill -9 -f "gz sim"; pkill -9 -f "sleep infinity"; pkill -9 -f px4_mission.py; sleep 2; }

kill_all
START=$(date +%s)
cd $RF
nohup bash -c "sleep infinity | env PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_$MODEL GZ_IP=127.0.0.1 HEADLESS=1 ../bin/px4" > $LOG 2>&1 &
ok=0
for i in $(seq 1 90); do sleep 2; grep -q "Ready for takeoff" $LOG && { ok=1; break; }; done
if [ $ok -ne 1 ]; then tail -15 $LOG; kill_all; echo "FLY_PX4 scenario=$SCEN model=$MODEL rc=98 (PX4 never ready)"; exit 98; fi

cd $R
timeout 240 python3 -u testing/benchmark/px4_mission.py $SCEN > /tmp/px4_mission.log 2>&1
rc=$?
sleep 2
kill_all
ULG=$(find $RF/log -name '*.ulg' -newermt "@$START" 2>/dev/null | sort | tail -1)
[ -z "$ULG" ] && ULG=$(find $RF/fs/log -name '*.ulg' -newermt "@$START" 2>/dev/null | sort | tail -1)
if [ -z "$ULG" ]; then echo "FLY_PX4 scenario=$SCEN model=$MODEL rc=97 (no new ulog)"; exit 97; fi
mkdir -p "$(dirname "$OUT")"; cp "$ULG" "$OUT"
grep -E "BENCH_RESULT|Traceback|Error" /tmp/px4_mission.log | tail -3
grep -iE "failsafe|preflight fail|emergency" $LOG | tail -3
echo "FLY_PX4 scenario=$SCEN model=$MODEL rc=$rc ulg=$OUT"
