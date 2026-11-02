#!/bin/bash
# OFFBOARD test in SITL (2026-11-02): your EKF + position/velocity loops fly PX4's x500 through
# attitude setpoints (run_sim.py OFFBOARD=1 BRIDGE=px4), exactly the path the real drone uses
# over TELEM2. Optionally breaks something mid-flight to prove PX4 takes over. Runs INSIDE WSL:
#   bash testing/suite/px4_offboard_flight.sh <out.csv> [mission] [fault]
#   mission: any testing/suite/missions.py name (default hover)
#   fault:   none (default) | hang (control loop freezes -> watchdog hands back)
#            | nan (EKF corrupted -> FALLBACK hands back) | kill (process killed -> PX4's
#            offboard-loss failsafe) - all at mission t=FAULT_AT (default 25 s)
# Then: python3 testing/suite/check_offboard.py <out.csv> <fault>
set -u
CSV=$(realpath -m "$1")   # absolute: the script cd's into PX4's tree before writing logs
MISSION=${2:-hover}
FAULT=${3:-none}
FAULT_AT=${FAULT_AT:-25}
R=/mnt/c/Users/Sasha/repos/python_drone
PX=~/PX4-Autopilot
RF=$PX/build/px4_sitl_default/rootfs
PXLOG=${CSV%.csv}.px4.log
RUNLOG=${CSV%.csv}.log
cmd() { (cd $RF && ../bin/px4-$1 "${@:2}"); }
kill_all() { pkill -9 -f bin/px4; pkill -9 -f "make px4_sitl"; pkill -9 -f "gz sim"; pkill -9 -f "sleep infinity"; pkill -9 -f run_sim.py; sleep 2; }
kill_all
mkdir -p "$(dirname "$CSV")"
cd $PX; nohup bash -c "sleep infinity | HEADLESS=1 make px4_sitl gz_x500" > $PXLOG 2>&1 &
for i in $(seq 1 90); do sleep 2; grep -q "Ready for takeoff" $PXLOG && break; done
grep -q "Ready for takeoff" $PXLOG || { echo "PX4 never ready"; kill_all; exit 1; }
# no RC / GCS in SITL; offboard loss -> land after 0.5 s (set the same on the real drone,
# where an RC switch back to Position mode comes first). COM_OBL_ACT = older PX4's name.
cmd param set COM_RC_IN_MODE 4
cmd param set NAV_RCL_ACT 0
cmd param set NAV_DLL_ACT 0
cmd param set COM_OF_LOSS_T 0.5
cmd param set COM_OBL_RC_ACT 4
cmd param set COM_OBL_ACT 1 2>/dev/null
cd $R
ENV="OFFBOARD=1 BRIDGE=px4 PX4_URL=udpin:0.0.0.0:14540 OFFBOARD_HOVER_THR=${OFFBOARD_HOVER_THR:-0.71} SENSOR_LOG=$CSV MISSION=$MISSION RUN_SECONDS=200"
case $FAULT in
    hang) ENV="$ENV SIM_HANG_AT=$FAULT_AT" ;;
    nan)  ENV="$ENV SIM_NAN_AT=$FAULT_AT" ;;
esac
env $ENV nohup python3 -u run_sim.py > $RUNLOG 2>&1 &
if [ "$FAULT" = kill ]; then
    for i in $(seq 1 60); do sleep 1; grep -q "ENGAGED" $RUNLOG && break; done
    sleep $FAULT_AT
    echo "FAULT kill: killing run_sim.py at mission t~${FAULT_AT}s"
    pkill -9 -f run_sim.py
fi
# wait for run_sim to finish (mission end / hand-back + PX4 disarmed), or for PX4 to disarm
for i in $(seq 1 120); do
    sleep 2
    pgrep -f run_sim.py > /dev/null || break
    [ "$FAULT" = hang ] && grep -q "handing control back" $RUNLOG && grep -qi "disarmed" $PXLOG && { sleep 3; break; }
done
for i in $(seq 1 30); do grep -qi "disarmed" $PXLOG && break; sleep 2; done
kill_all
echo "OFFBOARD_FLIGHT done: mission=$MISSION fault=$FAULT rows=$(($(wc -l < $CSV) - 1))"
