#!/bin/bash
# One Gazebo flight with a fresh restart, for the EKF test suite (EKF_TEST_PLAN.md).
# Runs INSIDE WSL. From PowerShell:
#   wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Sasha/repos/python_drone/testing/suite/fly.sh \
#       <mission> <csv, repo-relative> <log, repo-relative> [ENV=VAL ...]
# e.g. ... fly.sh hover testing/data/flight_hover.csv testing/data/logs/rec_hover.log ORACLE=1
#
# Follows HANDOFF.md's standard procedure: kill everything (fresh spawn), launch PX4+gz
# headless, wait for the 4 real motor subscribers, kill ONLY PX4 so the motor topic is
# ours, fly run_sim.py with MISSION=<mission>, then tear everything down (the gz server
# tends to die when the vehicle drops after run_sim exits anyway).
# Last line printed: FLY_RESULT mission=.. rc=.. gz_alive_at_end=..
set -u
REPO=/mnt/c/Users/Sasha/repos/python_drone
MISSION=$1; CSV=$2; LOG=$3; shift 3

kill_all() {
    pkill -9 -f run_sim.py 2>/dev/null
    pkill -9 -f bin/px4 2>/dev/null
    pkill -9 -f "make px4_sitl" 2>/dev/null
    pkill -9 -f "gz sim" 2>/dev/null
    pkill -9 -f "sleep infinity" 2>/dev/null
    sleep 2
}

subs() { gz topic -i -t /x500_0/command/motor_speed 2>/dev/null | grep -c "gz.msgs.Actuators"; }

kill_all
if pgrep -f "gz sim" >/dev/null; then echo "FLY_RESULT mission=$MISSION rc=97 (stale gz sim would not die)"; exit 97; fi

cd ~/PX4-Autopilot
# stdin = sleep infinity: PX4's pxh shell spins on EOF otherwise (floods the log, pegs a core)
nohup bash -c "sleep infinity | HEADLESS=1 make px4_sitl gz_x500" > /tmp/px4_launch.log 2>&1 &
ok=0
for i in $(seq 1 90); do
    sleep 2
    if [ "$(subs)" -ge 4 ]; then ok=1; break; fi
done
if [ $ok -ne 1 ]; then kill_all; echo "FLY_RESULT mission=$MISSION rc=98 (motor subscribers never appeared)"; exit 98; fi
sleep 3   # let the world finish settling before PX4 goes away
pkill -9 -f bin/px4; pkill -9 -f "make px4_sitl"; sleep 2
if ! pgrep -f "gz sim" >/dev/null; then echo "FLY_RESULT mission=$MISSION rc=96 (gz sim died after px4 kill)"; exit 96; fi
if gz topic -i -t /x500_0/command/motor_speed | grep -q "^Publishers"; then
    echo "WARN: a publisher is already on the motor topic"
fi

cd "$REPO"
mkdir -p "$(dirname "$CSV")" "$(dirname "$LOG")"
TOTAL=$(python3 -c "import sys; sys.path.insert(0,'testing/suite'); import missions; print(int(missions.MISSIONS['$MISSION'].total))")
env "$@" MISSION="$MISSION" SENSOR_LOG="$CSV" timeout $((TOTAL + 60)) python3 -u run_sim.py > "$LOG" 2>&1
rc=$?
alive=0; pgrep -f "gz sim" >/dev/null && alive=1
kill_all
echo "FLY_RESULT mission=$MISSION rc=$rc gz_alive_at_end=$alive rows=$(($(wc -l < "$CSV") - 1))"
