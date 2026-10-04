#!/bin/bash
# Shadow-mode pipeline test in SITL (2026-10-03): PX4 flies itself (not killed, unlike
# fly.sh) with full-rate sensor logging from boot, then the .ulg is copied out for
# testing/ulog_to_csv.py. Runs INSIDE WSL:  bash testing/suite/px4_shadow_flight.sh <out.ulg>
set -u
OUT=$1
PX=~/PX4-Autopilot
RF=$PX/build/px4_sitl_default/rootfs
LOG=/tmp/px4_shadow.log
cmd() { (cd $RF && ../bin/px4-$1 "${@:2}"); }
kill_all() { pkill -9 -f bin/px4; pkill -9 -f "make px4_sitl"; pkill -9 -f "gz sim"; pkill -9 -f "sleep infinity"; sleep 2; }
launch() {   # $1 = log text that means "ready"
    # stdin = sleep infinity: PX4's pxh shell spins on EOF otherwise (2.5 GB log, 140% CPU)
    cd $PX; nohup bash -c "sleep infinity | HEADLESS=1 make px4_sitl gz_x500" > $LOG 2>&1 &
    for i in $(seq 1 90); do sleep 2; grep -q "$1" $LOG && return 0; done
    echo "PX4 never reported: $1"; tail -15 $LOG; return 1
}
kill_all
launch "home set" || exit 1
# full-rate sensor logging from boot; SITL has no GCS or RC, so don't require them to arm
cmd param set SDLOG_PROFILE 3
cmd param set SDLOG_MODE 1
cmd param set NAV_DLL_ACT 0
cmd param set COM_RC_IN_MODE 4
cmd param set NAV_RCL_ACT 0
cmd param save
kill_all                       # logger params take effect on the next boot
START=$(date +%s)
launch "Ready for takeoff" || exit 1
sleep 8                        # at rest on the ground: becomes the calibration dwell
cmd commander takeoff
sleep 25                       # climb + hover
cmd commander land
sleep 20
kill_all
ULG=$(find $RF/fs/log -name '*.ulg' -newermt "@$START" | sort | tail -1)
[ -z "$ULG" ] && { echo "no new ulog"; exit 2; }
mkdir -p "$(dirname "$OUT")"; cp "$ULG" "$OUT"
echo "SHADOW_LOG $ULG -> $OUT ($(stat -c %s "$OUT") bytes)"
grep -E "Takeoff|takeoff|Landing|landed|Disarmed|Armed" $LOG | tail -6
