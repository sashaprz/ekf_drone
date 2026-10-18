#!/bin/bash
# Full benchmark campaign (resumable - existing flights/recordings are skipped). Runs INSIDE WSL:
#   bash testing/benchmark/campaign.sh [trials=3]
# 1. fly every scenario x model x trial on PX4 SITL (fresh boot each; trials interleaved so
#    any slow drift of the machine doesn't line up with one scenario)
# 2. prepare: stock x500 flights -> fault 'none' only (control: the near-ideal stock IMU);
#    x500_bench flights (realistic IMU) -> every fault
# Then, on Windows: python testing/benchmark/score.py
set -u
TRIALS=${1:-3}
R=/mnt/c/Users/Sasha/repos/python_drone
F=~/bench/flights
SCENARIOS="hover box circle_fast figure8 stops yaw_spin"
mkdir -p $F
cd $R
for t in $(seq 1 $TRIALS); do
  for model in x500_bench x500; do
    for s in $SCENARIOS; do
      out=$F/${s}__${model}__t$t.ulg
      [ -f $out ] && continue
      for attempt in 1 2; do
        res=$(bash testing/benchmark/fly_px4.sh $s $model $out 2>&1 | tail -1)
        echo "$(date +%H:%M:%S) $res"
        grep -q "BENCH_RESULT ok" /tmp/px4_mission.log && break
        rm -f $out   # failed flight (arming / crash): retry once, never keep a partial one
      done
    done
  done
done
echo "$(date +%H:%M:%S) FLIGHTS DONE: $(ls $F/*.ulg | wc -l)"
python3 testing/benchmark/prepare.py $F/*__x500__t*.ulg --faults none
python3 testing/benchmark/prepare.py $F/*__x500_bench__t*.ulg
echo "$(date +%H:%M:%S) CAMPAIGN DONE"
