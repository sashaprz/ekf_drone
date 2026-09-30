#!/bin/bash
# usage: bash run_rate_test.sh [axis] [step_value] [duration]
# e.g.:  bash run_rate_test.sh roll 1.0 5.0
pkill -f run_sim.py 2>/dev/null
pkill -f test_rate_loop.py 2>/dev/null
sleep 1
cd "$(dirname "$0")"
python3 test_rate_loop.py "$1" "$2" "$3" 2>&1 | tee rate_test.log
