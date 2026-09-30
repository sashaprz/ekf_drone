#!/bin/bash
# usage: bash run_attitude_test.sh [axis] [step_value_rad] [duration]
# e.g.:  bash run_attitude_test.sh roll 0.2 3.0
pkill -f run_sim.py 2>/dev/null
pkill -f test_rate_loop.py 2>/dev/null
pkill -f test_attitude_loop.py 2>/dev/null
sleep 1
cd "$(dirname "$0")"
python3 test_attitude_loop.py "$1" "$2" "$3" 2>&1 | tee attitude_test.log
