#!/bin/bash
R=/mnt/c/Users/Sasha/repos/python_drone
ULG=$(ls -t ~/PX4-Autopilot/build/px4_sitl_default/rootfs/fs/log/*/*.ulg | head -1)
mkdir -p $R/testing/data/shadow && cp "$ULG" $R/testing/data/shadow/sitl_flight1.ulg
cd $R && python3 testing/ulog_to_csv.py testing/data/shadow/sitl_flight1.ulg testing/data/shadow/sitl_flight1.csv
