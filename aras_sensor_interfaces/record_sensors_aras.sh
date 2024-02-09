#!/bin/bash
rosbag record -a -x "(.*)/compressedDepth(.*)" -b 1024 --chunksize=1024 -o ./Recordings/ARAS_bike
