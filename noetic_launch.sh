#!/bin/bash
sudo chmod -R 666 /dev/tty*
xhost +local:docker 
docker run --rm -it -v $(pwd)/aras_sensor_interfaces:/workspace/myspace -v /dev/:/dev/ --privileged -e DISPLAY \
-p 32925:32925 -p 11311:11311 -p 8888:8888 -p 8765:8765 \
-v /tmp/.X11-unix:/tmp/.X11-unix -w /workspace/myspace \
--name ros_noetic codesteller/ros-noetic-base:v24.02 bash
