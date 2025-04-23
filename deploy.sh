#!/bin/bash
#Usage:
#Option 1: ./deploy.sh
#Option 2: ./deploy.sh <control station ip>
cd ~/mars-jetson


if [ $# -ge 1 ]
    then
        export CONTROL_STATION_IP=$1
    else 
        export CONTROL_STATION_IP="192.168.0.200"
fi

# Build the packages
colcon build --symlink-install --packages-select webapp_comms teleop teleop_msgs startup
# Source the setup file
source install/setup.bash

# Launch everything
# run realsense executable in parallel
~/mars-jetson/src/obstacle_detection/src/build/obstacle_detect_node &
OBSTACLE_DETECT_PID=$!
ros2 launch startup launch.py

kill $OBSTACLE_DETECT_PID
