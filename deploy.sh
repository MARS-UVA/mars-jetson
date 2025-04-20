#!/bin/bash
cd ~/mars-jetson

# Build the packages
colcon build --symlink-install --packages-select webapp_comms teleop teleop_msgs startup
# Source the setup file
source install/setup.bash

# Launch everything
# run realsense executable in parallel
# ~/mars-jetson/src/obstacle_detection/src/build/obstacle_detect_node &
# OBSTACLE_DETECT_PID=$!
# ros2 launch startup launch.py

# kill $OBSTACLE_DETECT_PID
