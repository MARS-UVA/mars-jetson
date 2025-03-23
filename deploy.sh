#!/bin/bash
cd ~/mars-jetson

# Build the packages
colcon build --packages-select teleop_msgs teleop webapp_comms serial_ros startup

# Source the setup file
source install/setup.bash

# Launch everything
ros2 launch startup launch.py
