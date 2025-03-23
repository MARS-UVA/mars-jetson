#!/bin/bash
cd ~/mars-jetson

# Build the packages
colcon build --packages-select teleop_msgs teleop webapp_comms startup

# Source the setup file
source install/setup.bash

# Launch everything
ros2 launch startup launch.py
