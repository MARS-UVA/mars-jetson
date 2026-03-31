#!/bin/bash
#Usage:
#Option 1: ./deploy.sh
#Option 2: ./deploy.sh <control station ip>
cd ~/mars-jetson

# Avoid domain collisions with bucket drum
export ROS_DOMAIN_ID=10

# if [ $# -ge 1 ];
#     then
#         export CONTROL_STATION_IP=$1
#     else 
#         export CONTROL_STATION_IP="192.168.0.109"
# fi

ip="192.168.0.109" # default value for control station ip
domain="0" # default value for ROS domain id, between 0-101

while getopts "i:d:" flag
do
    case "${flag}" in
        i) ip=${OPTARG}
                ;;
        d) domain=${OPTARG}
                ;;
        *) echo "Invalid option: -$flag" ;;
        esac
done
# echo "Control Station IP: $ip"
# echo "Control Station Domain: $domain"

export CONTROL_STATION_IP=$ip
export ROS_DOMAIN_ID=$domain

# Build the packages
# colcon build --packages-select teleop_msgs serial_msgs
colcon build --symlink-install --packages-ignore zed_components zed_wrapper zed_ros2 --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=true
# Source the setup file
source install/setup.bash

# Launch everything
# run realsense executable in parallel
#ros2 launch startup launch.py
# ~/mars-jetson/src/obstacle_detection/src/build/obstacle_detect_node &
# OBSTACLE_DETECT_PID=$!

ros2 launch startup launch.py

# kill $OBSTACLE_DETECT_PID
