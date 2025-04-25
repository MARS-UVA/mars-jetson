#!/bin/bash
cd ~/mars-jetson/src
ROS_PACKAGES=(webapp_comms teleop teleop_msgs startup serial_ros nucleo_msgs)
for package in ${ROS_PACKAGES[@]}; do
    rm -r "$package/build"
    rm -r "$package/install"
done