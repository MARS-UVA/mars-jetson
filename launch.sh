#!/bin/bash

set -e

source /opt/ros/"$ROS_DISTRO"/setup.bash --
echo "source '/opt/ros/$ROS_DISTRO/setup.bash'" >> ~/.bashrc

ros2 launch launch.xml