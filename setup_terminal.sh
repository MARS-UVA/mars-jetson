#!/bin/bash
source install/setup.bash

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run rmw_zenoh_cpp rmw_zenohd

ros2 daemon stop

ros2 daemon start
