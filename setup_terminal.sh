#!/bin/bash
source install/setup.bash

ros2 run rmw_zenoh_cpp rmw_zenohd

ros2 daemon stop

ros2 daemon start
