#!/bin/bash
source install/setup.bash

export FASTRTPS_DEFAULT_PROFILES_FILE=/home/mars/mars-jetson/config.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=SHM

ros2 daemon stop

ros2 daemon start
