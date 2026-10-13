#!/bin/bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
source /opt/ros/jazzy/setup.bash
WORKSPACE_SETUP="$(dirname -- "${BASH_SOURCE[0]}")/install/setup.bash"
if [ -f "${WORKSPACE_SETUP}" ]; then
  source "${WORKSPACE_SETUP}"
fi

# The Zenoh router is now started by the launch file (see the rmw_zenohd Node in
# src/startup/launch/launch.py), so sourcing this file no longer starts one and
# deploy.sh needs no extra step.
#
# Fallback for starting the graph by hand: if you are running individual nodes
# with `ros2 run` instead of `ros2 launch startup launch.py`, there is no router,
# and nothing will discover anything. In that case uncomment the block below (or
# just run it in its own terminal) before starting your nodes.
#
# ros2 run rmw_zenoh_cpp rmw_zenohd &
# ros2 daemon stop
# ros2 daemon start
