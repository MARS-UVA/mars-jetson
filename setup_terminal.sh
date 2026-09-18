#!/bin/bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
source /opt/ros/jazzy/setup.bash
WORKSPACE_SETUP="$(dirname -- "${BASH_SOURCE[0]}")/install/setup.bash"
if [ -f "${WORKSPACE_SETUP}" ]; then
  source "${WORKSPACE_SETUP}"
fi
