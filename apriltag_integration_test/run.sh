#!/bin/sh
set -eu
TEST_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
export PYTHONDONTWRITEBYTECODE=1
# Source only installed ROS libraries, never the robot workspace overlay.
if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
    ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.sh"
    if [ ! -f "$ROS_SETUP" ]; then
        echo "Source your installed ROS 2 environment first." >&2
        exit 2
    fi
    set +u
    . "$ROS_SETUP"
    set -u
fi
exec /usr/bin/python3 "$TEST_ROOT/runner.py" "$@"
