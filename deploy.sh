#!/bin/bash
cd "$(dirname -- "${BASH_SOURCE[0]}")"
colcon build --symlink-install
rm -rf /tmp/rgbd_visual_test.db
source setup_terminal.sh && ros2 launch startup launch.py "$@"
