#!/bin/bash
# Builds the ROS workspace on container creation (called from postCreateCommand).
# Clears build/install/log first if they were generated at a different mount path,
# since colcon/CMake refuse to reuse a cache whose recorded paths don't match.
set -eo pipefail

WS="$(cd "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"

stale=""
for cache in build/*/CMakeCache.txt; do
    [ -f "$cache" ] || continue
    expected="$WS/$(dirname -- "$cache")"
    recorded="$(sed -n 's/^CMAKE_CACHEFILE_DIR:INTERNAL=//p' "$cache")"
    if [ -n "$recorded" ] && [ "$recorded" != "$expected" ]; then
        stale="$cache (built at $recorded, workspace is now $expected)"
        break
    fi
done

if [ -n "$stale" ]; then
    echo "build-workspace: stale CMake cache detected: $stale"
    echo "build-workspace: removing build/ install/ log/ and rebuilding from scratch"
    rm -rf build install log
fi

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
