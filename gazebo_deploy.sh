#!/bin/bash
# deploy.sh with robot_backend:=gazebo.
#
# gz sim's server does not die with the launch's process group, and a leftover
# server keeps the old world running for the next run's spawn to land in. So
# kill any leftover before starting, and ours on the way out.
cd "$(dirname -- "${BASH_SOURCE[0]}")"

kill_gz() {
    pkill -TERM -f 'gz sim' 2>/dev/null || return 0
    for _ in $(seq 1 50); do
        pgrep -f 'gz sim' >/dev/null || return 0
        sleep 0.1
    done
    pkill -KILL -f 'gz sim' 2>/dev/null
}

kill_gz

# Own session, so the whole launch tree can be signalled as one group. SIGTERM,
# not SIGINT: background commands start with SIGINT ignored.
deploy_pid=""
cleanup() {
    trap - EXIT
    trap '' INT TERM HUP
    if [ -n "$deploy_pid" ] && kill -TERM -- "-$deploy_pid" 2>/dev/null; then
        for _ in $(seq 1 100); do
            kill -0 -- "-$deploy_pid" 2>/dev/null || break
            sleep 0.1
        done
        kill -KILL -- "-$deploy_pid" 2>/dev/null
    fi
    kill_gz
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

setsid bash ./deploy.sh robot_backend:=gazebo "$@" <&0 &
deploy_pid=$!
wait "$deploy_pid"
