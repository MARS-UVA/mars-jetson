#!/bin/sh
# Stop Zenoh routers owned by the current user, including in other terminals.

user_id=$(id -u)
pattern='^(rmw_zenohd|zenohd)$'

if ! pgrep -u "$user_id" -x "$pattern" >/dev/null; then
    echo "No Zenoh routers running for the current user."
    exit 0
fi

echo "Stopping Zenoh routers:"
pgrep -u "$user_id" -a -x "$pattern"
pkill -TERM -u "$user_id" -x "$pattern"

attempt=0
while [ "$attempt" -lt 5 ]; do
    if ! pgrep -u "$user_id" -x "$pattern" >/dev/null; then
        echo "Zenoh routers stopped."
        exit 0
    fi
    sleep 1
    attempt=$((attempt + 1))
done

echo "Some Zenoh routers are still running (or were restarted):" >&2
pgrep -u "$user_id" -a -x "$pattern" >&2
echo "Stop their launch command or service, or use kill -9 <PID> if necessary." >&2
exit 1
