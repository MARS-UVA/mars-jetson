# Zenoh router launch (Jetson)

**Architecture:** rosbridge runs on **mars-control-station**, not on this repo's
target (the Jetson). This repo's only responsibility for the link is that its
**Zenoh router comes up automatically with the robot**. There is no rosbridge
package and no rosbridge launch here.

## 1. What was removed

Earlier work on this branch (`yogi/rosbridge-work`, uncommitted) had added
rosbridge to this repo on the assumption it ran here. All of it is now gone:

| File | Removed |
| --- | --- |
| `.devcontainer/Dockerfile` | `ros-jazzy-rosbridge-server` from the `apt-get install` line (line 23). The line now installs `ros-jazzy-rmw-zenoh-cpp` only. |
| `src/startup/launch/launch.py` | The `rosbridge_websocket_launch.xml` `IncludeLaunchDescription` block (and its `websocket_ping_interval` argument), plus the three imports that existed only for it: `AnyLaunchDescriptionSource`, `get_package_share_directory`, and `import os`. |
| `src/startup/package.xml` | The `<exec_depend>rosbridge_server</exec_depend>` entry and its comment. |

Two notes on scope:

- **`package.xml` was not in the task's grep filter** (`*.xml` wasn't included),
  but it carried a `rosbridge_server` exec_depend from the same change. Left in
  place it would have made `rosdep install` pull rosbridge back into every
  container on `postCreateCommand`, so it was removed too.
- **`docs/networking-audit-jetson.md` still mentions rosbridge** and was left
  alone. It is a prior analysis write-up (what a migration *would* involve), not
  a functional addition. Its conclusions about hosting rosbridge on the Jetson
  are superseded by the confirmed architecture; worth a revision pass, but that
  is a separate edit from this one.

## 2. Final state — the router node in `launch.py`

The router node already existed from earlier work; it gained `respawn=True` and
kept its inherited comment. Final form:

```python
nodes.append(Node(
    package="rmw_zenoh_cpp",
    executable="rmw_zenohd",
    name="rmw_zenohd",
    output="screen",
    respawn=True,
))
```

It is the **first** entry appended to `nodes`, which is spliced into the returned
`LaunchDescription([...])` after `SetEnvironmentVariable` and the launch args:

```python
return LaunchDescription([
    SetEnvironmentVariable(name="CONTROL_STATION_IP", value=control_station_ip),
    *args,
    *nodes,
])
```

Nothing else in that list was removed or reordered — `robot_state_publisher`,
the `robot.launch.py` include, `serial_node`, and the conditional
`gazebo.launch.py` include are untouched and still follow the router.

Two details worth keeping in mind:

- **`respawn=True` is a restart, not a reconnect.** If the router dies, launch
  restarts the process, but existing participants do not necessarily re-establish
  their sessions cleanly. It protects against a crash at startup more than it
  guarantees a seamless recovery mid-run.
- **The node deliberately passes no `parameters=[]`.** `rmw_zenohd` is not an
  rclcpp node, so `--ros-args -p ...` argv is silently ignored; it configures
  itself from `ZENOH_ROUTER_CONFIG_URI` / `ZENOH_CONFIG_OVERRIDE`. The packaged
  default already listens on `tcp/[::]:7447`. (An earlier version of this node
  passed `zenoh_router_port` / `zenoh_router_log_level` as parameters, which had
  no effect.)

## 3. Manual router start — retired

`setup_terminal.sh` **already had this done** by the earlier work; no change was
needed in this pass. The manual start is commented out, with a note that the
launch file now owns it and that the block is a fallback for running individual
nodes with `ros2 run`:

```bash
# ros2 run rmw_zenoh_cpp rmw_zenohd &
# ros2 daemon stop
# ros2 daemon start
```

## 4. `RMW_IMPLEMENTATION` in the devcontainers

The directories are `linux`, `macos`, and **`wslg`** (not `wsl`). State before
this pass and what changed:

| File | Before | Action |
| --- | --- | --- |
| `.devcontainer/wslg/devcontainer.json` | already had it (uncommitted, from earlier work) | left as-is |
| `.devcontainer/linux/devcontainer.json` | missing | added |
| `.devcontainer/macos/devcontainer.json` | missing | added |

All three now end their `containerEnv` with:

```json
"ROS_DOMAIN_ID": "42",
"RMW_IMPLEMENTATION": "rmw_zenoh_cpp"
```

All three parse as valid JSON. Note this is belt-and-braces: the shared
`.devcontainer/Dockerfile` already sets `ENV RMW_IMPLEMENTATION="rmw_zenoh_cpp"`
at build time. The `containerEnv` entries make it explicit per-container and
survive a base-image change that drops the `ENV`.

## 5. Not done here — verification

Per the task, nothing was rebuilt or launched: no devcontainer rebuild, no
`colcon build`, no `ros2 launch`. Syntax-level checks only, all passing:

- `python3 -m py_compile src/startup/launch/launch.py`
- `package.xml` parses as XML
- `bash -n setup_terminal.sh`
- all three `devcontainer.json` files parse as JSON

**Still to confirm inside a running container:** that `ros2 launch startup
launch.py` brings up `rmw_zenohd` and that the rest of the graph discovers
through it, and — since the Dockerfile changed — that a devcontainer rebuild
still succeeds without the rosbridge package.
