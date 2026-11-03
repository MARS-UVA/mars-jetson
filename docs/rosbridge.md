# Control-station link (rosbridge on the other side)

What this repo does so the control station can reach the robot over ROS 2, and
what to know before changing it. The rosbridge server itself lives in
mars-control-station; nothing here runs rosbridge.

## Topology

```
control station: browser ──> rosbridge :9090 ──Zenoh client──> Jetson container, rmw_zenohd :7447 ──> nodes
this host:       gstreamer streamers ──ws :6767 / :6969──> control-station signaling relay
                                      ──WebRTC──────────> browser
```

- `rmw_zenohd` is started by `src/startup/launch/launch.py`, with `respawn`, so
  the router comes up with the robot and `deploy.sh` needs no extra step. It
  takes no launch parameters: it is not an rclcpp node, so `--ros-args -p` is
  silently ignored, and the packaged default already listens on `tcp/[::]:7447`.
  `setup_terminal.sh` keeps the manual command commented out for `ros2 run`
  sessions that have no launch file.
- `RMW_IMPLEMENTATION=rmw_zenoh_cpp` is set in `setup_terminal.sh` and in every
  devcontainer profile's `containerEnv`.
- The three devcontainer profiles join the `mars-dev` Docker bridge network
  with the alias `mars-jetson` and publish TCP 7447 for the Zenoh router.
  The control station remains a Zenoh client; it does not need its own router.
- On the same Docker host, configure the control-station container with
  `JETSON_IP=mars-jetson` and `ROS_DOMAIN_ID=42`. Its `ros/ros-env.sh` should set
  `ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/mars-jetson:7447"]'`.
  Both containers must join `mars-dev`. The old `host.docker.internal` endpoint
  can also reach the published router port, but the container alias is direct.
- `CONTROL_STATION_IP=mars-control-station` makes the streamers dial the
  signaling relay directly over the shared bridge network on TCP 6767/6969.
  `127.0.0.1` inside this container refers to this container itself.
- Across Wi-Fi, Docker bridge networks are separate on each host. Set
  `JETSON_IP=<Jetson LAN IP>` on the control station, pass
  `control_station_ip:=<laptop LAN IP>` to `deploy.sh`, and publish TCP 6767/6969
  on the control-station host. The Jetson's published TCP 7447 must be reachable
  from the laptop. Match `ROS_DOMAIN_ID` on both sides.
- Camera media travels through WebRTC ICE, separately from signaling. The
  streamers currently configure STUN, but no TURN relay. Private container
  candidates may be unreachable from the browser, especially on Docker Desktop
  or across machines. Verify the selected ICE connection; a reachable TURN
  relay configured on the streamer and browser provides a fallback. Publishing
  the signaling ports alone does not expose the media connection.

## Running against the control station

Simulation, in the devcontainer:

```bash
./gazebo_deploy.sh
ros2 control list_controllers    # second terminal: all three active
```

`gazebo_deploy.sh` is `deploy.sh robot_backend:=gazebo`, plus cleanup: the
`gz sim` server outlives the launch's process group, and a leftover server keeps
the old world running for the next spawn to land in, so the script kills any
leftover before starting and its own on the way out.

Real robot: `./deploy.sh control_station_ip:=<laptop ip>`, unchanged.

Domain ID: the devcontainers force `ROS_DOMAIN_ID=42`; a bare-metal Jetson runs
on 0. The control station must match whichever one it is talking to. Zenoh
ignores `ROS_AUTOMATIC_DISCOVERY_RANGE`, so the `LOCALHOST` setting in the
profiles does nothing.

## Controller spawners

`robot.launch.py` no longer starts controller spawners for any backend. `serial`
and `mock` run no `ros2_control_node`, so there is no controller manager to
reach. For `gazebo` the controller manager lives inside `gz sim`
(`gz_ros2_control`) and only exists once the robot is spawned, so
`gazebo.launch.py` starts `joint_state_broadcaster` off the spawn's exit and
`base_controller` and `arm_drum_controller` off that one's exit. The spawner
timeouts are raised because at the default 10 s service timeout the spawner
re-sends `switch_controller` while the first switch is still in progress, the
manager rejects the retry, and the spawner exits with an error even though the
controller activated.

## Interface packages

`serial_msgs`, `teleop_msgs`, `robot_control_msgs`, and `autonomy_msgs` are
tracked in-tree under `src/`. The control station builds the same four from the
`mars-ros-interfaces` repo (submodule pinned at `c4b82fc`). They are identical
today and nothing enforces it. A field changed on one side only shows up as
rosbridge logging "Cannot infer topic type" on the other.

## What the control station uses

| Topic | Node here | Direction |
| --- | --- | --- |
| `/current_bus_voltage`, `/temperature`, `/position` | `serial_node` (or `actuator_position_feedback` in Gazebo) | out |
| `/robot_state` (transient local) | `robot_state_controller` | out |
| `/arm_control_mode` | `robot_controller` | out |
| `/human_input_state` | `teleop` | in |
| `/robot_state/toggle` (`std_msgs/UInt8`) | `robot_state_controller` | in |
| `/dig`, `/dump` actions | `digdump_actions` | in |

Dig and dump now use separate `autonomy_msgs/action/Dig` and
`autonomy_msgs/action/Dump` types, each with Bruno variant `index=0` and
`string phase` feedback. The legacy `/digdump` endpoint is no longer launched.

The control station treats the ESP as online when any of the three serial
topics arrived in the last 2 s, the same rule `udp_client` uses. In Gazebo there
is no `serial_node`, so it reports "not present" instead and keeps Dig/Dump
disabled.

## Known gaps

- `robot_state_controller` has no ESTOP latch: a non-ESTOP toggle received
  while in ESTOP becomes the new state. The new dig and dump servers abort
  when displaced from their mode and do not restore TELEOP over a different
  observed state. However, a new action still requests DIG or DUMP through
  the existing toggle mechanism, which can leave ESTOP. A controller-level
  latch remains a separate change.
- `udp_client` and `udp_server` (`network_communication`) still launch and talk
  to nothing, since the control station's UDP gateway is gone. `udp_client` is
  also what publishes ESTOP when ESP feedback stops for 2 s, which is why the
  robot boots latched in ESTOP in Gazebo. Removing the package means moving that
  watchdog somewhere else first. `src/network_communication/README.md` describes
  a `net_node` that no longer exists.
- Nothing publishes `/esp_working`. `udp_client` subscribes to it; the control
  station ignores it.
- `docker-compose.yaml` runs the gstreamer service with `rmw_fastrtps_cpp` and
  `ROS_DOMAIN_ID=0`. Whether the real robot's boot environment uses Zenoh has not
  been confirmed; only the devcontainers have.
- The `mock` backend looks, from the control station, like real hardware with a
  dead ESP.

## Checking the bridge connection

After rebuilding the Jetson devcontainer, start the robot and run
`./ros/verify-jetson-link.sh` in the control-station container with the environment
above. Verify ROS topics and command delivery before testing cameras. Test
cameras separately and inspect the browser's selected ICE candidate pair.
These bridge-mode runtime checks have not yet been performed.
