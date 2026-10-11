# ROS 2 development-container profiles

When VS Code asks which development-container configuration to use, choose the
profile matching the host:

- **Native Linux** forwards X11 directly and exposes `/dev/dri` for Mesa GPU
  acceleration. Before opening the container, permit local X11 clients with
  `xhost +local:`. Revoke that permission after use with `xhost -local:`.
- **WSLg** uses WSLg's X11 display with Mesa software rendering.
- **macOS** runs a desktop inside the container and exposes it through noVNC on
  port 6080. VS Code should open the forwarded port automatically.

All profiles build the shared `Dockerfile` in this directory. Changes to shared
extensions, environment variables, or lifecycle commands must currently be
applied to all three `devcontainer.json` files.

After creation, every profile runs rosdep and then `build-workspace.sh`, which
runs `colcon build --symlink-install`. If an existing `build/` directory holds a
`CMakeCache.txt` recorded under a different mount path, the script deletes
`build/`, `install/`, and `log/` before building. After that, you only need to
re-run `colcon build` by hand when you pull in new upstream packages.

All profiles create and join the `mars-dev` Docker bridge network, with the
alias `mars-jetson`. `CONTROL_STATION_IP=mars-control-station` addresses the
control-station container on the same Docker host. TCP 7447 is published for
Zenoh clients outside this network; UDP 8080 remains published for the legacy
gateway. Workspace builds and ROS environment settings are unchanged.

For local simulation, set `JETSON_IP=mars-jetson` in the control-station
container and use `ROS_DOMAIN_ID=42` on both sides. Rebuild both containers if
their networking or environment configuration changes. For separate machines,
use the Jetson's LAN IP as `JETSON_IP`, pass the laptop's LAN IP as
`control_station_ip` to `deploy.sh`, and publish TCP 6767/6969 on the
control-station host for camera signaling. Docker bridge networks do not span
machines. Camera media uses WebRTC ICE separately from these signaling ports;
bridge-mode camera connectivity must be verified and may need a TURN relay.
See `docs/rosbridge.md` for the connection details.

XQuartz may eventually replace or supplement noVNC in the macOS profile, but it
requires an X server and display-access configuration on each developer's Mac.
