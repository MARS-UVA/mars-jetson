# ROS 2 development-container profiles

When VS Code asks which development-container configuration to use, choose the
profile matching the host:

- **Native Linux** forwards X11 directly and exposes `/dev/dri` for Mesa GPU
  acceleration. Before opening the container, permit local X11 clients with
  `xhost +local:`. Revoke that permission after use with `xhost -local:`.
- **WSLg** uses WSLg's X11 display with Mesa software rendering.
  Hardware-accelerated D3D12 rendering is intentionally disabled because it
  caused framebuffer corruption and a Windows host crash on the tested Intel
  Arc system.
- **macOS** runs a desktop inside the container and exposes it through noVNC on
  port 6080. VS Code should open the forwarded port automatically.

All profiles build the shared `Dockerfile` in this directory. Changes to shared
extensions, environment variables, or lifecycle commands must currently be
applied to all three `devcontainer.json` files.

XQuartz may eventually replace or supplement noVNC in the macOS profile, but it
requires an X server and display-access configuration on each developer's Mac.

The WSLg profile prioritizes stability over rendering performance. Do not enable
Mesa's D3D12 driver on a machine that exhibits corruption without first updating
Windows, WSL, and the Windows graphics driver and testing outside the container.
