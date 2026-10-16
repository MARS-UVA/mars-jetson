# Optional Nav2 navigation

Nav2 plans and follows goals on RTAB-Map's occupancy grid. The current integrated
launch supports Gazebo with RGB-D and LiDAR enabled. Navigation is disabled by
default; starting Nav2 does not automatically select navigation as the driving mode.

## Start and send a goal

Run from the workspace root in the ROS 2 Jazzy development container. Dependencies
are declared in `package.xml` and installed through `rosdep`, rather than a separate
Dockerfile navigation-install block. Initialize the required source submodules
before resolving workspace dependencies and building, including the interface
submodule that defines `RobotState.NAVIGATION`.

```bash
./deploy.sh robot_backend:=gazebo enable_rtabmap:=true enable_nav2:=true
```

The deploy script builds the workspace and sources its overlay. The launch requires
`enable_rtabmap`, `enable_rgbd`, and `enable_lidar` to be true when navigation is
enabled. Enabling RTAB-Map supplies the latter two defaults. Hardware/serial Nav2
startup is not integrated yet.

The main startup launch includes `robot_navigation/launch/navigation.launch.py` when
`enable_nav2:=true`. `deploy.sh` forwards all ROS launch arguments. The optional
`./gazebo_deploy.sh enable_rtabmap:=true enable_nav2:=true` wrapper additionally
cleans up leftover Gazebo servers.

In another container terminal, use the same middleware and workspace overlay:

```bash
source setup_terminal.sh
ros2 topic pub --once /robot_state/toggle std_msgs/msg/UInt8 '{data: 4}'
```

In RViz, wait for the map and costmaps, then use **Navigation2 Goal** to click a
destination and drag its heading. The **Navigation 2** panel reports progress and
provides action cancellation. The fixed frame is `map`. The configuration displays
the planned path, both costmaps, and both planning footprints; sensor-cloud and TF
displays can be enabled for debugging.

Cancel the active goal in the panel before returning to teleop:

```bash
ros2 topic pub --once /robot_state/toggle std_msgs/msg/UInt8 '{data: 0}'
```

Mode switching currently changes the mux selection only. It does not cancel Nav2
or dig/dump actions. An action can continue running while deselected, and selecting
its source again can resume forwarding its commands. Do not treat a mode change
as action cancellation.

The existing mode values remain TELEOP=0, DIG=1, DUMP=2, ESTOP=3; NAVIGATION=4.
The existing ESTOP toggle behavior is also preserved: requesting 3 from another
mode enters ESTOP, while requesting 3 again returns to TELEOP. An independent stop
lock and cancellation on transitions remain future work.

## Topic and transform ownership

| Interface | Owner / consumer |
| --- | --- |
| `map -> odom` | RTAB-Map localization correction |
| `odom -> base_link` | RTAB-Map odometry; ICP with LiDAR, visual without it |
| Base-to-sensor/joint transforms | Robot state publisher using the robot description and joint states |
| `/rtabmap/map` | RTAB-Map occupancy grid; global costmap static layer subscribes |
| `/rtabmap/icp_odom` | Current integrated Nav2 odometry input, expected `nav_msgs/msg/Odometry` |
| `/rgbd_blue/points`, `/rgbd_orange/points` | Independent `PointCloud2` observations for the local voxel layer |
| `/unilidar/cloud` | LiDAR `PointCloud2` input for ICP odometry and RTAB-Map mapping |
| `/cmd_vel/navigation_raw` | Controller and recovery behaviors publish; velocity smoother subscribes |
| `/cmd_vel/navigation` | Velocity smoother publishes final `geometry_msgs/msg/Twist`; base-command mux subscribes |
| `/cmd_vel` | Mux-selected teleop, dig/dump, or navigation output |
| `/base_controller/cmd_vel` | Twist stamper output to the Gazebo base controller |
| `/arm_drum_control/stop` | State controller publishes four zero velocities at 20 Hz during navigation |
| `/arm_drum_controller/commands` | Arm/drum mux output to the Gazebo joint velocity controller |
| `/robot_state/toggle` | Mode requests as `std_msgs/msg/UInt8` |
| `/robot_state` | State controller publishes the selected `RobotState` with transient-local durability |

RTAB-Map's accumulated `/rtabmap/cloud_map` is a 3D cloud, not the occupancy grid
used by the global costmap. Confirm `/rtabmap/map` publication and QoS at runtime.
The drive controller has `enable_odom_tf: false`; keep one publisher for each TF
edge. This navigation launch starts neither AMCL nor a map server.

`robot_navigation/launch/navigation.launch.py` starts the controller, planner, smoother,
behavior, BT navigator, and velocity smoother servers plus their lifecycle
manager. Both controller and
recovery velocities pass through the same velocity smoother and mux. The launch
does not start a collision monitor, route server, docking server, or waypoint follower.

The configuration uses `/odom` as a placeholder parameter value. The navigation
launch rewrites each `odom_topic` to `/rtabmap/icp_odom` for the integrated launch;
it does not create a new `/odom` publisher. The standalone launch accepts
`odom_topic`, `config_dir`, `use_sim_time`, `autostart`, and `enable_local_lidar`,
assuming that its caller already provides map, TF, sensors, and command selection.
Avoid starting a second
Nav2 instance alongside the integrated launch.

During navigation, arms and drums are held at zero through their mux. Recovery
using the arms/drums to unbeach the robot needs an explicit future command source
or recovery mode. The serial controller currently subscribes to `/arm_drum_control`,
which differs from the mux output above; serial arm/drum routing was not changed
by this integration.

## Configuration and tuning assumptions

Edit the split YAML files in [robot_navigation/config](../robot_navigation/config) for
planning, controller, costmaps, behaviors, and both smoothers. See the
[package guide](../robot_navigation/README.md) for the file layout and launch arguments.
[navigation.rviz](../robot_navigation/rviz/navigation.rviz) defines the navigation view.

- **Planner:** Smac Hybrid-A* checks the rectangular footprint at each heading,
  with forward/reverse Reeds-Shepp paths and built-in smoothing. The initial
  curved-path turning radius is 0.40 m; this planner does not search in-place
  rotations, even though the differential drive can execute them. Planning through
  unknown space is allowed. The separate smoother server is configured, but the default
  navigation behavior tree does not call it; using it requires a `SmoothPath` step.
- **Motion:** MPPI uses `DiffDrive`, a 2.8-second prediction horizon, and 1,000
  sampled trajectories. Limits start at 0.60 m/s forward, 0.40 m/s reverse, and
  1.20 rad/s turning, below the drive controller's 0.80 m/s and 1.60 rad/s limits.
  The velocity smoother uses matching speed and acceleration limits and open-loop
  feedback. Linear acceleration is 0.60 m/s² and angular acceleration is 1.20 rad/s².
  MPPI samples with linear/angular standard deviations of 0.20/0.40. The path
  horizon is 2.0 m, covering 0.60 m/s times 2.8 seconds. MPPI follows Hybrid-A*
  headings (`PathAngleCritic.mode=2`, `PathAlignCritic.use_path_orientations=true`)
  and direction-change points (`enforce_path_inversion=true`); no forward-only
  critic competes with reverse segments. Profile
  controller performance on the Jetson before increasing the sample count.
- **Turning priority:** A rotation shim wraps MPPI. On each updated path, heading
  error above 30 degrees starts a zero-translation alignment phase, handing back
  to MPPI within 10 degrees. It uses the planned orientation 0.5 m along the path
  to preserve reverse-segment intent. Rotation speed/acceleration limits are
  1.20 rad/s and 1.20 rad/s². The shim collision-checks the rotating footprint;
  if rotation is blocked it falls back to MPPI. MPPI also favors alignment with
  path-angle weight 6.0, path-follow weight 3.0, and goal-angle weight 5.0.
- **Goal/progress:** Initial goal tolerances are 0.20 m and 0.20 rad; the progress
  checker requires 0.15 m movement within 20 seconds.
- **Controller obstacle costs:** MPPI uses `ObstaclesCritic` with zero repulsion
  weight and zero additional collision margin. Nearby inflation costs do not add
  a soft avoidance penalty; full rectangular footprint collision checks remain
  enabled with a nonzero collision penalty. Speed can still decrease for turns,
  acceleration limits, goals, or trajectories that would collide.
- **Local footprint:** The rectangle extends to x +/-0.90 m and y +/-0.38 m,
  with 0.03 m padding. It conservatively covers sampled URDF collision-mesh bounds
  across the arm limits and wheel/drum rotations, including protruding equipment.
  Compare it with the physical robot before hardware use.
- **Global footprint:** The same rectangle and 0.03 m padding as the local
  costmap replace the previous 1.02 m enclosing circle. Hybrid-A* checks oriented
  clearance near walls. The 1.25 m inflation radius adds a preference for clearance;
  it does not make that entire band an impassable obstacle.
- **Mapping body filter:** LiDAR occupancy grids exclude returns within the
  unpadded 1.80 m (base X) by 0.76 m (base Y) robot envelope, from frame Z=0
  to 0.50 m. The height is an initial simulation setting; inspect classified
  obstacle clouds to verify coverage. This filters grid generation, not ICP input.
  Start a fresh mapping database when evaluating it: saved local grids retain
  their previous classification.
- **Local obstacles:** A rolling 6 x 6 m voxel costmap runs at 10 Hz with 0.05 m
  horizontal resolution. Blue and orange RGB-D clouds are independent observation
  sources; no camera stitching or synchronization is required for these costmap
  inputs. Only these depth cameras feed the local costmap; LiDAR feeds ICP and
  RTAB-Map mapping. The cameras face the blue and orange ends, providing front/rear depth
  observations in the blue-forward base frame.
- **Filtering:** The initial ground assumption is approximately z=-0.13 m in
  `odom`. Obstacle returns must be between -0.08 and 1.45 m. The voxel grid spans
  -0.10 to 1.50 m. These are fixed-frame height limits, not optical-frame depth;
  verify them against actual TF and point clouds. A height cutoff alone does not
  segment ground on slopes.
- **Mapping ground classification:** With simulated LiDAR enabled, RTAB-Map
  builds occupancy grids using height passthrough instead of surface normals
  (`Grid/NormalsSegmentation=false`). `Grid/MaxGroundHeight=-0.08` classifies
  points at or below -0.08 m as ground in the gravity-aligned robot frame
  (`Grid/MapFrameProjection=false`, `Grid/GroundIsObstacle=false`). The wheel
  radius (0.1778 m) minus axle height (0.04445 m) predicts floor Z=-0.13335 m,
  leaving about 5 cm tolerance. This is a flat-arena setting; raised terrain
  above the cutoff remains an obstacle. Existing saved local grids must be
  rebuilt or a fresh database used to evaluate the new classification.
- **Observation ranges:** All sources mark to 2.5 m and clear to 3.0 m. Minimum
  ranges are 0.10 m for both RGB-D cameras. The expected-update interval
  is 0.5 seconds, so both cameras must remain available for a current local costmap.
- **Inflation:** Local/global radii are both 1.25 m. Local decay is 20.0;
  global decay is 6.0 and Hybrid-A* cost penalty is 2.0, restoring a moderate
  preference for global routes away from walls. The controller retains zero
  obstacle repulsion so clearance is primarily chosen by the global route. Actual obstacle cells and footprint collision checks remain.

The map database defaults to `/tmp/rgbd_visual_test.db` and resumes across runs.
Choose a separate `rtabmap_database_path` for a fresh test map. `RTABMap_sync:=true`
feeds synchronized camera streams to RTAB-Map; it does not alter Nav2's independent
point-cloud observations.

## Simulation checks to run

Runtime verification is pending and is performed by the operator. From another
terminal sourced with `setup_terminal.sh`, inspect the interfaces before sending
goals:

```bash
ros2 topic info /rtabmap/map --verbose
ros2 topic echo /rtabmap/icp_odom --once
ros2 run tf2_ros tf2_echo map base_link
ros2 topic hz /rgbd_blue/points --qos-reliability best_effort
ros2 topic hz /rgbd_orange/points --qos-reliability best_effort
ros2 topic hz /unilidar/cloud --qos-reliability best_effort
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /velocity_smoother
ros2 topic info /cmd_vel/navigation --verbose
ros2 topic info /base_controller/cmd_vel --verbose
```

Confirm the map is `nav_msgs/msg/OccupancyGrid`, TF is connected without duplicate
publishers, and odometry contains measured velocities expressed in the base child
frame with current timestamps. Confirm navigation publishes `Twist` and the
base controller accepts the twist stamper's output type. In RViz, check that
ground does not become an obstacle, real obstacles mark and clear appropriately,
and footprints enclose the robot.

Then test a short unobstructed goal, turning, an obstructed path, cancellation,
and mode switching. Observe `/cmd_vel/navigation`, `/cmd_vel`, and arm/drum output
to confirm command selection. Check that navigation does not move the robot when
another mode is selected and that arms/drums remain stopped in navigation. Test
stopping and re-entry explicitly; cancellation and retained-command handling are
not automatically managed by mode transitions in this version.

Reference: [Nav2 Jazzy Smac planner](https://api.nav2.org/nav2-jazzy/html/md_nav2_smac_planner_README.html)
and [adding a separate smoother](https://docs.nav2.org/jazzy/tutorials/general_tutorials/adding_smoother/adding_smoother/).

## Front convention

Blue is the default navigation front. `base_link` is centered at the CAD origin,
with +X toward the blue drums, +Y left, and +Z up. The fixed joint from `base_link`
to `frame_assembly` has yaw +pi/2, because blue is CAD -Y. RTAB-Map and wheel
odometry use `base_link`; Nav2 footprints are expressed in those axes.
Gazebo subtracts that fixed joint yaw from the original CAD spawn yaw so the
physical robot retains its original orientation in the arena. UI teleop
reversal remains owned by the UI. Selecting orange during a run is not implemented
by this fixed convention; it will need coordinated logical-frame, velocity,
footprint, and navigation-goal handling rather than rotating the odometry frame.
Use a fresh mapping database after changing the odometry base convention.
