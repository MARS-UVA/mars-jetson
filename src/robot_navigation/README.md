# Robot navigation configuration

This package owns the robot's Nav2 servers, configuration, RViz view, and optional
simulation LiDAR filter. It expects map, TF, odometry, sensor clouds, and command
selection to be supplied by the caller. It starts neither a simulator nor a map
server or localization system.

Build and launch in a sourced ROS 2 Jazzy workspace:

```bash
colcon build --symlink-install --packages-select robot_navigation
source install/setup.bash
ros2 launch robot_navigation navigation.launch.py use_sim_time:=true
```

The main `startup` launch includes this package when `enable_nav2:=true` and
selects its RViz configuration. To build and run the integrated simulation:

```bash
./deploy.sh robot_backend:=gazebo enable_rtabmap:=true enable_nav2:=true
```

`deploy.sh` forwards its arguments to the startup launch. `robot_backend:=gazebo`
selects simulation; `enable_rtabmap:=true` supplies mapping and defaults both
RGB-D and LiDAR sensors to enabled. Navigation remains disabled by default.
Add `enable_local_lidar:=true` to use the simulation LiDAR filter for local
obstacles. Start only one Nav2 instance.

Add `no_gui:=true` to disable all simulation windows (Gazebo, both RViz views,
and RTAB-Map's viewer). Add `enable_nav_view:=true` as well to show only the
navigation RViz view. `enable_nav_view:=false` hides that view while leaving
the other windows enabled. These options belong to the main startup launch;
the standalone navigation launch starts no GUI.

| File | Parameters |
| --- | --- |
| `config/planner.yaml` | Smac Hybrid-A* global path planner |
| `config/controller.yaml` | MPPI controller, progress checker, goal checker |
| `config/global_costmap.yaml` | RTAB-Map static layer and global inflation |
| `config/local_costmap.yaml` | Local voxel/inflation layers and simulation LiDAR filter |
| `config/bt_navigator.yaml` | NavigateToPose and NavigateThroughPoses navigators |
| `config/behaviors.yaml` | Spin, backup, drive-on-heading, and wait |
| `config/smoothers.yaml` | Path smoother server and velocity smoother |
| `rviz/navigation.rviz` | Navigation view used by the simulation integration |

The planner receives its server parameters and the global costmap file. The
controller receives its server parameters and the local costmap file. Keep the
costmaps' nested ROS node namespaces when editing these files.

Launch arguments:

- `config_dir`: directory containing all seven YAML files; defaults to this
  package's installed `config` directory.
- `use_sim_time`: defaults to `false`; rewritten across all loaded configurations.
- `odom_topic`: defaults to `/rtabmap/icp_odom`; replaces the `/odom` placeholders.
- `autostart`: defaults to `true` for the six managed Nav2 servers.
- `enable_local_lidar`: defaults to `false`. When enabled, starts
  `navigation_cloud_filter` and adds its output to local obstacle observations.
  Filter parameters live in `local_costmap.yaml` and assume the simulation's
  flat floor and robot geometry.

The controller and behaviors publish `/cmd_vel/navigation_raw`. The velocity
smoother publishes `/cmd_vel/navigation` for the robot's command mux. The path
smoother remains configured separately; calling it requires a `SmoothPath` step
in a behavior tree. This extraction preserves the current default trees and
planner/controller tuning.

Mission sequencing (for example, navigate, dig, navigate, dump) can call the Nav2
navigation actions from a separate autonomy package. Replacing the geometric
path planner later can be configured independently in `planner.yaml`.

## Arena markers

Arena markers are disabled by default so navigation can run outside the arena.
Providing a nonempty `arena_name` enables four fixed region outlines and labels
on `/arena/markers`.
In RViz, set the fixed frame to `map` and add a MarkerArray display for that topic.
Enable an arena on the navigation launch, or run only the visualization node:

```bash
ros2 launch robot_navigation navigation.launch.py arena_name:=ARENA_NASA
ros2 run robot_navigation arena_markers_node --ros-args -p arena_name:=ARENA_NASA
```

Add arena entries in `scripts/arena_configs.py` using zone vertices in meters,
plus the berm center and dimensions. The initial berm placement is approximate
and centered within the construction zone. Arena selection is read at startup.

To enable markers through the regular startup launch:

```bash
ros2 launch startup launch.py robot_backend:=gazebo enable_rtabmap:=true enable_nav2:=true arena_name:=ARENA_NASA
```

The startup launch forwards `arena_name` to navigation. Omitting it disables
markers; supplying it without `enable_nav2:=true` does not start the node.
Both RViz configurations include an Arena regions display. Navigation RViz uses
fixed frame `map`; the regular view keeps its existing fixed frame.
