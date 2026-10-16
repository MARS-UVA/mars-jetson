# Robot navigation configuration

This package owns the robot's Nav2 servers, configuration, RViz view, and optional
simulation LiDAR filter. It expects map, TF, odometry, sensor clouds, and command
selection to be supplied by the caller. It starts neither a simulator nor a map
server or localization system.

Build and launch in a sourced ROS 2 Jazzy workspace:

```bash
colcon build --symlink-install --packages-select nav2_config
source install/setup.bash
ros2 launch nav2_config navigation.launch.py use_sim_time:=true
```

Start only one Nav2 instance. During the initial extraction, startup still uses
its original navigation launch; switching startup to this package is a separate
step. The existing integrated entry point remains:

```bash
./deploy.sh robot_backend:=gazebo enable_rtabmap:=true enable_nav2:=true
```

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
