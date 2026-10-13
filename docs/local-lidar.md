# Local navigation LiDAR

RGB-D remains the default local obstacle source. After rebuilding `startup`,
enable filtered LiDAR in simulation with:

```bash
./gazebo_deploy.sh enable_rtabmap:=true enable_nav2:=true enable_local_lidar:=true
```

The filter reads `/unilidar/cloud` and publishes `/unilidar/navigation_points`.
It removes self returns and applies height limits in `base_link`, using only
the mounting transform. It preserves the input frame, sensor origin, and scan
timestamp. Nav2's TF message filter queues observations until odometry TF is
available. Raw clouds used by odometry and mapping are unchanged.

When enabled, LiDAR is a required observation source: stale data stops local
navigation rather than silently removing side obstacle coverage. Look for the
filter's `Publishing filtered LiDAR` startup message. `Waiting for LiDAR mounting
TF` means the cloud frame cannot be transformed into `base_link`; a filter process
exit means its startup traceback must be resolved before navigation.

## Hardware preparation

The integrated navigation launch currently requires Gazebo. The standalone
`nav2.launch.py` supports `enable_local_lidar:=true use_sim_time:=false`, but the
hardware map, odometry TF, command routing, and observation sources must be
configured separately before enabling robot motion.

The hardware driver uses cloud frame `unilidar_lidar`; the URDF currently has
`unilidar_link`. Supply a calibrated TF connecting the driver's actual sensor
origin and axes to the model. Do not merely relabel the cloud. Confirm points
line up with the robot and obstacles in RViz before using them for navigation.

Calibrate `navigation_cloud_filter`'s body bounds and `min_height_in_base` /
`max_height_in_base` in your hardware parameter file. The defaults assume this
simulation's flat floor at roughly Z=-0.13 m. They are not ground segmentation
for slopes or uneven terrain. The voxel layer's height limits are instead in
`odom`; those also need to match your hardware odometry origin. Check raised
arms and drums against the self-filter envelope.

Verify `/unilidar/navigation_points` publishes continuously, the local costmap
marks side obstacles and clears removed ones, and stale observations stop
navigation. Retain the 0.5-second freshness check; increasing that timeout does
not fix missing clouds or TF.
