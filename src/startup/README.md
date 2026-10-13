# Optional blue and orange RGB-D simulation

In the dev container, build and launch from the workspace root:

```bash
colcon build --packages-select startup --symlink-install
source install/setup.bash
ros2 launch startup launch.py robot_backend:=gazebo enable_rgbd:=true
```

This adds two Gazebo `rgbd_camera` sensors, facing opposite sides of the robot
(-Y/blue and +Y/orange in `frame_assembly`), each tilted 15 degrees downward.
The blue camera sits above the legacy `rear_camera` position. The
original front/rear cameras are independent. The option defaults to `false`.
Restart the robot launch to change it; the sensor is part of the spawned URDF.

`launch.py` passes `enable_rgbd` to both Xacro and `gazebo.launch.py`. The latter
conditionally includes `rgbd_sim.launch.py`, which starts the dedicated
ROS bridges and RViz with `config/RGBDRViz.rviz` and simulation time enabled.
It does not start another simulator or spawn a separate camera.
If including `gazebo.launch.py` from another launch file, pass
`enable_rgbd:=true` there **and** when generating that launch's robot description.
The existing Gazebo launch expects a robot description publisher upstream.

| ROS topic | Type |
| --- | --- |
| `/rgbd_blue/color/image_raw` | `sensor_msgs/msg/Image` |
| `/rgbd_blue/depth/image_raw` | `sensor_msgs/msg/Image` |
| `/rgbd_blue/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/rgbd_blue/points` | `sensor_msgs/msg/PointCloud2` |

The orange camera publishes the same four topics under `/rgbd_orange`, using
`rgbd_orange_link` and `rgbd_orange_optical_frame`. Both cameras are controlled by
`enable_rgbd`. RTAB-Map consumes the blue RGB-D stream by default. Enable `RTABMap_sync`
to feed both cameras to RTAB-Map.

The image and calibration topics use sensor-data QoS and `rgbd_blue_optical_frame`. Gazebo's RGB and depth
share the same viewpoint and calibration; `/rgbd_blue/camera_info` describes both.
Depth is expected as `32FC1` in meters. The optical frame uses +Z forward,
+X right, +Y down and is connected to `frame_assembly` through `rgbd_blue_link`.

The native point cloud uses +X forward, +Y left, +Z up. A separate bridge sets
its header frame to `rgbd_blue_link`, preserving the optical frame for images.
This corrects the cloud's frame label without rotating its point coordinates.

The initial settings are 640x480 at 30 Hz, 87-degree horizontal FOV and
0.1–10 m clipping. These are idealized simulation settings, not a calibrated
hardware profile. There is no IMU, stereo matching, hardware driver,
in this sensor model. RTAB-Map can be launched separately as described below.

Configuration locations:

- `urdf/robot/urdf/rgbd.gazebo.xacro`: mounting pose, frames and sensor settings.
- `config/rgbd_bridge.yaml`: Gazebo-to-ROS topic mappings.
- `launch/rgbd_sim.launch.py`: image/calibration and point-cloud bridges, plus RViz.
- `config/RGBDRViz.rviz`: RViz layout loaded when RGB-D is enabled.

With the simulation running, check in another sourced container terminal:

```bash
ros2 topic list | grep rgbd_
ros2 topic hz /rgbd_blue/color/image_raw --qos-reliability best_effort
ros2 topic hz /rgbd_blue/depth/image_raw --qos-reliability best_effort
ros2 topic echo /rgbd_blue/camera_info --once --qos-reliability best_effort
ros2 topic echo /rgbd_blue/depth/image_raw --field encoding --once --qos-reliability best_effort
ros2 run tf2_ros tf2_echo frame_assembly rgbd_blue_optical_frame
```

In RViz, use `frame_assembly` as the fixed frame and an Image display for each
image topic, selecting Best Effort reliability.
Add a PointCloud2 display on `/rgbd_blue/points` with Best Effort reliability and
RGB8 color to see the colored cloud. Stop any manually started bridge for this
topic before launching, to avoid duplicate publishers.
Check that both images look forward and that nearby obstacles appear at plausible
depths. Actual frame rate depends on rendering performance.

If ROS topics have no messages, check `gz topic -l` for `/rgbd_blue/image`,
`/rgbd_blue/depth_image`, `/rgbd_blue/camera_info` and `/rgbd_blue/points`, and check Gazebo's rendering logs.
The existing arena world already loads the required Sensors system with Ogre2.

## Optional RTAB-Map with RGB-D and LiDAR

Install `ros-jazzy-rtabmap-ros` in the dev container, rebuild `startup`, then run:

```bash
ros2 launch startup launch.py robot_backend:=gazebo enable_rtabmap:=true
```

`enable_rtabmap` defaults to false. Enabling it defaults both `enable_rgbd`
and `enable_lidar` to true. RTAB-Map uses LiDAR ICP odometry and subscribes to
both `/unilidar/cloud` and the RGB-D images. Registration and the occupancy
grid use LiDAR with the supplied planar ICP settings (5 cm voxels and
point-to-plane matching). Subscriptions use Best Effort QoS and simulation time.
RTAB-Map's own visualization window is enabled.

For RGB-D visual odometry alone, add `enable_lidar:=false`. For LiDAR alone,
add `enable_rgbd:=false`. Enabling RTAB-Map with both sensors disabled is an
error. Both sensor arguments also work independently of RTAB-Map; LiDAR defaults
to off when RTAB-Map is off. `enable_lidar` controls the simulated sensor and its
cloud/IMU bridges. The physical LiDAR launch for the serial backend is unchanged.
When launching `gazebo.launch.py` separately, pass the same sensor flags when
creating the robot description with Xacro.

Stop any manually launched RTAB-Map instance before using this option.
The existing controller configuration disables wheel odometry TF publication
(`enable_odom_tf: false`), allowing RTAB-Map odometry to own
`odom -> frame_assembly`. That controller setting also applies when RTAB-Map is
disabled; restore it to true if returning to wheel odometry, and do not run
RTAB-Map odometry TF alongside it.

The database defaults to `/tmp/rgbd_visual_test.db` and is resumed on subsequent
runs. Pass `rtabmap_database_path:=/tmp/another_test.db` for a separate map;
use a fresh database when switching sensor configurations.
In RViz, select fixed frame `map` and add `/rtabmap/cloud_map` as a PointCloud2
display to view the accumulated map. Odometry is published on
`/rtabmap/icp_odom` when LiDAR is enabled, otherwise `/rtabmap/visual_odom`.

Teleoperation uses `turn_speed_scale: 2.0` to double commanded turning speed.
The controller angular velocity limits are ±1.6 rad/s (previously ±0.8 rad/s).

## Synchronize both RGB-D cameras for RTAB-Map

```bash
ros2 launch startup launch.py robot_backend:=gazebo enable_rtabmap:=true RTABMap_sync:=true
```

`RTABMap_sync` defaults to false and requires `enable_rgbd:=true`. It starts
one `rtabmap_sync/rgbd_sync` node per camera, producing
`/rgbd_blue/rgbd_image` and `/rgbd_orange/rgbd_image`. A
`rtabmap_sync/rgbdx_sync` node synchronizes both into `/rgbd_images`
(`rtabmap_msgs/msg/RGBDImages`). This bundles both cameras with their own
calibration and frames; it does not stitch their depth images into one view.
Approximate synchronization allows at most 20 ms between inputs, with Best
Effort QoS and simulation time.

RTAB-Map, its visualization, and RGB-D odometry (when LiDAR is disabled) use
`subscribe_rgbd=true`, `rgbd_cameras=0` and the combined topic. Multi-camera
visual estimation uses `Vis/EstimationType=0`. LiDAR ICP settings remain active
when LiDAR is enabled. Use a fresh `rtabmap_database_path` when changing inputs.
The sync pipeline can also run with RTAB-Map disabled by setting
`enable_rgbd:=true RTABMap_sync:=true`.
