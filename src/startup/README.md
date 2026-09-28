# Optional front D435i RGB-D simulation

In the dev container, build and launch from the workspace root:

```bash
colcon build --packages-select startup --symlink-install
source install/setup.bash
ros2 launch startup launch.py robot_backend:=gazebo enable_d435i:=true
```

This adds **one** Gazebo `rgbd_camera` sensor to the robot's blue/front side
(-Y in `frame_assembly`), tilted 15 degrees downward. It sits above the position
named `rear_camera` in the legacy robot description. The
original front/rear cameras are independent. The option defaults to `false`.
Restart the robot launch to change it; the sensor is part of the spawned URDF.

`launch.py` passes `enable_d435i` to both Xacro and `gazebo.launch.py`. The latter
conditionally includes `d435i_sim.launch.py`, which starts the dedicated
ROS bridges and RViz with `config/RGBDRViz.rviz` and simulation time enabled.
It does not start another simulator or spawn a separate camera.
If including `gazebo.launch.py` from another launch file, pass
`enable_d435i:=true` there **and** when generating that launch's robot description.
The existing Gazebo launch expects a robot description publisher upstream.

| ROS topic | Type |
| --- | --- |
| `/d435i/color/image_raw` | `sensor_msgs/msg/Image` |
| `/d435i/depth/image_raw` | `sensor_msgs/msg/Image` |
| `/d435i/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/d435i/points` | `sensor_msgs/msg/PointCloud2` |

The image and calibration topics use sensor-data QoS and `d435i_optical_frame`. Gazebo's RGB and depth
share the same viewpoint and calibration; `/d435i/camera_info` describes both.
Depth is expected as `32FC1` in meters. The optical frame uses +Z forward,
+X right, +Y down and is connected to `frame_assembly` through `d435i_link`.

The native point cloud uses +X forward, +Y left, +Z up. A separate bridge sets
its header frame to `d435i_link`, preserving the optical frame for images.
This corrects the cloud's frame label without rotating its point coordinates.

The initial settings are 640x480 at 30 Hz, 87-degree horizontal FOV and
0.1–10 m clipping. These are idealized simulation settings, not a calibrated
D435i hardware profile. There is no IMU, stereo matching, hardware driver,
or RTAB-Map integration in this version.

Configuration locations:

- `urdf/robot/urdf/d435i.gazebo.xacro`: mounting pose, frames and sensor settings.
- `config/d435i_bridge.yaml`: Gazebo-to-ROS topic mappings.
- `launch/d435i_sim.launch.py`: image/calibration and point-cloud bridges, plus RViz.
- `config/RGBDRViz.rviz`: RViz layout loaded when the D435i is enabled.

With the simulation running, check in another sourced container terminal:

```bash
ros2 topic list | grep d435i
ros2 topic hz /d435i/color/image_raw --qos-reliability best_effort
ros2 topic hz /d435i/depth/image_raw --qos-reliability best_effort
ros2 topic echo /d435i/camera_info --once --qos-reliability best_effort
ros2 topic echo /d435i/depth/image_raw --field encoding --once --qos-reliability best_effort
ros2 run tf2_ros tf2_echo frame_assembly d435i_optical_frame
```

In RViz, use `frame_assembly` as the fixed frame and an Image display for each
image topic, selecting Best Effort reliability.
Add a PointCloud2 display on `/d435i/points` with Best Effort reliability and
RGB8 color to see the colored cloud. Stop any manually started bridge for this
topic before launching, to avoid duplicate publishers.
Check that both images look forward and that nearby obstacles appear at plausible
depths. Actual frame rate depends on rendering performance.

If ROS topics have no messages, check `gz topic -l` for `/d435i/image`,
`/d435i/depth_image`, `/d435i/camera_info` and `/d435i/points`, and check Gazebo's rendering logs.
The existing arena world already loads the required Sensors system with Ogre2.
