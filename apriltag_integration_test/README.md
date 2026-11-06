# Standalone AprilTag video integration test

Feed stationary, calibrated video recordings into the **installed `apriltag_ros`
node**, collect its actual detection arrays and pose TF, and compare every selected
frame with measured ground truth. No colcon build or robot launch is needed.

## Prepare recordings

Make a directory for each pose, with its video and `setup.yaml` together:

```text
apriltag_integration_test/
  setups/
    pose_1/
      setup.yaml
      video.mp4
    pose_2/
      setup.yaml
      video.mp4
    pose_3/
      setup.yaml
      video.mp4
```

Copy `setups/example/setup.example.yaml` into each directory as `setup.yaml`.
Replace the example camera calibration, tag ID/size, and measured pose. The example
is a template, not a usable calibration. Paths in YAML are relative to that YAML.
Videos and configuration must resolve inside this test directory, including symlinks.

The camera must remain stationary for the entire clip. Use calibration from that
camera at the recorded resolution. Optional `camera.output_width` and
`output_height` resize frames with OpenCV area interpolation and scale intrinsics
before rectification; source aspect ratio must be preserved. Calibration entries
`fx`, `fy`, `cx`, `cy` are in pixels. Distortion uses the OpenCV pinhole/plumb_bob
model, not fisheye coefficients. Raw frames are rectified locally with OpenCV using
the scaled intrinsic matrix. For already rectified recordings, set
`already_rectified: true` and supply the **rectified** intrinsics. Avoid additional
cropping, stabilization, or zoom after calibration.

## Ground-truth conventions

Measure the **tag center relative to the camera optical center**. Optical axes are
X right in the image, Y down, and Z forward. All distances are meters.

```yaml
ground_truth:
  offset_x_m: 0.10
  offset_y_m: -0.05
  distance_m: 1.50
  angle_deg: 20.0
```

`distance_m` is forward Z depth, not the diagonal tape-measure distance. For a slant
range `d` and known X/Y offsets, use `sqrt(d*d - x*x - y*y)` for depth.
`angle_deg` is tag rotation about camera optical **+Y**, using the right-hand rule.
At zero, a native upright AprilTag has axes X right, Y up, Z toward the camera,
matching the installed detector's PnP convention (camera-to-tag rotation Rx(180°)).
Use the native AprilTag print orientation; OpenCV's aruco AprilTag dictionary renders
the pattern rotated 180° relative to `apriltag_to_image`.
A positive angle makes the tag's right edge closer to the camera.
This is not a robot's floor-plane heading or the bearing to the tag center.

For arbitrary roll/pitch/yaw use this alternative, without the shorthand fields:

```yaml
ground_truth:
  translation_m: [0.10, -0.05, 1.50]
  rpy_deg: [180.0, 20.0, 0.0]
```

Rotation is `Rz(yaw) Ry(pitch) Rx(roll)` for the tag pose in camera coordinates.
Measure printed size at the detection corners (the outer black square for 36h11),
excluding the surrounding white margin. Add multiple `tags` entries if multiple
surveyed tags are expected in a clip; give each its own size and ground truth.
All configured tags are expected in every selected frame.

**For 41h12, do not assume a 20 cm overall print means `size_m: 0.2`.** Its data
extends beyond the detection border. Measure the edge between detection corners,
at the black/white border transition, for each family. Either print all families
at a 20 cm detection-corner width, or enter each family's measured corner width in
the generated setup files and record the common overall print footprint separately.
The official [AprilTag pose documentation](https://github.com/AprilRobotics/apriltag#pose-estimation)
defines this measurement. Incorrect size creates a systematic distance-scale error.

## Dorm experiment: generate all 36 configurations

From the repository root:

```sh
cp apriltag_integration_test/dorm.example.yaml apriltag_integration_test/dorm.yaml
cp apriltag_integration_test/camera.example.yaml apriltag_integration_test/camera.yaml
```

1. Fill `camera.yaml` with measured intrinsics and distortion for the actual camera
   and recording mode. Source dimensions are detected automatically with
   `width: auto` and `height: auto`; explicit dimensions also work. Intrinsics must
   correspond to that source resolution. Optional 640×480 resizing requires a 4:3
   source; for a 16:9 source use an output such as 640×360, or omit output dimensions
   to process the original size. Do not use photo resolution as video resolution.
2. In `dorm.yaml`, keep nominal distances `[0.5, 1, 2, 3]`, angles `[-45, 0, 45]`,
   families `[36h11, 41h12, 25h9]`, size 0.205 m, Hamming 0, FPS 60 and duration 5 s.
   Change `measured_distances_m` to your smaller physical distances if needed.
   Those measured distances, rather than nominal labels, are used as ground truth.
   Set `tag_ids` to the IDs you printed. Optional `tag_sizes_m` sets a measured
   detection-corner width per family. Accuracy limits can be shared in `thresholds`.
3. Generate the directories and configs:

   ```sh
   ./apriltag_integration_test/run.sh --generate-matrix apriltag_integration_test/dorm.yaml
   ```

4. Use `setups/dorm/manifest.csv` as the recording checklist. Put each recording
   beside its generated `setup.yaml` as `video.mp4` (or edit `video` to its actual
   filename). Edit individual measured distances, offsets, angles or sizes when
   your physical setup differs. The configured mount uses camera optical-center
   height 0.02 m, tag bottom height 0.025 m and tag height 0.205 m. With a level camera,
   the tag center is 0.1075 m above the lens, so generated optical offsets are
   `(0, -0.1075, distance)`. Floor heights must refer to the tag itself, excluding
   paper margins. A tilted camera requires manually measured camera-frame poses.
5. Validate and run:

   ```sh
   ./apriltag_integration_test/run.sh --validate
   ./apriltag_integration_test/run.sh
   ```

The generator creates **12 cases per family, 36 cases total, 300 frames per case**:
3,600 frames per family and 10,800 overall. It maps the convenient `41h12` label to
the installed detector's `Standard41h12` name. It refuses to overwrite existing
configs, preserving your measurements. The normal runner recursively discovers
all `setup.yaml` files under `setups/` (including any other experiments there).
To run only the dorm matrix:

```sh
./apriltag_integration_test/run.sh apriltag_integration_test/setups/dorm/*/setup.yaml
```

Each generated case requires 300 decoded samples and approximately 60 FPS (±0.1,
allowing 59.94). A shorter clip or mismatched FPS fails the protocol rather than
silently producing a smaller successful experiment. Set `start_frame` if the camera
settles after recording starts, and record enough additional footage to retain
300 frames. Videos use nominal FPS for frame-time labels; recordings should have
constant frame rate. Five seconds of wall-clock recording alone does not guarantee
300 decoded frames. This evaluates accuracy/reliability offline, not 60 FPS processing
performance on the robot.

Keep the recording lens and mode fixed, and avoid changing digital zoom or
stabilization between calibration and recording. Resizing scales the existing
camera intrinsics; it preserves the camera's view.

## Execute

For uncalibrated recordings, explicitly set `analysis_mode: detection_only` at the
top level of a setup. That runs the real detector with pose estimation disabled:
detection rates, corner coordinates, homographies, decision margins and annotations
are available, while metric position/angle errors are unavailable. Camera intrinsics
may be omitted in this mode. `errors.svg` then plots decision margin and detection
success. Switch back to `analysis_mode: localization` and supply actual calibration
to calculate localization errors.

Set `camera.rotation_degrees: 180` for upside-down footage. Rotation is applied to
the processed frames and annotated output, leaving the original recording intact.
With calibrated input, the image is rectified before rotation and the published
principal point is rotated consistently. Ground truth must describe the upright
processed camera frame. Only configured tag families receive detection overlays.

The supplied `1.0m.mov`, `1.5m.mov`, and `1.9m.mov` recordings have configs in
`setups/recorded_36h11_*/setup.yaml`: 36h11 ID 1, size 0.205 m, vertical offset
-0.1075 m, rotation 180°, localization mode, all decoded frames. They share
`camera_b0332.yaml`, with researched Arducam B0332 focal length and pixel pitch.
These are nominal manufacturer-specification parameters with centered optical
axis and crop assumptions, **not measured calibration**. See [B0332.md](B0332.md)
for sources, numeric values and limitations. The recording configurations use their
actual approximately 22 FPS, without relabeling them as 60 FPS. Rerun them with:

```sh
./apriltag_integration_test/run.sh apriltag_integration_test/setups/recorded_36h11_*/setup.yaml
```

To calculate **conditional** pose offsets from cached detection corners, using the
known 1.0 m clip as a calibration reference:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 apriltag_integration_test/estimate_offsets.py \
  apriltag_integration_test/results/<run-directory>
```

This writes populated per-frame pose offsets and `summary.csv` under that run's
`inferred_offsets/` directory. It assumes the configured tag size/pose are exact,
zero residual distortion and unchanged camera intrinsics across videos. The
reference clip is calibration data and cannot independently validate accuracy.
`calibration_assumptions.json` records these assumptions and the inferred matrix.

Requires installed ROS 2 packages `apriltag_ros`, `apriltag_msgs`, `sensor_msgs`,
`tf2_msgs`, `rclpy`, `ament_index_python`, and `rmw_fastrtps_cpp`, plus Python
OpenCV, NumPy and PyYAML. The script installs nothing. It uses `/usr/bin/python3`.
Source your installed ROS environment if needed; when no ROS environment is present,
the shell script sources `/opt/ros/${ROS_DISTRO:-jazzy}/setup.sh`.

From the repository root:

```sh
# Check video readability, pose fields and resolution before running ROS:
./apriltag_integration_test/run.sh --validate

# Run all setup.yaml files recursively under setups/:
./apriltag_integration_test/run.sh

# Or select cases explicitly:
./apriltag_integration_test/run.sh \
  apriltag_integration_test/setups/pose_1/setup.yaml \
  apriltag_integration_test/setups/pose_2/setup.yaml
```

The shell script is executable and works from any current directory; explicit CLI
paths are relative to your current directory. The default scan ignores the template.
Adjust `sampling.start_frame`, `stride` and `max_frames` in each YAML. Video FPS is
used to label sample times; processing deliberately runs one frame at a time and
waits for the detector, rather than testing real-time throughput. Detection arrays
and TF must match the exact published image timestamp. No stale pose is reused.

Exit status: **0** all cases pass, **1** one or more cases fail, **2** invalid input
or missing dependencies. Per-case runtime errors are saved and fail that case.
Default minimum pose success rate is 90%. Optional thresholds set the 95th-percentile
camera position error and rotation error; choose them for your application.
Any detector response timeout or missing pose TF fails the case independently.

## Read the output

Each execution creates `results/<timestamp>_<random>/`, containing a combined
`summary.json`, ROS logs/configuration, and a numbered directory for each case:

- `setup.yaml`: configuration snapshot with resolved video path.
- `detector.yaml` / `detector.log`: actual detector parameters and output.
- `annotated.avi`: paired MJPG video of the processed/rectified frames, with actual
  detector corner polygons, IDs, decision margins and per-frame status. All configured
  tags detected in a frame are boxed; missed tags never retain an old box. Frames with
  no detections remain in the output. The video uses source FPS / stride and contains
  exactly the selected samples, starting at `start_frame`, capped by `max_frames`.
  It has no audio. Set `annotation: {enabled: false}` to disable it.
- `frames.csv`: one row per selected frame per expected tag, including frame/time,
  status, confidence metadata, pose, quaternion and errors. Missing detections and
  missing TF remain explicit rows with empty error fields.
- `summary.json`: success rates, status counts, bias, population standard deviation,
  MSE, RMSE, absolute 95th-percentile errors, and threshold failures per tag.
- `errors.svg`: camera position and rotation errors against video frame, viewable
  directly in a browser. Missing poses are omitted from plots and counted in reports.
- `error.json`: diagnostic when a case cannot complete.

The detector reports `T_camera_tag = [R, t]`. We compare it to measured ground truth
and also invert it to obtain **camera localization in the tag coordinate frame**:
`camera_position = -R.T @ t`. `position_error_m` is the norm of the camera position
difference; `camera_dx_m`, `camera_dy_m`, `camera_dz_m` retain signed bias.
`tag_position_error_m` and `tag_dx_m`, etc. describe the direct tag-center estimate
in camera axes. `depth_error_m` measures Z bias; `range_error_m` measures slant-range
bias. `rotation_error_deg` is the shortest full 3D rotation angle, in [0, 180].
An orientation error can therefore increase camera localization error even when
the direct tag-center estimate is accurate.

`angle_error_deg` is the signed horizontal tag-normal angle error, wrapped to
[-180, 180); the dorm's rotations about optical Y use exactly this angle.
`rotation_error_deg` additionally catches roll and other 3D orientation errors.
`relative_position_error` is the camera position error divided by true camera-to-tag
range: multiply by 100 to express it as a percentage.

The installed `/detections` message contains a **2D homography**, not a metric 3D
pose matrix. We save that matrix and the detector's pixel corners in `frames.csv`,
and use the detector's timestamp-matched PnP TF for metric localization. All topics
are scoped to the test's namespace.

## Compare families and distances

### Analysis notebook

Open `apriltag_analysis.ipynb` in VS Code or Jupyter. It includes rendered charts
for the latest B0332 run: signed distance and angle offset bars with ±1 SD for each
distance and the pooled total, a compact metrics table, detection reliability,
3D position-error distributions and per-frame time traces. SVG figures are also
saved under that run's `notebook_figures/` directory for use in reports.

The best-fit diagnostic section fits a separate line to each recording/tag's
horizontal and vertical angle errors over time. It shows raw errors with fitted
baselines and residuals after baseline subtraction, plus raw, demeaned and
detrended SD and fitted drift. `angle_detrending_summary.csv` and
`angle_detrending_frames.csv` preserve the results. This describes repeatability
after fitting; it does not attribute bias to human setup or correct accuracy.
Subtracting a constant leaves a recording's SD unchanged; removing different
recording means reduces pooled SD, and removing slopes additionally removes drift.

The notebook uses NumPy, standard Python and inline SVG; it does not need an
additional plotting library. Outputs are already embedded for viewing. To rerun,
select a normal Python notebook kernel with NumPy and run all cells. Set
`RUN_DIRECTORY` in the first cell to analyze another localization run.
`SD_DDOF = 0` means population SD of the recorded frames. Total averages pool
all frames; error bars describe spread, not confidence intervals or calibration
uncertainty. The notebook builder can also execute and refresh the saved outputs
without a notebook kernel:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 apriltag_integration_test/notebooks/build_notebook.py
```

### Presentation CSV

`horizontal_vertical_angle_errors.csv` provides a separate comparison of tag-normal
horizontal and vertical orientation errors, with per-distance and pooled totals.
It includes signed mean offset, population SD, mean absolute error and P95 absolute
error, all in degrees. `horizontal_vertical_angle_errors_frames.csv` contains the
underlying estimated/true angles and errors for every frame, retaining misses.
Horizontal is normal azimuth, positive right; vertical is normal elevation,
positive up. These are tag tilt errors, not tag-center bearing errors; in-plane
roll remains part of the separate full 3D rotation error. Both compare against
each setup's actual configured orientation. The notebook displays the axis
comparison and separate vertical ±1 SD chart. Future runs export both CSVs.
To export these metrics from an existing run without rerunning the detector:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 apriltag_integration_test/angle_axes.py \
  apriltag_integration_test/results/<run-directory>
```

**Use `showcase.csv` to present results.** Every run produces this single compact
table, grouped by family, ground-truth distance/angle and calibration basis. It
contains sample count, detection percentage, signed mean depth offset, mean/P95
3D camera position error, mean absolute/P95 horizontal angle error, and calibration
provenance. Position/depth values are in centimeters; angles are in degrees; numeric
statistics are rounded to two decimals. P95 is the 95th percentile of absolute errors.
Missing localization stays blank. Samples count expected tag observations (one per
frame per expected tag); error statistics use valid poses and detection percentages
retain missing detections. Calibration provenance identifies nominal manufacturer
specifications so the compact table does not imply measured calibration.

At the run root, spreadsheet-friendly files aggregate the raw per-frame data:

| File | Grouping |
| --- | --- |
| `family_comparison.csv` | Family |
| `distance_comparison.csv` | Actual measured Z distance, pooling families |
| `family_distance_comparison.csv` | Family and measured distance |
| `family_angle_comparison.csv` | Family and angle |
| `case_comparison.csv` | Individual video and family |
| `comparison.json` | All groups, failed cases, and acquisition protocol status |

Each includes detection success percentage, valid-pose percentage, average decision
margin, and mean/std/MSE/P95 of position errors, normalized position error, angle
errors, signed camera/translation offsets, depth and range errors. Error statistics
use valid poses; detection percentages retain misses/timeouts in the denominator.
Decision margins average detected samples, including detections missing pose TF.
Rows are weighted equally per expected tag per frame. With the planned single-tag,
300-frame cases, each of the 12 configurations has equal weight within its family.
Check case failures and `protocol_complete` before treating an aggregate as complete;
runtime failures with no rows cannot contribute numerical error statistics.

For the family decision, first apply the same accuracy and detection requirements
to all families, then inspect their worst distance/angle cases and P95 errors.
Signed mean offsets can cancel, so also use mean position-error magnitude, MSE or
P95. The decision margin is a decoder score, **not a calibrated probability**, and
should not be the sole basis for ranking different families.

The 36 positive clips measure successful detection and localization under these
conditions. They do not measure false positives or wrong-family confusion. For risk
mitigation, also retain separate blank-wall/wrong-family controls, test at least a
few different IDs and repeat placements/lighting conditions before choosing a
family for deployment. A clip with no relevant detections stays visible as a failure
in this positive-case runner. Consecutive frames from one static placement are
correlated; 3,600 frames are not 3,600 independent setup trials.

Statistics are calculated over valid poses, with failures retained in the success-rate
denominator. There is no fitted alignment or correction that could hide bias.
Repeated frames quantify jitter at these poses; they are correlated observations,
not independent trials across the robot's entire operating range.

This evaluates tag-based camera localization. Evaluating robot `base_link` or the
complete RTAB-Map pipeline would additionally require mounting transforms and other
sensor streams; this standalone test never starts that pipeline.

## Isolation and verification

The runner starts only its video publisher/analyzer and one installed detector at
a time. It selects its own ROS domain (100–199), unique namespace, remaps global TF,
rosout and parameter-event topics, uses loopback-only Fast DDS UDP transport, and
does not use shared-memory transport or the ROS CLI daemon. Inherited discovery-server
settings are cleared. All generated configuration, reports, and ROS logs live under
this directory. The detector runs with its case output directory as its working
directory and is stopped after each case. Installed libraries are read externally;
robot packages, launch files, services and configurations are unused. This is ROS
topic/transport isolation, not an operating-system security sandbox.

Run the checks from the repository root with the installed ROS Python environment:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s apriltag_integration_test/tests -p 'test_*.py'
PYTHONDONTWRITEBYTECODE=1 python3 apriltag_integration_test/tests/smoke.py
```

The smoke test additionally uses the installed native AprilTag library through ctypes
to generate short videos for all three families, frontal/oblique poses and a blank
control. It runs the real detector, checks numerical pose limits, calibrated resizing,
annotation pixels/FPS/frame count, grouped reports and missing-detection accounting,
and leaves analysis results under `results/`.
Synthetic inputs are temporary and removed after the check. It needs permission
to open local ROS sockets.
