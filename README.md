# MARS ROS2 Development Respository

## Obstacle Detection

Obstacle detection will be done using the _libtrealsense2.0 SDK_ which provides the device drivers for the _Intel RealSense d435i Depth Camera_, C++ binaries and libraries, and header files to include librealsense functions in our source code. The source code in the _obstacle_detection_ project is responsible for receiving depth matrices from the Intel RealSense d435i camera through the SDK device driver and supported library functions, project depth frames onto an X, Y, Z coordinate frame to create a _Point Cloud_, rotate coordinates to offset the angle of the RealSense camera, fill holes produced by shadows behind _positive obstacles_, and compute gradients in the point cloud to understand elevation increase and decrease in the RealSense's field of view as well as the presence of positive and _negative obstacles_.

_Point Cloud_ - A series of point vectors (with X, Y, Z coordinates) in 3D space
_Postive obstacles_ - obstacles portruding above ground level
_Negative obstacles_ - obstacles like crates or potholes below average ground level

## Usage

Before using source code in the _obstacle_detection/src_ directory, the _librealsense2.0_ C++ binaries must be linked with our source code. There are 2 targets in our environment, the librealsense SDK libraries and any source code we wish to build. Our environment follows a _bash_ framework for testing source code since we have an SDK as a dependency to extract, build, and link. In the _obstacle_detection/src/shell_ directory, there are PowerShell and Linux Shell scripts for both Windows and Linux platforms to extract _librealsense2.0_, build it, build any source code targets you specify, and link librealsense libraries with source code targets. To build your target code:

1. Add all source code targets you wish to build to _obstacle_detection/src/CMakeLists.txt_ as executables in order to specify it as a cmake build target.
2. Navigate to _obstacle_detection/src/shell_ and run _artifact_build.ps1_ or _artifact_build.sh_ depending on your host platform to build your target source codes and the librealsense SDK if not already built as well as perform linking.
3. All binaries will be found in _obstacle_detection/src/build/Debug_ since this is just test/debug code. You can run and use GDB on your target binary from this folder.

## Testing/Experimentation

Our SIL testing is done through emulating the RealSense d435 using mock pointcloud data.
The _obstacle_detection/test/assets_ directory has 2 pointclouds in a PLY file format. To view one of these ply files, use the _ply_file_reader.py_ file in the _obstacle_detection/test/_ directory, select whether to use Matplotlib or Open3d as well as the path to PLY file you want to view relative to your present working directory.

You can also view the RealSense field of view by running _pointcloud_viewer.py_. Before running, run _pip install pyrealsense2_ first if you haven't already.
