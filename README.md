# MARS ROS2 Development Respository

## Obstacle Detection
Obstacle detection will be done using the *libtrealsense2.0 SDK* which provides the device drivers for the *Intel RealSense d435i Depth Camera*, C++ binaries and libraries, and header files to include librealsense functions in our source code. The source code in the *obstacle_detection* project is responsible for receiving depth matrices from the Intel RealSense d435i camera through the SDK device driver and supported library functions, project depth frames onto an X, Y, Z coordinate frame to create a *Point Cloud*, rotate coordinates to offset the angle of the RealSense camera, fill holes produced by shadows behind *positive obstacles*, and compute gradients in the point cloud to understand elevation increase and decrease in the RealSense's field of view as well as the presence of positive and *negative obstacles*.

*Point Cloud* - A series of point vectors (with X, Y, Z coordinates) in 3D space
*Postive obstacles* - obstacles portruding above ground level
*Negative obstacles* - obstacles like crates or potholes below average ground level

## Usage
Before using source code in the *obstacle_detection/src* directory, the *librealsense2.0* C++ binaries must be linked with our source code. There are 2 targets in our environment, the librealsense SDK libraries and any source code we wish to build. Our environment follows a *bash* framework for testing source code since we have an SDK as a dependency to extract, build, and link. In the *obstacle_detection/src/shell* directory, there are PowerShell and Linux Shell scripts for both Windows and Linux platforms to extract *librealsense2.0*, build it, build any source code targets you specify, and link librealsense libraries with source code targets. To build your target code:
1) Add all source code targets you wish to build to *obstacle_detection/src/CMakeLists.txt* as executables in order to specify it as a cmake build target.
2) Navigate to *obstacle_detection/src/shell* and run *artifact_build.ps1* or *artifact_build.sh* depending on your host platform to build your target source codes and the librealsense SDK if not already built as well as perform linking.
3) All binaries will be found in *obstacle_detection/src/build/Debug* since this is just test/debug code. You can run and use GDB on your target binary from this folder.
