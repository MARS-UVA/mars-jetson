# MARS ROS2 Development Repository

**main branch is behind feature branches due to independent feature development, please refer to each branch for implementation and documentation**\
This repository is still in development and documentation is only available for each feature branch. The main branch is behind feature branches and accordingly, please see a feature branch's README.md file for instructions on building and to see implementations:
- Obstacle Detection and RealSense Depth frame processing: please visit **obstacle-detection** branch
- Multi-process socket communication implementation and protocols: please visit **communication** branch
- April Tag Pose Estimation implementation: please visit **apriltag** branch

# Structure
This repository is a ROS2 repository and has a hierarchical structure. The root folder or workspace directory is *src/*. Under this workspace directory exists project directories containing sub-modules of the larger project:
- *src/actions/* includes programs that make up the **actions** node. These programs decide what motor and actuator actions to perform upon subscribing to the **communications** nodes. All motor and actuator current values (all motor are controlled by current levels) are passed through a buffer or array of bytes and constructed by the **actions** node.
- *src/hero_comms* and *src/webapp_comms* are both part of the **communications** node responsible for setting up a series of sockets for communicating chunks of data. For instance, the *frame_sender.cpp* implementation is responsible for dividing up an RGB frame from the **webcam** node into simultaneously sent packets for quick transmission time. Each packet is designed to be a lightweight UDP packet with nothing but a sequence number and CRC checksum to validate transmission integrity. It is also designed to be reconstructed on the fly by a load-balancing server on the receiving end.
- *src/obstacle_detection* is part of the **perception** node. This node is responsible for ingesting depth frames from a RealSense 435 Depth Camera, filling holes produced by occlusions, rotating the frame of reference to a X, Y, Z coordinate system, and detecting positive and negative obstacles in the ground.

# Usage
1) Create local *ssh* private/public key pair and add public key to your GitHub account.
2) Clone this repository (git@github.com:MARS-UVA/mars-jetson.git)
3) To build specific features (each of which are on separate branches as of 1/15/2025), please refer to the README.md for each branch on how to run shell scripts and CMakeLists for building.
4) To build ROS2 nodes, run *colcon build* from *src/*
     * To run specific project such as *src/webapp_comms*, navigate to the desired project directory and run *colcon build*
  
** Make sure to source ROS2 Humble (/opt/ros2/humble/setup.sh) in order to ensure that the *ros2* command and make files during build will work.

## Communications
There are 2 communication channels that our Nvidia Jetson Orin computer board will utilize:
* Internet sockets to communicate packets between the Jetson and our control station laptop
* Serial Bus to communicate streams of data between the Jetson a Nucleo STM32 microcontroller board

There are 3 types of data that will be communicated between the Jetson and Control Station:
1) RGB image frames from our Webcam's video stream
2) List of motor and actuator feedbacks listed in order
3) Position of robot and detected obstacles as X, Y coordinates

## Usage
Only testing usage is supported as of now.
To test, navigate to *webapp_comms/test/shell* and run *test.ps1* or *test.sh* depending on whether you are using a Windows or Linux platform. These shell scripts will build and install Catch2, build sourc code using the CMakeLists.txt in the *webapp_comms/test/* directory, and run tests. This CMakeLists.txt will build any target source code you wish to test in *webapp_comms/src/* and test suite code in *webapp_comms/test/*.

If you write new source code in *webapp_comms/src/* that you wish to test, add the source code file as an executable in *webapp_comms/test/CMakeLists.txt*. If you create a new source code file for new Catch2 test functions, do the same... add that source code file as an executable in *webapp_comms/test/CMakeLists.txt*.
