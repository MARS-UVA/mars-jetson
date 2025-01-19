# MARS ROS2 Development Repository

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