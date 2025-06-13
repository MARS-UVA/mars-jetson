# MARS ROS2 Development Repository

## How to Run
`./deploy.sh [CONTROL_STATION_IP]`

## Structure
This repository is a ROS2 **workspace**. Under `src`, almost each directory is a ROS2 **package**. Each package has code directories (`scripts`, `src`, or the name of the package) and a build file, either `setup.py` or `CMakeLists.txt`.

## Development
1) Create local *ssh* private/public key pair and add public key to your GitHub account.
2) Clone this repository (git@github.com:MARS-UVA/mars-jetson.git)
3) To build ROS2 nodes, run *colcon build* from *src/*
     * To run specific project such as *src/webapp_comms*, navigate to the desired project directory and run *colcon build*

** Make sure to source ROS2 Humble (/opt/ros2/humble/setup.sh) in order to ensure that the `ros2` command works.

## Communications
There are 2 communication channels that our Nvidia Jetson Orin computer board will utilize:
* Internet sockets to communicate packets between the Jetson and our control station laptop
* Serial Bus to communicate streams of data between the Jetson and the Nucleo STM32 microcontroller board