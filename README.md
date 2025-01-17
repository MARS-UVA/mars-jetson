# MARS ROS2 Development Repository

## Communications
There are 2 communication channels that our Nvidia Jetson Orin computer board will utilize:
* Internet sockets to communicate packets between the Jetson and our control station laptop
* Serial Bus to communicate streams of data between the Jetson a Nucleo STM32 microcontroller board

There are 3 types of data that will be communicated between the Jetson and Control Station:
1) RGB image frames from our Webcam's video stream
2) List of motor and actuator feedbacks listed in order
3) Position of robot and detected obstacles as X, Y coordinates
