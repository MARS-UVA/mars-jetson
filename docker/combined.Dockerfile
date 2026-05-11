# Combined image

# Build (from repo root):
#   docker build -f docker/combined.Dockerfile -t mars-jetson:combined .

# Run (no gpio):
#   sudo docker run\
#     --privileged \
#     --network host \
#     -v /dev:/dev \
#     -v /sys:/sys \
#     -e CONTROL_STATION_IP=192.168.50.101 \
#     mars-jetson:combined

FROM ros:kilted-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-opencv \
    python3-serial \
    python3-websockets \
    busybox \
    gstreamer1.0-tools \
    gstreamer1.0-nice \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-plugins-good \
    libgstreamer1.0-dev \
    libglib2.0-dev \
    python3-gst-1.0 \
    gir1.2-gst-plugins-bad-1.0 \
    ros-kilted-cv-bridge \
    ros-kilted-vision-opencv \
    ros-kilted-sensor-msgs \
    ros-kilted-v4l2-camera \
    ros-kilted-rclcpp-action \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install Jetson.GPIO --break-system-packages

COPY ./src /mars-jetson/src
WORKDIR /mars-jetson

RUN source /opt/ros/kilted/setup.bash && \
    colcon build --symlink-install \
      --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=true

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# ENV FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ENV JETSON_MODEL_NAME=JETSON_ORIN_NANO
ENV CONTROL_STATION_IP=192.168.50.101

ENTRYPOINT ["/bin/bash", "-c", "\
    source /opt/ros/kilted/setup.bash && \
    source /mars-jetson/install/setup.bash ; \
    ros2 launch gstreamer gstreamer_launch.py & \
    ros2 launch startup launch.py"]
