FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

SHELL ["/bin/bash", "-c"]

# Install GStreamer, Python bindings, and OpenCV
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    gstreamer1.0-tools \
    gstreamer1.0-nice \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-plugins-good \
    libgstreamer1.0-dev \
    libglib2.0-dev \
    python3-gst-1.0 \
    gir1.2-gst-plugins-bad-1.0 \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    ros-humble-sensor-msgs \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Install python networking libs
RUN pip3 install websockets asyncio

# Create workspace
RUN mkdir -p /ws/.ros && chown -R 1000:1000 /ws
WORKDIR /ws

# Copy required files into the container
COPY ./src /ws/src

# Build packages
RUN source /opt/ros/humble/setup.bash && colcon build --packages-select gstreamer

# Source ROS 2 and run startup
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch gstreamer gstreamer_launch.py"]
