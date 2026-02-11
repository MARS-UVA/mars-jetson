FROM ros:jazzy-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

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
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-opencv \
    ros-jazzy-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install python networking libs
RUN pip3 install --break-system-packages websockets asyncio

# Create workspace
WORKDIR /ws

# Copy required files into the container
COPY ./src /ws/src

# Source ROS 2 and run startup
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && colcon build --packages-select gstreamer &&
    source install/setup.bash && ros2 run gstreamer webrtc_stream"]