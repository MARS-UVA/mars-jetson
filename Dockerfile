FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

RUN apt-get update -q && apt-get upgrade -q -y
RUN apt-get update -q && apt-get install -y --no-install-recommends \
    ros-jazzy-v4l2-camera \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

WORKDIR /ros2_ws/

COPY ./ ./

# Build
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --packages-select cameras

CMD [ "./launch.sh" ]