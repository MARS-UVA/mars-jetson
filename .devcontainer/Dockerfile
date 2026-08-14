FROM ros:jazzy-perception
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if getent passwd ${USER_UID} > /dev/null; then userdel $(getent passwd ${USER_UID} | cut -d: -f1); fi \
    && if getent group ${USER_GID} > /dev/null; then groupdel $(getent group ${USER_GID} | cut -d: -f1); fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL /bin/bash

RUN apt-get update && apt-get install -y libglfw3-dev libx11-dev xorg-dev libopencv-dev libpcl-dev libxkbcommon-dev

RUN apt-get update && apt-get install -y gstreamer1.0-tools gstreamer1.0-nice gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-plugins-good libgstreamer1.0-dev libglib2.0-dev gir1.2-gst-plugins-bad-1.0

RUN apt-get update && apt-get install -y ros-jazzy-topic-tools ros-jazzy-rmw-zenoh-cpp ros-jazzy-v4l2-camera ros-jazzy-controller-interface ros-jazzy-rqt-console ros-jazzy-rqt-graph

# [Optional] Automatically source ROS setup script when a new shell is launched
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]
