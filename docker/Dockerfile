ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG DEBIAN_FRONTEND=noninteractive
ARG MVSDK_DIR=linuxSDK_V2.1.0.45(240521)
ARG ROS_DISTRO

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libopencv-dev \
    python3-colcon-common-extensions \
    python3-opencv \
    python3-pip \
    python3-yaml \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-launch-ros \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/mvsdk

# Build context is expected to be the parent GigE_SDK directory.
COPY ${MVSDK_DIR}/include/ /usr/include/
COPY ${MVSDK_DIR}/lib/x64/libMVSDK.so /usr/lib/libMVSDK.so
COPY ${MVSDK_DIR}/88-mvusb.rules /etc/udev/rules.d/88-mvusb.rules
COPY ${MVSDK_DIR}/99-mvusb.rules /etc/udev/rules.d/99-mvusb.rules
RUN ldconfig

WORKDIR /work/Academic_Immersion_Skoltech

COPY Academic_Immersion_Skoltech/docker/entrypoint.sh /entrypoint.sh
COPY Academic_Immersion_Skoltech/docker/start_camera.sh /usr/local/bin/start_camera.sh
COPY Academic_Immersion_Skoltech/docker/start_viewer.sh /usr/local/bin/start_viewer.sh
RUN chmod +x /entrypoint.sh /usr/local/bin/start_camera.sh /usr/local/bin/start_viewer.sh

RUN printf '%s\n' \
    'if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then' \
    '  source "/opt/ros/${ROS_DISTRO}/setup.bash"' \
    'fi' \
    'if [ -f /work/Academic_Immersion_Skoltech/aida-camera/install/setup.bash ]; then' \
    '  source /work/Academic_Immersion_Skoltech/aida-camera/install/setup.bash' \
    'fi' \
    > /etc/profile.d/ros_workspace.sh \
 && printf '\nsource /etc/profile.d/ros_workspace.sh\n' >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
