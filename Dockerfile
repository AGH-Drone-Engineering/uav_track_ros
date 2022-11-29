FROM ros:noetic

RUN apt-get update && \
    apt-get install -y \
        ros-${ROS_DISTRO}-mavros-msgs \
    && rm -rf /var/lib/apt/lists/*
