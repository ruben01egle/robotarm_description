FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    python3-colcon-common-extensions \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /robotarm_description

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc