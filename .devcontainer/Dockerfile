FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-kinematics-interface \
    ros-${ROS_DISTRO}-eigen3-cmake-module \
    libeigen3-dev \
    python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*

# trimesh has no apt package; pip is externally-managed in this image (PEP 668)
RUN pip3 install --break-system-packages trimesh

RUN rosdep init || true && rosdep update

WORKDIR /robotarm_description

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc