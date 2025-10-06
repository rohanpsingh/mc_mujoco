FROM ubuntu:22.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Set locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8

# Set ROS_DISTRO environment variable
ENV ROS_DISTRO=humble

# Update and install basic dependencies
RUN apt-get update && apt-get install -y \
    emacs \
    curl \
    git \
    build-essential \
    cmake \
    software-properties-common \
    lsb-release \
    gnupg2 \
    ca-certificates \
    apt-transport-https \
    wget

# Setup ROS 2 Humble repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 base and required dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-common-interfaces \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-rosbag2 \
    python3-colcon-common-extensions

# Setup mc-rtc repository and install mc-rtc
RUN curl -1sLf 'https://dl.cloudsmith.io/public/mc-rtc/stable/setup.deb.sh' | bash && \
    apt-get update && apt-get install -y \
    libmc-rtc-dev \
    mc-rtc-utils

# Install ROS 2 mc-rtc packages
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-mc-rtc-plugin \
    ros-${ROS_DISTRO}-mc-rtc-tools

# Install mc_mujoco dependencies
RUN apt-get update && apt-get install -y \
    mesa-utils \
    libgl1-mesa-dri \
    libglx-mesa0 \
    libgl1-mesa-glx \
    x11-apps \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    libglew-dev \
    libboost-all-dev \
    libeigen3-dev \
    libyaml-cpp-dev \
    libgeos++-dev \
    libtinyxml2-dev \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR /workspace

# Clone and build mc_mujoco
RUN git clone --recursive https://github.com/rohanpsingh/mc_mujoco.git && \
    cd mc_mujoco && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    make -j$(nproc) && \
    make install


# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Reset DEBIAN_FRONTEND
ENV DEBIAN_FRONTEND=

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["/bin/bash"]
