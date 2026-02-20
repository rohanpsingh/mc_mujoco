FROM ubuntu:22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV ROS_DISTRO=humble

# Setup locales and add repositories in one layer
RUN apt-get update && apt-get install -y \
    locales curl gnupg lsb-release wget ca-certificates apt-transport-https && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    # ROS 2 repository
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    # mc-rtc repository
    curl -1sLf 'https://dl.cloudsmith.io/public/mc-rtc/stable/setup.deb.sh' | bash && \
    rm -rf /var/lib/apt/lists/*

# Install all dependencies in one layer (heavy but cached)
RUN apt-get update && apt-get install -y \
    # Build tools
    build-essential cmake git emacs \
    # ROS 2
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-common-interfaces \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-rosbag2 \
    ros-${ROS_DISTRO}-mc-rtc-plugin \
    ros-${ROS_DISTRO}-mc-rtc-tools \
    python3-colcon-common-extensions \
    # mc-rtc
    libmc-rtc-dev mc-rtc-utils \
    # Graphics/X11
    mesa-utils libgl1-mesa-dri libglx-mesa0 libgl1-mesa-glx x11-apps \
    libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libglew-dev \
    # Libraries
    libboost-all-dev libeigen3-dev libyaml-cpp-dev libgeos++-dev libtinyxml2-dev \
    && rm -rf /var/lib/apt/lists/*

# Pre-install MuJoCo (cached, won't re-download)
COPY MUJOCO_VERSION /tmp/MUJOCO_VERSION
RUN MUJOCO_VERSION=$(cat /tmp/MUJOCO_VERSION | tr -d '[:space:]') && \
    mkdir -p /opt/mujoco && cd /opt && \
    wget --quiet https://github.com/deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz && \
    tar xzf mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz && \
    mv mujoco-${MUJOCO_VERSION}/* mujoco/ && \
    rm -rf mujoco-${MUJOCO_VERSION} *.tar.gz

ENV LD_LIBRARY_PATH=/opt/mujoco/lib:/opt/mujoco/bin:$LD_LIBRARY_PATH
ENV MUJOCO_ROOT_DIR=/opt/mujoco

# Build stage - uses local source code
FROM base AS builder

WORKDIR /workspace/mc_mujoco

# Copy source code (uses .dockerignore to exclude build artifacts)
COPY . .

# Build mc_mujoco
RUN mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DMUJOCO_ROOT_DIR=/opt/mujoco && \
    make -j$(nproc) && \
    make install

# Final stage
FROM base AS runtime

# Copy built artifacts from builder
COPY --from=builder /usr/local /usr/local
COPY --from=builder /workspace/mc_mujoco/build /workspace/mc_mujoco/build

# Copy entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /workspace

ENV DEBIAN_FRONTEND=

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
