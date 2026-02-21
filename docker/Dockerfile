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
    # mc-rtc repository (for JRL dependency packages)
    curl -1sLf 'https://dl.cloudsmith.io/public/mc-rtc/stable/setup.deb.sh' | bash && \
    rm -rf /var/lib/apt/lists/*

# Install all dependencies in one layer (heavy but cached)
# Note: ros-humble-mc-rtc-plugin/tools are excluded — they pull in libmc-rtc-dev
# (binary built against old RBDyn 128-byte Joint). We build mc_rtc from source instead.
RUN apt-get update && apt-get install -y \
    # Build tools
    build-essential cmake ninja-build ccache git emacs \
    # ROS 2
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-common-interfaces \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-rosbag2 \
    python3-colcon-common-extensions \
    # mc-rtc build dependencies (JRL libs from cloudsmith, system libs from Ubuntu)
    # libgeos++-inline-dev provides missing .inl files for Ubuntu 22.04 GEOS
    # mc-rtc-data provides robot/environment descriptions (JVRC1, etc.)
    libeigen3-dev libboost-all-dev libtinyxml2-dev \
    libyaml-cpp-dev libspdlog-dev libnanomsg-dev libltdl-dev libnotify-dev \
    libtasks-dev libtvm-dev libeigen-quadprog-dev libstate-observation-dev \
    libndcurves-dev libmesh-sampling-dev libgeos++-inline-dev \
    mc-rtc-data \
    # Graphics/X11
    mesa-utils libgl1-mesa-dri libglx-mesa0 libgl1-mesa-glx x11-apps \
    libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libglew-dev \
    # Additional libraries
    libgeos++-dev \
    && rm -rf /var/lib/apt/lists/*

# The cloudsmith Tasks 1.8.2 binary was compiled against RBDyn pre-1.9.3,
# where rbd::Joint was 128 bytes. RBDyn 1.9.3 added Ir_/gr_ fields making
# rbd::Joint 144 bytes, causing Tasks' nrVars to crash (wrong array stride).
# Rebuild Tasks from source against the installed RBDyn 1.9.3.
# /usr/local/lib is searched before /lib/x86_64-linux-gnu (via libc.conf),
# so the rebuilt library takes precedence after ldconfig.
RUN git clone --depth=1 --recurse-submodules --branch v1.8.2 https://github.com/jrl-umi3218/Tasks.git /tmp/Tasks && \
    cmake -S /tmp/Tasks -B /tmp/Tasks/build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DPYTHON_BINDING=OFF && \
    cmake --build /tmp/Tasks/build && \
    cmake --install /tmp/Tasks/build && \
    ldconfig && \
    rm -rf /tmp/Tasks

# Build mc_rtc from source against RBDyn 1.9.3 (avoids ABI mismatch from
# the cloudsmith binary which was compiled against the old 128-byte rbd::Joint)
RUN git clone --depth=1 --recurse-submodules https://github.com/jrl-umi3218/mc_rtc.git /tmp/mc_rtc && \
    cmake -S /tmp/mc_rtc -B /tmp/mc_rtc/build -G Ninja \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        -DPYTHON_BINDING=OFF && \
    cmake --build /tmp/mc_rtc/build && \
    cmake --install /tmp/mc_rtc/build && \
    ldconfig && \
    rm -rf /tmp/mc_rtc

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
