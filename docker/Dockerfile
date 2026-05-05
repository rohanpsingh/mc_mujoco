# syntax=docker/dockerfile:1
FROM ubuntu:22.04

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
    curl -1sLf 'https://dl.cloudsmith.io/public/mc-rtc/head/setup.deb.sh' | bash && \
    rm -rf /var/lib/apt/lists/*

# Install all dependencies in one layer (heavy but cached)
# Note: libmc-rtc-dev excluded — binary was compiled against RBDyn pre-1.9.3
# where rbd::Joint was 128 bytes. RBDyn 1.9.3 added Ir_/gr_ fields making
# rbd::Joint 144 bytes. We build mc_rtc from source instead (see below).
RUN apt-get update && apt-get install -y \
    # Build tools
    build-essential cmake ninja-build ccache git emacs \
    # ROS 2
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-common-interfaces \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-rosbag2 \
    ros-${ROS_DISTRO}-ament-index-cpp \
    ros-${ROS_DISTRO}-mc-rtc-msgs \
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
    libgeos++-dev \
    && rm -rf /var/lib/apt/lists/*

# The cloudsmith Tasks 1.8.2 binary was compiled against RBDyn pre-1.9.3,
# where rbd::Joint was 128 bytes. RBDyn 1.9.3 added Ir_/gr_ fields making
# rbd::Joint 144 bytes, causing Tasks' nrVars to crash (wrong array stride).
# Rebuild Tasks from source against the installed RBDyn 1.9.3.
# /usr/local/lib is searched before /lib/x86_64-linux-gnu (via ld.so.conf),
# so the rebuilt library takes precedence after ldconfig.
RUN --mount=type=cache,target=/root/.ccache \
    git clone --depth=1 --recurse-submodules --branch v1.8.2 \
        https://github.com/jrl-umi3218/Tasks.git /tmp/Tasks && \
    cmake -S /tmp/Tasks -B /tmp/Tasks/build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        -DPYTHON_BINDING=OFF && \
    cmake --build /tmp/Tasks/build && \
    cmake --install /tmp/Tasks/build && \
    ldconfig && \
    rm -rf /tmp/Tasks

# Build mc_rtc from source against RBDyn 1.9.3 (avoids ABI mismatch from
# the cloudsmith binary which was compiled against the old 128-byte rbd::Joint)
# Update MC_RTC_VERSION to a branch, tag, or commit SHA to pin and invalidate
# this layer's cache.
COPY MC_RTC_VERSION /tmp/MC_RTC_VERSION
RUN --mount=type=cache,target=/root/.ccache \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    MC_RTC_VERSION=$(cat /tmp/MC_RTC_VERSION | tr -d '[:space:]') && \
    git clone --depth=1 --recurse-submodules --branch ${MC_RTC_VERSION} \
        https://github.com/jrl-umi3218/mc_rtc.git /tmp/mc_rtc && \
    cmake -S /tmp/mc_rtc -B /tmp/mc_rtc/build -G Ninja \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        -DPYTHON_BINDING=OFF && \
    cmake --build /tmp/mc_rtc/build && \
    cmake --install /tmp/mc_rtc/build && \
    # Disable autoload of the ROS plugin: rclcpp's static teardown segfaults
    # at process exit. Users who want the plugin can opt in via mc_rtc.yaml.
    rm -f /usr/local/lib/mc_plugins/autoload/ROS.yaml && \
    ldconfig && \
    rm -rf /tmp/mc_rtc

# Build the mc_rtc-ecosystem libraries that example controllers depend on
# (BaselineWalkingController and its transitive deps). Pinned via
# examples/MC_RTC_DEPS_VERSION — bump that file to invalidate this stack.
# SHAs match grasp-fsm-sample-controller/README.md "Tested with" table.
COPY examples/MC_RTC_DEPS_VERSION /tmp/MC_RTC_DEPS_VERSION

RUN --mount=type=cache,target=/root/.ccache \
    git clone --depth=1 --recurse-submodules \
        https://github.com/arntanguy/gram_savitzky_golay.git /tmp/gsg && \
    cmake -S /tmp/gsg -B /tmp/gsg/build -G Ninja \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        -DBUILD_TESTING=OFF && \
    cmake --build /tmp/gsg/build && \
    cmake --install /tmp/gsg/build && \
    ldconfig && rm -rf /tmp/gsg

RUN --mount=type=cache,target=/root/.ccache \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    git clone --depth=1 --recurse-submodules \
        https://github.com/jrl-umi3218/mc_state_observation.git /tmp/mso && \
    cmake -S /tmp/mso -B /tmp/mso/build -G Ninja \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        -DBUILD_TESTING=OFF -DINSTALL_DOCUMENTATION=OFF && \
    cmake --build /tmp/mso/build && \
    cmake --install /tmp/mso/build && \
    ldconfig && rm -rf /tmp/mso

RUN --mount=type=cache,target=/root/.ccache \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    git clone --recurse-submodules \
        https://github.com/isri-aist/QpSolverCollection.git /tmp/qsc && \
    git -C /tmp/qsc checkout 42cb58674e692bea0bf64c7ab7a4a80137023aee && \
    cmake -S /tmp/qsc -B /tmp/qsc/build -G Ninja \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        -DBUILD_TESTING=OFF -DINSTALL_DOCUMENTATION=OFF \
        -DENABLE_QUADPROG=ON && \
    cmake --build /tmp/qsc/build && \
    cmake --install /tmp/qsc/build && \
    ldconfig && rm -rf /tmp/qsc

RUN --mount=type=cache,target=/root/.ccache \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    git clone --recurse-submodules \
        https://github.com/isri-aist/TrajectoryCollection.git /tmp/tc && \
    git -C /tmp/tc checkout 3f18e7a8748e40dbccdd28b0b3ec98353a33f58d && \
    cmake -S /tmp/tc -B /tmp/tc/build -G Ninja \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        -DBUILD_TESTING=OFF -DINSTALL_DOCUMENTATION=OFF && \
    cmake --build /tmp/tc/build && \
    cmake --install /tmp/tc/build && \
    ldconfig && rm -rf /tmp/tc

RUN --mount=type=cache,target=/root/.ccache \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    git clone --recurse-submodules \
        https://github.com/isri-aist/ForceControlCollection.git /tmp/fcc && \
    git -C /tmp/fcc checkout 55de5119347987e9e3bc76783aec439748202cf5 && \
    cmake -S /tmp/fcc -B /tmp/fcc/build -G Ninja \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        -DBUILD_TESTING=OFF -DINSTALL_DOCUMENTATION=OFF && \
    cmake --build /tmp/fcc/build && \
    cmake --install /tmp/fcc/build && \
    ldconfig && rm -rf /tmp/fcc

RUN --mount=type=cache,target=/root/.ccache \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    git clone --recurse-submodules \
        https://github.com/isri-aist/NMPC.git /tmp/nmpc && \
    git -C /tmp/nmpc checkout d88792ccb1a77e78bca7572ba2afd35b0418cd71 && \
    cmake -S /tmp/nmpc -B /tmp/nmpc/build -G Ninja \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        -DBUILD_TESTING=OFF -DINSTALL_DOCUMENTATION=OFF && \
    cmake --build /tmp/nmpc/build && \
    cmake --install /tmp/nmpc/build && \
    ldconfig && rm -rf /tmp/nmpc

RUN --mount=type=cache,target=/root/.ccache \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    git clone --recurse-submodules \
        https://github.com/isri-aist/CentroidalControlCollection.git /tmp/ccc && \
    git -C /tmp/ccc checkout a874f6a09e4f1431509c166fb622af25781fe396 && \
    cmake -S /tmp/ccc -B /tmp/ccc/build -G Ninja \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        -DBUILD_TESTING=OFF -DINSTALL_DOCUMENTATION=OFF && \
    cmake --build /tmp/ccc/build && \
    cmake --install /tmp/ccc/build && \
    ldconfig && rm -rf /tmp/ccc

RUN --mount=type=cache,target=/root/.ccache \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    git clone --recurse-submodules \
        https://github.com/isri-aist/BaselineWalkingController.git /tmp/bwc && \
    git -C /tmp/bwc checkout 9cba7e2fcc95b990774599f405471b86cc0f9e07 && \
    cmake -S /tmp/bwc -B /tmp/bwc/build -G Ninja \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        -DBUILD_TESTING=OFF -DINSTALL_DOCUMENTATION=OFF && \
    cmake --build /tmp/bwc/build && \
    cmake --install /tmp/bwc/build && \
    ldconfig && rm -rf /tmp/bwc

# Install MuJoCo (cached — update MUJOCO_VERSION to re-download)
COPY MUJOCO_VERSION /tmp/MUJOCO_VERSION
RUN MUJOCO_VERSION=$(cat /tmp/MUJOCO_VERSION | tr -d '[:space:]') && \
    cd /opt && \
    wget --quiet https://github.com/deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz && \
    tar xzf mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz && \
    mv mujoco-${MUJOCO_VERSION} mujoco && \
    rm -f mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz

ENV LD_LIBRARY_PATH=/opt/mujoco/lib:$LD_LIBRARY_PATH
ENV MUJOCO_ROOT_DIR=/opt/mujoco

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /workspace/mc_mujoco

ENV DEBIAN_FRONTEND=

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
