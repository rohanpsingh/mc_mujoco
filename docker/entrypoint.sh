#!/bin/bash
set -e

# Create mc_rtc config directory if it doesn't exist
mkdir -p ~/.config/mc_rtc/

# Create mc_rtc.yaml with MainRobot configuration
cat > ~/.config/mc_rtc/mc_rtc.yaml << EOF
MainRobot: JVRC1
Timestep: 0.002
Enabled: [Posture, EndEffector, CoM, SampleNeckPolicy]
EOF

# Source ROS setup
source /opt/ros/humble/setup.bash

# Execute the command passed to the container
exec "$@"
