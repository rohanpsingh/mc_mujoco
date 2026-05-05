#!/bin/bash
set -e

# Write a default mc_rtc.yaml if the user hasn't provided one. Idempotent so
# that copying in a sample's preset (e.g. examples/grasp-fsm/etc/mc_rtc.in.yaml)
# survives subsequent `make run` invocations.
mkdir -p ~/.config/mc_rtc/
if [ ! -f ~/.config/mc_rtc/mc_rtc.yaml ]; then
  cat > ~/.config/mc_rtc/mc_rtc.yaml << EOF
MainRobot: JVRC1
Timestep: 0.002
Enabled: [Posture, EndEffector, CoM]
EOF
fi

# Source ROS setup
source /opt/ros/humble/setup.bash

# Execute the command passed to the container
exec "$@"
