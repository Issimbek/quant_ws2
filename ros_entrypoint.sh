#!/bin/bash
set -e

# Setup ROS2 environment
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Source workspace if it exists
if [ -f "${ROS_WS}/install/setup.bash" ]; then
    source "${ROS_WS}/install/setup.bash"
fi

exec "$@"
