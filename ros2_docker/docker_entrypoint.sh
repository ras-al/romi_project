#!/bin/bash
set -e

# Source ROS2 setup
source /opt/ros/humble/setup.bash

# Set Gazebo Fortress environment variables
export GZ_VERSION=fortress
# Append to existing GZ_SIM_RESOURCE_PATH if set, otherwise just set it
if [ -n "${GZ_SIM_RESOURCE_PATH}" ]; then
    export GZ_SIM_RESOURCE_PATH=/usr/share/gz-fortress:${GZ_SIM_RESOURCE_PATH}
else
    export GZ_SIM_RESOURCE_PATH=/usr/share/gz-fortress
fi

# Set Gazebo system plugin path to include ROS2 plugins
if [ -n "${GZ_SIM_SYSTEM_PLUGIN_PATH}" ]; then
    export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}
else
    export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib
fi

# If a workspace exists, source it
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Execute the command passed to the container
exec "$@"

