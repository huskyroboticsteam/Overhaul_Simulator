#!/bin/bash
# shellcheck disable=SC1090,SC1091
set -e

source /opt/ros/"$ROS_DISTRO"/setup.bash --
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash --
fi

# GUI apps (Gazebo, RViz) render into the VNC desktop; set START_VNC=0 to run headless
if [ "${START_VNC:-1}" = "1" ]; then
    start-desktop
fi

exec "$@"
