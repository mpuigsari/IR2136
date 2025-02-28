#!/bin/bash
set -e

# Ensure NVIDIA OpenGL works
export LIBGL_ALWAYS_INDIRECT=0
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# Fix XDG_RUNTIME_DIR warning
export XDG_RUNTIME_DIR=/tmp/runtime-root
mkdir -p $XDG_RUNTIME_DIR
chmod 0700 $XDG_RUNTIME_DIR

# Source ROS setup
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

# Start ROS Bridge in the background
echo "Starting ROS Bridge..."
roslaunch rosbridge_server rosbridge_websocket.launch &

# Keep the container running
exec "$@"

