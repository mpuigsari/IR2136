#!/bin/bash
set -e  # Exit if any command fails

# Source ROS environment
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

# Start ROS core in the background
roscore &

# Wait for ROS core to initialize
sleep 5

roslaunch rosbridge_server rosbridge_websocket.launch

# Keep container running
exec bash
