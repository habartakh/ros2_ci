#!/bin/bash
set -e

# Source ROS2 workspace
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Launch Gazebo in the background
echo "Starting Gazebo..."
ros2 launch fastbot_gazebo one_fastbot_room.launch.py
GAZEBO_PID=$!

# Optionally sleep to allow Gazebo to load
sleep 5