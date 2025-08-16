#!/bin/bash
set -e

# Source ROS2 workspace
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Launch Gazebo in the background
echo "Starting Gazebo..."
ros2 launch fastbot_gazebo one_fastbot_room.launch.py &
GAZEBO_PID=$!

# sleep to allow Gazebo to load
sleep 35

# Start the ROS 2 action server
echo "Launching Fastbot Waypoints action server..."
ros2 run fastbot_waypoints fastbot_action_server &
SERVICE_PID=$!

# Wait a bit before starting test
sleep 8

# Run the test file 
echo "Running test file..."
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
TEST_PID=$?


# After test finishes, shut everything down
echo "Shutting down Gazebo and service server..."
sleep 5
kill $SERVICE_PID || true
kill $GAZEBO_PID || true

# Exit with the result of the test
exit $TEST_RESULT

