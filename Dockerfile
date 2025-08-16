##  This Docker image will contain everything necessary for starting the Gazebo simulation in ROS2
# Base image
FROM osrf/ros:humble-desktop

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]

# Install Gazebo 11 and other dependencies
RUN apt-get update && apt-get install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-robot-localization \
  ros-humble-xacro \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions \
  libxcb-xinerama0 \
  && rm -rf /var/lib/apt/lists/*

# Create workspace and download simulation repository
RUN source /opt/ros/humble/setup.bash \
 && mkdir -p /ros2_ws/src \
 && cd /ros2_ws/src 

# Copy the necessary package files to spawn the robot in the simulation
COPY ./fastbot_description/ /ros2_ws/src/fastbot_description
COPY ./fastbot_gazebo/ /ros2_ws/src/fastbot_gazebo
COPY ./fastbot_waypoints/ /ros2_ws/src/fastbot_waypoints
COPY ./entrypoint.sh /ros2_ws/ros2_entrypoint.sh

# Build the Colcon workspace and ensure it's sourced
RUN source /opt/ros/humble/setup.bash \
 && cd /ros2_ws \
 && colcon build

RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Gazebo is unable to detect the fastbot meshes even with an updated GAZEBO_MODEL_PATH
# so we just copy them inside the models of Gazebo
COPY ./fastbot_description/ /usr/share/gazebo-11/models/fastbot_description

# Set up a workspace directory
WORKDIR /ros2_ws/

# Set environment variables
ENV DISPLAY=:1
ENV ROS_DOMAIN_ID=1
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/ros2_ws/src/
ENV GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}

# Start a bash shell when the container starts
CMD ["/bin/bash", "-c","bash ros2_entrypoint.sh"]