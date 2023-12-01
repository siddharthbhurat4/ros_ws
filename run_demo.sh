#!/bin/bash

grep -qxF "$export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/workspaces/ros_ws/src/basic_mobile_robot/models/" ~/.bashrc || echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/workspaces/ros_ws/src/basic_mobile_robot/models/" >> ~/.bashrc 
source ~/.bashrc

# Run colcon build
colcon build
 
# Get the current directory
current_directory=$(pwd)

# Source directories
controller_source="/workspaces/ros_ws/install/mobile_robot_localization/lib/mobile_robot_localization/controller_node"
localization_source="/workspaces/ros_ws/install/mobile_robot_localization/lib/mobile_robot_localization/localization_node"

# Destination directory
destination_directory="/workspaces/ros_ws/src/install/mobile_robot_localization/lib/mobile_robot_localization"


# Copy files
cp "$controller_source" "$destination_directory"
cp "$localization_source" "$destination_directory"

# # Script to build, setup, and launch ROS2 package

# Source the setup.bash file
source install/setup.bash

export GAZEBO_MODEL_PATH=$(grep 'export GAZEBO_MODEL_PATH=' ~/.bashrc | cut -d'=' -f2)
# Launch the ROS2 package
ros2 launch final_demo final_demo.launch.py