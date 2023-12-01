#!/bin/bash

grep -qxF "$export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/workspaces/ros_ws/src/basic_mobile_robot/models/" ~/.bashrc || echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/workspaces/ros_ws/src/basic_mobile_robot/models/" >> ~/.bashrc 
source ~/.bashrc

# Script to build, setup, and launch ROS2 package
cd src
# Run colcon build
colcon build
# Source the setup.bash file
source install/setup.bash
# Launch the ROS2 package
ros2 launch final_demo final_demo.launch.py