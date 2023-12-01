#!/bin/bash

# Script to build, setup, and launch ROS2 package
cd src

# Run colcon build
colcon build

# Source the setup.bash file
source install/setup.bash

# Launch the ROS2 package
ros2 launch final_demo final_demo.launch.py