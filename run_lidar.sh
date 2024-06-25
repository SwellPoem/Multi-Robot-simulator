#!/bin/bash

# Navigate to the workspace directory
cd "./workspace"

# Source the setup file
source /opt/ros/noetic/setup.bash

# Source the setup file
source ./devel/setup.bash

# Run the simulation node
rosrun multi_robot_sim lidar_node 3