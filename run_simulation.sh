#!/bin/bash

# Define cleanup procedure
cleanup() {
    echo "Caught Interrupt signal, cleaning up..."
    # Kill the background job if it's running
    kill $! 2>/dev/null
    exit 0
}

# Setup trap to call cleanup function when SIGINT is received
trap "cleanup" INT

# Navigate to the workspace directory
cd "./workspace"

# Source the setup file
source /opt/ros/noetic/setup.bash

# Build the project
catkin build

# Wait for the build to complete
wait

# Source the setup file
source ./devel/setup.bash

# Run the simulation node
rosrun multi_robot_sim simulation_node config.json &

# Wait indefinitely until receiving a signal to exit
wait