# Multi-Robot Simulator - robot programming project

## World and Objects
The world is represented by a grid and an image stored in the map folder. The image is loaded using OpenCV.<br />
In the world 3 robots and 3 lidars are present, represented as black circles with a series of rays navigating in the map.<br />
The robot and the lidar parameters are declared in the Json file called ``` config.json ```, and include several features like the maximum rotational and translational velocity for the robot, and the field of view aperture and the number of beams fdor the lidar. Robot and the lidars are all world items, so their classes are declared as WorldItem subclasses.

## ROS nodes
The ROS version used for the project is ROS Noetic.<br />
Two ROS nodes files are present:

``` simulation_node ``` : controls the representational part of the world, extracting the map from the Json file and loading it into a world object. From the Json file it also reads the properties of the objects, robots and lidars, creating and storing them in vectors of shared pointers. The main loop redraws the world advancing by a certain time step.

``` mover_node ``` : handles the control of the multiple robots. The user can choose the robot to control from command line based on the number of drones present in the simulation, and control their movement with the keyboard:
- ‘w a s d’ for forward, backward and rotation movement
- 'space bar' to stop the movement
- ‘c’ for changing the robot to control
- ‘q’ for exit the motion control

The print of the linear and angular velocity of the drones, and the odometry measurements are present in the terminal feeds, updating as the drones move.

## Structure
``` bash
├── robot_programming_project
    ├── config
    │    └── config.json
    ├── map
    │    └── mappa.png
    ├── workspace
    │    └── src
    │         ├── CMakeLists.txt ->
    │         └── multi_robot_sim
    │               ├── CMakeLists.txt
    │               ├── package.xml
    │               ├── nodes
    │               │    ├── mover_node.cpp
    │               │    └── simulation_node.cpp
    │               ├── include
    │               │    ├── lidar.h
    │               │    ├── robot.h
    │               │    ├── definitions.h
    │               │    ├── utils.h
    │               │    └── world.h
    │               ├── msg
    │               │    └── robot_odometry.msg
    │               └── src
    │                    ├── lidar.cpp
    │                    ├── robot.cpp
    │                    ├── utils.cpp
    │                    └── world.cpp
    └── README.md
```

## How to use
Start Master:
In bash nr. 1:
```code
source /opt/ros/noetic/setup.bash
roscore
```
How to compile:
In bash nr. 2:
```code
cd ./workspace
source /opt/ros/noetic/setup.bash
catkin build
source ./devel/setup.bash
```
To start the sumulation node:
In bash nr. 2:
```code
rosrun multi_robot_sim simulation_node config.json
```
To start the mover node:
In bash nr. 3:
```code
cd ./workspace
source /opt/ros/noetic/setup.bash
source ./devel/setup.bash
rosrun multi_robot_sim mover_node 'NUM_ROBOTS'
```

