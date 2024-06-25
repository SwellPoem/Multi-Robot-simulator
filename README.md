# Multi-Robot Simulator - robot programming project

## World and Objects
The world is represented by a grid and an image stored in the map folder. The image is loaded using OpenCV.<br />
In the world 3 robots and 3 lidars are present, represented as black circles with a series of rays navigating in the map.<br />
The robot and the lidar parameters are declared in the Json file called ``` config.json ```, and include several features like the maximum rotational and translational velocity for the robot, and the field of view aperture and the number of beams for the lidar. Robot and the lidars are all world items, so their classes are declared as WorldItem subclasses.

## ROS nodes
The ROS version used for the project is ROS Noetic.<br />
Three ROS nodes files are present:

``` simulation_node ``` : controls the representational part of the world, extracting the map from the Json file and loading it into a world object. From the Json file it also reads the properties of the objects, robots and lidars, creating and storing them in vectors of shared pointers. The main loop redraws the world advancing by a certain time step.

``` mover_node ``` : handles the control of the multiple robots. The user can choose the robot to control from command line based on the number of drones present in the simulation, and control their movement with the keyboard:
- ‘w a s d’ for forward, backward and rotation movement
- 'space bar' to stop the movement
- ‘c’ for changing the robot to control
- ‘q’ for exit the motion control

``` lidar_node ``` : handles the print of the point cloud data retrieved from the lidars. The prints are obtained selecting the lidar of interest in the terminal feed and pressing the 'enter' key to display.

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
    │               │    ├── lidar_node.cpp
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
    ├──open_all.sh
    ├──run_lidar.sh
    ├──run_mover.sh
    ├──run_simulation.sh
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
In bash nr. 2 in the folder ``` robot_programming_project ``` :
```code
chmod +x open_all.sh
./open_all.sh
```
The ``` open_all.sh ``` file allows the execution of other three bash scripts, ``` run_simulation.sh ```, ``` run_mover.sh ``` and ``` run_lidar.sh ```, that start the thee ROS nodes of the project.
