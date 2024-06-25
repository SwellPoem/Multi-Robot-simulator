//Description: This file contains all the necessary includes and definitions for the project.

#pragma once

#include <map>
#include <tuple>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include <ros/ros.h>

//for lidar_node.cpp
#include <termios.h>

//for simulation_node.cpp
#include <sys/time.h>
#include <thread>   //for multithreading
#include <chrono>   //for time functions
#include <cmath>

//for mover_node.cpp
#include <termios.h>    //for terminal settings
#include <unistd.h>   
#include <signal.h>   //for signal handling
#include <iomanip> // for std::fixed and std::setprecision

//for world.h
#include <string>
#include <vector>

//for utils.h
#include <optional>
#include <variant>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

//for lidar.h
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h> 

#include <geometry_msgs/TransformStamped.h>

//for robot.h
#include <multi_robot_sim/robot_odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>


#include <Eigen/Dense>

using namespace std;

using Point_Int = Eigen::Vector2i;
using Point = Eigen::Vector2f;
using Point3D = Eigen::Vector3f;
using Pose = Eigen::Isometry2f;
using Rot2D = Eigen::Rotation2Df;

const int TERMINAL_QUEUE_SIZE = 100;
const int POINT_CLOUD_QUEUE_SIZE = 1000;
const string PATH = "./..";
