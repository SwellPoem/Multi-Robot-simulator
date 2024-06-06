cmake_minimum_required(VERSION 3.16)
project(multi_robot_sim)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(jsoncpp REQUIRED)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    # required ROS packages to use in this project
    roscpp
    tf2
    tf2_ros
    tf2_eigen
    std_msgs
    pcl_ros
    pcl_conversions
    sensor_msgs
    geometry_msgs
    message_generation
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    INCLUDE_DIRS "include/"
    CATKIN_DEPENDS  message_runtime 
    roscpp
    tf2 
    tf2_ros 
    tf2_eigen
)    


add_library(multi_robot_sim src/robot.cpp src/world.cpp src/lidar.cpp src/utils.cpp) 
target_include_directories(multi_robot_sim PUBLIC 
    include 
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(multi_robot_sim PUBLIC 
    ${OpenCV_LIBRARIES} 
    Eigen3::Eigen
    ${catkin_LIBRARIES}
    jsoncpp_lib
    
)

