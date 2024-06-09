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

#add custom message file and generate ncessary files
add_message_files(
    FILES
    robot_odometry.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

#specify the package dependencies
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


# Add simulation_node
add_executable(simulation_node nodes/simulation_node.cpp)
target_include_directories(simulation_node PUBLIC 
    include 
    ${catkin_INCLUDE_DIRS}    
    ${PCL_INCLUDE_DIRS}

)
target_link_libraries(simulation_node 
    multi_robot_sim
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

# Add mover_node
add_executable(mover_node nodes/mover_node.cpp)
target_include_directories(mover_node PUBLIC 
    include 
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(mover_node
    multi_robot_sim
    ${catkin_LIBRARIES}
)

