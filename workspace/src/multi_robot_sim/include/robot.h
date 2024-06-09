//Description: Header file for the Robot class.
#pragma once

#include "definitions.h"
#include "world.h"

struct Robot : public WorldItem {

      //first constructor
      Robot(string namespace_, float radius_, float max_rv_, float max_tv_, shared_ptr<World> w_, const Pose& pose_ = Pose::Identity());

      //second constructor
      Robot(string namespace_, float radius_, float max_rv_, float max_tv_, shared_ptr<WorldItem> parent_, const Pose& pose_ = Pose::Identity());

      void draw() override;

      void timeTick(float dt) override;
      
      //function to update robot velocity
      void velUpdate(const geometry_msgs::Twist::ConstPtr& msg);
      
      //function to print the odometry data
      void printOdometry();

      float radius;
      float tv = 0, rv = 0;
      float max_rv, max_tv;

      string velocity_command_topic;
      string odom_topic;

      // ROS
      ros::NodeHandle nh;  // ROS Node Handle
      ros::Publisher odom_pub;  // Publisher to send odometry data
      ros::Subscriber velocity_command_sub;  // Subscriber to receive velocity commands
}; 
