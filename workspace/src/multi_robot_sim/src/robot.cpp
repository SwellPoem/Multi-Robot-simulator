#include "robot.h"

//first constructor, it initializes the robot with the namespace, radius, max rotational velocity, max translational velocity, world and pose
Robot::Robot( string namespace_, float radius_, float max_rv_, float max_tv_, shared_ptr<World> w_, const Pose& pose_): 
      radius(radius_), 
      WorldItem(w_, pose_), 
      tv(0.0), 
      rv(0.0),
      max_rv(max_rv_), 
      max_tv(max_tv_), 
      odom_pub(nh.advertise<nav_msgs::Odometry>("/" + namespace_ + "/odom", TERMINAL_QUEUE_SIZE)),
      velocity_command_sub(nh.subscribe("/" + namespace_ + "/velocity_command", TERMINAL_QUEUE_SIZE, &Robot::velUpdate, this)) {}

//second constructor, it initializes the robot with the namespace, radius, max rotational velocity, max translational velocity, parent and pose
Robot::Robot( string namespace_, float radius_, float max_rv_, float max_tv_, shared_ptr<WorldItem> parent_, const Pose& pose_): 
      radius(radius_), 
      WorldItem(parent_, pose_), 
      tv(0.0), 
      rv(0.0),
      max_rv(max_rv_), 
      max_tv(max_tv_), 
      odom_topic("/" + namespace_ + "/odom"),
      velocity_command_topic("/" + namespace_ + "/velocity_command"),
      odom_pub(nh.advertise<nav_msgs::Odometry>(odom_topic, TERMINAL_QUEUE_SIZE)),
      velocity_command_sub(nh.subscribe(velocity_command_topic, TERMINAL_QUEUE_SIZE, &Robot::velUpdate, this)) {}

//draw function to visualize the robot
void Robot::draw() {
  int int_radius = radius * world->i_res;
  Point_Int p = world->world2grid(poseInWorld().translation());
  cv::circle(world->display_image, cv::Point(p.y(), p.x()), int_radius,
             cv::Scalar::all(0), -1);
}

//update the robot's pose based on the translational and rotational velocities
void Robot::timeTick(float dt) {
  Pose motion = Pose::Identity();
  motion.translation() << tv * dt, 0;
  motion.rotate(rv * dt);

  Pose next_pose = pose_in_parent * motion;
  Point_Int ip = world->world2grid(next_pose.translation());
  int int_radius = radius * world->i_res;
  if (!world->collides(ip, int_radius)) pose_in_parent = next_pose;

  // Translational component extraction
  Eigen::Vector2f msg_translation = pose_in_parent.translation();
  float msg_x = msg_translation.x();
  float msg_y = msg_translation.y();

  // Rotational component extraction
  Rot2D msg_rotation(pose_in_parent.linear());
  float msg_theta = msg_rotation.angle();

  // Publish odometry data
  multi_robot_sim::rodom odom;
  odom.x = msg_x;
  odom.y = msg_y;
  odom.theta = msg_theta;
  odom_pub.publish(odom);
}

//update the robot's translational and rotational velocities based on incoming commands
//velocities are not allowed to exceed the maximum values
//commands are of the type geometry_msgs::Twist, ROS message type to represent velocity in free space
//the method takes a constant pointer to a geometry_msgs::Twist message as argument
void Robot::velUpdate(const geometry_msgs::Twist::ConstPtr& msg) {
  if (msg->linear.x > max_tv) tv = max_tv;
  else tv = msg->linear.x;
  if (msg->angular.z > max_rv) rv = max_rv;
  else rv = msg->angular.z;

  printOdometry();
}

//print the robot's odometry
void Robot::printOdometry() {
  // translational component extraction
  Point msg_translation = pose_in_parent.translation();
  float msg_x = msg_translation.x();
  float msg_y = msg_translation.y();

  // rotational component extraction
  Rot2D msg_rotation(pose_in_parent.linear());
  float msg_theta = msg_rotation.angle();

  // print of odometry
  cout << "Odometry - X: " << msg_x << ", Y: " << msg_y << ", Theta: " << msg_theta << std::endl;
}

