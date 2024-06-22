//Description: This file is the main file for the simulation node.#include "definitions.h"

#include "world.h"
#include "robot.h"
#include "lidar.h"
#include "utils.h"

int main(int argc, char** argv) {

  //if no config file is provided as a command line argument, print error message and exit
  if (argc < 2) {
    cerr << "Error: no config.jsosn file provided:\n" << endl;
    return 1;
  }

  ros::init(argc, argv, "simulation_node");
  ros::NodeHandle nh("/");

  const string CONFIG_PATH = PATH + "/config/" + argv[1];

  //configuration file expected to be in config folder
  //expected to be a JSON file
  Json::Value root = readJson(CONFIG_PATH);
  string map_name = root["map"].asString();
  cout << "Map -> " << map_name << endl;
  string IMAGE_PATH = PATH + "/map/" +  map_name;

  //create the world object
  World world(1);
  World* world_ptr = &world;
  world.loadFromImage(IMAGE_PATH);	//load the specified image into the world object

  int NUM_ROBOTS = 0;

  //vectors to store robots and lidars
  vector<Robot*> robots;
  vector<Lidar*> lidars;
  //map to store robot id and robot object
  map<int, Robot*> id2robots;

  //iterate over the items in the json file
  for(const Json::Value& item: root["items"]) {

    //read the id, type, namespace, pose, parent from the JSON file
    const int id = item["id"].asInt();
    const  string type = item["type"].asString();
    const string namespace_ = item["namespace"].asString();

    double pose_x = item["pose"][0].asInt();
    double pose_y = item["pose"][1].asInt();
    double theta = item["pose"][2].asDouble();

    const int id_parent = item["parent"].asInt(); 

    //robot type
    if (type == "robot") {
      //read robot parameters and create a new robot object
      double radius = item["radius"].asDouble();
      float max_rv = item["max_rv"].asFloat();
      float max_tv = item["max_tv"].asFloat();
      
      //create a pose object to represent the robot pose
      //convert the position from grid coordinates to world coordinates
      //orientation is set to a 2D rotation matrix from the angle theta
      Pose robot_pose = Pose::Identity();
      robot_pose.translation() = world_ptr->grid2world(Point_Int(pose_x, pose_y));
      robot_pose.linear() = Rot2D(theta).matrix();
      Robot* r;

      //the robot usually doesn't have a parent
      //create a new robot object with the world as a parent
      r = new Robot(namespace_, radius, max_rv, max_tv, world_ptr, robot_pose);
      
      //add the robot to the map and to the vector updating the number of robots in the simulation
      id2robots[id] = r;
      robots.push_back(r);
      NUM_ROBOTS++;
      
      ros::NodeHandle nh;
      nh.setParam(namespace_ + "/max_tv", max_tv);
      nh.setParam(namespace_ + "/max_rv", max_rv);
    }
    //lidar type
    else {
      //read the id, type, namespace, pose, parent from the JSON file
      float fov = item["fov"].asFloat();
      double max_range_l = item["max_range"].asDouble();
      int num_rays_l = item["num_rays"].asInt();
  
      //create a pose object to represent the lidar pose
      //convert the position from grid coordinates to world coordinates
      //orientation is set to a 2D rotation matrix from the angle theta
      Pose lidar_pose = Pose::Identity();
      lidar_pose.translation() = world_ptr->grid2world(Point_Int(pose_x, pose_y));
      lidar_pose.linear() = Rot2D(theta).matrix();
      Lidar* l;

      //the lidar ususlly has a parent
      //create a new object lidar with the parent as the parent
      Robot* parent_ptr= id2robots[id_parent]; 
      l = new Lidar(namespace_, fov, max_range_l, num_rays_l, parent_ptr, lidar_pose);

      //add the lidart to the vector
      lidars.push_back(l);
    }
  }
 
  //draw the initial state
  world.draw();
  cv::waitKey(1);

  float wait_time = 0.1;
  ros::Rate loop_rate(10);

  cout << "Running primary node" << endl;

  //main loop
  while (ros::ok()) {

    //redraw the world
    world.draw();
    int k = cv::waitKey(1);
    if (k == 27) break;

    ros::spinOnce();	//handle the callbacks
    world.timeTick(wait_time);	//advance the world by wait time
    this_thread::sleep_for(chrono::milliseconds(10));

  }

  cv::destroyAllWindows();

  // Delete all dynamically allocated memory
  for (auto robot : robots) {
    delete robot;
  }
  for (auto lidar : lidars) {
    delete lidar;
  }

  return 0;
} 
