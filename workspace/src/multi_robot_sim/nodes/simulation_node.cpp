//Description: This file is the main file for the simulation node.
#include "definitions.h"
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

  ros::init(argc, argv, "simulation_node");  //initialize of ros node
  ros::NodeHandle nh("/");  //create a node handling for interacting

  const string CONFIG_PATH = PATH + "/config/" + argv[1];

  //configuration file expected to be in config folder
  //expected to be a JSON file
  Json::Value root = readJson(CONFIG_PATH);
  string map_name = root["map"].asString();   //the map name is read from the JSON file
  cout << "Map -> " << map_name << endl;
  string IMAGE_PATH = PATH + "/map/" +  map_name;

//create the world object
  World world(1);  //
  WorldPointer world_ptr(&world, [](World*){ });   //creating a shared pointer to the world object => &world is the object, [](World*){} is the deleter
  world.loadFromImage(IMAGE_PATH);  //load the specified image into the world object

  int NUM_ROBOTS = 0;

  //vectors to store robots and lidars
  vector<shared_ptr<Robot>> robots; 
  vector<shared_ptr<Lidar>> lidars;
  //map to store robot id and robot object
  map<int, shared_ptr<Robot>> id2robots;

  
  //Iterate over the items in the JSON file
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
      // read robot parameters and create a new robot object
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

      //if the robot has a parent, create a new robot object with the parent as the parent
      if (id_parent != -1){
        shared_ptr<Robot> parent_ptr= id2robots[id_parent]; 
        Pose parent_pose = parent_ptr->poseInWorld();
        r = new Robot(namespace_, radius, max_rv, max_tv, parent_ptr, parent_pose); 
      }
      //if the robot does not have a parent, create a new robot object with the world as the parent
      else {
        r = new Robot(namespace_, radius, max_rv, max_tv, world_ptr, robot_pose);
      }
      shared_ptr<Robot> r_(r, [](Robot* r){ });   //create a shared pointer to manage the memory of the robot obj => r is the object, [](Robot*){} is the deleter
      id2robots[id] = r_;   //add the shared pointer to the map => to access it by it's position in the vector
      robots.push_back(r_);   //add the shared pointer to the vector
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

      //if the lidar has a parent, create a new lidar object with the parent as the parent
      if (id_parent != -1){
        shared_ptr<Robot> parent_ptr= id2robots[id_parent]; 
        l = new Lidar(namespace_, fov, max_range_l, num_rays_l, parent_ptr, lidar_pose);
      }
      //if the lidar does not have a parent, create a new lidar object with the world as the parent
      else {
        l = new Lidar(namespace_, fov, max_range_l, num_rays_l, world_ptr, lidar_pose);
      }

      shared_ptr<Lidar> l_(l, [](Lidar* l){ });   //create a shared pointer to manage the memory of the lidar
      lidars.push_back(l_);   //add the shared pointer to the vector
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
    if (k == 27) break;  //esc key to exit

    //to handle callbacks
    ros::spinOnce();
    world.timeTick(wait_time);    //advance the world by wait_time
    this_thread::sleep_for(chrono::milliseconds(10));

  }

  cv::destroyAllWindows();
  return 0;
} 
