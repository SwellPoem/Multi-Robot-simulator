//Description: This file is the main file for the simulation node.#include "definitions.h"

#include "world.h"
#include "robot.h"
#include "lidar.h"
#include "utils.h"

int main(int argc, char** argv) {

  if (argc < 2) {
    cerr << "Error: no config.jsosn file provided:\n" << endl;
    return 1;
  }

  ros::init(argc, argv, "simulation_node");
  ros::NodeHandle nh("/");

  const string CONFIG_PATH = PATH + "/config/" + argv[1];

  Json::Value root = readJson(CONFIG_PATH);
  string map_name = root["map"].asString();
  cout << "Map -> " << map_name << endl;
  string IMAGE_PATH = PATH + "/map/" +  map_name;

  World world(1);
  World* world_ptr = &world;
  world.loadFromImage(IMAGE_PATH);

  int NUM_ROBOTS = 0;

  vector<Robot*> robots;
  vector<Lidar*> lidars;
  map<int, Robot*> id2robots;

  for(const Json::Value& item: root["items"]) {

    const int id = item["id"].asInt();
    const  string type = item["type"].asString();
    const string namespace_ = item["namespace"].asString();

    double pose_x = item["pose"][0].asInt();
    double pose_y = item["pose"][1].asInt();
    double theta = item["pose"][2].asDouble();

    const int id_parent = item["parent"].asInt(); 

    if (type == "robot") {
      double radius = item["radius"].asDouble();
      float max_rv = item["max_rv"].asFloat();
      float max_tv = item["max_tv"].asFloat();
      
      Pose robot_pose = Pose::Identity();
      robot_pose.translation() = world_ptr->grid2world(Point_Int(pose_x, pose_y));
      robot_pose.linear() = Rot2D(theta).matrix();
      Robot* r;

      if (id_parent != -1){
        Robot* parent_ptr= id2robots[id_parent]; 
        Pose parent_pose = parent_ptr->poseInWorld();
        r = new Robot(namespace_, radius, max_rv, max_tv, parent_ptr, parent_pose); 
      }
      else {
        r = new Robot(namespace_, radius, max_rv, max_tv, world_ptr, robot_pose);
      }
      id2robots[id] = r;
      robots.push_back(r);
      NUM_ROBOTS++;
      ros::NodeHandle nh;
      nh.setParam(namespace_ + "/max_tv", max_tv);
      nh.setParam(namespace_ + "/max_rv", max_rv);
    }
    else {
      float fov = item["fov"].asFloat();
      double max_range_l = item["max_range"].asDouble();
      int num_rays_l = item["num_rays"].asInt();
  
      Pose lidar_pose = Pose::Identity();
      lidar_pose.translation() = world_ptr->grid2world(Point_Int(pose_x, pose_y));
      lidar_pose.linear() = Rot2D(theta).matrix();
      Lidar* l;

      if (id_parent != -1){
        Robot* parent_ptr= id2robots[id_parent]; 
        l = new Lidar(namespace_, fov, max_range_l, num_rays_l, parent_ptr, lidar_pose);
      }
      else {
        l = new Lidar(namespace_, fov, max_range_l, num_rays_l, world_ptr, lidar_pose);
      }

      lidars.push_back(l);
    }
  }
 
  world.draw();
  cv::waitKey(1);

  float wait_time = 0.1;
  ros::Rate loop_rate(10);

  cout << "Running primary node" << endl;

  while (ros::ok()) {

    world.draw();
    int k = cv::waitKey(1);
    if (k == 27) break;

    ros::spinOnce();
    world.timeTick(wait_time);
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
