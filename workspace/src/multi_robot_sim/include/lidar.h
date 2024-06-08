#pragma once

#include "definitions.h"
#include "world.h"

class Lidar : public WorldItem {
 public:
    //first constructor
    Lidar(string namespace_, float fov_, float max_range_, int num_rays_, shared_ptr<World> w, const Pose& pose_ = Pose::Identity());

    //second constructor
    Lidar(string namespace_, float fov_, float max_range_, int num_rays_, shared_ptr<WorldItem> p_, const Pose& pose_ = Pose::Identity());

    void timeTick(float dt) override;

    void draw() override;

    float fov, max_range;
    int num_rays;
    vector<float> ranges;

    // ROS
    ros::NodeHandle node_handler;  // ROS Node Handle
    ros::Publisher scan_publisher;  // Publisher to send odometry data
};
