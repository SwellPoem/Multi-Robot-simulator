//Description: Lidar class implementation file.
#include "lidar.h"

//constructor, it initializes the lidar with the namespace, field of view, max range, number of rays, parent and pose
Lidar::Lidar( string namespace_, float fov_, float max_range_, int num_rays_, WorldItem* p_, const Pose& pose_):
      WorldItem(p_, pose_),
      fov(fov_),
      max_range(max_range_),
      num_rays(num_rays_),
      ranges(num_rays, -1.0),
      //create a publisher to publish messages of type PointCloud2
      //the topic is the namespace + scan
      scan_publisher(node_handler.advertise<sensor_msgs::PointCloud2>("/" +namespace_+ "/" +"base_scan", POINT_CLOUD_QUEUE_SIZE)) {}


//update the Lidar sensor readings by calculating the range values for each ray based on the Lidar sensor's pose and the obstacles in the world.
void Lidar::timeTick(float dt) {
  Pose piw = poseInWorld(); //pose of the lidar in the world
  Point_Int origin = world->world2grid(piw.translation()); //lidar position from world coordinates to grid coordinates
  
  if (!world->inside(origin)) return;

  float ang_resolution = fov / num_rays;  //angle between each ray
  float alpha = Rot2D(piw.linear()).angle() - fov / 2; //starting angle for the firts ray
  float int_range = max_range * world->i_res; //max range of the lidar
  
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.header.frame_id = "lidar_frame";

  //for each ray, calculate the range value
  for (int i = 0; i < num_rays; ++i) {
    Point_Int endpoint; //end point of the ray
    ranges[i] = max_range;  //range of the current ray = max_range
    //traverseRay call: update of the end point and retunr true if obstacle is hit
    bool result = world->traverseRay(endpoint, origin, alpha, int_range);
    //if an obstacle is hit
    if (result) {
      Point_Int delta = endpoint - origin;   //range to the obstacle
      ranges[i] = delta.norm() * world->res;  //update the range value
    }
    //endpoint grid coordinates to world coordinates
    Point p_world = world->grid2world(endpoint);

    //add the endpoint to the point cloud
    cloud.points.push_back(pcl::PointXYZ(p_world.x(), p_world.y(), 0));
    
    //prepare alpha for the next ray
    alpha += ang_resolution;
  }
  
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(cloud, output);
  scan_publisher.publish(output);
}

//visualize the lidar readings
void Lidar::draw() {
  Pose piw = poseInWorld();
  Point_Int origin = world->world2grid(piw.translation());

  if (!world->inside(origin)) return;

  float ang_resolution = fov / num_rays;
  float alpha = -fov / 2;
  for (int i = 0; i < num_rays; ++i) {
    float r = ranges[i];
    Point p_lidar(r * cos(alpha), r * sin(alpha));
    Point p_world = piw * p_lidar;
    Point_Int epi = world->world2grid(p_world);
    //opencv function to draw a line
    //image where to draw, start point, end point, color, thickness
    cv::line(world->display_image, cv::Point(origin.y(), origin.x()), cv::Point(epi.y(), epi.x()), cv::Scalar(200, 200, 200), 1);
    alpha += ang_resolution;
  }
}
