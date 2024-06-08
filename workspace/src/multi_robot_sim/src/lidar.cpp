#include "lidar.h"

//first constructor, it initializes the lidar with the namespace, field of view, max range, number of rays, world and pose
Lidar::Lidar( string namespace_, float fov_, float max_range_, int num_rays_, shared_ptr<World> w, const Pose& pose_):
      WorldItem(w, pose_),
      fov(fov_),
      max_range(max_range_),
      num_rays(num_rays_),
      ranges(num_rays, -1.0),
      //create a publisher to publish messages of type PointCloud2
      //the topic is the namespace + scan
      scan_publisher(node_handler.advertise<sensor_msgs::PointCloud2>("/" +namespace_+ "/" +"scan", POINT_CLOUD_QUEUE_SIZE)) {}

//second constructor, it initializes the lidar with the namespace, field of view, max range, number of rays, parent and pose
Lidar::Lidar( string namespace_, float fov_, float max_range_, int num_rays_, shared_ptr<WorldItem> p_, const Pose& pose_):
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

  //for each ray, calculate the range value
  for (int i = 0; i < num_rays; ++i) {
    Point_Int endpoint;
    ranges[i] = max_range;
    bool result = world->traverseRay(endpoint, origin, alpha, int_range);
    if (result) {
      Point_Int delta = endpoint - origin;
      ranges[i] = delta.norm() * world->res;
    }
    alpha += ang_resolution;
  }
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
};
