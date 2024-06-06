#include "lidar.h"

#include <opencv2/imgproc.hpp>

#include "definitions.h"

//first constructor, it initializes the lidar with the namespace, field of view, max range, number of beams, world and pose
Lidar::Lidar( string namespace_, float fov_, float max_range_, int num_beams_, shared_ptr<World> w, const Pose& pose_):
      WorldItem(w, pose_),
      fov(fov_),
      max_range(max_range_),
      num_beams(num_beams_),
      ranges(num_beams, -1.0),
      scan_publisher(node_handler.advertise<sensor_msgs::PointCloud2>("/" +namespace_+ "/" +"scan", 1000)) {}

//second constructor, it initializes the lidar with the namespace, field of view, max range, number of beams, parent and pose
Lidar::Lidar( string namespace_, float fov_, float max_range_, int num_beams_, shared_ptr<WorldItem> p_, const Pose& pose_):
      WorldItem(p_, pose_),
      fov(fov_),
      max_range(max_range_),
      num_beams(num_beams_),
      ranges(num_beams, -1.0),
      scan_publisher(node_handler.advertise<sensor_msgs::PointCloud2>("/" +namespace_+ "/" +"base_scan", 1000)) {}


//update the Lidar sensor readings by calculating the range values for each beam based on the Lidar sensor's pose and the obstacles in the world.
void Lidar::timeTick(float dt) {
  Pose piw = poseInWorld(); //pose of the lidar in the world
  Point_Int origin = world->world2grid(piw.translation()); //lidar position from world coordinates to grid coordinates
  
  if (!world->inside(origin)) return;

  float d_alpha = fov / num_beams;  //angle between each beam
  float alpha = Rot2D(piw.linear()).angle() - fov / 2; //starting angle for the firts beam
  float int_range = max_range * world->i_res; //max range of the lidar

  //for each beam, calculate the range value
  for (int i = 0; i < num_beams; ++i) {
    Point_Int endpoint;
    ranges[i] = max_range;
    bool result = world->traverseBeam(endpoint, origin, alpha, int_range);
    if (result) {
      Point_Int delta = endpoint - origin;
      ranges[i] = delta.norm() * world->res;
    }
    alpha += d_alpha;
  }
}

//visualize the lidar readings
void Lidar::draw() {
  Pose piw = poseInWorld();
  Point_Int origin = world->world2grid(piw.translation());

  if (!world->inside(origin)) return;

  float d_alpha = fov / num_beams;
  float alpha = -fov / 2;
  for (int i = 0; i < num_beams; ++i) {
    float r = ranges[i];
    Point p_lidar(r * cos(alpha), r * sin(alpha));
    Point p_world = piw * p_lidar;
    Point_Int epi = world->world2grid(p_world);
    cv::line(world->display_image, cv::Point(origin.y(), origin.x()),
             cv::Point(epi.y(), epi.x()), cv::Scalar(127, 127, 127), 1);
    alpha += d_alpha;
  }
};
