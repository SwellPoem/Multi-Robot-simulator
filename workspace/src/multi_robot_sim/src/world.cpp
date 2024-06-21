//Description: Implementation of the World and WorldItem classes.,
#include "world.h"

/////////////World class/////////////

World::World(int id) {_id = id;}

//load an image from a file to initialize the world grid
//convert it to grayscale, and store its pixel data in a 1D grid
void World::loadFromImage(const string filename_) {
  cv::Mat frame = cv::imread(filename_);  //store the image read in a cv::Mat object -> opencv stuff
  if (frame.rows == 0) {
    throw runtime_error("unable to load image");
  }
  cv::cvtColor(frame, display_image, cv::COLOR_BGR2GRAY); //need it grayscale
  size = display_image.rows * display_image.cols; //size of the image in pixels
  rows = display_image.rows;  
  cols = display_image.cols;
  _grid = vector<uint8_t>(size, 0x00); //initialize the grid with 0x00
  memcpy(_grid.data(), display_image.data, size);
}

void World::draw() {
  for (const auto item : _items) item->draw();
  cv::imshow("Map", display_image);
  // Clean the display image
  memcpy(display_image.data, _grid.data(), size);
}

void World::timeTick(float dt) {
  for (const auto item : _items) item->timeTick(dt);
}

void World::add(WorldItem* item) { _items.push_back(item); }

//simulation of ray traveling in the world
//the ray stops when it hits an obstacle or reaches the maximum range
bool World::traverseRay(Point_Int& endpoint, const Point_Int& origin, const float angle, const int max_range) {
  Point p0 = origin.cast<float>();    //start point of the ray cast to float
  const Point dp(cos(angle), sin(angle));   //direction of the ray
  int range_to_go = max_range;
  while (range_to_go > 0) {
    endpoint = Point_Int(p0.x(), p0.y());   //set the endpoint to the integer coordinates of the current point
    if (!inside(endpoint)) return false;    //endpoint must be inside the world
    if (at(endpoint) < 127) return true;    //if the value at the endpoint is less than 127, it is an obstacle
    p0 = p0 + dp;
    --range_to_go;    //reduce the remaining range
  }
  return true;
}

bool World::checkCollision(const Point_Int& p, const int radius) const {
  if (!inside(p)) return true;    //point must be inside the world
  int r2 = radius * radius;
  //itearate over a square of side 2*radius centered at p
  for (int r = -radius; r <= radius; ++r) {
    for (int c = -radius; c <= radius; ++c) {
      Point_Int off(r, c);    //point offset
      if (off.squaredNorm() > r2) continue;
      Point_Int p_test = p + Point_Int(r, c);   //calculate the point to test adding the offset to p
      if (!inside(p_test)) return true;   //point to test must be inside the world

      if (at(p_test) < 127) return true;    //collision detected
    }
  }
  return false;
}


////////WorldItem class/////////////

WorldItem::WorldItem(World* w_, const Pose& p_)
    : world(w_), parent(nullptr), pose_in_parent(p_) {
  if (world) world->add(this);
}

WorldItem::WorldItem(WorldItem* parent_, const Pose& p)
    : world(parent_->world), parent(parent_), pose_in_parent(p) {
  if (world) world->add(this);
}

Pose WorldItem::poseInWorld() {
  if (!parent) return pose_in_parent;
  return parent->poseInWorld() * pose_in_parent;
}

WorldItem::~WorldItem() {}
