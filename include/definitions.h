#pragma once

#include <map>
#include <tuple>
#include <string>
#include <vector>
#include <cstdio>
#include <fstream>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Dense>

using namespace std;

using Point_Int = Eigen::Vector2i;
using Point = Eigen::Vector2f;
using Point3D = Eigen::Vector3f;
using Pose = Eigen::Isometry2f;
using Rot2D = Eigen::Rotation2Df;

const int TERMINAL_QUEUE_SIZE = 100;
const string PATH = "./..";
