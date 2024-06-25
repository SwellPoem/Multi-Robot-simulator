//Description: This file contains the function definitions for the utility functions used in the project.
#include "definitions.h"
#include "world.h"
#include "robot.h"
#include "lidar.h"

#include <jsoncpp/json/json.h> 

using WorldPointer = World*;

// Functions definitions
void killTerminal();
int runShellCommand(string command);
Json::Value readJson(const string& in_path);
