#include "definitions.h"
#include "world.h"
#include "robot.h"
#include "lidar.h"

#include <jsoncpp/json/json.h> 

using WorldPointer = shared_ptr<World>;

// Functions definitions
void killTerminal();
void clearTerminal();
int runShellCommand(string command);
Json::Value readJson(string in_path);
