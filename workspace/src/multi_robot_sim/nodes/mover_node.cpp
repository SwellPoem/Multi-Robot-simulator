//Description: This node is used to control the robots in the simulation.
#include "definitions.h"
#include "world.h"
#include "robot.h"
#include "lidar.h"
#include "utils.h"

// callback function to store the received velocities
//called whenever a velocity message is recieved 
//prints the linear and angular velocities of the robot
void velocity_commandCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  ROS_INFO("Recieved velocity command:\n linear=%.2f\n angular=%.2f", msg->linear.x, msg->angular.z);
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "mover_node");    //initialization of ros node
  ros::NodeHandle nh("/");    //create a node handling for interacting 
 
  int NUM_ROBOTS = -1;
  NUM_ROBOTS = stoi(argv[1]); // get number of robots from command line

  //creates a vector of ros publishers, one for each robot that will pusblish messages
  vector<ros::Publisher> publishers_vector;
  vector<ros::Subscriber> subscribers_vector;
  for (int i=0; i < NUM_ROBOTS; i++) {
    ros::Publisher foo_pub  = nh.advertise<geometry_msgs::Twist>("robot_" + to_string(i) + "/velocity_command", 1000);
    ros::Subscriber foo_sub = nh.subscribe("robot_" + to_string(i) + "/velocity_command", 1000, velocity_commandCallback);
    publishers_vector.push_back(foo_pub);
    subscribers_vector.push_back(foo_sub);
  }

  //modifications to terminal settings in order to read character by character instead of line by line
  struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  bool select_robot = true; 
  int robot_idx = -1;
  bool clear = false;
  bool quit = false;

  double max_tv, max_rv;
  
  string input;
  geometry_msgs::Twist msg;
  ros::Rate rate(10);

  // Variables to keep track of current velocities
  double current_linear_velocity = 0.0;
  double current_angular_velocity = 0.0;

  //Main loop
  while (ros::ok()) {

    //quit chioice
    if (quit){
      cout << "Killing..." << endl; 
      break;
    }

    //robot selection
    if (select_robot) {
      // deactivate getchar terminal func
      tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

      if (clear) {
        system("clear"); // clear terminal screen
        clear = false;
      }

      while (true) {

        cout << "\nSelect a Robot inputing a number from 0 to " << NUM_ROBOTS-1 << endl; 
        cin >> input;

        //kill terminal condition
        if (input == "quit") killTerminal();

        try {
          robot_idx = stoi(input);
          //if number inserted is not valid, print message
          if (robot_idx < 0 || robot_idx > NUM_ROBOTS-1) {
            system("cls||clear"); 
            cout << "Invalid input." << endl;
          }
          else {
            //otherwise if robot selected is valid, set that robot as current robot to control
            //print message of selected robot
            cout << "\nRobot " << robot_idx << " selected" << "\n" << endl;
            // for(int i =0; i<6; i++) {
            //     cout << "\n";
            // }
            cout << "Move the robot with 'w', 'a', 's', 'd', and stop it with 'space'.\n" << endl;
            cout << "Press 'c' to change Robot or 'q' to quit.\n" << endl;

            //get the max linear and angular velocities of the selected robot
            //assumiong that the simulation_node is started befor the mover_node
            if (!nh.getParam("robot_" + to_string(robot_idx) + "/max_tv", max_tv)) {
              ROS_ERROR("Failed to get parameter max_tv");
            }
            if (!nh.getParam("robot_" + to_string(robot_idx) + "/max_rv", max_rv)) {
              ROS_ERROR("Failed to get parameter max_rv");
            }

            break;
          }
        }
        //if string inserted is not a number, print message
        catch (const exception& e) {
          system("cls||clear"); 
          cout << "Invalid input." << endl;
        }
      }
      select_robot = false;
      char none = getchar(); // just to ignore Enter signal
      tcsetattr(STDIN_FILENO, TCSANOW, &newt); // reactivate getchar term functionalities
    }

    //reset of linear and angular velocities of the selected robot to 0
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;

    //wait for 25 milliseconds
    this_thread::sleep_for(chrono::milliseconds(25));
    //first call to publish the message with the velocities of the selected robot
    //the msg object has been reset to 0 velocities
    //this stops the robot before the next command
    publishers_vector[robot_idx].publish(msg);
    ros::spinOnce();
 
    char ch = getchar(); 

    //wasd to move robot, c to change robot, q to quit
    switch (ch) {
      //forward
      case 'w':
        system("clear");
        cout << "forward\n";
        if (current_linear_velocity + 0.2 <= max_tv) {
          current_linear_velocity += 0.2;
        }
        cout.flush(); 
        break;
      
      //backward
      case 's': 
        system("clear"); 
        cout << "back\n"; 
        if (current_linear_velocity - 0.2 >= -max_tv) {
          current_linear_velocity -= 0.2;
        }
        cout.flush(); 
        break;
      
      //right
      case 'd': 
        system("clear"); 
        cout << "right\n"; 
        if (current_angular_velocity - 0.1 >= -max_rv) {
          current_angular_velocity -= 0.1;
        }
        cout.flush(); 
        break;
      
      //left
      case 'a': 
        system("clear"); 
        cout << "left\n"; 
        if (current_angular_velocity + 0.1 <= max_rv) {
          current_angular_velocity += 0.1;
        }
        cout.flush(); 
        break;
      
      //space to reset velocities to 0
      case ' ':
        system("clear");
        cout << "stop\n";
        current_linear_velocity = 0.0;
        current_angular_velocity = 0.0;
        cout.flush();
        break;
      
      //change robot
      case 'c': {
        system("clear"); 
        select_robot = true; 
        clear = true;
        break;
      }

      //close
      case 'q': {
        system("clear"); 
        cout << "Closing...\n"; cout.flush(); 
        quit = true;
        break;
      }
      default: 
        system("clear");
        cerr << "Invalid command: " << ch << endl;
        cout.flush();
        break;
    }

    // Update velocities
    msg.linear.x = current_linear_velocity;
    msg.angular.z = current_angular_velocity;
     
    //second call to publish the message with the velocities of the selected robot
    //the msg object has been updated with the new velocities
    //send new velocities to the robot
    publishers_vector[robot_idx].publish(msg);
    ros::spinOnce();

  }
  //restore terminal settings deactivating getchar func
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  killTerminal();
  return 0;
}
