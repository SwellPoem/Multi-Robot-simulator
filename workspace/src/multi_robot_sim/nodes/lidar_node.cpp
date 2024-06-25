// Description: This node subscribes to the point cloud data from multiple Lidars and prints the data of the selected Lidar.
#include "definitions.h"
#include "utils.h"

vector<pcl::PointCloud<pcl::PointXYZ>> lidar_clouds;
int selected_lidar = -1;

int counter = 0;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg, int lidar_id) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    lidar_clouds[lidar_id] = cloud;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_node");
    ros::NodeHandle nh;

    int NUM_LIDARS = -1;
    if (argc > 1) {
        NUM_LIDARS = stoi(argv[1]);
    } else {
        ROS_ERROR("Number of Lidars not specified. Exiting.");
        return -1;
    }

    ROS_INFO("Number of Lidars: %d", NUM_LIDARS);

    // Create a vector of subscribers, one for each Lidar
    vector<ros::Subscriber> subscribers_vector;
    lidar_clouds.resize(NUM_LIDARS);
    for (int i = 0; i < NUM_LIDARS; ++i) {
        string topic = "/robot_" + to_string(i) + "/base_scan";
        ROS_INFO("Subscribing to topic: %s", topic.c_str());
        ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(topic, POINT_CLOUD_QUEUE_SIZE, boost::bind(pointCloudCallback, _1, i));
        subscribers_vector.push_back(sub);
    }

    // Modifications to terminal settings in order to read character by character instead of line by line
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    bool select_lidar = true;
    bool clear = false;
    bool quit = false;

    string input;

    // Main loop
    while (ros::ok()) {

        //quit chioice
        if (quit) {
            cout << "Killing..." << endl;
            break;
        }

        //lidar selection
        if (select_lidar) {
            // deactivate getchar terminal func
            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

            if (clear) {
                system("clear");
                clear = false;
            }

            while (true) {
                cout << "\nSelect a Lidar by inputting a number from 0 to " << NUM_LIDARS - 1 << endl;
                cin >> input;

                //kill terminal condition
                if (input == "quit") killTerminal();

                try {
                    selected_lidar = stoi(input);
                    //if number inserted is not valid, print message
                    if (selected_lidar < 0 || selected_lidar > NUM_LIDARS - 1) {
                        system("cls||clear");
                        cout << "Invalid input." << endl;
                    } 
                    else {
                        //otherwise if lidar selected is valid, set that lidar as current pointCLoud to display
                        //print message of selected lidar
                        cout << "\nLidar " << selected_lidar << " selected\n" << endl;
                        cout << "Press 'c' to change Lidar, 'q' to quit or 'Enter' to print point cloud data.\n" << endl;
                        break;
                    }
                } catch (const exception& e) {
                    system("clear");
                    cout << "Invalid input." << endl;
                }
            }
            select_lidar = false;
            char none = getchar();
            tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        }

        this_thread::sleep_for(chrono::milliseconds(25));
        ros::spinOnce();

        char ch = getchar();

        switch (ch) {
            case '\n': {  // Enter key
                system("cls||clear");
                const auto& cloud = lidar_clouds[selected_lidar];
                cout << "Printing PointCloud for Lidar" << selected_lidar << endl;
                for (const auto& point : cloud.points) {
                    ROS_INFO("Point (x: %.2f, y: %.2f)", point.x, point.y);
                }
                break;
            }
            case 'c': {
                system("clear");
                select_lidar = true;
                clear = true;
                break;
            }
            case 'q': {
                system("clear");
                cout << "closing...\n";
                quit = true;
                break;
            }
            default:
                system("clear");
                cerr << "Invalid command: " << ch << endl;
                cout.flush();
                break;
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    killTerminal();
    return 0;
}

