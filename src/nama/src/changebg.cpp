#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "change_background_color");
    ros::NodeHandle nh;

    // Set the background color parameters
    nh.setParam("/turtlesim/background_r", 255);
    nh.setParam("/turtlesim/background_g", 255);
    nh.setParam("/turtlesim/background_b", 255);

    // Call the /clear service to apply the changes
    ros::ServiceClient clear_client = nh.serviceClient<std_srvs::Empty>("/clear");
    std_srvs::Empty srv;
    if (clear_client.call(srv)) {
        ROS_INFO("Background color changed successfully!");
    }
    else {
        ROS_ERROR("Failed to change background color.");
    }

    return 0;
}
