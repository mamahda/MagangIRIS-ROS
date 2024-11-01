#include <ros/ros.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "spawn_turtle2");
    ros::NodeHandle nh;

    ros::service::waitForService("/spawn");

    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn spawn_srv;

    spawn_srv.request.x = 5.0;
    spawn_srv.request.y = 5.0;
    spawn_srv.request.theta = 0.0;
    spawn_srv.request.name = "turtle2";

    if (spawn_client.call(spawn_srv)) {
        ROS_INFO("Successfully spawned turtle2 at (5.0, 5.0) with theta 0.0");
    }
    else {
        ROS_ERROR("Failed to spawn turtle2");
        return 1;
    }

    return 0;
}
