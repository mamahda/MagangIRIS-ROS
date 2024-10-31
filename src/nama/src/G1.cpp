#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "control_turtle1");
    ros::NodeHandle nh;

    ros::Publisher turtle1_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::ServiceClient turtle1_teleport_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");

    geometry_msgs::Twist move;
    turtlesim::TeleportAbsolute tele;

    ros::Rate rate(10);
    while (ros::ok()) {
        move.angular.z =
            turtle1_pub.publish(move);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}