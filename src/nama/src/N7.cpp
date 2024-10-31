#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "control_turtle1");
    ros::NodeHandle nh;

    ros::Publisher turtle1_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = 1.0;
    move_cmd.angular.z = 0.5;

    ros::Rate rate(10);
    while (ros::ok()) {
        turtle1_pub.publish(move_cmd);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}