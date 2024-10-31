#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include <std_srvs/Empty.h>
#include <math.h>
#define _USE_MATH_DEFINES

ros::Publisher turtle1_pub;
ros::ServiceClient turtle1_set_pen_client;
ros::ServiceClient turtle1_teleport_client;
turtlesim::SetPen pen;
turtlesim::TeleportAbsolute tele;
geometry_msgs::Twist move;

void moveTurtle(double speed) {
    move.linear.x = speed;
    turtle1_pub.publish(move);
    ros::Duration(1).sleep();
    move.linear.x = 0;
    turtle1_pub.publish(move);
}

void rotate(double degree) {
    move.angular.z = degree * M_PI / 180;
    turtle1_pub.publish(move);
    ros::Duration(1.5).sleep();
    move.angular.z = 0;
    turtle1_pub.publish(move);
}

void setpen(int red, int green, int blue) {
    pen.request.r = red;
    pen.request.g = green;
    pen.request.b = blue;
    pen.request.width = 2;
    pen.request.off = 0;

    if (!turtle1_set_pen_client.call(pen)) {
        ROS_ERROR("Failed to call set_pen service");
    }
}

void teleport(float x, float y, float theta = 0) {
    tele.request.x = x;
    tele.request.y = y;
    tele.request.theta = theta;
    ros::Duration(1).sleep();

    if (!turtle1_teleport_client.call(tele)) {
        ROS_ERROR("Failed to call teleport_absolute service");
    }
}

void write_G(int r, int g, int b, double x, double y) {

    setpen(255, 255, 255);
    teleport(x, y);
    setpen(r, g, b);

    rotate(180.0);
    moveTurtle(1.0);

    rotate(90.0);
    moveTurtle(2.0);

    rotate(90.0);
    moveTurtle(1.0);

    rotate(90);
    moveTurtle(1.0);

    rotate(90);
    moveTurtle(0.7);
}

void write_I(int r, int g, int b, double x, double y) {
    setpen(255, 255, 255);
    teleport(x, y);
    setpen(r, g, b);

    rotate(-90);
    moveTurtle(2);
}

void write_L(int r, int g, int b, double x, double y) {
    setpen(255, 255, 255);
    teleport(x, y);
    setpen(r, g, b);

    rotate(-90);
    moveTurtle(2);

    rotate(90);
    moveTurtle(1);
}

void write_B(int r, int g, int b, double x, double y) {
    setpen(255, 255, 255);
    teleport(x, y);
    setpen(r, g, b);

    rotate(-90);
    moveTurtle(2);

    rotate(120);
    moveTurtle(1);

    rotate(120);
    moveTurtle(1);

    rotate(-120);
    moveTurtle(1);

    rotate(120);
    moveTurtle(1);
}

void write_R(int r, int g, int b, double x, double y) {
    setpen(255, 255, 255);
    teleport(x, y);
    setpen(r, g, b);

    rotate(90);
    moveTurtle(2);

    rotate(-120);
    moveTurtle(1);

    rotate(-120);
    moveTurtle(1);

    rotate(100);
    moveTurtle(sqrt(2));
}

void write_A(int r, int g, int b, double x, double y) {
    setpen(255, 255, 255);
    teleport(x, y);
    setpen(r, g, b);

    rotate(75);
    moveTurtle(2);

    rotate(-150);
    moveTurtle(2);

    setpen(255, 255, 255);
    teleport(7.3, 5);
    setpen(r, g, b);
    moveTurtle(0.5);
}

void write_N(int r, int g, int b, double x, double y) {
    setpen(255, 255, 255);
    teleport(x, y);
    setpen(r, g, b);

    rotate(90);
    moveTurtle(2);

    rotate(-150);
    moveTurtle(1 + sqrt(2));

    rotate(150);
    moveTurtle(2);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "control_turtle1_GILBRAN");
    ros::NodeHandle nh;

    nh.setParam("/turtlesim/background_r", 255);
    nh.setParam("/turtlesim/background_g", 255);
    nh.setParam("/turtlesim/background_b", 255);

    turtle1_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    turtle1_set_pen_client = nh.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
    turtle1_teleport_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
    ros::ServiceClient clear_client = nh.serviceClient<std_srvs::Empty>("/clear");

    std_srvs::Empty srv;
    ros::Rate rate(0.1);

    while (ros::ok()) {
        write_G(0, 0, 0, 1.5, 6);
        write_I(0, 0, 0, 2, 6);
        write_L(0, 0, 0, 2.5, 6);
        write_B(0, 0, 0, 4, 6);
        write_R(0, 0, 0, 5.5, 4);
        write_A(0, 0, 0, 7, 4);
        write_N(0, 0, 0, 8.5, 4);

        write_G(255, 255, 255, 1.5, 6);
        write_I(255, 255, 255, 2, 6);
        write_L(255, 255, 255, 2.5, 6);
        write_B(255, 255, 255, 4, 6);
        write_R(255, 255, 255, 5.5, 4);
        write_A(255, 255, 255, 7, 4);
        write_N(255, 255, 255, 8.5, 4);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
