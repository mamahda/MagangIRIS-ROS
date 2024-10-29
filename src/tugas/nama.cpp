#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/TeleportAbsolute.h"

ros::Publisher pub;
ros::ServiceClient set_pen_client;
ros::ServiceClient teleport_client;

geometry_msgs::Twist move;


// void move(double speed, double distance) {
//     move_cmd.linear.x = speed;
//     pub.publish(move_cmd);
//     ros::Duration(distance / fabs(speed)).sleep();
//     move_cmd.linear.x = 0;
//     pub.publish(move_cmd);
// }

// void rotate(double angular_speed) {
//     geometry_msgs::Twist rotate_cmd;
//     rotate_cmd.angular.z = angular_speed;
//     pub.publish(rotate_cmd);
//     ros::Duration(1.5).sleep(); // Approximate rotation for 90 degrees
//     rotate_cmd.angular.z = 0;
//     pub.publish(rotate_cmd);
// }

void setPen(bool on) {
    turtlesim::SetPen pen_srv;
    pen_srv.request.off = !on;
    pen_srv.request.r = on ? 0 : 255;
    pen_srv.request.g = on ? 0 : 255;
    pen_srv.request.b = on ? 0 : 255;
    pen_srv.request.width = 3;
    set_pen_client.call(pen_srv);
}

void teleport(double x, double y, double theta = 0) {
    turtlesim::TeleportAbsolute teleport_srv;
    teleport_srv.request.x = x;
    teleport_srv.request.y = y;
    teleport_srv.request.theta = theta;
    teleport_client.call(teleport_srv);
    ros::Duration(1).sleep();
}

void letter_G() {
    move(1, 2);
    rotate(-1.57);
    move(1, 1);
    rotate(-1.57);
    move(1, 1);
    rotate(-1.57);
    move(1, 1);
    rotate(1.57);
    move(1, 1);
}

void letter_I() {
    move(1, 2);
}

void letter_L() {
    move(1, 2);
    rotate(-1.57);
    move(1, 1);
}

void letter_B() {
    move(1, 2);
    rotate(-1.57);
    move(1, 1);
    rotate(-1.57);
    move(1, 1);
    rotate(1.57);
    move(1, 1);
    rotate(1.57);
    move(1, 1);
}

void letter_R() {
    move(1, 2);
    rotate(-1.57);
    move(1, 1);
    rotate(-1.57);
    move(1, 1);
    rotate(2);
    move(1, 1);
}

void letter_A() {
    rotate(1.57);
    move(1, 2);
    rotate(-1.2);
    move(1, 1);
    rotate(2.4);
    move(1, 1);
}

void letter_N() {
    move(1, 2);
    rotate(2);
    move(1, 2);
    rotate(-2);
    move(1, 2);
}

void write_gilbran() {
    // Draw "G"
    teleport(1.5, 5);
    setPen(true);

    letter_G();
    setPen(false);

    // Draw "I"
    teleport(3, 5);
    setPen(true);
    letter_I();
    setPen(false);

    // Draw "L"
    teleport(4, 5);
    setPen(true);
    letter_L();
    setPen(false);

    // Draw "B"
    teleport(5.5, 5);
    setPen(true);
    letter_B();
    setPen(false);

    // Draw "R"
    teleport(7, 5);
    setPen(true);
    letter_R();
    setPen(false);

    // Draw "A"
    teleport(8.5, 5);
    setPen(true);
    letter_A();
    setPen(false);

    // Draw "N"
    teleport(10, 5);
    setPen(true);
    letter_N();
    setPen(false);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtlesim_name_writer_no_oop");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    set_pen_client = nh.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
    teleport_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");

    ros::Duration(2).sleep(); // Wait for services to be ready
    write_gilbran();

    return 0;
}
