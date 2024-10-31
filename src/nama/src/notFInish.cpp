#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/TeleportAbsolute.h"
#include "turtlesim/Spawn.h"

ros::Publisher pub1, pub2, pub3, pub4, pub5, pub6, pub7;
ros::ServiceClient set_pen_client1, set_pen_client2, set_pen_client3, set_pen_client4, set_pen_client5, set_pen_client6, set_pen_client7;
ros::ServiceClient teleport_client1, teleport_client2, teleport_client3, teleport_client4, teleport_client5, teleport_client6, teleport_client7;
ros::ServiceClient spawn_client; // Service client for spawning turtles

// Function to move the turtle
void move(ros::Publisher& pub, double speed, double distance) {
    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = speed;
    pub.publish(move_cmd);
    ros::Duration(distance / fabs(speed)).sleep();
    move_cmd.linear.x = 0;
    pub.publish(move_cmd);
}

// Function to rotate the turtle
void rotate(ros::Publisher& pub, double angular_speed) {
    geometry_msgs::Twist rotate_cmd;
    rotate_cmd.angular.z = angular_speed;
    pub.publish(rotate_cmd);
    ros::Duration(1.5).sleep(); // Approximate rotation for 90 degrees
    rotate_cmd.angular.z = 0;
    pub.publish(rotate_cmd);
}

// Function to set the pen state
void setPen(ros::ServiceClient& client, bool on) {
    turtlesim::SetPen pen_srv;
    pen_srv.request.off = !on;
    pen_srv.request.r = on ? 0 : 255;
    pen_srv.request.g = on ? 0 : 255;
    pen_srv.request.b = on ? 0 : 255;
    pen_srv.request.width = 3;
    client.call(pen_srv);
}

// Function to teleport the turtle
void teleport(ros::ServiceClient& client, double x, double y, double theta = 0) {
    turtlesim::TeleportAbsolute teleport_srv;
    teleport_srv.request.x = x;
    teleport_srv.request.y = y;
    teleport_srv.request.theta = theta;
    client.call(teleport_srv);
    ros::Duration(1).sleep();
}

// Function to spawn a turtle
void spawnTurtle(double x, double y, const std::string& name) {
    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = x;
    spawn_srv.request.y = y;
    spawn_srv.request.theta = 0; // Default orientation
    spawn_srv.request.name = name; // Name of the new turtle
    spawn_client.call(spawn_srv);
}

// Function to draw letter G
void letter_G(ros::Publisher& pub, ros::ServiceClient& set_pen_client) {
    setPen(set_pen_client, true);
    move(pub, 1, 2);
    rotate(pub, -1.57);
    move(pub, 1, 1);
    rotate(pub, -1.57);
    move(pub, 1, 1);
    rotate(pub, -1.57);
    move(pub, 1, 1);
    rotate(pub, 1.57);
    move(pub, 1, 1);
    setPen(set_pen_client, false);
}

// Function to draw letter I
void letter_I(ros::Publisher& pub, ros::ServiceClient& set_pen_client) {
    setPen(set_pen_client, true);
    move(pub, 1, 2);
    setPen(set_pen_client, false);
}

// Function to draw letter L
void letter_L(ros::Publisher& pub, ros::ServiceClient& set_pen_client) {
    setPen(set_pen_client, true);
    move(pub, 1, 2);
    rotate(pub, -1.57);
    move(pub, 1, 1);
    setPen(set_pen_client, false);
}

// Function to draw letter B
void letter_B(ros::Publisher& pub, ros::ServiceClient& set_pen_client) {
    setPen(set_pen_client, true);
    move(pub, 1, 2);
    rotate(pub, -1.57);
    move(pub, 1, 1);
    rotate(pub, -1.57);
    move(pub, 1, 1);
    rotate(pub, 1.57);
    move(pub, 1, 1);
    rotate(pub, 1.57);
    move(pub, 1, 1);
    setPen(set_pen_client, false);
}

// Function to draw letter R
void letter_R(ros::Publisher& pub, ros::ServiceClient& set_pen_client) {
    setPen(set_pen_client, true);
    move(pub, 1, 2);
    rotate(pub, -1.57);
    move(pub, 1, 1);
    rotate(pub, -1.57);
    move(pub, 1, 1);
    rotate(pub, 2);
    move(pub, 1, 1);
    setPen(set_pen_client, false);
}

// Function to draw letter A
void letter_A(ros::Publisher& pub, ros::ServiceClient& set_pen_client) {
    setPen(set_pen_client, true);
    rotate(pub, 1.57);
    move(pub, 1, 2);
    rotate(pub, -1.2);
    move(pub, 1, 1);
    rotate(pub, 2.4);
    move(pub, 1, 1);
    setPen(set_pen_client, false);
}

// Function to draw letter N
void letter_N(ros::Publisher& pub, ros::ServiceClient& set_pen_client) {
    setPen(set_pen_client, true);
    move(pub, 1, 2);
    rotate(pub, 2);
    move(pub, 1, 2);
    rotate(pub, -2);
    move(pub, 1, 2);
    setPen(set_pen_client, false);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtlesim_name_writer_multiple_turtles");
    ros::NodeHandle nh;

    // Initialize service client for spawning turtles
    spawn_client = nh.serviceClient<turtlesim::Spawn>("spawn");

    // Spawn turtles for each letter
    spawnTurtle(1, 5, "turtle2");
    spawnTurtle(3, 5, "turtle3");
    spawnTurtle(4, 5, "turtle4");
    spawnTurtle(5.5, 5, "turtle5");
    spawnTurtle(7, 5, "turtle6");
    spawnTurtle(8.5, 5, "turtle7");

    // Initialize publishers and service clients for each turtle
    pub1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    pub2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);
    pub3 = nh.advertise<geometry_msgs::Twist>("/turtle3/cmd_vel", 10);
    pub4 = nh.advertise<geometry_msgs::Twist>("/turtle4/cmd_vel", 10);
    pub5 = nh.advertise<geometry_msgs::Twist>("/turtle5/cmd_vel", 10);
    pub6 = nh.advertise<geometry_msgs::Twist>("/turtle6/cmd_vel", 10);
    pub7 = nh.advertise<geometry_msgs::Twist>("/turtle7/cmd_vel", 10);

    set_pen_client1 = nh.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
    set_pen_client2 = nh.serviceClient<turtlesim::SetPen>("/turtle2/set_pen");
    set_pen_client3 = nh.serviceClient<turtlesim::SetPen>("/turtle3/set_pen");
    set_pen_client4 = nh.serviceClient<turtlesim::SetPen>("/turtle4/set_pen");
    set_pen_client5 = nh.serviceClient<turtlesim::SetPen>("/turtle5/set_pen");
    set_pen_client6 = nh.serviceClient<turtlesim::SetPen>("/turtle6/set_pen");
    set_pen_client7 = nh.serviceClient<turtlesim::SetPen>("/turtle7/set_pen");

    teleport_client1 = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
    teleport_client2 = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle2/teleport_absolute");
    teleport_client3 = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle3/teleport_absolute");
    teleport_client4 = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle4/teleport_absolute");
    teleport_client5 = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle5/teleport_absolute");
    teleport_client6 = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle6/teleport_absolute");
    teleport_client7 = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle7/teleport_absolute");

    ros::Duration(2).sleep(); // Wait for services to be ready
    // Draw "G"
    teleport(teleport_client1, 1, 5);
    letter_G(pub1, set_pen_client1);

    // Draw "I"
    teleport(teleport_client2, 3, 5);
    letter_I(pub2, set_pen_client2);

    // Draw "L"
    teleport(teleport_client3, 4, 5);
    letter_L(pub3, set_pen_client3);

    // Draw "B"
    teleport(teleport_client4, 5.5, 5);
    letter_B(pub4, set_pen_client4);

    // Draw "R"
    teleport(teleport_client5, 7, 5);
    letter_R(pub5, set_pen_client5);

    // Draw "A"
    teleport(teleport_client6, 8.5, 5);
    letter_A(pub6, set_pen_client6);

    // Draw "N"
    teleport(teleport_client7, 10, 5);
    letter_N(pub7, set_pen_client7);

    return 0;
}
