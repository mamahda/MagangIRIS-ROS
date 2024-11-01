#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <turtlesim/TeleportAbsolute.h>
#include <opencv2/opencv.hpp>
#include <turtlesim/SetPen.h>
#include <opencv2/videoio.hpp>
#include <iostream>

using namespace cv;

void setpen(int red, int green, int blue) {
    ros::NodeHandle nh;
    ros::ServiceClient turtle1_set_pen_client = nh.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");

    turtlesim::SetPen pen;
    pen.request.r = red;
    pen.request.g = green;
    pen.request.b = blue;
    pen.request.width = 2;
    pen.request.off = 0;
}

void robotPositionCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    ros::NodeHandle nh;
    ros::ServiceClient robot_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");

    turtlesim::TeleportAbsolute tele;
    tele.request.x = msg->position.x;
    tele.request.y = msg->position.y;

    if (robot_client.call(tele)) {
        ROS_INFO("Moved robot turtle to: (%f, %f)", msg->position.x, msg->position.y);
    }
}

void ballPositionCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    ros::NodeHandle nh;
    ros::ServiceClient ball_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle2/teleport_absolute");

    turtlesim::TeleportAbsolute tele;
    tele.request.x = msg->position.x;
    tele.request.y = msg->position.y;

    if (ball_client.call(tele)) {
        ROS_INFO("Moved ball turtle to: (%f, %f)", msg->position.x, msg->position.y);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "position_publisher");
    ros::NodeHandle nh;

    ros::Duration(1).sleep();

    ros::Publisher robot_pos_pub = nh.advertise<geometry_msgs::Pose>("robot_position", 10);
    ros::Publisher ball_pos_pub = nh.advertise<geometry_msgs::Pose>("ball_position", 10);

    ros::Subscriber robot_sub = nh.subscribe("robot_position", 10, robotPositionCallback);
    ros::Subscriber ball_sub = nh.subscribe("ball_position", 10, ballPositionCallback);

    VideoCapture cap("/home/gilbran/MagangIRIS/MagangIRIS-ROS/src/gerakrobot/src/robotcamera.mp4");
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open video file");
        return -1;
    }

    Mat frame, hsvFrame, mask;
    double x_robot = 0.0, y_robot = 0.0;
    double prev_ballX_cm = 0.0, prev_ballY_cm = 0.0;
    const float px_to_cm = 0.1;
    const float cm_to_lapangan = 10.0;
    bool is_first_frame = true;
    const double min_area_ball = 30.0;
    const double max_area_ball = 5000.0;

    const double staticBallX_px = 89.75;
    const double staticBallY_px = 11.25;

    Point2f centerField(cap.get(CAP_PROP_FRAME_WIDTH) / 2, cap.get(CAP_PROP_FRAME_HEIGHT) / 2);
    int exclusionRadius = 100;

    ros::Rate loop_rate(30);
    ros::Duration(0.5).sleep();

    while (ros::ok()) {
        cap >> frame;
        if (frame.empty()) break;

        cvtColor(frame, hsvFrame, COLOR_BGR2HSV);
        inRange(hsvFrame, Scalar(5, 150, 150), Scalar(15, 255, 255), mask);

        circle(mask, centerField, exclusionRadius, Scalar(0), -1);

        std::vector<std::vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        setpen(0, 0, 0);

        if (!contours.empty()) {
            double maxArea = 0;
            int largestContourIndex = -1;

            for (int i = 0; i < contours.size(); i++) {
                double area = contourArea(contours[i]);
                if (area > maxArea && area >= min_area_ball && area <= max_area_ball) {
                    maxArea = area;
                    largestContourIndex = i;
                }
            }

            if (largestContourIndex != -1) {
                RotatedRect rect = minAreaRect(contours[largestContourIndex]);
                Point2f center = rect.center;

                double object_x_cm = center.x * px_to_cm;
                double object_y_cm = center.y * px_to_cm;

                if (!is_first_frame) {
                    setpen(255, 255, 255);
                    x_robot -= object_x_cm - prev_ballX_cm;
                    y_robot += object_y_cm - prev_ballY_cm;
                }

                prev_ballX_cm = object_x_cm;
                prev_ballY_cm = object_y_cm;

                x_robot = (x_robot < 0) ? -x_robot : x_robot;
                y_robot = (y_robot < 0) ? -y_robot : y_robot;

                is_first_frame = false;

                geometry_msgs::Pose robot_position;
                robot_position.position.x = (x_robot + 20) / cm_to_lapangan;
                robot_position.position.y = (y_robot + 20) / cm_to_lapangan;
                robot_position.position.z = 0.0;
                robot_position.orientation.w = 1.0;

                robot_pos_pub.publish(robot_position);

                geometry_msgs::Pose ball_position;
                ball_position.position.x = (staticBallX_px - 34.75) / cm_to_lapangan;
                ball_position.position.y = (staticBallY_px + 43.75) / cm_to_lapangan;
                ball_position.position.z = 0.0;
                ball_position.orientation.w = 1.0;

                ball_pos_pub.publish(ball_position);
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
