#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>

using namespace cv;

int main(int argc, char** argv) {
    ros::init(argc, argv, "position_publisher");
    ros::NodeHandle nh;

    ros::Publisher robot_pos_pub = nh.advertise<geometry_msgs::Pose>("robot_position", 10);
    ros::Publisher ball_pos_pub = nh.advertise<geometry_msgs::Pose>("ball_position", 10);

    VideoCapture cap("/home/gilbran/MagangIRIS/MagangIRIS-ROS/src/gerakrobot/src/robotcamera.avi");
    if (!cap.isOpened()) {
        return -1;
    }

    Mat frame, hsvFrame, mask;
    double x_robot = 0.0, y_robot = 0.0;
    double prev_x_cm = 0.0, prev_y_cm = 0.0;
    const float px_to_cm = 0.1;
    const float cm_to_field_scale = 10.0;
    bool is_first_frame = true;
    const double min_contour_area = 30.0;
    const double max_contour_area = 5000.0;

    // Static ball position
    const double staticBallX_cm = 8.975;
    const double staticBallY_cm = 1.125;

    // Exclusion zone around the center field to ignore the static ball region
    Point2f centerField(cap.get(CAP_PROP_FRAME_WIDTH) / 2, cap.get(CAP_PROP_FRAME_HEIGHT) / 2);
    int exclusionRadius = 100;

    ros::Rate loop_rate(30);

    while (ros::ok()) {
        cap >> frame;
        if (frame.empty()) break;

        // Convert to HSV and apply mask for robot detection
        cvtColor(frame, hsvFrame, COLOR_BGR2HSV);
        Scalar lower_orange(5, 150, 150);
        Scalar upper_orange(15, 255, 255);
        inRange(hsvFrame, lower_orange, upper_orange, mask);

        // Exclude the center field region
        circle(mask, centerField, exclusionRadius, Scalar(0), -1);

        // Find contours for detecting the robot
        std::vector<std::vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            double maxArea = 0;
            int largestContourIdx = -1;

            for (int i = 0; i < contours.size(); i++) {
                double area = contourArea(contours[i]);
                if (area > maxArea && area >= min_contour_area && area <= max_contour_area) {
                    maxArea = area;
                    largestContourIdx = i;
                }
            }

            if (largestContourIdx != -1) {
                RotatedRect rect = minAreaRect(contours[largestContourIdx]);
                Point2f center = rect.center;

                double current_x_cm = center.x * px_to_cm;
                double current_y_cm = center.y * px_to_cm;

                if (!is_first_frame) {
                    x_robot -= current_x_cm - prev_x_cm;
                    y_robot += current_y_cm - prev_y_cm;
                }

                prev_x_cm = current_x_cm;
                prev_y_cm = current_y_cm;

                if (x_robot < 0) {
                    x_robot -= x_robot - 1;
                }

                if (y_robot < 0) {
                    y_robot -= y_robot - 1;
                }

                is_first_frame = false;

                // Publish robot position
                geometry_msgs::Pose robot_position;
                robot_position.position.x = x_robot / cm_to_field_scale;
                robot_position.position.y = y_robot / cm_to_field_scale;
                robot_position.position.z = 0.0;
                robot_position.orientation.w = 1.0;

                robot_pos_pub.publish(robot_position);

                // Publish static ball position
                geometry_msgs::Pose ball_position;
                ball_position.position.x = staticBallX_cm / cm_to_field_scale;
                ball_position.position.y = staticBallY_cm / cm_to_field_scale;
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