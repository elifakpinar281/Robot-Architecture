#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "turtlebot3_control/RobotStatus.h"
#include <cmath>

double linear_speed  = 0.0; // aus /cmd_vel
double angular_speed  = 0.0; // aus /cmd_vel
double distance_front = 0.0; // aus /scan
double distance_left  = 0.0;
double distance_right = 0.0;

double readRange(const sensor_msgs::LaserScan::ConstPtr& msg, int index) {
    if (index < 0 || index >= (int)msg->ranges.size()) {
        return msg->range_max;
    }
    double d = msg->ranges[index];
    if (!std::isfinite(d) || d < msg->range_min || d > msg->range_max) {
        return msg->range_max;
    }
    return d;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    distance_front = readRange(msg, 0);
    distance_left  = readRange(msg, 90);
    distance_right = readRange(msg, 270);
}

void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    linear_speed  = msg->linear.x;
    angular_speed = msg->angular.z;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "status_publisher");
    ros::NodeHandle nh;

    // Zwei Subscriber: einer für den LiDAR, einer für den Fahrbefehl.
    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scanCallback);
    ros::Subscriber cmd_sub  = nh.subscribe("/cmd_vel", 10, cmdCallback);

    // Publisher für unsere eigene Nachricht.
    ros::Publisher pub =
        nh.advertise<turtlebot3_control::RobotStatus>("/robot_status", 10);

    ROS_INFO("status_publisher gestartet");

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        turtlebot3_control::RobotStatus status;
        status.linear_speed   = linear_speed;
        status.angular_speed  = angular_speed;
        status.distance_front = distance_front;
        status.distance_left  = distance_left;
        status.distance_right = distance_right;

        pub.publish(status);
        rate.sleep();
    }
    return 0;
}
