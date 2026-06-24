#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "turtlebot3_control/RobotStatus.h"
#include <cmath>
#include <limits>

// aus /cmd_vel
double linear_speed  = 0.0;
double angular_speed = 0.0;
sensor_msgs::LaserScan::ConstPtr latest_scan;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    latest_scan = msg;
}

void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    linear_speed  = msg->linear.x;
    angular_speed = msg->angular.z;
}

double minRange(const sensor_msgs::LaserScan::ConstPtr& msg, double center_deg, double window_rad) {
    const int N = (int)msg->ranges.size();
    if (N == 0) return msg->range_max;

    const double inc = msg->angle_increment;
    double center_rad = center_deg * M_PI / 180.0;
    int half = std::max(1, (int)std::lround(window_rad / inc));
    int center_idx = (int)std::lround((center_rad - msg->angle_min) / inc);

    double best = std::numeric_limits<double>::infinity();
    for (int o = -half; o <= half; ++o) {
        int idx = (((center_idx + o) % N) + N) % N;
        double r = msg->ranges[idx];
        if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max && r < best) {
            best = r;
        }
    }
    if (!std::isfinite(best)) return msg->range_max;
    return best;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "status_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); // privater Handle

    double rate_hz, window_deg, front_deg, left_deg, right_deg;
    std::string scan_topic, cmd_topic, status_topic, frame_id;
    pnh.param("rate_hz", rate_hz, 10.0);
    pnh.param("window_deg", window_deg, 5.0);
    pnh.param("front_deg", front_deg, 0.0);
    pnh.param("left_deg", left_deg, 90.0);
    pnh.param("right_deg", right_deg, 270.0);
    pnh.param<std::string>("scan_topic", scan_topic, "/scan");
    pnh.param<std::string>("cmd_topic", cmd_topic, "/cmd_vel");
    pnh.param<std::string>("status_topic", status_topic, "/robot_status");
    pnh.param<std::string>("frame_id", frame_id, "base_scan");

    double window_rad = window_deg * M_PI / 180.0;

    ros::Subscriber scan_sub = nh.subscribe(scan_topic, 10, scanCallback);
    ros::Subscriber cmd_sub = nh.subscribe(cmd_topic, 10, cmdCallback);
    ros::Publisher pub = nh.advertise<turtlebot3_control::RobotStatus>(status_topic, 10);

    ROS_INFO("status_publisher gestartet (rate=%.1f Hz, window=%.1f deg)", rate_hz, window_deg);

    ros::Rate rate(rate_hz);
    while (ros::ok()) {
        ros::spinOnce();

        turtlebot3_control::RobotStatus status;
        status.header.stamp = ros::Time::now(); // Zeitstempel der Messung
        status.header.frame_id = frame_id;
        status.linear_speed = linear_speed;
        status.angular_speed = angular_speed;

        if (latest_scan) {
            status.distance_front = minRange(latest_scan, front_deg, window_rad);
            status.distance_left = minRange(latest_scan, left_deg,  window_rad);
            status.distance_right = minRange(latest_scan, right_deg, window_rad);
        }

        pub.publish(status);
        rate.sleep();
    }
    return 0;
}
