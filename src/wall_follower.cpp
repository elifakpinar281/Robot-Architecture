#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <limits>

double dist_front = std::numeric_limits<double>::infinity();
double dist_right = std::numeric_limits<double>::infinity();
double dist_front_right = std::numeric_limits<double>::infinity();

double minRange(const sensor_msgs::LaserScan::ConstPtr& msg, int center_deg, int window_deg) {
    int n = (int)msg->ranges.size();
    if (n == 0) return std::numeric_limits<double>::infinity();

    double kleinster = std::numeric_limits<double>::infinity();
    for (int offset = -window_deg; offset <= window_deg; ++offset) {
        int deg = center_deg + offset;
        int grad = ((deg % 360) + 360) % 360;
        int index = (grad * n) / 360;
        if (index < 0 || index >= n) continue;

        double d = msg->ranges[index];
        if (std::isfinite(d) && d >= msg->range_min && d <= msg->range_max) {
            if (d < kleinster) {
                kleinster = d;
            }
        }
    }
    return kleinster;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    dist_front = minRange(msg,   0, 15);   // ca. 345..15 Grad
    dist_right = minRange(msg, 270, 15);   // ca. 255..285 Grad
    dist_front_right = minRange(msg, 315, 15);   // ca. 300..330 Grad
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_follower");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/scan", 10, scanCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ROS_INFO("wall_follower gestartet");

    const double WAND_ABSTAND = 0.3;    // gewuenschter Abstand zur Wand (m)
    const double TOLERANZ     = 0.1;    // Pufferzone um den Wunschabstand
    const double FRONT_STOP   = 0.5;    // Wand voraus -> auf der Stelle drehen
    const double SPEED        = 0.15;   // Vorwaertsgeschwindigkeit (m/s)
    const double DREHEN       = 0.5;    // Drehen in der Ecke (rad/s)
    const double KORREKTUR    = 0.3;    // sanftes Nachlenken (rad/s)

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        geometry_msgs::Twist cmd;

        if (dist_front < FRONT_STOP || dist_front_right < WAND_ABSTAND) {
            cmd.linear.x  = 0.0;
            cmd.angular.z = DREHEN;
        } else if (dist_right > WAND_ABSTAND + TOLERANZ) {
            cmd.linear.x  = SPEED;
            cmd.angular.z = -KORREKTUR;
        } else if (dist_right < WAND_ABSTAND - TOLERANZ) {
            cmd.linear.x  = SPEED;
            cmd.angular.z = KORREKTUR;
        } else {
            cmd.linear.x  = SPEED;
            cmd.angular.z = 0.0;
        }

        pub.publish(cmd);
        rate.sleep();
    }
    return 0;
}
