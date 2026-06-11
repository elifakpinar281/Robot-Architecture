#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <limits>

double front_distance = std::numeric_limits<double>::infinity();

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    double d = msg->ranges[0];                    // ranges[0] = direkt vorne
    front_distance = (std::isfinite(d) && d > 0.0)
                     ? d : std::numeric_limits<double>::infinity();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_between");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/scan", 10, scanCallback);
    ros::Publisher  pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ROS_INFO("move_between gestartet");

    const double STOP_DISTANCE = 0.5;             // wie nah ans Objekt
    const double FORWARD_SPEED = 0.15;            // m/s
    const double TURN_SPEED    = 0.6;             // rad/s
    const double TURN_DURATION = M_PI / TURN_SPEED;  // ~180°

    enum State { FAHREN, DREHEN } state = FAHREN;
    ros::Time turn_start;
    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce();                          // front_distance aktualisieren
        geometry_msgs::Twist cmd;

        if (state == FAHREN) {
            if (front_distance > STOP_DISTANCE) {
                cmd.linear.x = FORWARD_SPEED;     // vorwärts
            } else {
                state = DREHEN;                   // Objekt erreicht
                turn_start = ros::Time::now();
            }
        } else { // DREHEN
            if ((ros::Time::now() - turn_start).toSec() < TURN_DURATION)
                cmd.angular.z = TURN_SPEED;       // auf der Stelle drehen
            else
                state = FAHREN;                   // fertig -> wieder fahren
        }

        pub.publish(cmd);
        rate.sleep();
    }
    return 0;
}