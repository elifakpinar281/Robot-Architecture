#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <limits>
#include <csignal>

volatile std::sig_atomic_t g_stop = 0;
void onSigint(int) { g_stop = 1; }

double dist_front = std::numeric_limits<double>::infinity();
double dist_right = std::numeric_limits<double>::infinity();
double dist_front_right = std::numeric_limits<double>::infinity();

double window_deg = 15.0;

double minRange(const sensor_msgs::LaserScan::ConstPtr& msg, double center_deg, double window_deg) {
    const int N = (int)msg->ranges.size();
    if (N == 0) return std::numeric_limits<double>::infinity();

    const double inc = msg->angle_increment;
    double center_rad = center_deg * M_PI / 180.0;
    int half = std::max(1, (int)std::lround((window_deg * M_PI / 180.0) / inc));
    int center_idx = (int)std::lround((center_rad - msg->angle_min) / inc);

    double kleinster = std::numeric_limits<double>::infinity();
    for (int o = -half; o <= half; ++o) {
        int idx = (((center_idx + o) % N) + N) % N;
        double d = msg->ranges[idx];
        if (std::isfinite(d) && d >= msg->range_min && d <= msg->range_max && d < kleinster) {
            kleinster = d;
        }
    }
    return kleinster;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    dist_front = minRange(msg, 0, window_deg);
    dist_right = minRange(msg, 270, window_deg);
    dist_front_right = minRange(msg, 315, window_deg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_follower", ros::init_options::NoSigintHandler);
    std::signal(SIGINT, onSigint);
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    double wand_abstand, toleranz, front_stop, speed, drehen, korrektur, rate_hz, wall_range;
    std::string scan_topic, cmd_topic;
    pnh.param("wand_abstand", wand_abstand, 0.3); 
    pnh.param("toleranz", toleranz, 0.1);
    pnh.param("front_stop", front_stop, 0.5);
    pnh.param("speed", speed, 0.15);
    pnh.param("drehen", drehen, 0.5);
    pnh.param("korrektur", korrektur, 0.3);
    pnh.param("window_deg", window_deg, 15.0);
    pnh.param("rate_hz", rate_hz, 10.0);
    pnh.param("wall_range", wall_range, 1.0);
    pnh.param<std::string>("scan_topic", scan_topic, "/scan");
    pnh.param<std::string>("cmd_topic", cmd_topic, "/cmd_vel");

    ros::Subscriber sub = nh.subscribe(scan_topic, 10, scanCallback);
    ros::Publisher  pub = nh.advertise<geometry_msgs::Twist>(cmd_topic, 10);
    ROS_INFO("wall_follower gestartet (abstand=%.2f m, speed=%.2f m/s)", wand_abstand, speed);

    ros::Rate rate(rate_hz);
    while (ros::ok() && !g_stop) {
        ros::spinOnce();
        geometry_msgs::Twist cmd;

        // Warten bis der erste Scan da ist
        // Sonst würde inf als "frei" durchgehen und der Roboter fährt blind los
        if (!std::isfinite(dist_front) && !std::isfinite(dist_right) && !std::isfinite(dist_front_right)) {
            pub.publish(cmd);
            rate.sleep();
            continue;
        }

        if (dist_front < front_stop || dist_front_right < wand_abstand) {
            cmd.linear.x  = 0.0;
            cmd.angular.z = drehen;
        } else if (dist_right > wall_range) {
            cmd.linear.x  = speed;
            cmd.angular.z = 0.0;
        } else if (dist_right > wand_abstand + toleranz) {
            cmd.linear.x  = speed;
            cmd.angular.z = -korrektur;
        } else if (dist_right < wand_abstand - toleranz) {
            cmd.linear.x  = speed;
            cmd.angular.z = korrektur;
        } else {
            cmd.linear.x  = speed;
            cmd.angular.z = 0.0;
        }

        pub.publish(cmd);
        rate.sleep();
    }

    geometry_msgs::Twist stop;
    pub.publish(stop);
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    ros::shutdown();
    return 0;
}