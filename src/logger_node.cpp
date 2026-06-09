#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "logger_node");
    ros::NodeHandle nh;
    ROS_INFO("logger_node gestartet");
    ros::spin();
    return 0;
}
