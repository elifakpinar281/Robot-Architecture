#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "status_publisher");
    ros::NodeHandle nh;
    ROS_INFO("status_publisher gestartet");
    ros::spin();
    return 0;
}
