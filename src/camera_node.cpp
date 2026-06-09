#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;
    ROS_INFO("camera_node gestartet");
    ros::spin();
    return 0;
}
