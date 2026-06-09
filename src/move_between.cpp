#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_between");
    ros::NodeHandle nh;
    ROS_INFO("move_between gestartet");
    ros::spin();
    return 0;
}
