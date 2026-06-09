#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_follower");
    ros::NodeHandle nh;
    ROS_INFO("wall_follower gestartet");
    ros::spin();
    return 0;
}
