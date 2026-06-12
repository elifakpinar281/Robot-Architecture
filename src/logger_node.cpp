#include <ros/ros.h>
#include "turtlebot3_control/RobotStatus.h"
#include <std_srvs/SetBool.h>
#include <fstream>

std::ofstream logfile;
bool logging_enabled = true;

// Service: schaltet das Logging an/aus
bool setLogging(std_srvs::SetBool::Request& req,
                std_srvs::SetBool::Response& res) {
    logging_enabled = req.data;
    res.success = true;
    res.message = req.data ? "Logging an" : "Logging aus";
    ROS_INFO("%s", res.message.c_str());
    return true;
}

// Wird bei jeder RobotStatus-Nachricht aufgerufen
void statusCallback(const turtlebot3_control::RobotStatus::ConstPtr& msg) {
    if (!logging_enabled) return;                 // bei "aus" nichts schreiben
    logfile << ros::Time::now().toSec() << ","
            << msg->linear_speed   << ","
            << msg->angular_speed  << ","
            << msg->distance_front << ","
            << msg->distance_left  << ","
            << msg->distance_right << std::endl;  // endl = direkt auf Platte
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "logger_node");
    ros::NodeHandle nh;

    logfile.open("robot_status_log.csv");         
    logfile << std::fixed;    // keine 1.78e+09-Schreibweise mehr
    logfile.precision(3);
    logfile << "timestamp,linear_speed,angular_speed,"
               "distance_front,distance_left,distance_right" << std::endl;

    ros::Subscriber sub = nh.subscribe("/robot_status", 10, statusCallback);
    ros::ServiceServer srv = nh.advertiseService("/set_logging", setLogging);

    ROS_INFO("logger_node gestartet");
    ros::spin();
    logfile.close();
    return 0;
}