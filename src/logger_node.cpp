#include <ros/ros.h>
#include "turtlebot3_control/RobotStatus.h"
#include <std_srvs/SetBool.h>
#include <fstream>
#include <ros/package.h>
#include <ctime>
#include <sys/stat.h>

std::ofstream logfile;
bool logging_enabled = true;

bool setLogging(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    logging_enabled = req.data;
    res.success = true;
    res.message = req.data ? "Logging an" : "Logging aus";
    ROS_INFO("%s", res.message.c_str());
    return true;
}

void statusCallback(const turtlebot3_control::RobotStatus::ConstPtr& msg) {
    if (!logging_enabled) return; 

    // Zeitstempel aus der Nachricht (Messzeit)
    double stamp = msg->header.stamp.toSec();
    logfile << stamp << ","
            << msg->linear_speed << ","
            << msg->angular_speed << ","
            << msg->distance_front << ","
            << msg->distance_left << ","
            << msg->distance_right << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "logger_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string status_topic, service_name, log_dir;
    bool start_enabled;
    pnh.param<std::string>("status_topic", status_topic, "/robot_status");
    pnh.param<std::string>("service_name", service_name, "/set_logging");
    pnh.param<std::string>("log_dir", log_dir, ros::package::getPath("turtlebot3_control") + "/logs");
    pnh.param("start_enabled", start_enabled, true);
    logging_enabled = start_enabled;

    mkdir(log_dir.c_str(), 0775); // schlaegt fehl, falls schon vorhanden (man kann trotzdem ausführen)

    time_t now = time(0);
    struct tm* t = localtime(&now);
    char fname[48];
    strftime(fname, sizeof(fname), "log_%d_%m_%Y_%H%M%S.csv", t);
    std::string path = log_dir + "/" + fname;

    logfile.open(path.c_str());
    if (!logfile.is_open()) { 
        ROS_ERROR("Logdatei konnte nicht geoeffnet werden: %s", path.c_str());
        return 1;
    }
    logfile << std::fixed;
    logfile.precision(3);
    logfile << "timestamp,linear_speed,angular_speed,"
               "distance_front,distance_left,distance_right" << std::endl;

    ros::Subscriber sub = nh.subscribe(status_topic, 10, statusCallback);
    ros::ServiceServer srv = nh.advertiseService(service_name, setLogging);

    ROS_INFO("logger_node gestartet -> %s (logging %s)", path.c_str(), logging_enabled ? "an" : "aus");
    ros::spin();
    logfile.close();
    return 0;
}
