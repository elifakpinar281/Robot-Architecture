#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <limits>
#include <vector>
#include <csignal>

volatile std::sig_atomic_t g_stop = 0;
void onSigint(int) { g_stop = 1; }

double FRONT_SECTOR;
double CLUSTER_GAP;
int  MIN_POINTS;
double STOP_DISTANCE;
double CLEAR_FACTOR; 
double CENTER_TOL;
double FORWARD_SPEED;
double TURN_SPEED;
double SEARCH_TURN;
double STEER_GAIN;

struct Cluster {
    double bearing;
    double distance; 
};

sensor_msgs::LaserScan::ConstPtr latest_scan;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    latest_scan = msg;
}

std::vector<Cluster> findFrontClusters(const sensor_msgs::LaserScan::ConstPtr& msg) {
    std::vector<Cluster> clusters;
    const int N = (int)msg->ranges.size();
    if (N == 0) return clusters;

    const double inc = msg->angle_increment;
    int sector = (int)std::lround(FRONT_SECTOR / inc);
    if (sector > N / 2) sector = N / 2;

    bool open = false;
    double minDist = 0.0;
    double sumAng = 0.0;
    int count = 0;
    double lastDist = 0.0;

    for (int k = -sector; k <= sector; ++k) {
        int idx = (((k % N) + N) % N);
        double r = msg->ranges[idx];
        double ang = k * inc; 
        bool valid = std::isfinite(r) && r >= msg->range_min && r <= msg->range_max;

        bool belongs = valid && open && std::fabs(r - lastDist) < CLUSTER_GAP;

        if (belongs) {
            if (r < minDist) minDist = r;
            sumAng += ang;
            count++;
            lastDist = r;
        } else {
            if (open && count >= MIN_POINTS) {
                Cluster c; c.distance = minDist; c.bearing = sumAng / count;
                clusters.push_back(c);
            }
            if (valid) { open = true; minDist = r; sumAng = ang; count = 1; lastDist = r; }
            else { 
                open = false; count = 0; 
            }
        }
    }
    if (open && count >= MIN_POINTS) {
        Cluster c; c.distance = minDist; c.bearing = sumAng / count;
        clusters.push_back(c);
    }
    return clusters;
}

// Liefert true + den am staerksten zentrierten Cluster (kleinster |bearing|).
bool pickAhead(const std::vector<Cluster>& clusters, Cluster& out) {
    bool found = false;
    double bestAbs = 0.0;
    for (size_t i = 0; i < clusters.size(); ++i) {
        double a = std::fabs(clusters[i].bearing);
        if (!found || a < bestAbs) {
            found = true; bestAbs = a; out = clusters[i];
        }
    }
    return found;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_between", ros::init_options::NoSigintHandler);
    std::signal(SIGINT, onSigint);
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    double front_sector_deg, center_tol_deg, rate_hz;
    std::string scan_topic, cmd_topic;
    pnh.param("front_sector_deg", front_sector_deg, 90.0);
    pnh.param("cluster_gap", CLUSTER_GAP, 0.20);
    pnh.param("min_points", MIN_POINTS, 3);
    pnh.param("stop_distance", STOP_DISTANCE, 0.5);
    pnh.param("clear_factor", CLEAR_FACTOR, 2.0);
    pnh.param("center_tol_deg", center_tol_deg, 8.0);
    pnh.param("forward_speed", FORWARD_SPEED, 0.15);
    pnh.param("turn_speed", TURN_SPEED, 0.6);
    pnh.param("search_turn", SEARCH_TURN, 0.4);
    pnh.param("steer_gain", STEER_GAIN, 1.2);
    pnh.param("rate_hz", rate_hz, 20.0);
    pnh.param<std::string>("scan_topic", scan_topic, "/scan");
    pnh.param<std::string>("cmd_topic", cmd_topic, "/cmd_vel");

    FRONT_SECTOR = front_sector_deg * M_PI / 180.0;
    CENTER_TOL = center_tol_deg  * M_PI / 180.0;
    const double MAX_TURN_TIME = (2.0 * M_PI / TURN_SPEED);

    ros::Subscriber sub = nh.subscribe(scan_topic, 10, scanCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(cmd_topic, 10);
    ROS_INFO("move_between gestartet (stop=%.2f m, sektor=%.0f deg)", STOP_DISTANCE, front_sector_deg);

    enum State { DRIVE, TURN } state = DRIVE;
    bool cleared = false;
    ros::Time turn_start;
    ros::Rate rate(rate_hz);

    while (ros::ok() && !g_stop) {
        ros::spinOnce();
        geometry_msgs::Twist cmd;

        if (!latest_scan) { pub.publish(cmd); rate.sleep(); continue; }

        std::vector<Cluster> clusters = findFrontClusters(latest_scan);
        Cluster ahead;
        bool haveAhead = pickAhead(clusters, ahead);

        switch (state) {
        case DRIVE:
            if (!haveAhead) {
                cmd.angular.z = SEARCH_TURN;
            } else if (ahead.distance > STOP_DISTANCE) {
                cmd.linear.x  = FORWARD_SPEED;
                cmd.angular.z = STEER_GAIN * ahead.bearing;
            } else {
                state = TURN; cleared = false; turn_start = ros::Time::now();
            }
            break;

        case TURN:
            cmd.angular.z = TURN_SPEED;

            // "cleared": das verlassene nahe Objekt ist nicht mehr voraus
            if (!cleared) {
                if (!haveAhead || ahead.distance > CLEAR_FACTOR * STOP_DISTANCE)
                    cleared = true;
            }
            {
                bool reacquired = cleared && haveAhead&& std::fabs(ahead.bearing) < CENTER_TOL;
                bool timeout  = (ros::Time::now() - turn_start).toSec() > MAX_TURN_TIME;
                if (reacquired || timeout) state = DRIVE;
            }
            break;
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