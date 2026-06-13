#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <limits>

// Aus dem letzten Scan abgeleitet: nicht ein Strahl, sondern ein Front-Kegel.
static double front_min = std::numeric_limits<double>::infinity(); // nächste Distanz im Kegel
static double front_bearing = 0.0; // Winkel dazu, + = links

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    const int N = (int)msg->ranges.size();
    if (N == 0) return;

    const double inc  = msg->angle_increment; // rad pro Index
    const int HALF = std::max(1, (int)std::lround((25.0 * M_PI / 180.0) / inc)); // ±25° in Indizes

    double best = std::numeric_limits<double>::infinity();
    double best_ang = 0.0;
    for (int k = -HALF; k <= HALF; ++k) {
        int idx = ((k % N) + N) % N; // negatives k auf die hinteren Indizes (rechte Seite) wickeln
        double r = msg->ranges[idx];
        if (std::isfinite(r) && r > 0.05 && r < best) {  // ungültige Werte raus, Minimum behalten
            best = r;
            best_ang = k * inc;
        }
    }
    front_min = best;
    front_bearing = best_ang;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_between");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/scan", 10, scanCallback);
    ros::Publisher  pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ROS_INFO("move_between gestartet");

    const double STOP_DISTANCE = 0.5; // so nah ans Objekt heranfahren
    const double CLEAR_DISTANCE = 3.4; // Front gilt als frei
    const double REACQUIRE_DISTANCE = 3.4; // ab hier zählt ein Objekt wieder als voraus
    const double CENTER_TOL = 10.0 * M_PI / 180.0;  // so mittig muss es liegen (±10°)
    const double FORWARD_SPEED = 0.15;
    const double TURN_SPEED = 0.6;
    const double STEER_GAIN = 1.2; // P-Korrektur: hält das Objekt mittig
    const double MAX_TURN_TIME = (M_PI / TURN_SPEED) * 1.5;  // Fallback gegen Hängenbleiben

    enum State { DRIVE, TURN } state = DRIVE;
    bool cleared = false; // schon vom alten Objekt weggedreht?
    ros::Time turn_start;
    ros::Rate rate(20);

    while (ros::ok()) {
        ros::spinOnce();
        geometry_msgs::Twist cmd;

        switch (state) {
        case DRIVE:
            if (front_min > STOP_DISTANCE) {
                cmd.linear.x  = FORWARD_SPEED;
                cmd.angular.z = STEER_GAIN * front_bearing; // schräges Auflaufen verhindern
            } else {
                state = TURN; // Objekt erreicht
                cleared = false;
                turn_start = ros::Time::now();
            }
            break;

        case TURN:
            cmd.angular.z = TURN_SPEED; // auf der Stelle drehen

            if (!cleared && front_min > CLEAR_DISTANCE)
                cleared = true; // 1. altes Objekt aus dem Blick gedreht

            // 2. weiter, bis das andere Objekt zentriert voraus liegt (Sensor entscheidet, keine blinde 180°-Zeit)
            bool reacquired = cleared && front_min < REACQUIRE_DISTANCE
                              && std::fabs(front_bearing) < CENTER_TOL;
            bool timeout    = (ros::Time::now() - turn_start).toSec() > MAX_TURN_TIME;

            if (reacquired || timeout)
                state = DRIVE;
            break;
        }

        pub.publish(cmd);
        rate.sleep();
    }
    return 0;
}