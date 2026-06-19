#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// Wird bei JEDEM neuen Kamerabild aufgerufen.
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv::Mat bild;
    try {
        // ROS-Bild -> OpenCV-Bild (cv::Mat). "bgr8" = Farbformat von OpenCV.
        bild = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge Fehler: %s", e.what());
        return;
    }

    // Bildverarbeitung
    cv::Mat grau, kanten;
    cv::cvtColor(bild, grau, cv::COLOR_BGR2GRAY); // in Graustufen
    cv::Canny(grau, kanten, 100, 200); // Kantendetektion

    // Anzeige
    cv::imshow("Kamera (original)", bild);
    cv::imshow("Kanten", kanten);
    cv::waitKey(1);   // ohne das öffnet sich kein Fenster (GUI-Events)
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;

    // Abonniert das Kamera-Topic; bei jedem Bild läuft imageCallback.
    ros::Subscriber sub = nh.subscribe("/camera/rgb/image_raw", 1, imageCallback);

    ROS_INFO("camera_node gestartet");
    ros::spin();   // wartet auf Bilder
    return 0;
}