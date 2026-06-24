#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

int canny_low  = 100;
int canny_high = 200;
bool show_windows = true;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv::Mat bild;
    try {
        bild = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge Fehler: %s", e.what());
        return;
    }

    cv::Mat grau, kanten;
    cv::cvtColor(bild, grau, cv::COLOR_BGR2GRAY);
    cv::Canny(grau, kanten, canny_low, canny_high);

    if (show_windows) {
        cv::imshow("Kamera (original)", bild);
        cv::imshow("Kanten", kanten);
        cv::waitKey(1);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv,"camera_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string image_topic;
    pnh.param<std::string>("image_topic", image_topic, "/camera/rgb/image_raw");
    pnh.param("canny_low", canny_low, 100);
    pnh.param("canny_high", canny_high, 200);
    pnh.param("show_windows", show_windows, true);

    ros::Subscriber sub = nh.subscribe(image_topic, 1, imageCallback);

    ROS_INFO("camera_node gestartet (topic=%s, canny=%d/%d)", image_topic.c_str(), canny_low, canny_high);
    ros::spin();

    cv::destroyAllWindows();
    return 0;
}