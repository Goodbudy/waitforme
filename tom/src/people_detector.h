#ifndef PEOPLE_DETECTOR_H
#define PEOPLE_DETECTOR_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class PeopleDetector : public rclcpp::Node {
public:
    PeopleDetector();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void processImage(cv::Mat& image);
    void publishMarkers();

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

    cv::HOGDescriptor hog_;
    std::vector<cv::Rect> detected_people_;
    std::vector<geometry_msgs::msg::Point> people_positions_;
};

#endif // PEOPLE_DETECTOR_H
