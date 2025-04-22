// detection.h
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <opencv2/opencv.hpp>

class ObjectDetector : public rclcpp::Node {
public:
    ObjectDetector();

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void processScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void detectShapes(const cv::Mat &image);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr object_pub_;
};
