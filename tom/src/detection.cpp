// detection.cpp
#include "detection.h"

ObjectDetector::ObjectDetector() : Node("object_detector") {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&ObjectDetector::scanCallback, this, std::placeholders::_1));

    object_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("detected_object", 10);
}

void ObjectDetector::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    processScan(msg);
}

void ObjectDetector::processScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    const int image_size = 500;
    const int scale = 100; // meters to pixels
    cv::Mat map = cv::Mat::zeros(image_size, image_size, CV_8UC1);
    cv::Point center(image_size / 2, image_size / 2);

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float range = msg->ranges[i];
        if (std::isfinite(range) && range > msg->range_min && range < msg->range_max) {
            float angle = msg->angle_min + i * msg->angle_increment;
            float x = range * cos(angle);
            float y = range * sin(angle);
            int img_x = static_cast<int>(center.x + x * scale);
            int img_y = static_cast<int>(center.y - y * scale); // invert y for image coordinates

            if (img_x >= 0 && img_x < image_size && img_y >= 0 && img_y < image_size) {
                map.at<uchar>(img_y, img_x) = 255;
            }
        }
    }

    // Apply morphology to help connect shape outlines
    cv::dilate(map, map, cv::Mat(), cv::Point(-1, -1), 1);
    cv::erode(map, map, cv::Mat(), cv::Point(-1, -1), 1);

    detectShapes(map);
}

void ObjectDetector::detectShapes(const cv::Mat &image) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(image.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < 100) continue; // filter noise

        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, 5, true);

        std::string shape;
        if (approx.size() == 3) shape = "triangle";
        else if (approx.size() == 4) shape = "square";
        else if (approx.size() > 5) shape = "circle";
        else continue;

        cv::Moments M = cv::moments(contour);
        if (M.m00 == 0) continue;
        int cx = static_cast<int>(M.m10 / M.m00);
        int cy = static_cast<int>(M.m01 / M.m00);

        // Convert back to robot-relative position
        float x = (cx - image.cols / 2) / 100.0;
        float y = -(cy - image.rows / 2) / 100.0;

        geometry_msgs::msg::Pose2D msg;
        msg.x = x;
        msg.y = y;
        if (shape == "circle") msg.theta = 1.0;
        else if (shape == "square") msg.theta = 2.0;
        else msg.theta = 3.0;

        RCLCPP_INFO(this->get_logger(), "Detected %s at (%.2f, %.2f)", shape.c_str(), x, y);
        object_pub_->publish(msg);
    }
}
// Main function (entry point)
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);  // Initialize ROS
    rclcpp::spin(std::make_shared<ObjectDetector>());  // Create and spin the node
    rclcpp::shutdown();  // Shutdown ROS
    return 0;
}